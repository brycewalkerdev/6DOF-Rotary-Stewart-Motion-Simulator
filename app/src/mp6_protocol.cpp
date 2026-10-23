/*
 * MP6 protocol codec implementation. See mp6_protocol.h for wire layout and
 * evidence citations. This translation unit is platform-independent C++17.
 */

#include "mp6_protocol.h"

#include <algorithm>
#include <cstring>

namespace mp6 {
namespace {

constexpr std::uint8_t payloadWordLow(std::uint16_t word) noexcept {
    return static_cast<std::uint8_t>(word & 0xFFu);
}

constexpr std::uint8_t payloadWordHigh(std::uint16_t word) noexcept {
    return static_cast<std::uint8_t>((word >> 8) & 0xFFu);
}

std::uint16_t readBigEndian16(const std::uint8_t* p) noexcept {
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(p[0]) << 8) |
        static_cast<std::uint16_t>(p[1]));
}

// The recovered host uses Math.Round(word / 8.0), whose default is
// round-half-to-even. Implement that exactly with integer arithmetic so the
// codec does not depend on the floating-point rounding mode.
int sensorRawCounts(std::uint16_t word) noexcept {
    unsigned quotient = static_cast<unsigned>(word / 8u);
    unsigned remainder = static_cast<unsigned>(word % 8u);
    if (remainder > 4u) {
        ++quotient;
    } else if (remainder == 4u && (quotient & 1u) != 0u) {
        ++quotient;
    }
    return static_cast<int>(quotient);
}

bool hasTypeAndLength(const RxFrame& frame, std::uint8_t type,
                      std::size_t length) noexcept {
    return frame.type == type && frame.length == length &&
           frame.length <= kMaxRxFrameBytes;
}

}  // namespace

// ---------------------------------------------------------------------------
// Host TX
// ---------------------------------------------------------------------------

std::uint8_t txChecksum(
    const std::array<std::uint8_t, kHostFrameBytes>& frame) noexcept {
    unsigned sum = 0;
    for (std::size_t i = 0; i < kHostSequenceOffset; ++i) {
        sum += frame[i];
    }
    return static_cast<std::uint8_t>(sum & 0xFFu);
}

std::uint8_t SequenceCounter::nextOrdinary() noexcept {
    if (value_ >= kLastOrdinarySequence) {
        value_ = kFirstOrdinarySequence;
    } else {
        ++value_;
    }
    return value_;
}

std::uint8_t SequenceCounter::nextForced() noexcept {
    // The recovered writer advances/wraps TXCount first, then overwrites the
    // emitted byte with 127, leaving the member at 127. Reproduce that order.
    (void)nextOrdinary();
    value_ = kForcedSequence;
    return value_;
}

EncodeResult encodeFrame(
    std::uint8_t command,
    const std::array<std::uint8_t, kHostPayloadBytes>& payload,
    std::uint8_t sequence) noexcept {
    EncodeResult result;
    if (sequence > kForcedSequence) {
        result.error = EncodeError::InvalidSequence;
        return result;
    }

    result.bytes[0] = kHostMarker0;
    result.bytes[1] = kHostMarker1;
    result.bytes[2] = command;
    for (std::size_t i = 0; i < kHostPayloadBytes; ++i) {
        result.bytes[kHostPayloadOffset + i] = payload[i];
    }
    result.bytes[kHostSequenceOffset] = sequence;
    result.bytes[kHostChecksumOffset] = txChecksum(result.bytes);
    return result;
}

EncodeResult encodeTargetFrame(
    const std::array<AxisTarget, 6>& targets,
    std::uint8_t flags,
    std::uint8_t sequence) noexcept {
    EncodeResult result;
    if (sequence > kForcedSequence) {
        result.error = EncodeError::InvalidSequence;
        return result;
    }

    std::array<std::uint8_t, kHostPayloadBytes> payload{};
    for (std::size_t i = 0; i < targets.size(); ++i) {
        std::uint16_t word = 0;
        if (targets[i].kind == AxisTarget::Kind::Special) {
            // PEP[i] > 0 replaces the axis word with 1 (Update_Stepper_4).
            word = 1;
        } else {
            const std::int32_t value = targets[i].value;
            if (value < -32767 || value > 32768) {
                result.error = EncodeError::ValueOutOfRange;
                result.fieldIndex = static_cast<int>(i);
                return result;
            }
            word = static_cast<std::uint16_t>(value + 32767);
            if (word == 1) {
                // -32766 is arithmetically representable but the host/firmware
                // reserves raw word 1 for the per-axis special value.
                result.error = EncodeError::ReservedTargetWord;
                result.fieldIndex = static_cast<int>(i);
                return result;
            }
        }
        payload[i * 2] = payloadWordLow(word);
        payload[i * 2 + 1] = payloadWordHigh(word);
    }
    // Payload byte 12: ACPower bit 6, MotorDisable bit 7, lower six bits pass
    // through as supplied by the caller.
    payload[12] = flags;
    return encodeFrame(static_cast<std::uint8_t>(HostCommand::Target), payload,
                       sequence);
}

EncodeResult encodeTargetFrame(
    const std::array<std::int32_t, 6>& values,
    std::uint8_t flags,
    std::uint8_t sequence) noexcept {
    std::array<AxisTarget, 6> targets{};
    for (std::size_t i = 0; i < values.size(); ++i) {
        targets[i] = AxisTarget::ordinary(values[i]);
    }
    return encodeTargetFrame(targets, flags, sequence);
}

EncodeResult encodeConfigFrame(
    const std::array<ConfigPair, 3>& pairs,
    bool sendConfig,
    std::uint8_t sequence) noexcept {
    EncodeResult result;
    if (sequence > kForcedSequence) {
        result.error = EncodeError::InvalidSequence;
        return result;
    }

    std::array<std::uint8_t, kHostPayloadBytes> payload{};
    for (std::size_t pair = 0; pair < pairs.size(); ++pair) {
        const std::int32_t fields[2] = {pairs[pair].id, pairs[pair].value};
        for (std::size_t f = 0; f < 2; ++f) {
            if (fields[f] < -32767 || fields[f] > 32768) {
                result.error = EncodeError::ValueOutOfRange;
                result.fieldIndex = static_cast<int>(pair * 2 + f);
                return result;
            }
            const std::size_t payloadIndex = pair * 4 + f * 2;
            const auto word = static_cast<std::uint16_t>(fields[f] + 32767);
            payload[payloadIndex] = payloadWordLow(word);
            payload[payloadIndex + 1] = payloadWordHigh(word);
        }
    }
    // Payload byte 12 is the SendConfigToBoard marker, not part of a word.
    payload[12] = sendConfig ? 1u : 0u;
    return encodeFrame(static_cast<std::uint8_t>(HostCommand::Config), payload,
                       sequence);
}

// ---------------------------------------------------------------------------
// RX checksums and lengths
// ---------------------------------------------------------------------------

std::uint16_t rxChecksumSum(const std::uint8_t* frame,
                            std::size_t length) noexcept {
    if (frame == nullptr || length < 3) {
        return 0;
    }
    unsigned sum = 0;
    for (std::size_t i = 0; i + 2 < length; ++i) {
        sum += frame[i];
    }
    return static_cast<std::uint16_t>(sum);
}

bool rxChecksumValid(const std::uint8_t* frame, std::size_t length) noexcept {
    if (frame == nullptr || length < 3) {
        return false;
    }
    const std::uint16_t expected =
        readBigEndian16(frame + (length - 2));
    return rxChecksumSum(frame, length) == expected;
}

// ---------------------------------------------------------------------------
// Typed frame accessors
// ---------------------------------------------------------------------------

std::optional<Type32Data> RxFrame::asType32() const noexcept {
    if (!hasTypeAndLength(*this, 32, 21)) {
        return std::nullopt;
    }
    Type32Data out;
    out.byte7 = raw[7];
    out.byte11 = raw[11];
    out.byte12 = raw[12];
    out.byte7Below20 = raw[7] < 20;
    out.byte11IsOne = raw[11] == 1;
    out.versionByte12 = raw[12];
    return out;
}

std::optional<Type36Data> RxFrame::asType36() const noexcept {
    if (!hasTypeAndLength(*this, 36, 27)) {
        return std::nullopt;
    }
    Type36Data out;
    for (std::size_t i = 0; i < out.characters.size(); ++i) {
        out.characters[i] = raw[7 + i * 2];
    }
    return out;
}

std::optional<Type38Data> RxFrame::asType38() const noexcept {
    if (!hasTypeAndLength(*this, 38, 27)) {
        return std::nullopt;
    }
    Type38Data out;
    for (std::size_t i = 0; i < out.avControl.size(); ++i) {
        out.avControl[i] = readBigEndian16(raw.data() + 6 + i * 2);
    }
    return out;
}

std::optional<Type42Data> RxFrame::asType42() const noexcept {
    if (!hasTypeAndLength(*this, 42, 21)) {
        return std::nullopt;
    }
    Type42Data out;
    out.byte7 = raw[7];
    out.byte9 = raw[9];
    out.byte11 = raw[11];
    out.byte12 = raw[12];
    out.byte13 = raw[13];
    if (raw[7] == 3) {
        out.handshakeRxM = static_cast<int>(raw[7]) * 1000 +
                           static_cast<int>(raw[9]);
    }
    out.runCodeIn1To3 = raw[11] > 0 && raw[11] < 4;
    out.motorBoardVersion = readBigEndian16(raw.data() + 12);
    return out;
}

std::optional<Type43Data> RxFrame::asType43() const noexcept {
    if (!hasTypeAndLength(*this, 43, 21)) {
        return std::nullopt;
    }
    Type43Data out;
    out.configCount = raw[7];
    return out;
}

std::optional<Type48Data> RxFrame::asType48() const noexcept {
    if (!hasTypeAndLength(*this, 48, 43)) {
        return std::nullopt;
    }
    Type48Data out;
    out.rawPacking = raw[4];
    out.selector = static_cast<std::uint8_t>(raw[4] / 10u);
    out.status = static_cast<std::uint8_t>(raw[4] % 10u);
    out.configReceived = (out.status & 0x01u) != 0u;
    out.avOk = (out.status & 0x02u) != 0u;
    out.statusBit2 = (out.status & 0x04u) != 0u;

    for (std::size_t i = 0; i < out.axes.size(); ++i) {
        const std::uint16_t word = readBigEndian16(raw.data() + 6 + i * 2);
        out.axes[i].word = word;
        out.axes[i].rawCounts = sensorRawCounts(word);
        if (out.axes[i].rawCounts < 100) {
            out.axes[i].potRange = PotRange::Low;
        } else if (out.axes[i].rawCounts > 3996) {
            out.axes[i].potRange = PotRange::High;
        } else {
            out.axes[i].potRange = PotRange::Ok;
        }
    }
    out.digInput = readBigEndian16(raw.data() + 18);
    out.boardCounter = raw[18];
    for (std::size_t i = 0; i < out.ardValues.size(); ++i) {
        out.ardValues[i] = readBigEndian16(raw.data() + 20 + i * 2);
    }
    return out;
}

std::optional<Type49Data> RxFrame::asType49() const noexcept {
    if (!hasTypeAndLength(*this, 49, 27)) {
        return std::nullopt;
    }
    Type49Data out;
    for (std::size_t i = 0; i < out.testInputs.size(); ++i) {
        out.testInputs[i] = readBigEndian16(raw.data() + 6 + i * 2);
    }
    out.torque = static_cast<double>(out.testInputs[2]) * 0.161 - 2.8;
    return out;
}

// ---------------------------------------------------------------------------
// Type 48 multiplexing
// ---------------------------------------------------------------------------

SelectorData decodeSelector(const Type48Data& frame) noexcept {
    const auto& v = frame.ardValues;
    switch (frame.selector) {
    case 1: {
        Selector1 out;
        out.platformNumber =
            static_cast<int>(v[0]) * 256 + static_cast<int>(v[1]);
        out.boardType = static_cast<int>(v[2]);
        return out;
    }
    case 2: {
        Selector2 out;
        out.minutes = static_cast<std::int64_t>(v[0]) * 65536 +
                      static_cast<std::int64_t>(v[1]) * 256 +
                      static_cast<std::int64_t>(v[2]);
        return out;
    }
    case 3: {
        Selector3 out;
        out.loopCounter = static_cast<int>(v[0]) * 20;
        out.pneumaticSolenoid = static_cast<int>(v[1]) - 16000;
        out.rxDelay = v[2];
        return out;
    }
    case 4: {
        Selector4 out;
        out.runCounter =
            static_cast<int>(v[0]) * 256 + static_cast<int>(v[1]);
        out.accCount = static_cast<int>(v[2]);
        return out;
    }
    case 5: {
        Selector5 out;
        out.proportional = (static_cast<double>(v[0]) - 16000.0) * 0.1;
        out.derivative = (static_cast<double>(v[1]) - 16000.0) * 0.1;
        out.stepCorrection = static_cast<double>(v[2]) * 0.0001;
        return out;
    }
    case 6: {
        Selector6 out;
        out.pneumaticAim = v[0];
        out.avInput10 = v[1];
        out.avInput11 = v[2];
        return out;
    }
    case 7: {
        Selector7 out;
        out.avInput12 = v[0];
        out.avInput13 = v[1];
        out.avInput14 = v[2];
        const std::uint16_t packed = v[2];
        out.supplyVolts[0] = static_cast<int>(packed & 31u) * 3;
        out.supplyVolts[1] = static_cast<int>((packed >> 5) & 31u) * 3;
        out.supplyVolts[2] = static_cast<int>((packed >> 10) & 31u) * 3;
        return out;
    }
    case 8: {
        Selector8 out;
        out.runMode = static_cast<int>(v[0] % 4u);
        out.okToRun = (v[0] & 4u) != 0u;
        out.microsSec = v[2];
        return out;
    }
    case 9: {
        Selector9 out;
        out.checksumCurrent = v[0];
        return out;
    }
    default:
        return std::monostate{};
    }
}

ModeTelemetry decodeModeTelemetry(const Type48Data& frame,
                                  int runMode) noexcept {
    const auto& v = frame.ardValues;
    const int frequencyOffset = static_cast<int>(v[5]) - 16000;
    const int targetOffset = static_cast<int>(v[7]) - 16000;

    if (runMode == 3) {
        FreeplayTelemetry out;
        out.freeplay = static_cast<int>(v[3]) - 16000;
        out.calculatedPosition = static_cast<int>(v[4]) - 16000;
        out.frequencyOffset = frequencyOffset;
        out.velocityError = static_cast<int>(v[6]) - 16000;
        out.targetOffset = targetOffset;
        out.txCount = v[9];
        return out;
    }

    NonFreeplayTelemetry out;
    out.mainBoardAdc = v[3];
    out.solenoidSupplyAdc = v[4];
    out.mainBoardVolts =
        static_cast<double>(out.mainBoardAdc) * 2.7843 / 1024.0 * 5.0;
    out.solenoidSupplyVolts =
        static_cast<double>(out.solenoidSupplyAdc) * 2.7843 / 1024.0 * 5.0;
    out.frequencyOffset = frequencyOffset;
    out.targetOffset = targetOffset;
    return out;
}

std::optional<FaultCode> decodeFaultCode(const Type48Data& frame) noexcept {
    switch (frame.ardValues[8]) {
    case 210:
        return FaultCode::InternalPark;
    case 220:
        return FaultCode::AdPotError;
    case 230:
        return FaultCode::LowVolts;
    case 240:
        return FaultCode::EStop;
    case 250:
        return FaultCode::CommsDrop;
    default:
        return std::nullopt;
    }
}

// ---------------------------------------------------------------------------
// Incremental parser
// ---------------------------------------------------------------------------

Parser::Parser(std::size_t capacity)
    : capacity_(std::max(capacity, kMinParserCapacity)) {}

std::size_t Parser::feed(const std::uint8_t* data, std::size_t length) {
    if (data == nullptr || length == 0) {
        return 0;
    }

    stats_.bytesFed += length;
    std::size_t emitted = 0;

    while (length > 0) {
        if (buffer_.size() >= capacity_) {
            // A full buffer with no consumable frame means a truncated or
            // corrupt prefix; discard the oldest byte and keep scanning.
            buffer_.erase(buffer_.begin());
            ++stats_.overflowDroppedBytes;
            ++stats_.skippedBytes;
        }

        const std::size_t space = capacity_ - buffer_.size();
        const std::size_t take = std::min(space, length);
        buffer_.insert(buffer_.end(), data, data + take);
        data += take;
        length -= take;

        emitted += parseAvailable();
    }
    return emitted;
}

std::size_t Parser::feed(const std::vector<std::uint8_t>& data) {
    return feed(data.data(), data.size());
}

std::size_t Parser::parseAvailable() {
    std::size_t i = 0;
    std::size_t emitted = 0;

    for (;;) {
        const std::size_t remaining = buffer_.size() - i;
        if (remaining < kRxMarkerBytes) {
            break;
        }

        const bool marker = buffer_[i] == kRxMarker[0] &&
                            buffer_[i + 1] == kRxMarker[1] &&
                            buffer_[i + 2] == kRxMarker[2];
        if (!marker) {
            ++i;
            ++stats_.skippedBytes;
            continue;
        }

        // Type lives at byte 5, so the marker plus bytes 3 and 4 must be
        // present before its length can be known.
        if (remaining < kRxTypeOffset + 1) {
            break;
        }

        const std::uint8_t type = buffer_[i + kRxTypeOffset];
        const std::size_t frameLength = rxFrameLength(type);
        if (frameLength == 0) {
            ++stats_.unknownTypes;
            ++i;
            ++stats_.skippedBytes;
            continue;
        }

        if (remaining < frameLength) {
            break;  // truncated tail: retain and wait for more bytes
        }

        if (!rxChecksumValid(buffer_.data() + i, frameLength)) {
            ++stats_.badChecksums;
            ++i;
            ++stats_.skippedBytes;
            continue;
        }

        RxFrame frame;
        frame.type = type;
        frame.length = frameLength;
        std::memcpy(frame.raw.data(), buffer_.data() + i, frameLength);
        frame.ardTimeRaw = frame.raw[frameLength - 3];
        frame.ardTimeSeconds = static_cast<double>(frame.ardTimeRaw) * 0.1;

        queueFrame(std::move(frame));
        ++stats_.framesEmitted;
        ++emitted;
        i += frameLength;
    }

    if (i > 0) {
        buffer_.erase(buffer_.begin(),
                      buffer_.begin() + static_cast<std::ptrdiff_t>(i));
    }
    return emitted;
}

void Parser::queueFrame(RxFrame&& frame) {
    if (queue_.size() >= kMaxQueuedFrames) {
        queue_.pop_front();
        ++stats_.queueDroppedFrames;
    }
    queue_.push_back(std::move(frame));
}

bool Parser::poll(RxFrame& out) {
    if (queue_.empty()) {
        return false;
    }
    out = std::move(queue_.front());
    queue_.pop_front();
    return true;
}

void Parser::reset() noexcept {
    buffer_.clear();
    queue_.clear();
    stats_ = ParserStats{};
}

}  // namespace mp6
