/*
 * Offline unit tests for the MP6 binary codec.
 *
 * Every fixture is built explicitly here from the recovered protocol
 * description and cross-checked against the byte layouts in
 * motum_resources/motum-replacement-handoff/PROTOCOL.md and
 * recovered-source/MotSim/frm_Main.cs (RS232_Tx_Motor, SerialPortRead).
 * The test file does not use the production checksum or length helpers to
 * construct its fixtures; those helpers are asserted against constant values.
 */

#include "mp6_protocol.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <vector>

namespace {

int g_checks = 0;
int g_failures = 0;

void expectTrue(bool ok, const char* expression, int line) {
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL line %d: %s\n", line, expression);
    }
}

void expectEq(long long actual, long long expected, const char* actualExpr,
              const char* expectedExpr, int line) {
    ++g_checks;
    if (actual != expected) {
        ++g_failures;
        std::printf("FAIL line %d: %s == %s (actual %lld, expected %lld)\n",
                    line, actualExpr, expectedExpr, actual, expected);
    }
}

void expectNear(double actual, double expected, double epsilon, int line) {
    ++g_checks;
    if (!(std::fabs(actual - expected) <= epsilon)) {
        ++g_failures;
        std::printf("FAIL line %d: |%f - %f| > %f\n", line, actual, expected,
                    epsilon);
    }
}

void expectBytes(const std::array<std::uint8_t, mp6::kHostFrameBytes>& actual,
                 const std::array<std::uint8_t, mp6::kHostFrameBytes>& expected,
                 int line) {
    ++g_checks;
    for (std::size_t i = 0; i < expected.size(); ++i) {
        if (actual[i] != expected[i]) {
            ++g_failures;
            std::printf("FAIL line %d: byte %zu = 0x%02X, expected 0x%02X\n",
                        line, i, static_cast<unsigned>(actual[i]),
                        static_cast<unsigned>(expected[i]));
            return;
        }
    }
}

#define CHECK(cond) expectTrue(static_cast<bool>(cond), #cond, __LINE__)
#define CHECK_EQ(a, b) expectEq((a), (b), #a, #b, __LINE__)
#define CHECK_NEAR(a, b, eps) expectNear((a), (b), (eps), __LINE__)
#define CHECK_BYTES(a, b) expectBytes((a), (b), __LINE__)

using Bytes = std::vector<std::uint8_t>;

template <typename Container>
void putBe16(Container& bytes, std::size_t offset, std::uint16_t value) {
    bytes[offset] = static_cast<std::uint8_t>(value >> 8);
    bytes[offset + 1] = static_cast<std::uint8_t>(value & 0xFFu);
}

// Test-local checksum, intentionally independent of mp6::rxChecksumSum.
void seal(Bytes& frame) {
    std::uint32_t sum = 0;
    for (std::size_t i = 0; i + 2 < frame.size(); ++i) {
        sum += frame[i];
    }
    frame[frame.size() - 2] = static_cast<std::uint8_t>((sum >> 8) & 0xFFu);
    frame[frame.size() - 1] = static_cast<std::uint8_t>(sum & 0xFFu);
}

// Test-local length table, intentionally independent of mp6::rxFrameLength.
std::size_t fixtureLength(std::uint8_t type) {
    switch (type) {
    case 32:
    case 42:
    case 43:
        return 21;
    case 36:
    case 38:
    case 49:
        return 27;
    case 48:
        return 43;
    default:
        return 0;
    }
}

Bytes makeRx(std::uint8_t type) {
    Bytes frame(fixtureLength(type), 0);
    frame[0] = 0x80;
    frame[1] = 0x81;
    frame[2] = 0x82;
    frame[5] = type;
    return frame;
}

void append(Bytes& target, const Bytes& source) {
    target.insert(target.end(), source.begin(), source.end());
}

void feedFrame(mp6::Parser& parser, const Bytes& frame) {
    parser.feed(frame.data(), frame.size());
}

// ---------------------------------------------------------------------------
// TX: framing and checksum
// ---------------------------------------------------------------------------

void testTxFraming() {
    std::array<std::uint8_t, mp6::kHostPayloadBytes> payload{};
    payload[0] = 0x03;  // command 6: RunMode in payload byte 0

    const mp6::EncodeResult result =
        mp6::encodeFrame(static_cast<std::uint8_t>(mp6::HostCommand::RunMode),
                         payload, 5);
    CHECK(result);
    CHECK_EQ(static_cast<int>(result.error),
             static_cast<int>(mp6::EncodeError::None));

    // sum[0..15] = FE + FC + 06 + 03 = 515; 515 % 256 = 3. Sequence 5 is
    // excluded from the checksum.
    const std::array<std::uint8_t, 18> expected{
        0xFE, 0xFC, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x03};
    CHECK_BYTES(result.bytes, expected);
    CHECK_EQ(mp6::txChecksum(result.bytes), 3);
    CHECK_EQ(result.bytes[16], 5);

    // Sequence byte is the only excluded byte: changing it must not change
    // the checksum, changing a payload byte must.
    payload[1] = 0x01;
    const auto payloadChanged = mp6::encodeFrame(6, payload, 5);
    CHECK(payloadChanged);
    CHECK(payloadChanged.bytes[17] != result.bytes[17]);

    const auto sequenceChanged = mp6::encodeFrame(
        static_cast<std::uint8_t>(mp6::HostCommand::RunMode), payload, 6);
    CHECK(sequenceChanged);
    CHECK_EQ(sequenceChanged.bytes[17], payloadChanged.bytes[17]);

    // Wire sequence range is 0..127 only.
    const auto badSequence = mp6::encodeFrame(6, payload, 128);
    CHECK(!badSequence);
    CHECK_EQ(static_cast<int>(badSequence.error),
             static_cast<int>(mp6::EncodeError::InvalidSequence));
}

// ---------------------------------------------------------------------------
// TX: command 2 target conversion
// ---------------------------------------------------------------------------

void testTargetZeroBias() {
    const std::array<std::int32_t, 6> values{0, 0, 0, 0, 0, 0};
    const auto result = mp6::encodeTargetFrame(values, 0, 42);
    CHECK(result);

    // Each 0 + 32767 = 0x7FFF, little-endian FF 7F.
    // sum[0..15] = FE + FC + 02 + 6*(FF + 7F) = 2800; 2800 % 256 = 0xF0.
    const std::array<std::uint8_t, 18> expected{
        0xFE, 0xFC, 0x02, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F,
        0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0x00, 0x2A, 0xF0};
    CHECK_BYTES(result.bytes, expected);
    CHECK_EQ(mp6::txChecksum(result.bytes), 0xF0);
}

void testTargetBoundariesAndSpecial() {
    const std::array<mp6::AxisTarget, 6> targets{
        mp6::AxisTarget::ordinary(-32767),   // raw 0
        mp6::AxisTarget::ordinary(32768),    // raw 65535 = FF FF
        mp6::AxisTarget::ordinary(-1),       // raw 32766 = 7F FE -> FE 7F
        mp6::AxisTarget::ordinary(1),        // raw 32768 = 80 00 -> 00 80
        mp6::AxisTarget::ordinary(32767),    // raw 65534 = FF FE -> FE FF
        mp6::AxisTarget::specialWord()};     // raw 1 = 01 00

    const auto result = mp6::encodeTargetFrame(targets, 0xC0, 126);
    CHECK(result);

    // Byte sum: FE+FC+02 = 508; the six little-endian words contribute
    // 00+00 + FF+FF + FE+7F + 00+80 + FE+FF + 01+00 = 1529; flags C0 = 192.
    // 508 + 1529 + 192 = 2229; 2229 % 256 = 0xB5.
    const std::array<std::uint8_t, 18> expected{
        0xFE, 0xFC, 0x02, 0x00, 0x00, 0xFF, 0xFF, 0xFE, 0x7F,
        0x00, 0x80, 0xFE, 0xFF, 0x01, 0x00, 0xC0, 0x7E, 0xB5};
    CHECK_BYTES(result.bytes, expected);
    CHECK_EQ(result.bytes[15], 0xC0);  // ACPower | MotorDisable
    CHECK_EQ(mp6::kFlagAcPowerRequest, 0x40);
    CHECK_EQ(mp6::kFlagMotorDisable, 0x80);
}

void testTargetRejections() {
    std::array<std::int32_t, 6> values{};
    values[0] = -32768;
    auto result = mp6::encodeTargetFrame(values, 0, 0);
    CHECK(!result);
    CHECK_EQ(static_cast<int>(result.error),
             static_cast<int>(mp6::EncodeError::ValueOutOfRange));
    CHECK_EQ(result.fieldIndex, 0);

    values = {};
    values[2] = 32769;
    result = mp6::encodeTargetFrame(values, 0, 0);
    CHECK(!result);
    CHECK_EQ(static_cast<int>(result.error),
             static_cast<int>(mp6::EncodeError::ValueOutOfRange));
    CHECK_EQ(result.fieldIndex, 2);

    // -32766 is arithmetic but its biased word is 1, reserved for the special
    // per-axis value. It must not be silently coerced.
    values = {};
    values[1] = -32766;
    result = mp6::encodeTargetFrame(values, 0, 0);
    CHECK(!result);
    CHECK_EQ(static_cast<int>(result.error),
             static_cast<int>(mp6::EncodeError::ReservedTargetWord));
    CHECK_EQ(result.fieldIndex, 1);

    values = {};
    result = mp6::encodeTargetFrame(values, 0, 127);
    CHECK(result);
    result = mp6::encodeTargetFrame(values, 0, 128);
    CHECK(!result);
    CHECK_EQ(static_cast<int>(result.error),
             static_cast<int>(mp6::EncodeError::InvalidSequence));
}

// ---------------------------------------------------------------------------
// TX: command 5 configuration transaction
// ---------------------------------------------------------------------------

void testConfigEncoding() {
    const std::array<mp6::ConfigPair, 3> pairs{{
        {21, 1},
        {22, 100},
        {111, -50},
    }};
    const auto result = mp6::encodeConfigFrame(pairs, true, 7);
    CHECK(result);

    // 21+32767=0x8014, 1+32767=0x8000, 22+32767=0x8015,
    // 100+32767=0x8063, 111+32767=0x806E, -50+32767=0x7FCD.
    // Byte sum = FE+FC+05 + 148+128+149+227+238+332 + flag 1 = 1734;
    // 1734 % 256 = 0xC6.
    const std::array<std::uint8_t, 18> expected{
        0xFE, 0xFC, 0x05, 0x14, 0x80, 0x00, 0x80, 0x15, 0x80,
        0x63, 0x80, 0x6E, 0x80, 0xCD, 0x7F, 0x01, 0x07, 0xC6};
    CHECK_BYTES(result.bytes, expected);
    CHECK_EQ(result.bytes[15], 1);

    // Non-SendConfigToBoard calls put 0 in payload byte 12; only byte 15 and
    // the checksum change.
    const auto ordinary = mp6::encodeConfigFrame(pairs, false, 7);
    CHECK(ordinary);
    CHECK_EQ(ordinary.bytes[15], 0);
    CHECK_EQ(ordinary.bytes[17], 0xC5);
    for (std::size_t i = 0; i < 15; ++i) {
        CHECK_EQ(ordinary.bytes[i], result.bytes[i]);
    }

    // Begin packet from SendConfigToBoard: parameter 10 and five zero words.
    // Byte sum = FE+FC+05 + 09+80 + 5*(FF+7F) + flag 1 = 2559; %256 = 0xFF.
    const std::array<mp6::ConfigPair, 3> begin{{
        {10, 0},
        {0, 0},
        {0, 0},
    }};
    const auto beginFrame = mp6::encodeConfigFrame(begin, true, 0);
    CHECK(beginFrame);
    const std::array<std::uint8_t, 18> beginExpected{
        0xFE, 0xFC, 0x05, 0x09, 0x80, 0xFF, 0x7F, 0xFF, 0x7F,
        0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0x01, 0x00, 0xFF};
    CHECK_BYTES(beginFrame.bytes, beginExpected);

    // Boundaries accepted for all three pairs.
    const std::array<mp6::ConfigPair, 3> boundary{{
        {-32767, 32768},
        {-32767, 32768},
        {-32767, 32768},
    }};
    CHECK(mp6::encodeConfigFrame(boundary, false, 0));

    auto rejected = pairs;
    rejected[0].id = -32768;
    auto bad = mp6::encodeConfigFrame(rejected, true, 0);
    CHECK(!bad);
    CHECK_EQ(static_cast<int>(bad.error),
             static_cast<int>(mp6::EncodeError::ValueOutOfRange));
    CHECK_EQ(bad.fieldIndex, 0);

    rejected = pairs;
    rejected[2].value = 32769;
    bad = mp6::encodeConfigFrame(rejected, true, 0);
    CHECK(!bad);
    CHECK_EQ(static_cast<int>(bad.error),
             static_cast<int>(mp6::EncodeError::ValueOutOfRange));
    CHECK_EQ(bad.fieldIndex, 5);
}

// ---------------------------------------------------------------------------
// TX: sequence counter
// ---------------------------------------------------------------------------

void testSequenceCounter() {
    mp6::SequenceCounter counter;
    CHECK_EQ(counter.value(), 0);
    CHECK_EQ(counter.nextOrdinary(), 1);
    CHECK_EQ(counter.nextOrdinary(), 2);

    std::array<bool, 127> seen{};
    mp6::SequenceCounter cycle;
    for (int i = 0; i < 127; ++i) {
        const std::uint8_t value = cycle.nextOrdinary();
        CHECK(value <= 126);
        seen[value] = true;
    }
    for (std::size_t i = 0; i < seen.size(); ++i) {
        CHECK(seen[i]);
    }

    // 126 wraps to 0.
    mp6::SequenceCounter wrap(125);
    CHECK_EQ(wrap.nextOrdinary(), 126);
    CHECK_EQ(wrap.nextOrdinary(), 0);

    // Forced sequence reproduces the recovered writer: advance/wrap the
    // ordinary byte, override with 127, leave 127 stored.
    mp6::SequenceCounter forced;
    CHECK_EQ(forced.nextForced(), 127);
    CHECK_EQ(forced.value(), 127);
    CHECK_EQ(forced.nextOrdinary(), 0);
    CHECK_EQ(forced.nextForced(), 127);
    CHECK_EQ(forced.nextForced(), 127);

    // A duplicate frame (TxRepeat) is the same bytes with the same sequence.
    const std::array<std::int32_t, 6> values{1, 2, 3, 4, 5, 6};
    const auto first = mp6::encodeTargetFrame(values, 0, 33);
    const auto duplicate = mp6::encodeTargetFrame(values, 0, 33);
    CHECK(first);
    CHECK(duplicate);
    CHECK(std::memcmp(first.bytes.data(), duplicate.bytes.data(),
                      mp6::kHostFrameBytes) == 0);
}

// ---------------------------------------------------------------------------
// RX: length table and checksum
// ---------------------------------------------------------------------------

void testRxLengths() {
    CHECK_EQ(mp6::rxFrameLength(32), 21);
    CHECK_EQ(mp6::rxFrameLength(36), 27);
    CHECK_EQ(mp6::rxFrameLength(38), 27);
    CHECK_EQ(mp6::rxFrameLength(42), 21);
    CHECK_EQ(mp6::rxFrameLength(43), 21);
    CHECK_EQ(mp6::rxFrameLength(48), 43);
    CHECK_EQ(mp6::rxFrameLength(49), 27);

    const std::uint8_t unknown[] = {0, 1, 31, 33, 37, 39, 40, 41, 44, 47, 50, 255};
    for (std::uint8_t type : unknown) {
        CHECK_EQ(mp6::rxFrameLength(type), 0);
    }

    // Explicit checksum fixture: 21 bytes, bytes[0..18] all 1 -> sum 19.
    std::array<std::uint8_t, 21> frame{};
    frame.fill(1);
    frame[19] = 0x00;
    frame[20] = 0x13;
    CHECK_EQ(mp6::rxChecksumSum(frame.data(), frame.size()), 19);
    CHECK(mp6::rxChecksumValid(frame.data(), frame.size()));
    frame[20] = 0x12;
    CHECK(!mp6::rxChecksumValid(frame.data(), frame.size()));

    CHECK(!mp6::rxChecksumValid(nullptr, 0));
    CHECK(!mp6::rxChecksumValid(frame.data(), 2));
    CHECK_EQ(mp6::rxChecksumSum(frame.data(), 2), 0);
}

// ---------------------------------------------------------------------------
// RX: typed frames
// ---------------------------------------------------------------------------

void testType43AndCommonFields() {
    Bytes frame = makeRx(43);
    frame[3] = 0xEE;  // type 43 does not assign this byte
    frame[4] = 0xDD;
    frame[6] = 0xCC;
    frame[7] = 0x12;  // configuration count
    frame[18] = 10;   // ArdTime 1.0 s, included in the checksum
    seal(frame);

    mp6::Parser parser;
    CHECK_EQ(parser.feed(frame.data(), frame.size()), 1);
    mp6::RxFrame parsed;
    CHECK(parser.poll(parsed));
    CHECK_EQ(parsed.type, 43);
    CHECK_EQ(parsed.length, 21);
    CHECK_EQ(parsed.raw[3], 0xEE);
    CHECK_EQ(parsed.raw[4], 0xDD);
    CHECK_EQ(parsed.ardTimeRaw, 10);
    CHECK_NEAR(parsed.ardTimeSeconds, 1.0, 1e-9);

    const auto typed = parsed.asType43();
    CHECK(typed.has_value());
    if (typed) {
        CHECK_EQ(typed->configCount, 0x12);
    }
    CHECK(!parsed.asType48().has_value());
    CHECK(!parsed.asType42().has_value());
}

void testType42MotorHandshake() {
    Bytes frame = makeRx(42);
    frame[7] = 3;
    frame[9] = 9;
    frame[11] = 2;
    frame[12] = 0x01;
    frame[13] = 0x02;
    seal(frame);

    mp6::Parser parser;
    feedFrame(parser, frame);
    mp6::RxFrame parsed;
    CHECK(parser.poll(parsed));
    const auto typed = parsed.asType42();
    CHECK(typed.has_value());
    if (typed) {
        CHECK_EQ(typed->byte7, 3);
        CHECK_EQ(typed->byte9, 9);
        CHECK_EQ(typed->byte11, 2);
        CHECK(typed->handshakeRxM.has_value());
        CHECK_EQ(typed->handshakeRxM.value_or(0), 3009);
        CHECK(typed->runCodeIn1To3);
        CHECK_EQ(typed->motorBoardVersion, 258);
    }

    frame[7] = 4;
    seal(frame);
    CHECK(parser.feed(frame.data(), frame.size()) == 1);
    CHECK(parser.poll(parsed));
    const auto noHandshake = parsed.asType42();
    CHECK(noHandshake.has_value());
    if (noHandshake) {
        CHECK(!noHandshake->handshakeRxM.has_value());
    }
}

void testType32ControlStatus() {
    Bytes frame = makeRx(32);
    frame[7] = 19;
    frame[11] = 1;
    frame[12] = 7;
    seal(frame);

    mp6::Parser parser;
    feedFrame(parser, frame);
    mp6::RxFrame parsed;
    CHECK(parser.poll(parsed));
    const auto typed = parsed.asType32();
    CHECK(typed.has_value());
    if (typed) {
        CHECK(typed->byte7Below20);
        CHECK(typed->byte11IsOne);
        CHECK_EQ(typed->versionByte12, 7);
        CHECK_EQ(typed->byte7, 19);
        CHECK_EQ(typed->byte11, 1);
    }
}

void testType36Keyboard() {
    Bytes frame = makeRx(36);
    for (std::size_t i = 0; i < 8; ++i) {
        frame[7 + i * 2] = static_cast<std::uint8_t>('A' + i);
    }
    seal(frame);

    mp6::Parser parser;
    feedFrame(parser, frame);
    mp6::RxFrame parsed;
    CHECK(parser.poll(parsed));
    const auto typed = parsed.asType36();
    CHECK(typed.has_value());
    if (typed) {
        for (std::size_t i = 0; i < 8; ++i) {
            CHECK_EQ(typed->characters[i], static_cast<int>('A' + i));
        }
    }
}

void testType38ControlInputs() {
    Bytes frame = makeRx(38);
    putBe16(frame, 6, 0x1234);
    putBe16(frame, 8, 0x0001);
    putBe16(frame, 22, 0xBEEF);
    seal(frame);

    mp6::Parser parser;
    feedFrame(parser, frame);
    mp6::RxFrame parsed;
    CHECK(parser.poll(parsed));
    const auto typed = parsed.asType38();
    CHECK(typed.has_value());
    if (typed) {
        CHECK_EQ(typed->avControl[0], 0x1234);
        CHECK_EQ(typed->avControl[1], 0x0001);
        CHECK_EQ(typed->avControl[8], 0xBEEF);
    }
}

void testType49TestInputs() {
    Bytes frame = makeRx(49);
    for (std::size_t i = 0; i < 9; ++i) {
        putBe16(frame, 6 + i * 2, static_cast<std::uint16_t>(i));
    }
    seal(frame);

    mp6::Parser parser;
    feedFrame(parser, frame);
    mp6::RxFrame parsed;
    CHECK(parser.poll(parsed));
    const auto typed = parsed.asType49();
    CHECK(typed.has_value());
    if (typed) {
        for (std::size_t i = 0; i < 9; ++i) {
            CHECK_EQ(typed->testInputs[i], static_cast<int>(i));
        }
        // TestInput[3] = 2 -> 2 * 0.161 - 2.8 = -2.478.
        CHECK_NEAR(typed->torque, -2.478, 1e-9);
    }
}

// ---------------------------------------------------------------------------
// RX: type 48
// ---------------------------------------------------------------------------

void testType48TelemetryAndSelectors() {
    Bytes frame = makeRx(48);
    frame[3] = 0xEE;  // uninterpreted by the Windows type-48 branch
    frame[4] = 73;    // selector 7, status 3
    frame[5] = 48;

    const std::uint16_t words[6] = {4, 12, 20, 0, 32768, 32767};
    for (std::size_t i = 0; i < 6; ++i) {
        putBe16(frame, 6 + i * 2, words[i]);
    }
    putBe16(frame, 18, 0x1234);  // DigInput

    const std::uint16_t ardValues[10] = {
        7, 300, 31809, 1000, 2000, 17000, 16001, 15000, 240, 1234};
    for (std::size_t i = 0; i < 10; ++i) {
        putBe16(frame, 20 + i * 2, ardValues[i]);
    }
    frame[40] = 25;  // ArdTime 2.5 s
    seal(frame);

    mp6::Parser parser;
    feedFrame(parser, frame);
    mp6::RxFrame parsed;
    CHECK(parser.poll(parsed));
    const auto typed = parsed.asType48();
    CHECK(typed.has_value());
    if (!typed) {
        return;
    }

    CHECK_EQ(typed->rawPacking, 73);
    CHECK_EQ(typed->selector, 7);
    CHECK_EQ(typed->status, 3);
    CHECK(typed->configReceived);  // bit 0
    CHECK(typed->avOk);            // bit 1
    CHECK(!typed->statusBit2);     // status 3 has bit 2 clear
    CHECK_EQ(typed->digInput, 0x1234);
    CHECK_EQ(typed->boardCounter, 0x12);
    CHECK_NEAR(parsed.ardTimeSeconds, 2.5, 1e-9);

    // Banker's rounding of word/8.0: 4/8 -> 0, 12/8 -> 2, 20/8 -> 2.
    CHECK_EQ(typed->axes[0].rawCounts, 0);
    CHECK_EQ(typed->axes[1].rawCounts, 2);
    CHECK_EQ(typed->axes[2].rawCounts, 2);
    CHECK_EQ(typed->axes[3].rawCounts, 0);
    CHECK_EQ(typed->axes[4].rawCounts, 4096);
    CHECK_EQ(typed->axes[5].rawCounts, 4096);

    CHECK_EQ(static_cast<int>(typed->axes[0].potRange),
             static_cast<int>(mp6::PotRange::Low));
    CHECK_EQ(static_cast<int>(typed->axes[1].potRange),
             static_cast<int>(mp6::PotRange::Low));
    CHECK_EQ(static_cast<int>(typed->axes[4].potRange),
             static_cast<int>(mp6::PotRange::High));

    for (std::size_t i = 0; i < 10; ++i) {
        CHECK_EQ(typed->ardValues[i], ardValues[i]);
    }

    const auto selector = mp6::decodeSelector(*typed);
    CHECK(selector.index() == 7);
    const auto* supply = std::get_if<mp6::Selector7>(&selector);
    CHECK(supply != nullptr);
    if (supply != nullptr) {
        CHECK_EQ(supply->avInput12, 7);
        CHECK_EQ(supply->avInput13, 300);
        CHECK_EQ(supply->avInput14, 31809);
        CHECK_EQ(supply->supplyVolts[0], 3);   // (31809 & 31) * 3
        CHECK_EQ(supply->supplyVolts[1], 6);   // ((31809 >> 5) & 31) * 3
        CHECK_EQ(supply->supplyVolts[2], 93);  // ((31809 >> 10) & 31) * 3
    }

    const auto fault = mp6::decodeFaultCode(*typed);
    CHECK(fault.has_value());
    CHECK_EQ(static_cast<int>(fault.value_or(mp6::FaultCode::InternalPark)),
             240);

    // Non-freeplay context (RunMode 0): v4/v5 are board ADC values; v7 and
    // v10 are deliberately unassigned.
    const auto normal = mp6::decodeModeTelemetry(*typed, 0);
    CHECK(normal.index() == 1);
    const auto* other = std::get_if<mp6::NonFreeplayTelemetry>(&normal);
    CHECK(other != nullptr);
    if (other != nullptr) {
        CHECK_EQ(other->mainBoardAdc, 1000);
        CHECK_EQ(other->solenoidSupplyAdc, 2000);
        CHECK_NEAR(other->mainBoardVolts, 1000.0 * 2.7843 / 1024.0 * 5.0,
                   1e-9);
        CHECK_NEAR(other->solenoidSupplyVolts,
                   2000.0 * 2.7843 / 1024.0 * 5.0, 1e-9);
        CHECK_EQ(other->frequencyOffset, 1000);
        CHECK_EQ(other->targetOffset, -1000);
    }

    // Explicit freeplay context (RunMode 3).
    const auto freeplay = mp6::decodeModeTelemetry(*typed, 3);
    CHECK(freeplay.index() == 0);
    const auto* play = std::get_if<mp6::FreeplayTelemetry>(&freeplay);
    CHECK(play != nullptr);
    if (play != nullptr) {
        CHECK_EQ(play->freeplay, -15000);
        CHECK_EQ(play->calculatedPosition, -14000);
        CHECK_EQ(play->frequencyOffset, 1000);
        CHECK_EQ(play->velocityError, 1);
        CHECK_EQ(play->targetOffset, -1000);
        CHECK_EQ(play->txCount, 1234);
    }
}

void testType48StatusBitsAndPotRange() {
    for (int status = 0; status <= 9; ++status) {
        mp6::RxFrame frame;
        frame.type = 48;
        frame.length = 43;
        frame.raw[4] = static_cast<std::uint8_t>(status);
        const auto typed = frame.asType48();
        CHECK(typed.has_value());
        if (typed) {
            CHECK_EQ(typed->status, status);
            CHECK_EQ(typed->configReceived, (status & 1) != 0);
            CHECK_EQ(typed->avOk, (status & 2) != 0);
            CHECK_EQ(typed->statusBit2, (status & 4) != 0);
        }
    }

    // Pot thresholds: raw < 100 Low, raw > 3996 High, otherwise Ok. Words are
    // chosen so the division is exact (word = raw * 8).
    const std::uint16_t words[5] = {799, 800, 31968, 31976, 65535};
    const int expectedCounts[5] = {100, 100, 3996, 3997, 8192};
    for (int i = 0; i < 5; ++i) {
        mp6::RxFrame frame;
        frame.type = 48;
        frame.length = 43;
        frame.raw.fill(0);
        putBe16(frame.raw, 6, words[i]);
        const auto typed = frame.asType48();
        CHECK(typed.has_value());
        if (typed) {
            CHECK_EQ(typed->axes[0].rawCounts, expectedCounts[i]);
        }
    }
}

void testSelectorRotation() {
    mp6::Type48Data frame;

    frame.selector = 1;
    frame.ardValues = {0x01, 0x02, 0x03, 0, 0, 0, 0, 0, 0, 0};
    {
        const auto decoded = mp6::decodeSelector(frame);
        const auto* value = std::get_if<mp6::Selector1>(&decoded);
        CHECK(value != nullptr);
        if (value) {
            CHECK_EQ(value->platformNumber, 258);
            CHECK_EQ(value->boardType, 3);
        }
    }

    frame.selector = 2;
    frame.ardValues = {1, 2, 3, 0, 0, 0, 0, 0, 0, 0};
    {
        const auto decoded = mp6::decodeSelector(frame);
        const auto* value = std::get_if<mp6::Selector2>(&decoded);
        CHECK(value != nullptr);
        if (value) {
            CHECK_EQ(value->minutes, 65536 + 512 + 3);
        }
    }

    frame.selector = 3;
    frame.ardValues = {5, 16010, 77, 0, 0, 0, 0, 0, 0, 0};
    {
        const auto decoded = mp6::decodeSelector(frame);
        const auto* value = std::get_if<mp6::Selector3>(&decoded);
        CHECK(value != nullptr);
        if (value) {
            CHECK_EQ(value->loopCounter, 100);
            CHECK_EQ(value->pneumaticSolenoid, 10);
            CHECK_EQ(value->rxDelay, 77);
        }
    }

    frame.selector = 4;
    frame.ardValues = {2, 3, 4, 0, 0, 0, 0, 0, 0, 0};
    {
        const auto decoded = mp6::decodeSelector(frame);
        const auto* value = std::get_if<mp6::Selector4>(&decoded);
        CHECK(value != nullptr);
        if (value) {
            CHECK_EQ(value->runCounter, 515);
            CHECK_EQ(value->accCount, 4);
        }
    }

    frame.selector = 5;
    frame.ardValues = {16010, 15990, 1234, 0, 0, 0, 0, 0, 0, 0};
    {
        const auto decoded = mp6::decodeSelector(frame);
        const auto* value = std::get_if<mp6::Selector5>(&decoded);
        CHECK(value != nullptr);
        if (value) {
            CHECK_NEAR(value->proportional, 1.0, 1e-9);
            CHECK_NEAR(value->derivative, -1.0, 1e-9);
            CHECK_NEAR(value->stepCorrection, 0.1234, 1e-12);
        }
    }

    frame.selector = 6;
    frame.ardValues = {11, 12, 13, 0, 0, 0, 0, 0, 0, 0};
    {
        const auto decoded = mp6::decodeSelector(frame);
        const auto* value = std::get_if<mp6::Selector6>(&decoded);
        CHECK(value != nullptr);
        if (value) {
            CHECK_EQ(value->pneumaticAim, 11);
            CHECK_EQ(value->avInput10, 12);
            CHECK_EQ(value->avInput11, 13);
        }
    }

    frame.selector = 8;
    frame.ardValues = {7, 0, 99, 0, 0, 0, 0, 0, 0, 0};
    {
        const auto decoded = mp6::decodeSelector(frame);
        const auto* value = std::get_if<mp6::Selector8>(&decoded);
        CHECK(value != nullptr);
        if (value) {
            CHECK_EQ(value->runMode, 3);
            CHECK(value->okToRun);
            CHECK_EQ(value->microsSec, 99);
        }
    }

    frame.selector = 9;
    frame.ardValues = {0xBEEF, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    {
        const auto decoded = mp6::decodeSelector(frame);
        const auto* value = std::get_if<mp6::Selector9>(&decoded);
        CHECK(value != nullptr);
        if (value) {
            CHECK_EQ(value->checksumCurrent, 0xBEEF);
        }
    }

    // Selector 0 and >9 have no recovered meaning; preserve raw, do not guess.
    frame.selector = 0;
    CHECK(mp6::decodeSelector(frame).index() == 0);
    frame.selector = 10;
    CHECK(mp6::decodeSelector(frame).index() == 0);

    // Only the five recovered fault codes are named.
    frame.ardValues = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    CHECK(!mp6::decodeFaultCode(frame).has_value());
    frame.ardValues[8] = 210;
    CHECK_EQ(static_cast<int>(mp6::decodeFaultCode(frame).value_or(
                 mp6::FaultCode::InternalPark)),
             210);
    frame.ardValues[8] = 220;
    CHECK_EQ(static_cast<int>(mp6::decodeFaultCode(frame).value_or(
                 mp6::FaultCode::InternalPark)),
             220);
    frame.ardValues[8] = 230;
    CHECK_EQ(static_cast<int>(mp6::decodeFaultCode(frame).value_or(
                 mp6::FaultCode::InternalPark)),
             230);
    frame.ardValues[8] = 240;
    CHECK_EQ(static_cast<int>(mp6::decodeFaultCode(frame).value_or(
                 mp6::FaultCode::InternalPark)),
             240);
    frame.ardValues[8] = 250;
    CHECK_EQ(static_cast<int>(mp6::decodeFaultCode(frame).value_or(
                 mp6::FaultCode::InternalPark)),
             250);
    frame.ardValues[8] = 211;
    CHECK(!mp6::decodeFaultCode(frame).has_value());
}

// ---------------------------------------------------------------------------
// Parser behaviour
// ---------------------------------------------------------------------------

void testParserSplitAndMultipleFrames() {
    Bytes first = makeRx(43);
    first[7] = 1;
    seal(first);
    Bytes second = makeRx(42);
    second[7] = 3;
    second[9] = 4;
    seal(second);
    Bytes third = makeRx(49);
    seal(third);

    mp6::Parser split;
    std::size_t emitted = 0;
    for (std::size_t i = 0; i < first.size(); ++i) {
        emitted += split.feed(first.data() + i, 1);
        if (i + 1 < first.size()) {
            CHECK_EQ(split.pendingFrames(), 0);
            CHECK_EQ(split.pendingBytes(), i + 1);
        }
    }
    CHECK_EQ(emitted, 1);
    CHECK_EQ(split.pendingFrames(), 1);
    CHECK_EQ(split.pendingBytes(), 0);
    mp6::RxFrame parsed;
    CHECK(split.poll(parsed));
    CHECK_EQ(parsed.type, 43);

    Bytes combined;
    append(combined, first);
    append(combined, second);
    append(combined, third);
    mp6::Parser parser;
    CHECK_EQ(parser.feed(combined.data(), combined.size()), 3);
    CHECK_EQ(parser.pendingFrames(), 3);
    CHECK(parser.poll(parsed));
    CHECK_EQ(parsed.type, 43);
    CHECK(parser.poll(parsed));
    CHECK_EQ(parsed.type, 42);
    CHECK(parser.poll(parsed));
    CHECK_EQ(parsed.type, 49);
    CHECK(!parser.poll(parsed));
}

void testParserCorruptionResync() {
    Bytes first = makeRx(43);
    first[7] = 1;
    seal(first);
    Bytes second = makeRx(43);
    second[7] = 2;
    seal(second);

    // Damage only the big-endian checksum tail of the first frame.
    first[20] ^= 0xFF;

    Bytes combined;
    append(combined, first);
    append(combined, second);

    mp6::Parser parser;
    CHECK_EQ(parser.feed(combined.data(), combined.size()), 1);
    CHECK_EQ(parser.stats().badChecksums, 1);
    mp6::RxFrame parsed;
    CHECK(parser.poll(parsed));
    CHECK_EQ(parsed.type, 43);
    const auto typed = parsed.asType43();
    CHECK(typed.has_value());
    if (typed) {
        CHECK_EQ(typed->configCount, 2);
    }
}

void testParserMarkerInsidePayload() {
    Bytes frame = makeRx(48);
    // Put 80 81 82 across the first two ArdTVal words.
    frame[20] = 0x80;
    frame[21] = 0x81;
    frame[22] = 0x82;
    seal(frame);

    mp6::Parser parser;
    CHECK_EQ(parser.feed(frame.data(), frame.size()), 1);
    CHECK_EQ(parser.stats().badChecksums, 0);
    mp6::RxFrame parsed;
    CHECK(parser.poll(parsed));
    CHECK_EQ(parsed.type, 48);
    CHECK_EQ(parsed.length, 43);
    CHECK_EQ(parsed.raw[20], 0x80);
    CHECK_EQ(parsed.raw[21], 0x81);
    CHECK_EQ(parsed.raw[22], 0x82);
    CHECK(!parser.poll(parsed));

    // A corrupt frame with an embedded marker must still resync to the next
    // real frame rather than emitting the embedded marker as a frame.
    Bytes corrupt = makeRx(48);
    corrupt[20] = 0x80;
    corrupt[21] = 0x81;
    corrupt[22] = 0x82;
    corrupt[25] = 0;  // byte after the embedded marker: unknown type
    seal(corrupt);
    corrupt[42] ^= 0xFF;

    Bytes good = makeRx(43);
    good[7] = 9;
    seal(good);

    Bytes combined;
    append(combined, corrupt);
    append(combined, good);

    mp6::Parser resync;
    CHECK_EQ(resync.feed(combined.data(), combined.size()), 1);
    CHECK(resync.stats().badChecksums >= 1);
    CHECK(resync.poll(parsed));
    CHECK_EQ(parsed.type, 43);
    const auto typed = parsed.asType43();
    CHECK(typed.has_value());
    if (typed) {
        CHECK_EQ(typed->configCount, 9);
    }
}

void testParserUnknownType() {
    Bytes unknown(9, 0);
    unknown[0] = 0x80;
    unknown[1] = 0x81;
    unknown[2] = 0x82;
    unknown[5] = 37;  // no recovered length
    unknown[8] = 0xAA;

    Bytes good = makeRx(43);
    good[7] = 5;
    seal(good);

    Bytes combined;
    append(combined, unknown);
    append(combined, good);

    mp6::Parser parser;
    CHECK_EQ(parser.feed(combined.data(), combined.size()), 1);
    CHECK(parser.stats().unknownTypes >= 1);
    mp6::RxFrame parsed;
    CHECK(parser.poll(parsed));
    CHECK_EQ(parsed.type, 43);
    CHECK_EQ(parsed.length, 21);
}

void testParserTruncation() {
    Bytes frame = makeRx(43);
    frame[7] = 0x33;
    seal(frame);

    mp6::Parser parser;
    CHECK_EQ(parser.feed(frame.data(), 10), 0);
    CHECK_EQ(parser.pendingFrames(), 0);
    CHECK_EQ(parser.pendingBytes(), 10);
    CHECK_EQ(parser.feed(frame.data() + 10, frame.size() - 10), 1);
    CHECK_EQ(parser.pendingBytes(), 0);
    mp6::RxFrame parsed;
    CHECK(parser.poll(parsed));
    CHECK_EQ(parsed.type, 43);
    const auto typed = parsed.asType43();
    CHECK(typed.has_value());
    if (typed) {
        CHECK_EQ(typed->configCount, 0x33);
    }

    // Truncated prefix followed by a new complete frame: discard the dead
    // prefix and recover the complete frame.
    Bytes tail(frame.begin(), frame.begin() + 10);
    Bytes good = makeRx(43);
    good[7] = 0x44;
    seal(good);
    Bytes combined;
    append(combined, tail);
    append(combined, good);

    mp6::Parser resync;
    CHECK_EQ(resync.feed(combined.data(), combined.size()), 1);
    CHECK(resync.stats().badChecksums >= 1);
    CHECK(resync.poll(parsed));
    const auto recovered = parsed.asType43();
    CHECK(recovered.has_value());
    if (recovered) {
        CHECK_EQ(recovered->configCount, 0x44);
    }

    // A lone partial marker is retained, not dropped.
    mp6::Parser partial;
    const std::uint8_t markerPrefix[2] = {0x80, 0x81};
    CHECK_EQ(partial.feed(markerPrefix, 2), 0);
    CHECK_EQ(partial.pendingBytes(), 2);
}

void testParserBounds() {
    // Garbage must not accumulate beyond the configured capacity.
    mp6::Parser parser(64);
    CHECK_EQ(parser.capacity(), 64);
    Bytes garbage(5000, 0x55);
    CHECK_EQ(parser.feed(garbage.data(), garbage.size()), 0);
    CHECK(parser.pendingBytes() <= parser.capacity());
    CHECK(parser.pendingBytes() <= 2);  // at most a split-marker prefix is held
    CHECK_EQ(parser.stats().skippedBytes + parser.pendingBytes(), 5000);

    Bytes good = makeRx(43);
    good[7] = 0x77;
    seal(good);
    CHECK_EQ(parser.feed(good.data(), good.size()), 1);
    mp6::RxFrame parsed;
    CHECK(parser.poll(parsed));
    CHECK_EQ(parsed.type, 43);

    // Capacity below the documented minimum is clamped.
    mp6::Parser tiny(1);
    CHECK_EQ(tiny.capacity(), mp6::kMinParserCapacity);

    // Queued frames are bounded; overflow drops the oldest and counts it.
    mp6::Parser bounded;
    Bytes stream;
    for (int i = 0; i < 100; ++i) {
        Bytes frame = makeRx(43);
        frame[7] = static_cast<std::uint8_t>(i);
        seal(frame);
        append(stream, frame);
    }
    CHECK_EQ(bounded.feed(stream.data(), stream.size()), 100);
    CHECK_EQ(bounded.stats().framesEmitted, 100);
    CHECK_EQ(bounded.stats().queueDroppedFrames,
             100 - static_cast<int>(mp6::kMaxQueuedFrames));
    CHECK_EQ(bounded.pendingFrames(), mp6::kMaxQueuedFrames);

    int last = -1;
    int count = 0;
    while (bounded.poll(parsed)) {
        const auto typed = parsed.asType43();
        CHECK(typed.has_value());
        if (typed) {
            last = typed->configCount;
        }
        ++count;
    }
    CHECK_EQ(count, 64);
    CHECK_EQ(last, 99);

    bounded.reset();
    CHECK_EQ(bounded.pendingFrames(), 0);
    CHECK_EQ(bounded.pendingBytes(), 0);
    CHECK_EQ(bounded.stats().bytesFed, 0);
}

}  // namespace

int main() {
    testTxFraming();
    testTargetZeroBias();
    testTargetBoundariesAndSpecial();
    testTargetRejections();
    testConfigEncoding();
    testSequenceCounter();

    testRxLengths();
    testType43AndCommonFields();
    testType42MotorHandshake();
    testType32ControlStatus();
    testType36Keyboard();
    testType38ControlInputs();
    testType49TestInputs();
    testType48TelemetryAndSelectors();
    testType48StatusBitsAndPotRange();
    testSelectorRotation();

    testParserSplitAndMultipleFrames();
    testParserCorruptionResync();
    testParserMarkerInsidePayload();
    testParserUnknownType();
    testParserTruncation();
    testParserBounds();

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
