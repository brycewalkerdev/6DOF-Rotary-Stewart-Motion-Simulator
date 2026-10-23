#pragma once
/*
 * MP6 host/controller binary protocol codec (pure C++17, namespace mp6).
 *
 * This module owns only wire encoding/decoding for the unmodified MOTUM MP6
 * Arduino Mega controller. It has no hardware, Win32, filesystem, threading or
 * UI dependencies, and it never opens a COM port.
 *
 * Evidence:
 *   motum_resources/motum-replacement-handoff/PROTOCOL.md
 *   motum_resources/motum-replacement-handoff/recovered-source/MotSim/frm_Main.cs
 *     - RS232_Tx_Motor   (host TX builder)
 *     - SerialPortRead   (host RX parser)
 *     - SendConfigToBoard (command 5 transaction)
 *   motum_resources/motum-replacement-handoff/recovered-source/MotSim/Module2.cs
 *
 * Host -> Mega: fixed 18-byte frame
 *   [0..1]  FE FC marker
 *   [2]     command
 *   [3..15] 13-byte payload (zero-filled unless a command defines it)
 *   [16]    sequence: ordinary 0..126, forced 127
 *   [17]    additive checksum: sum(bytes[0..15]) modulo 256
 *
 * Mega -> host: variable frame
 *   [0..2]  80 81 82 marker
 *   [3..4]  type-specific bytes (type 48: [3] uninterpreted, [4] selector/status)
 *   [5]     type
 *   ...     payload
 *   [len-3] ArdTime (tenths of a second in the Windows parser)
 *   [len-2..len-1] big-endian checksum of bytes[0..len-3]
 *
 * The parser is incremental and bounded. It accepts arbitrary chunk splits,
 * multiple frames per feed, unknown types, corruption and markers embedded in
 * payloads, and it resynchronises by scanning for the next marker.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <variant>
#include <vector>

namespace mp6 {

// ---------------------------------------------------------------------------
// Host -> Mega wire constants
// ---------------------------------------------------------------------------

inline constexpr std::size_t kHostFrameBytes = 18;
inline constexpr std::size_t kHostPayloadBytes = 13;
inline constexpr std::size_t kHostPayloadOffset = 3;
inline constexpr std::size_t kHostSequenceOffset = 16;
inline constexpr std::size_t kHostChecksumOffset = 17;
inline constexpr std::uint8_t kHostMarker0 = 0xFE;
inline constexpr std::uint8_t kHostMarker1 = 0xFC;

inline constexpr std::uint8_t kFirstOrdinarySequence = 0;
inline constexpr std::uint8_t kLastOrdinarySequence = 126;
inline constexpr std::uint8_t kForcedSequence = 127;

// Payload byte 12 of command 2 (absolute byte 15). Lower six bits are motor
// direction/control bits in the Windows writer; the codec passes them through
// without assigning meanings not established by PROTOCOL.md.
inline constexpr std::uint8_t kFlagAcPowerRequest = 0x40;
inline constexpr std::uint8_t kFlagMotorDisable = 0x80;

// Commands whose payload layouts are established by PROTOCOL.md. The generic
// encoder accepts any command byte because the firmware dispatch table is not
// fully enumerated, but these names document the known host usages.
enum class HostCommand : std::uint8_t {
    MotorRate = 1,       // TxDat[0..12]
    Target = 2,          // six biased target words + flags byte
    Handshake = 3,       // generic writer leaves payload zero; usage not traced
    Pneumatic = 4,       // TxPneu[0..7], remaining payload zero
    Config = 5,          // three biased ID/value pairs + send flag
    RunMode = 6,         // RunMode in payload byte 0
    TorquePulse = 8,     // TorquePulseLen, TorquePulseFreq in payload bytes 0,1
    BoardValue = 9,      // BoardVal[0..2]
    BoardValueB = 11,    // BoardVal[0..1]
    BoardColor = 12,     // three little-endian colour words
    BoardValueC = 13,    // BoardVal[0..1]
};

enum class EncodeError : std::uint8_t {
    None = 0,
    ValueOutOfRange,     // biased 16-bit word would leave 0..65535
    ReservedTargetWord,  // an ordinary target would encode as the special word 1
    InvalidSequence,     // sequence must be 0..127
};

struct EncodeResult {
    std::array<std::uint8_t, kHostFrameBytes> bytes{};
    EncodeError error = EncodeError::None;
    // Index of the axis/pair that failed, or -1 when not applicable.
    int fieldIndex = -1;

    explicit operator bool() const noexcept { return error == EncodeError::None; }
};

// One of the six command-2 axis words. Ordinary targets are biased by +32767;
// the firmware/host conventions also use the raw word 1 as a per-axis special
// value (PEP[i] > 0 in Update_Stepper_4). An ordinary target that would encode
// as word 1 is rejected instead of being silently treated as that special.
struct AxisTarget {
    enum class Kind : std::uint8_t {
        Ordinary,
        Special,
    };

    Kind kind = Kind::Ordinary;
    std::int32_t value = 0;

    static constexpr AxisTarget ordinary(std::int32_t v) noexcept {
        return AxisTarget{Kind::Ordinary, v};
    }
    static constexpr AxisTarget specialWord() noexcept {
        return AxisTarget{Kind::Special, 0};
    }
};

// Command 5 carries three parameter ID/value pairs. Each field is biased by
// +32767 and written little-endian. Payload byte 12 (absolute byte 15) is 1
// for SendConfigToBoard calls and 0 otherwise.
struct ConfigPair {
    std::int32_t id = 0;
    std::int32_t value = 0;
};

// Mirrors the Windows TXCount byte. The recovered writer resets 126 to 0,
// otherwise increments, then overrides the emitted byte with 127 for a
// positive-Typ call. The member is left at 127 after that, so the next
// ordinary frame emits 0. nextForced() reproduces that order exactly.
class SequenceCounter {
public:
    explicit constexpr SequenceCounter(std::uint8_t initial = 0) noexcept
        : value_(initial) {}

    std::uint8_t nextOrdinary() noexcept;
    std::uint8_t nextForced() noexcept;
    std::uint8_t value() const noexcept { return value_; }

private:
    std::uint8_t value_;
};

// Checksum over bytes[0..15], i.e. the sequence byte at [16] is excluded.
std::uint8_t txChecksum(
    const std::array<std::uint8_t, kHostFrameBytes>& frame) noexcept;

// Generic 18-byte frame builder. Payload bytes not supplied are zero. The
// sequence must be 0..127; checksum is calculated, never caller-supplied.
EncodeResult encodeFrame(
    std::uint8_t command,
    const std::array<std::uint8_t, kHostPayloadBytes>& payload,
    std::uint8_t sequence) noexcept;

// Command 2. Each ordinary target must be in -32767..32768 and must not encode
// as word 1 (that is, -32766). SpecialWord() emits the reserved word 1.
EncodeResult encodeTargetFrame(
    const std::array<AxisTarget, 6>& targets,
    std::uint8_t flags,
    std::uint8_t sequence) noexcept;

// Convenience overload for six ordinary values.
EncodeResult encodeTargetFrame(
    const std::array<std::int32_t, 6>& values,
    std::uint8_t flags,
    std::uint8_t sequence) noexcept;

// Command 5. Every ID and value must be in -32767..32768. sendConfig sets
// payload byte 12 to 1; otherwise it is 0.
EncodeResult encodeConfigFrame(
    const std::array<ConfigPair, 3>& pairs,
    bool sendConfig,
    std::uint8_t sequence) noexcept;

// ---------------------------------------------------------------------------
// Mega -> host wire constants
// ---------------------------------------------------------------------------

inline constexpr std::array<std::uint8_t, 3> kRxMarker{{0x80, 0x81, 0x82}};
inline constexpr std::size_t kRxMarkerBytes = 3;
inline constexpr std::size_t kRxTypeOffset = 5;
inline constexpr std::size_t kRxStatusOffset = 4;
inline constexpr std::size_t kMaxRxFrameBytes = 43;
inline constexpr std::size_t kDefaultParserCapacity = 256;
inline constexpr std::size_t kMinParserCapacity = 64;
inline constexpr std::size_t kMaxQueuedFrames = 64;

enum class RxType : std::uint8_t {
    ControlStatus = 32,   // 21 bytes
    Keyboard = 36,        // 27 bytes
    ControlInputs = 38,   // 27 bytes
    MotorHandshake = 42,  // 21 bytes
    ConfigAck = 43,       // 21 bytes
    Telemetry = 48,       // 43 bytes
    TestInputs = 49,      // 27 bytes
};

// Returns the known total frame length including marker and checksum, or 0 for
// a type with no recovered length. The type is byte 5, not byte 4.
constexpr std::size_t rxFrameLength(std::uint8_t type) noexcept {
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

// Sum of bytes[0..length-3] inclusive. Returns 0 when length < 3.
std::uint16_t rxChecksumSum(const std::uint8_t* frame,
                            std::size_t length) noexcept;

// Compares rxChecksumSum() with the big-endian final two bytes.
bool rxChecksumValid(const std::uint8_t* frame, std::size_t length) noexcept;

// ---------------------------------------------------------------------------
// Typed Mega -> host frames
// ---------------------------------------------------------------------------

// Current-frame potentiometer range check. The Windows host applies this only
// when pScale != 0 and latches alarms; this codec reports the instantaneous
// condition and leaves latching/validity to the session layer.
enum class PotRange : std::uint8_t {
    Ok,
    Low,   // rawCounts < 100
    High,  // rawCounts > 3996
};

struct FeedbackAxis {
    std::uint16_t word = 0;  // big-endian sensor word from bytes 6..17
    int rawCounts = 0;       // Math.Round(word / 8.0), banker's rounding
    PotRange potRange = PotRange::Ok;
};

struct Type48Data {
    std::uint8_t rawPacking = 0;  // byte 4: selector = /10, status = %10
    std::uint8_t selector = 0;
    std::uint8_t status = 0;
    bool configReceived = false;  // status bit 0
    bool avOk = false;            // status bit 1
    bool statusBit2 = false;      // read by Windows but unused in this branch
    std::array<FeedbackAxis, 6> axes{};
    std::uint16_t digInput = 0;   // bytes 18..19
    std::uint8_t boardCounter = 0;  // DigInput / 256
    std::array<std::uint16_t, 10> ardValues{};  // index 0 is ArdTVal[1]
};

struct Type49Data {
    std::array<std::uint16_t, 9> testInputs{};  // index 0 is TestInput[1]
    double torque = 0.0;  // TestInput[3] * 0.161 - 2.8
};

struct Type42Data {
    std::uint8_t byte7 = 0;
    std::uint8_t byte9 = 0;
    std::uint8_t byte11 = 0;
    std::uint8_t byte12 = 0;
    std::uint8_t byte13 = 0;
    std::optional<int> handshakeRxM;  // byte7 * 1000 + byte9 when byte7 == 3
    bool runCodeIn1To3 = false;       // Windows MotorBoardRun condition
    std::uint16_t motorBoardVersion = 0;  // bytes 12..13, big-endian
};

struct Type43Data {
    std::uint8_t configCount = 0;  // byte 7
};

struct Type32Data {
    std::uint8_t byte7 = 0;
    std::uint8_t byte11 = 0;
    std::uint8_t byte12 = 0;
    bool byte7Below20 = false;
    bool byte11IsOne = false;
    std::uint8_t versionByte12 = 0;
};

struct Type36Data {
    std::array<std::uint8_t, 8> characters{};  // bytes 7,9,...,21
};

struct Type38Data {
    std::array<std::uint16_t, 9> avControl{};  // index 0 is AVControl[1]
};

struct RxFrame {
    std::uint8_t type = 0;
    std::size_t length = 0;
    std::array<std::uint8_t, kMaxRxFrameBytes> raw{};
    std::uint8_t ardTimeRaw = 0;  // byte length-3; also covered by checksum
    double ardTimeSeconds = 0.0;  // ardTimeRaw * 0.1 as decoded by Windows

    // Typed views require the matching type and exact length. Parser-produced
    // frames have already passed the checksum check; a manually assembled
    // RxFrame is the caller's responsibility.
    std::optional<Type32Data> asType32() const noexcept;
    std::optional<Type36Data> asType36() const noexcept;
    std::optional<Type38Data> asType38() const noexcept;
    std::optional<Type42Data> asType42() const noexcept;
    std::optional<Type43Data> asType43() const noexcept;
    std::optional<Type48Data> asType48() const noexcept;
    std::optional<Type49Data> asType49() const noexcept;
};

// ---------------------------------------------------------------------------
// Type 48 multiplexed values
// ---------------------------------------------------------------------------

// Selector 1: PlatformNumber_OB = v1*256+v2; Board_Typ = v3.
struct Selector1 {
    int platformNumber = 0;
    int boardType = 0;
};

// Selector 2: hour meter minutes = v1*65536+v2*256+v3. 64-bit because the
// three-word expression exceeds 32-bit signed range at its maximum.
struct Selector2 {
    std::int64_t minutes = 0;
};

// Selector 3: loop counter, pneumatic solenoid, RX delay.
struct Selector3 {
    int loopCounter = 0;
    int pneumaticSolenoid = 0;
    std::uint16_t rxDelay = 0;
};

// Selector 4: run counter and acceleration count.
struct Selector4 {
    int runCounter = 0;
    int accCount = 0;
};

// Selector 5: onboard P, D and step correction.
struct Selector5 {
    double proportional = 0.0;
    double derivative = 0.0;
    double stepCorrection = 0.0;
};

// Selector 6: pneumatic aim and two AV inputs.
struct Selector6 {
    std::uint16_t pneumaticAim = 0;
    std::uint16_t avInput10 = 0;
    std::uint16_t avInput11 = 0;
};

// Selector 7: three AV inputs; v3 packs three 5-bit supply readings at 3 V per
// count (low bits -> supplyVolts[0], then [1], then [2]).
struct Selector7 {
    std::uint16_t avInput12 = 0;
    std::uint16_t avInput13 = 0;
    std::uint16_t avInput14 = 0;
    std::array<int, 3> supplyVolts{};
};

// Selector 8: onboard run mode, OK-to-run bit and microseconds.
struct Selector8 {
    int runMode = 0;
    bool okToRun = false;
    std::uint16_t microsSec = 0;
};

// Selector 9: current checksum.
struct Selector9 {
    std::uint16_t checksumCurrent = 0;
};

// Selector 0 or >9 is std::monostate; no meaning is invented.
using SelectorData = std::variant<std::monostate, Selector1, Selector2,
                                  Selector3, Selector4, Selector5, Selector6,
                                  Selector7, Selector8, Selector9>;

SelectorData decodeSelector(const Type48Data& frame) noexcept;

// RunMode == 3 freeplay interpretation of ArdTVal[4..8] and [10].
struct FreeplayTelemetry {
    int freeplay = 0;             // v4 - 16000
    int calculatedPosition = 0;   // v5 - 16000
    int frequencyOffset = 0;      // v6 - 16000
    int velocityError = 0;        // v7 - 16000
    int targetOffset = 0;         // v8 - 16000
    std::uint16_t txCount = 0;    // v10
};

// Every other RunMode: v4/v5 are board ADC values, v6/v8 frequency/target.
// v7 and v10 are intentionally not assigned meanings in this branch.
struct NonFreeplayTelemetry {
    std::uint16_t mainBoardAdc = 0;      // v4
    std::uint16_t solenoidSupplyAdc = 0; // v5
    double mainBoardVolts = 0.0;         // ADC * 2.7843 / 1024 * 5
    double solenoidSupplyVolts = 0.0;
    int frequencyOffset = 0;  // v6 - 16000
    int targetOffset = 0;     // v8 - 16000
};

// The caller supplies the explicit mode context. Only RunMode 3 selects the
// freeplay interpretation, matching the recovered Windows branch; 0..2 and any
// other value use the non-freeplay fields rather than guessed names.
using ModeTelemetry = std::variant<FreeplayTelemetry, NonFreeplayTelemetry>;

ModeTelemetry decodeModeTelemetry(const Type48Data& frame,
                                  int runMode) noexcept;

enum class FaultCode : std::uint16_t {
    InternalPark = 210,
    AdPotError = 220,
    LowVolts = 230,
    EStop = 240,
    CommsDrop = 250,
};

// ArdTVal[9] == one of the recovered codes, otherwise nullopt.
std::optional<FaultCode> decodeFaultCode(const Type48Data& frame) noexcept;

// ---------------------------------------------------------------------------
// Incremental RX parser
// ---------------------------------------------------------------------------

struct ParserStats {
    std::uint64_t bytesFed = 0;
    std::uint64_t framesEmitted = 0;
    std::uint64_t badChecksums = 0;
    std::uint64_t unknownTypes = 0;
    std::uint64_t skippedBytes = 0;  // garbage and resync bytes
    std::uint64_t overflowDroppedBytes = 0;
    std::uint64_t queueDroppedFrames = 0;
};

class Parser {
public:
    explicit Parser(std::size_t capacity = kDefaultParserCapacity);

    // Appends bytes and emits every complete, checksum-valid frame it can.
    // Returns the number of frames decoded by this call. Decoding may also
    // evict an older queued frame when the bounded queue is full.
    std::size_t feed(const std::uint8_t* data, std::size_t length);
    std::size_t feed(const std::vector<std::uint8_t>& data);

    // Removes the oldest queued frame. Returns false when the queue is empty.
    bool poll(RxFrame& out);

    std::size_t pendingFrames() const noexcept { return queue_.size(); }
    std::size_t pendingBytes() const noexcept { return buffer_.size(); }
    std::size_t capacity() const noexcept { return capacity_; }
    const ParserStats& stats() const noexcept { return stats_; }

    // Clears buffered bytes, queued frames and statistics.
    void reset() noexcept;

private:
    std::size_t parseAvailable();
    void queueFrame(RxFrame&& frame);
    std::size_t capacity_;
    std::vector<std::uint8_t> buffer_;
    std::deque<RxFrame> queue_;
    ParserStats stats_{};
};

}  // namespace mp6
