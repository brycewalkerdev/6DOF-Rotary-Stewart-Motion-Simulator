/*
 * Offline tests for the MP6 session layer and the in-memory link.
 *
 * No device is opened here. Every controller frame is scripted from the
 * recovered wire description (marker 0x80 0x81 0x82, type at byte 5, big-endian
 * checksum over all but the final two bytes) and every host write is captured
 * by mp6::MemoryLink.
 *
 * The SerialLink documentation-mode guard is asserted only when the process is
 * started with STEWART_DOCUMENTATION_MODE=1, because mp6::IsDocumentationMode()
 * is deliberately process-immutable:
 *
 *   test_mp6_session.exe               (guard test reports SKIP)
 *   STEWART_DOCUMENTATION_MODE=1 test_mp6_session.exe
 *
 * Source citations:
 *   recovered-source/MotSim/frm_Main.cs:32503 (fault latch blocks TX)
 *   frm_Main.cs:31639-31911 (22-frame / 57-count command-5 transaction)
 *   frm_Main.cs:33817 (identity echo byte7=3 byte9=253 clears host latch)
 *   frm_Main.cs:32347-32362 (mode context and supply volts)
 *   Module2.cs Motor_Power / Stop_All_Motors
 */

#include "automation.h"
#include "mp6_link.h"
#include "mp6_session.h"

#include <algorithm>
#include <array>
#include <cfenv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

int g_checks = 0;
int g_failures = 0;
int g_skips = 0;

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

#define CHECK(cond) expectTrue(static_cast<bool>(cond), #cond, __LINE__)
#define CHECK_EQ(a, b) expectEq((a), (b), #a, #b, __LINE__)
#define CHECK_NEAR(a, b, eps) expectNear((a), (b), (eps), __LINE__)

using Bytes = std::vector<std::uint8_t>;
using Axes = std::array<std::uint16_t, 6>;
using Ard = std::array<std::uint16_t, 10>;

void putBe16(Bytes& bytes, std::size_t offset, std::uint16_t value) {
    bytes[offset] = static_cast<std::uint8_t>(value >> 8);
    bytes[offset + 1] = static_cast<std::uint8_t>(value & 0xFFu);
}

// Test-local checksum (sum of every byte except the last two, big-endian).
void seal(Bytes& frame) {
    std::uint32_t sum = 0;
    for (std::size_t i = 0; i + 2 < frame.size(); ++i) {
        sum += frame[i];
    }
    frame[frame.size() - 2] = static_cast<std::uint8_t>((sum >> 8) & 0xFFu);
    frame[frame.size() - 1] = static_cast<std::uint8_t>(sum & 0xFFu);
}

Bytes header(std::uint8_t type, std::size_t length) {
    Bytes frame(length, 0);
    frame[0] = 0x80;
    frame[1] = 0x81;
    frame[2] = 0x82;
    frame[5] = type;
    return frame;
}

Bytes makeType42(std::uint8_t byte7, std::uint8_t byte9) {
    Bytes frame = header(42, 21);
    frame[7] = byte7;
    frame[9] = byte9;
    seal(frame);
    return frame;
}

Bytes makeType43(std::uint8_t count) {
    Bytes frame = header(43, 21);
    frame[7] = count;
    seal(frame);
    return frame;
}

Bytes makeType48(int selector, int status, const Axes& axes, const Ard& ard) {
    Bytes frame = header(48, 43);
    frame[4] = static_cast<std::uint8_t>(selector * 10 + status);
    for (std::size_t i = 0; i < axes.size(); ++i) {
        putBe16(frame, 6 + i * 2, axes[i]);
    }
    for (std::size_t i = 0; i < ard.size(); ++i) {
        putBe16(frame, 20 + i * 2, ard[i]);
    }
    seal(frame);
    return frame;
}

Axes axesAt(std::uint16_t word) {
    Axes axes{};
    axes.fill(word);
    return axes;
}

Ard zeroArd() {
    Ard ard{};
    ard.fill(0);
    return ard;
}

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

struct Harness {
    mp6::MemoryLink link;
    mp6::Session session;
    std::uint64_t t = 0;
    // Acks a command-5 transaction once its 22nd frame is written, as the
    // controller does. Tests that withhold or corrupt the ack switch this off.
    bool autoAckConfig = true;

    Harness() {
        link.open("test");
        session.attach(&link);
    }

    explicit Harness(const mp6::SessionConfig& config) : session(config) {
        link.open("test");
        session.attach(&link);
    }

    void step(std::uint64_t ms = 1) {
        t += ms;
        session.tick(t);
        if (autoAckConfig && session.state() == mp6::State::Configuring &&
            session.configStatus().framesSent == mp6::kConfigFrameCount) {
            feed(makeType43(static_cast<std::uint8_t>(mp6::kConfigParameterCount)));
        }
    }

    void feed(const Bytes& bytes) { link.inject(bytes); }

    std::vector<Bytes> take() { return link.takeWrites(); }
};

int countCommand(const std::vector<Bytes>& writes, std::uint8_t command) {
    int count = 0;
    for (const Bytes& write : writes) {
        if (write.size() >= 3 && write[2] == command) {
            ++count;
        }
    }
    return count;
}

int lastCommand(const std::vector<Bytes>& writes) {
    return writes.empty() ? -1 : static_cast<int>(writes.back()[2]);
}

std::uint8_t sequenceOf(const Bytes& write) {
    return write.size() > 16 ? write[16] : 0;
}

int payloadPairId(const Bytes& write, int pair) {
    const std::size_t base = 3 + static_cast<std::size_t>(pair) * 4;
    const unsigned raw = static_cast<unsigned>(write[base]) |
                         (static_cast<unsigned>(write[base + 1]) << 8);
    return static_cast<int>(raw) - 32767;
}

int payloadPairValue(const Bytes& write, int pair) {
    const std::size_t base = 3 + static_cast<std::size_t>(pair) * 4;
    const unsigned raw = static_cast<unsigned>(write[base + 2]) |
                         (static_cast<unsigned>(write[base + 3]) << 8);
    return static_cast<int>(raw) - 32767;
}

int targetWord(const Bytes& write, int axis) {
    const std::size_t base = 3 + static_cast<std::size_t>(axis) * 2;
    const unsigned raw = static_cast<unsigned>(write[base]) |
                         (static_cast<unsigned>(write[base + 1]) << 8);
    return static_cast<int>(raw) - 32767;
}

// ---------------------------------------------------------------------------
// Validated inputs
// ---------------------------------------------------------------------------

mp6::ControllerConfig fullControllerConfig() {
    mp6::ControllerConfig config;
    config.benchTest = 0;
    config.commsTimeoutMs = 1000;
    config.stepRev = 1000;
    config.onboardShutdown = 0;
    config.accelP1Times100 = 100;
    config.accelP0Times10 = 10;
    config.decelTimes100 = 100;
    for (int i = 0; i < 6; ++i) {
        const std::size_t index = static_cast<std::size_t>(i);
        config.potLo[index] = 0;
        config.angLoTimes10[index] = 100;
        config.pScaleTimes1000[index] = 1000;
        config.pOffsetTimes10[index] = 0;
        config.gearboxTimes100[index] = 100;
    }
    config.motValFinal = 100;
    config.deadbandEnable = 0;
    config.deadbandValue = 0;
    config.stoppedCountLimit = 10;
    config.avFilterTimes10000 = 100;
    config.limitsEnable = 1;
    config.maxPps = 10000;
    config.infoMotor = 0;
    config.rotationDirection = 1;
    return config;
}

// Per-mode PID profiles. Distinct proportional values make each command-5
// transaction observable: Configure 3.0, Lifecycle 2.0, Normal 1.0
// (wire P*10 = 30, 20, 10).
mp6::PidProfile pidProfileFor(mp6::PidMode mode) {
    mp6::PidProfile profile;
    profile.proportionalTimes10 = mode == mp6::PidMode::Configure ? 30
                                  : mode == mp6::PidMode::Lifecycle ? 20
                                                                    : 10;
    profile.derivativeTimes10 = 10;
    profile.dFilter = 10;
    profile.tConst = 10;
    profile.stepCorrTimes10000 = 0;
    profile.sensorFilterTimes10000 = 500;
    profile.historyLen = 10;
    return profile;
}

bool supplyPidProfiles(Harness& harness) {
    std::string why;
    for (mp6::PidMode mode : {mp6::PidMode::Normal, mp6::PidMode::Configure,
                              mp6::PidMode::Lifecycle}) {
        if (!harness.session.setPidProfile(mode, pidProfileFor(mode), &why)) {
            std::printf("PID profile refused: %s\n", why.c_str());
            return false;
        }
    }
    return true;
}

mp6::Calibration fullCalibration(std::int32_t steps) {
    mp6::Calibration calibration;
    calibration.valid = true;
    calibration.description = "test calibration";
    for (int i = 0; i < 6; ++i) {
        mp6::AxisCalibration& axis = calibration.axis[static_cast<std::size_t>(i)];
        axis.pScale = 8.0;  // calibrated == raw sensor word
        axis.pOffset = 0.0;
        axis.dockedSteps = steps;
        axis.valid = true;
    }
    return calibration;
}

std::array<std::int32_t, 6> neutralAt(std::int32_t steps) {
    std::array<std::int32_t, 6> neutral{};
    neutral.fill(steps);
    return neutral;
}

void feedSupplyAxes(Harness& harness, const Axes& axes) {
    Ard selector6 = zeroArd();
    selector6[0] = 30;   // pneumatic aim
    selector6[1] = 800;  // AVInput10 -> main board 10.88 V
    selector6[2] = 800;  // AVInput11 -> solenoid supply 10.88 V
    harness.feed(makeType48(6, 3, axes, selector6));

    Ard selector7 = zeroArd();
    selector7[0] = 60;     // strut pressure
    selector7[2] = 15855;  // three 5-bit rails at 15 counts * 3 V = 45 V
    harness.feed(makeType48(7, 3, axes, selector7));
}

void feedSupply(Harness& harness, std::uint16_t word = 16000) {
    feedSupplyAxes(harness, axesAt(word));
}

bool identify(Harness& harness, int platform = 1) {
    std::string why;
    if (!harness.session.requestIdentify(&why)) {
        std::printf("identify refused: %s\n", why.c_str());
        return false;
    }
    harness.step();
    harness.feed(makeType42(3, 253));
    harness.step();
    if (harness.session.state() != mp6::State::Identified) {
        return false;
    }
    Ard pairing = zeroArd();
    pairing[0] = 0;
    pairing[1] = static_cast<std::uint16_t>(platform);
    pairing[2] = 3;
    harness.feed(makeType48(1, 3, axesAt(16000), pairing));
    harness.step();
    harness.session.setExpectedPlatformNumber(platform);
    return harness.session.telemetry().platformNumber == platform;
}

bool prepareInputs(Harness& harness) {
    std::string why;
    if (!harness.session.setControllerConfig(fullControllerConfig(), &why)) {
        std::printf("controller config refused: %s\n", why.c_str());
        return false;
    }
    if (!supplyPidProfiles(harness)) {
        return false;
    }
    if (!harness.session.setCalibration(fullCalibration(16000), &why)) {
        std::printf("calibration refused: %s\n", why.c_str());
        return false;
    }
    if (!harness.session.setNeutralSteps(neutralAt(16000), &why)) {
        std::printf("neutral refused: %s\n", why.c_str());
        return false;
    }
    return true;
}

// Sends exactly one command-5 frame per 1 ms tick.
std::vector<Bytes> driveConfigFrames(Harness& harness, int frames,
                                     int perFrameMs = 1) {
    std::vector<Bytes> all;
    for (int i = 0; i < frames; ++i) {
        harness.step(static_cast<std::uint64_t>(perFrameMs));
        std::vector<Bytes> write = harness.take();
        for (Bytes& frame : write) {
            all.push_back(std::move(frame));
        }
    }
    return all;
}

bool bringToConfigured(Harness& harness) {
    if (!identify(harness) || !prepareInputs(harness)) {
        return false;
    }
    std::string why;
    if (!harness.session.requestConfigure(&why)) {
        std::printf("configure refused: %s\n", why.c_str());
        return false;
    }
    driveConfigFrames(harness, mp6::kConfigFrameCount);
    harness.feed(makeType43(static_cast<std::uint8_t>(mp6::kConfigParameterCount)));
    harness.step();
    return harness.session.configured();
}

bool bringToParked(Harness& harness) {
    if (!bringToConfigured(harness)) {
        return false;
    }
    // Feedback is required before initialization.
    feedSupply(harness);
    harness.step();
    std::string why;
    if (!harness.session.requestInitialize(&why)) {
        std::printf("initialize refused: %s\n", why.c_str());
        return false;
    }
    for (int i = 0; i < 25000; ++i) {
        feedSupply(harness);
        harness.step();
        if (harness.session.state() == mp6::State::Parked) {
            return true;
        }
    }
    std::printf("initialize did not reach Parked (state %s, stage %s)\n",
                mp6::toString(harness.session.state()),
                harness.session.lifecycleStatus().stage.c_str());
    return false;
}

bool bringToRunning(Harness& harness) {
    if (!bringToParked(harness)) {
        return false;
    }
    feedSupply(harness);
    harness.step();
    mp6::StepTarget target;
    target.steps = {16000, 16000, 16000, 16000, 16000, 16000};
    target.valid = true;
    target.generation = 1;
    harness.session.setTarget(target);
    std::string why;
    if (!harness.session.requestStartMotion(&why)) {
        std::printf("start refused: %s\n", why.c_str());
        return false;
    }
    // Start applies the normal PID profile first; Running follows its ACK.
    for (int i = 0; i < 200 && harness.session.state() != mp6::State::Running; ++i) {
        feedSupply(harness);
        harness.step();
    }
    return harness.session.state() == mp6::State::Running;
}

// A scripted plant: feedback follows the last command-2 target with one tick
// of delay. This makes observed-settle logic meaningful for nonzero ramps.
struct Plant {
    Axes axes;
    std::array<int, 6> lastTarget{};
    bool haveTarget = false;
    // Type-48 telemetry is sent on ticks where t % telemetryPeriodMs ==
    // telemetryPhase. The default (every tick) keeps the existing tests intact.
    std::uint64_t telemetryPeriodMs = 1;
    std::uint64_t telemetryPhase = 0;

    explicit Plant(std::uint16_t start) {
        axes.fill(start);
        lastTarget.fill(static_cast<int>(start));
    }

    std::vector<Bytes> step(Harness& harness, std::uint64_t ms = 1) {
        if (harness.t % telemetryPeriodMs == telemetryPhase) {
            feedSupplyAxes(harness, axes);
        }
        harness.step(ms);
        std::vector<Bytes> writes = harness.take();
        for (const Bytes& write : writes) {
            if (write.size() >= 18 && write[2] == 2) {
                for (int i = 0; i < 6; ++i) {
                    lastTarget[static_cast<std::size_t>(i)] =
                        targetWord(write, i);
                    // The recovered host feeds the sensor word into the same
                    // calibrated-step domain used by the test calibration
                    // (pScale 8, pOffset 0). Keep the plant in that domain;
                    // only the command-2 wire word is biased by 32767.
                    axes[static_cast<std::size_t>(i)] =
                        static_cast<std::uint16_t>(
                            std::max(0, lastTarget[static_cast<std::size_t>(i)]));
                }
                haveTarget = true;
            }
        }
        return writes;
    }
};

bool prepareInputsAt(Harness& harness, int dock, int neutral) {
    std::string why;
    if (!harness.session.setControllerConfig(fullControllerConfig(), &why)) {
        std::printf("controller config refused: %s\n", why.c_str());
        return false;
    }
    if (!supplyPidProfiles(harness)) {
        return false;
    }
    if (!harness.session.setCalibration(
            fullCalibration(static_cast<std::int32_t>(dock)), &why)) {
        std::printf("calibration refused: %s\n", why.c_str());
        return false;
    }
    if (!harness.session.setNeutralSteps(
            neutralAt(static_cast<std::int32_t>(neutral)), &why)) {
        std::printf("neutral refused: %s\n", why.c_str());
        return false;
    }
    return true;
}

bool bringToConfiguredAt(Harness& harness, int dock, int neutral) {
    if (!identify(harness) || !prepareInputsAt(harness, dock, neutral)) {
        return false;
    }
    std::string why;
    if (!harness.session.requestConfigure(&why)) {
        std::printf("configure refused: %s\n", why.c_str());
        return false;
    }
    driveConfigFrames(harness, mp6::kConfigFrameCount);
    harness.feed(makeType43(static_cast<std::uint8_t>(mp6::kConfigParameterCount)));
    harness.step();
    return harness.session.configured();
}

bool bringToParkedPlant(Harness& harness, Plant& plant, int dock, int neutral) {
    if (!bringToConfiguredAt(harness, dock, neutral)) {
        return false;
    }
    feedSupplyAxes(harness, plant.axes);
    harness.step();
    std::string why;
    if (!harness.session.requestInitialize(&why)) {
        std::printf("initialize refused: %s\n", why.c_str());
        return false;
    }
    for (int i = 0; i < 60000; ++i) {
        plant.step(harness);
        if (harness.session.state() == mp6::State::Parked) {
            return true;
        }
    }
    std::printf("plant initialization did not reach Parked (state %s, stage %s,"
                " reason %s, detail %s, ramp endpoint %d timeout %llu elapsed %llu)\n",
                mp6::toString(harness.session.state()),
                harness.session.lifecycleStatus().stage.c_str(),
                mp6::toString(harness.session.stopReason()),
                harness.session.diagnostics().detail.c_str(),
                harness.session.rampStatus().endpointReached ? 1 : 0,
                static_cast<unsigned long long>(
                    harness.session.rampStatus().timeoutMs),
                static_cast<unsigned long long>(
                    harness.session.nowMs() - harness.session.rampStatus().startMs));
    return false;
}

bool startMotionPlant(Harness& harness, Plant& plant) {
    feedSupplyAxes(harness, plant.axes);
    harness.step();
    mp6::StepTarget target;
    target.steps = {plant.lastTarget[0], plant.lastTarget[1],
                    plant.lastTarget[2], plant.lastTarget[3],
                    plant.lastTarget[4], plant.lastTarget[5]};
    target.valid = true;
    target.generation = 1;
    harness.session.setTarget(target);
    std::string why;
    if (!harness.session.requestStartMotion(&why)) {
        std::printf("plant start refused: %s\n", why.c_str());
        return false;
    }
    for (int i = 0; i < 200 && harness.session.state() != mp6::State::Running; ++i) {
        plant.step(harness);
    }
    return harness.session.state() == mp6::State::Running;
}

bool bringToDockedPlant(Harness& harness, Plant& plant, int dock, int neutral) {
    if (!bringToParkedPlant(harness, plant, dock, neutral)) {
        return false;
    }
    std::string why;
    if (!harness.session.requestPark(&why)) {
        std::printf("park refused: %s\n", why.c_str());
        return false;
    }
    for (int i = 0; i < 40000; ++i) {
        plant.step(harness);
        if (harness.session.state() == mp6::State::Docked) {
            return true;
        }
    }
    std::printf("plant park did not reach Docked (state %s, stage %s, reason %s,"
                " detail %s)\n",
                mp6::toString(harness.session.state()),
                harness.session.lifecycleStatus().stage.c_str(),
                mp6::toString(harness.session.stopReason()),
                harness.session.diagnostics().detail.c_str());
    return false;
}

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

void testIdentifyEcho() {
    Harness harness;
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Opened));

    CHECK(harness.session.requestIdentify());
    harness.step();
    std::vector<Bytes> writes = harness.take();
    CHECK_EQ(writes.size(), static_cast<std::size_t>(1));
    CHECK_EQ(lastCommand(writes), 3);
    CHECK_EQ(sequenceOf(writes.front()), 127);
    for (std::size_t i = 3; i < 16; ++i) {
        CHECK_EQ(writes.front()[i], 0);  // cmd3 is the zero-payload probe
    }

    // Wrong byte 9 is reported but is not an identity.
    harness.feed(makeType42(3, 251));
    harness.step();
    CHECK(harness.session.diagnostics().identityMismatch);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Identifying));

    harness.feed(makeType42(3, 253));
    harness.step();
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Identified));
    CHECK(harness.session.identityAccepted());
}

void testIdentityMismatchExhaustsProbes() {
    Harness harness;
    CHECK(harness.session.requestIdentify());
    for (int i = 0; i < 40; ++i) {
        harness.feed(makeType42(3, 17));
        harness.step(200);
    }
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Opened));
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::IdentityMismatch));
    std::vector<Bytes> writes = harness.take();
    // 10 probes, each carrying the forced sequence byte 127.
    CHECK_EQ(countCommand(writes, 3), 10);
    CHECK(harness.session.diagnostics().identityMismatch);
}

// ---------------------------------------------------------------------------
// Configuration transaction
// ---------------------------------------------------------------------------

void testConfigurationTransaction() {
    // Additional near-tie tests verifying exact represented single-precision half-even without fuzzy epsilon:
    // Ties: 2.5f -> 2 (even), 3.5f -> 4 (even)
    CHECK_EQ(mp6::roundHalfEvenToInt(2.5), 2);
    CHECK_EQ(mp6::roundHalfEvenToInt(3.5), 4);
    // Near ties (strictly < 0.5 or strictly > 0.5):
    CHECK_EQ(mp6::roundHalfEvenToInt(static_cast<double>(std::nextafter(2.5f, 2.0f))), 2);
    CHECK_EQ(mp6::roundHalfEvenToInt(static_cast<double>(std::nextafter(2.5f, 3.0f))), 3);
    CHECK_EQ(mp6::roundHalfEvenToInt(static_cast<double>(std::nextafter(3.5f, 3.0f))), 3);
    CHECK_EQ(mp6::roundHalfEvenToInt(static_cast<double>(std::nextafter(3.5f, 4.0f))), 4);

    Harness harness;

    harness.autoAckConfig = false;
    CHECK(identify(harness));
    harness.take();  // discard the identify probe

    // Missing controller parameters are named diagnostics, not fabricated.
    std::string why;
    CHECK(!harness.session.requestConfigure(&why));
    CHECK(why.find("missing controller parameters") != std::string::npos);
    CHECK(why.find("5 BenchTest") != std::string::npos);
    CHECK(!harness.session.diagnostics().missing.empty());
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Identified));
    CHECK(!harness.session.configured());

    CHECK(prepareInputs(harness));
    CHECK(harness.session.requestConfigure(&why));
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Configuring));
    // startConfiguration only schedules; it does not transmit.
    CHECK(harness.take().empty());

    std::vector<Bytes> frames;
    for (int i = 0; i < mp6::kConfigFrameCount; ++i) {
        harness.step(1);
        std::vector<Bytes> write = harness.take();
        if (i == 0) {
            // A second tick at the same timestamp must not violate spacing.
            CHECK_EQ(write.size(), static_cast<std::size_t>(1));
            harness.session.tick(harness.t);
            CHECK(harness.take().empty());
        }
        for (Bytes& frame : write) {
            frames.push_back(std::move(frame));
        }
    }
    CHECK_EQ(frames.size(), static_cast<std::size_t>(mp6::kConfigFrameCount));
    CHECK_EQ(countCommand(frames, 5),
             static_cast<int>(mp6::kConfigFrameCount));
    for (const Bytes& frame : frames) {
        CHECK_EQ(sequenceOf(frame), 127);  // positive Typ is forced to 127
    }

    // Exact grouping: begin (10,0), end (11,57), forced param values.
    CHECK_EQ(payloadPairId(frames[0], 0), 10);
    CHECK_EQ(payloadPairValue(frames[0], 0), 0);
    CHECK_EQ(payloadPairId(frames[1], 0), 5);
    CHECK_EQ(payloadPairId(frames[1], 1), 6);
    CHECK_EQ(payloadPairId(frames[2], 1), 43);
    CHECK_EQ(payloadPairValue(frames[2], 1), 40);
    CHECK_EQ(payloadPairId(frames[15], 2), 45);
    CHECK_EQ(payloadPairValue(frames[15], 2), 5);
    CHECK_EQ(payloadPairId(frames[16], 0), 0);  // unused slot stays (0,0)
    CHECK_EQ(payloadPairId(frames[20], 2), 20);
    CHECK_EQ(payloadPairValue(frames[20], 2), 1);
    CHECK_EQ(payloadPairId(frames[21], 0), 11);
    CHECK_EQ(payloadPairValue(frames[21], 0), 57);

    int counted = 0;
    for (const Bytes& frame : frames) {
        for (int pair = 0; pair < 3; ++pair) {
            const int id = payloadPairId(frame, pair);
            if (id != 0) {
                ++counted;
            }
        }
    }
    // 56 real parameter values plus the (11,57) end marker make the ack 57;
    // the (10,0) begin marker is not counted.
    CHECK_EQ(counted - 1, 57);
    CHECK_EQ(counted - 2, 56);
    CHECK_EQ(harness.session.configStatus().parametersTransmitted, 56);

    CHECK_EQ(harness.session.configStatus().framesSent,
             mp6::kConfigFrameCount);
    CHECK_EQ(harness.session.configStatus().parametersExpected, 57);
    CHECK(!harness.session.configured());
    CHECK(!harness.session.requestStartMotion(&why));  // no ack yet

    harness.feed(makeType43(57));
    harness.step(1);
    CHECK(harness.session.configured());
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Configured));
    CHECK_EQ(harness.session.configStatus().lastAckCount, 57);
    CHECK_EQ(harness.session.configStatus().attempts, 1);

    // A late/unsolicited 57 ack must not change a non-configuring state.
    harness.feed(makeType43(57));
    harness.step(1);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Configured));
}

void testConfigurationWrongAckAndRetries() {
    Harness harness;
    harness.autoAckConfig = false;
    CHECK(identify(harness));
    CHECK(prepareInputs(harness));
    std::string why;
    CHECK(harness.session.requestConfigure(&why));

    driveConfigFrames(harness, mp6::kConfigFrameCount);
    harness.feed(makeType43(50));
    harness.step(1);
    CHECK(harness.session.configStatus().wrongAckCount >= 1);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Configuring));

    // Three more full transactions; each waits out the 200 ms ack timeout.
    std::vector<Bytes> all;
    for (int attempt = 2; attempt <= 4; ++attempt) {
        harness.step(201);
        std::vector<Bytes> frames =
            driveConfigFrames(harness, mp6::kConfigFrameCount);
        for (Bytes& frame : frames) {
            all.push_back(std::move(frame));
        }
    }
    harness.step(201);

    CHECK_EQ(harness.session.configStatus().attempts, 4);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Identified));
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::ConfigRejected));
    CHECK(harness.session.diagnostics().configRejected);
    CHECK(!harness.session.configured());

    // Note: the first transaction's frames were consumed above by the helper,
    // so only transactions 2..4 are in all; verify 22 frames per attempt and 88
    // command-5 transmissions in total via the session counters.
    CHECK_EQ(countCommand(all, 5), 22 * 3);
    CHECK_EQ(harness.session.txCounters().command5, 22ull * 4);
    CHECK_EQ(countCommand(all, 6), 0);
    CHECK_EQ(countCommand(all, 1), 0);
}

// ---------------------------------------------------------------------------
// Telemetry, selectors and mode context
// ---------------------------------------------------------------------------

void testSelectorsAndModeContext() {
    Harness harness;
    CHECK(identify(harness));

    Ard ard = zeroArd();
    ard[0] = 0;      // selector 1: platform = 0*256 + 1
    ard[1] = 1;
    ard[2] = 3;      // board type
    harness.feed(makeType48(1, 3, axesAt(16000), ard));

    ard = zeroArd();
    ard[0] = 1;
    ard[1] = 2;
    ard[2] = 3;
    harness.feed(makeType48(2, 3, axesAt(16000), ard));

    ard = zeroArd();
    ard[0] = 2;
    ard[1] = 16005;
    ard[2] = 7;
    harness.feed(makeType48(3, 3, axesAt(16000), ard));

    ard = zeroArd();
    ard[0] = 1;
    ard[1] = 2;
    ard[2] = 7;
    harness.feed(makeType48(4, 3, axesAt(16000), ard));

    ard = zeroArd();
    ard[0] = 16100;
    ard[1] = 15900;
    ard[2] = 5000;
    harness.feed(makeType48(5, 3, axesAt(16000), ard));

    ard = zeroArd();
    ard[0] = 7;  // run mode 3 plus ok-to-run bit
    ard[2] = 1234;
    harness.feed(makeType48(8, 3, axesAt(16000), ard));

    ard = zeroArd();
    ard[0] = 4321;
    harness.feed(makeType48(9, 3, axesAt(16000), ard));

    ard = zeroArd();
    ard[0] = 30;
    ard[1] = 800;   // main board ADC (selector 6)
    ard[2] = 700;   // solenoid ADC
    harness.feed(makeType48(6, 3, axesAt(16000), ard));

    // Selector 7 is last so its mode-dependent words win, as they do in the
    // recovered host's per-frame branch.
    ard = zeroArd();
    ard[0] = 60;
    ard[2] = 15855;  // 45 V, 45 V, 45 V
    ard[3] = 900;    // ArdTVal[4] -> main board ADC in non-freeplay mode
    ard[4] = 700;    // ArdTVal[5] -> solenoid ADC
    harness.feed(makeType48(7, 3, axesAt(16000), ard));

    harness.step();

    const mp6::Telemetry& telemetry = harness.session.telemetry();
    CHECK_EQ(telemetry.platformNumber, 1);
    CHECK_EQ(telemetry.boardType, 3);
    CHECK_EQ(telemetry.loopCounter, 40);
    CHECK_EQ(telemetry.pneuSolenoid, 5);
    CHECK_EQ(static_cast<int>(telemetry.rxDelay), 7);
    CHECK_EQ(telemetry.runCounter, 258);
    CHECK_EQ(telemetry.accCount, 7);
    CHECK_NEAR(telemetry.onboardPid.proportional, 10.0, 1e-9);
    CHECK_NEAR(telemetry.onboardPid.derivative, -10.0, 1e-9);
    CHECK_NEAR(telemetry.onboardPid.stepCorrection, 0.5, 1e-9);
    CHECK_EQ(telemetry.pneuAim, 30);
    CHECK_EQ(telemetry.avInput10, 800);
    CHECK_EQ(telemetry.avInput11, 700);
    CHECK_EQ(telemetry.pressure, 60);
    CHECK_EQ(telemetry.motorSupplyVolts[0], 45);
    CHECK_EQ(telemetry.motorSupplyVolts[1], 45);
    CHECK_EQ(telemetry.motorSupplyVolts[2], 45);
    CHECK_EQ(telemetry.runMode, 3);
    CHECK(telemetry.okToRun);
    CHECK_EQ(static_cast<int>(telemetry.checksumCurrent), 4321);
    // One selector-1 frame from the pairing step, then the nine above.
    CHECK_EQ(telemetry.type48Frames, static_cast<std::uint64_t>(10));
    for (int selector = 1; selector <= 9; ++selector) {
        CHECK(telemetry.selectorFrames[selector] >= 1);
        CHECK(telemetry.selectorSeenMs[selector] != 0);
    }

    // commandedRunMode is still 1, so ArdTVal[4]/[5] supply the volts.
    const double mainBoard =
        static_cast<double>(900) * 2.7843 / 1024.0 * 5.0;
    const double solenoid =
        static_cast<double>(700) * 2.7843 / 1024.0 * 5.0;
    CHECK_NEAR(telemetry.mainBoardVolts, mainBoard, 0.01);
    CHECK_NEAR(telemetry.solenoidVolts, solenoid, 0.01);
    CHECK(telemetry.voltageSource ==
          mp6::Telemetry::VoltageSource::NonFreeplayAdc);

    // No configuration ack exists here, so calibrated feedback is not usable
    // even though raw sensor words arrived.
    CHECK(!harness.session.feedbackFresh());
    CHECK(harness.session.supplyFresh());
    CHECK(harness.session.pressureFresh());
    CHECK(harness.session.statusFresh());
    harness.step(600);  // feedback/status data is now older than the TTLs
    CHECK(!harness.session.feedbackFresh());
    CHECK(harness.session.statusFresh());  // status TTL is 2000 ms
}

// ---------------------------------------------------------------------------
// Fault latch
// ---------------------------------------------------------------------------

void testFaultLatchBlocksTransmission() {
    Harness harness;
    CHECK(identify(harness));
    harness.take();

    Ard ard = zeroArd();
    ard[8] = 240;  // EStop
    harness.feed(makeType48(9, 3, axesAt(16000), ard));
    harness.step();

    CHECK(harness.session.controllerFaultLatched());
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Faulted));
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::ControllerFault));
    CHECK_EQ(harness.session.telemetry().controllerFaultCode, 240);

    harness.take();
    for (int i = 0; i < 20; ++i) {
        feedSupply(harness);
        harness.step(100);
    }
    CHECK(harness.take().empty());  // no frame whatsoever while latched
    std::string why;
    CHECK(!harness.session.requestIdentify(&why));
    CHECK(why.find("latched") != std::string::npos);

    // Each recovered code latches the raw word; none is decoded as "clear".
    const std::uint16_t codes[5] = {210, 220, 230, 250, 240};
    for (std::uint16_t code : codes) {
        Ard frame = zeroArd();
        frame[8] = code;
        harness.feed(makeType48(9, 3, axesAt(16000), frame));
        harness.step();
        CHECK_EQ(harness.session.telemetry().controllerFaultCode, code);
        CHECK(harness.session.controllerFaultLatched());
    }
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::ControllerFault));

    // Recovery is a new host identify session, never a frame that pretends to
    // clear the controller's latch.
    harness.session.attach(&harness.link);
    CHECK(!harness.session.controllerFaultLatched());
    CHECK(harness.session.requestIdentify());
}

// ---------------------------------------------------------------------------
// Configuration loss
// ---------------------------------------------------------------------------

void testConfigurationLossStops() {
    Harness harness;
    harness.autoAckConfig = false;
    CHECK(bringToConfigured(harness));
    harness.take();

    // status bit 0 clear: controller no longer reports the configuration.
    Ard ard = zeroArd();
    for (int i = 0; i < 3; ++i) {
        harness.feed(makeType48(0, 2, axesAt(16000), ard));
        harness.step(100);
    }
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Stopped));
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::ConfigLost));
    CHECK(harness.session.diagnostics().configLost);
    CHECK(!harness.session.configured());
}

// ---------------------------------------------------------------------------
// Full lifecycle, supply freshness and motion
// ---------------------------------------------------------------------------

void testLifecycleAndSupplyGuards() {
    mp6::SessionConfig config;
    config.parkSettleMs = 200;
    Harness harness(config);

    CHECK(identify(harness));
    CHECK(prepareInputs(harness));
    std::string why;
    CHECK(harness.session.requestConfigure(&why));
    std::vector<Bytes> configFrames =
        driveConfigFrames(harness, mp6::kConfigFrameCount);
    CHECK_EQ(countCommand(configFrames, 5), 22);
    harness.feed(makeType43(57));
    harness.step();
    CHECK(harness.session.configured());

    feedSupply(harness);
    harness.step();
    CHECK(harness.session.feedbackFresh());
    CHECK(harness.session.requestInitialize(&why));
    harness.take();

    // Command order: run mode 3 on the first tick, then the first of three
    // power frames (the power burst driver runs before the lifecycle driver).
    harness.step();
    std::vector<Bytes> writes = harness.take();
    std::vector<Bytes> lifecycle;
    for (Bytes& frame : writes) {
        lifecycle.push_back(frame);
    }
    CHECK_EQ(writes.size(), static_cast<std::size_t>(1));
    CHECK_EQ(writes[0][2], 6);
    CHECK_EQ(writes[0][3], 3);  // RunMode 3
    CHECK_EQ(sequenceOf(writes[0]), 127);

    // The lifecycle PID profile is a 22-frame command-5 transaction that runs
    // before motor power (PID_SetValues(2), frm_Main.cs:23019). Drain it here
    // and check that no power frame leaves before it is acked.
    std::vector<Bytes> profileFrames;
    for (int i = 0; i < 200; ++i) {
        harness.step();
        for (Bytes& frame : harness.take()) {
            profileFrames.push_back(std::move(frame));
        }
        if (harness.session.lifecycleStatus().stage == "Motor power on") {
            break;
        }
    }
    CHECK_EQ(countCommand(profileFrames, 5), 22);
    CHECK_EQ(countCommand(profileFrames, 1), 0);

    // commandedRunMode is now 3: freeplay words must decode without
    // overwriting the supply snapshot taken from selector 6.
    feedSupply(harness);
    Ard freeplay = zeroArd();
    freeplay[0] = 60;
    freeplay[2] = 15855;
    freeplay[3] = 20000;  // freeplay 4000
    freeplay[4] = 17000;  // calculated position 1000
    freeplay[6] = 16345;  // velocity error 345
    freeplay[7] = 16789;  // target offset 789
    freeplay[9] = 42;     // TX count
    harness.feed(makeType48(7, 3, axesAt(16000), freeplay));

    harness.step(1);
    for (Bytes& frame : harness.take()) {
        lifecycle.push_back(std::move(frame));
    }
    CHECK_EQ(harness.session.telemetry().freeplay.freeplay, 4000);
    CHECK_EQ(harness.session.telemetry().freeplay.calculatedPosition, 1000);
    CHECK_EQ(harness.session.telemetry().freeplay.velocityError, 345);
    CHECK_EQ(harness.session.telemetry().freeplay.targetOffset, 789);
    CHECK_EQ(harness.session.telemetry().freeplay.txCount, 42);
    CHECK_NEAR(harness.session.telemetry().mainBoardVolts,
               800.0 * 2.7843 / 1024.0 * 5.0, 0.01);
    CHECK(harness.session.telemetry().voltageSource ==
          mp6::Telemetry::VoltageSource::Selector6);
    CHECK_EQ(countCommand(lifecycle, 1), 1);
    CHECK_EQ(lifecycle.back()[2], 1);
    CHECK_EQ(lifecycle.back()[15], 0x40);  // AC request, motor not disabled
    CHECK_EQ(sequenceOf(lifecycle.back()), 127);

    // Power frames are spaced by 50 ms.
    harness.step(49);
    CHECK(harness.take().empty());
    harness.step(1);
    for (Bytes& frame : harness.take()) {
        lifecycle.push_back(std::move(frame));
    }
    CHECK_EQ(countCommand(lifecycle, 1), 2);
    harness.step(50);
    for (Bytes& frame : harness.take()) {
        lifecycle.push_back(std::move(frame));
    }
    CHECK_EQ(countCommand(lifecycle, 1), 3);

    // Reaching Parked requires observed power, pressure and two settled ramps.
    bool parked = false;
    for (int i = 0; i < 25000; ++i) {
        feedSupply(harness);
        harness.step();
        std::vector<Bytes> tickWrites = harness.take();
        for (Bytes& frame : tickWrites) {
            lifecycle.push_back(std::move(frame));
        }
        if (harness.session.state() == mp6::State::Parked) {
            parked = true;
            break;
        }
    }
    CHECK(parked);
    if (!parked) {
        return;
    }
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::None));
    CHECK(harness.session.lifecycleStatus().powerOnObserved);
    CHECK(harness.session.lifecycleStatus().supplyObserved);
    CHECK(harness.session.lifecycleStatus().pressureObserved);
    CHECK_EQ(harness.session.lifecycleStatus().pressureAimPsi, 30);

    // Command order through initialization: 6 before 1 before 4 before 2.
    int first6 = -1;
    int first1 = -1;
    int first4 = -1;
    int first2 = -1;
    for (std::size_t i = 0; i < lifecycle.size(); ++i) {
        const int command = lifecycle[i][2];
        if (command == 6 && first6 < 0) first6 = static_cast<int>(i);
        if (command == 1 && first1 < 0) first1 = static_cast<int>(i);
        if (command == 4 && first4 < 0) first4 = static_cast<int>(i);
        if (command == 2 && first2 < 0) first2 = static_cast<int>(i);
    }
    CHECK(first6 >= 0 && first1 > first6 && first4 > first1 && first2 > first4);
    CHECK(countCommand(lifecycle, 4) >= 4);  // pressure set + refresh frames
    for (const Bytes& frame : lifecycle) {
        if (frame[2] == 4) {
            CHECK_EQ(frame[3], 35);  // aim 30 + 5
            CHECK_EQ(sequenceOf(frame), 127);
        }
    }

    // Motion is blocked while the retained supply snapshot is stale.
    for (int i = 0; i < 2200; ++i) {
        Ard selector7 = zeroArd();
        selector7[0] = 60;
        selector7[2] = 15855;
        harness.feed(makeType48(7, 3, axesAt(16000), selector7));
        harness.step();
    }
    mp6::StepTarget target;
    target.steps = {16000, 16000, 16000, 16000, 16000, 16000};
    target.valid = true;
    harness.session.setTarget(target);
    CHECK(!harness.session.requestStartMotion(&why));
    CHECK(why.find("supply") != std::string::npos);

    feedSupply(harness);
    harness.step();
    harness.session.setTarget(target);
    harness.take();
    CHECK(harness.session.requestStartMotion(&why));
    // The first target frame goes out on the tick that enters Running.
    writes.clear();
    for (int i = 0; i < 200 && harness.session.state() != mp6::State::Running; ++i) {
        feedSupply(harness);
        harness.step();
        for (Bytes& write : harness.take()) {
            writes.push_back(std::move(write));
        }
    }
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Running));
    CHECK_EQ(countCommand(writes, 2), 1);
    CHECK_EQ(targetWord(writes.back(), 0), 16000);
    CHECK_EQ(writes.back()[15], 0x40);
    CHECK_EQ(sequenceOf(writes.back()), 0);  // ordinary sequence after forced 127

    // Keep the target fresh but let supply telemetry expire: Runnning motion
    // must stop with a reason rather than keep commanding targets.
    for (int i = 0; i < 2200; ++i) {
        Ard selector7 = zeroArd();
        selector7[0] = 60;
        selector7[2] = 15855;
        harness.feed(makeType48(7, 3, axesAt(16000), selector7));
        harness.session.setTarget(target);
        harness.step();
        if (harness.session.state() != mp6::State::Running) {
            break;
        }
    }
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Stopped));
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::SupplyStale));
    CHECK(harness.session.targetFresh());  // target was kept fresh
}

// ---------------------------------------------------------------------------
// Targets and stop paths
// ---------------------------------------------------------------------------

void testInvalidTargetAndPep() {
    Harness harness;
    CHECK(bringToRunning(harness));
    harness.take();

    std::string why;
    CHECK(!harness.session.armPepSpecialTarget(0, 1, &why));
    CHECK(why.find("unresolved") != std::string::npos);

    mp6::StepTarget target;
    target.steps = {16000, -32766, 16000, 16000, 16000, 16000};
    target.valid = true;
    harness.session.setTarget(target);
    harness.step(17);  // next 60 Hz target slot
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Stopped));
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::InvalidTarget));

    // The stop path re-asserts run mode 1, then the two zero-rate frames are
    // emitted by the burst driver on later ticks (50 ms spacing).
    std::vector<Bytes> writes = harness.take();
    CHECK_EQ(countCommand(writes, 6), 1);
    harness.step(1);
    harness.step(50);
    for (Bytes& frame : harness.take()) {
        writes.push_back(std::move(frame));
    }
    CHECK(countCommand(writes, 1) >= 2);
}

void testPepOptInEmitsReservedWord() {
    mp6::SessionConfig config;
    config.allowPepSpecialTargets = true;
    Harness harness(config);
    CHECK(bringToRunning(harness));
    harness.take();
    CHECK(harness.session.armPepSpecialTarget(0, 1));

    mp6::StepTarget target;
    target.steps = {16000, 16000, 16000, 16000, 16000, 16000};
    target.valid = true;
    harness.session.setTarget(target);
    harness.step(17);
    std::vector<Bytes> writes = harness.take();
    CHECK_EQ(countCommand(writes, 2), 1);
    // Raw word 1 on axis 1 only for the deliberately armed frame.
    CHECK_EQ(writes.back()[3], 1);
    CHECK_EQ(writes.back()[4], 0);
    CHECK_EQ(targetWord(writes.back(), 1), 16000);
}

void testRxStaleStopsMotion() {
    // Make the comms TTL the first deadline to expire: feedback freshness is
    // deliberately longer so the RX-stale path itself is exercised.
    mp6::SessionConfig config;
    config.feedbackFreshMs = 3000;
    Harness harness(config);
    CHECK(bringToRunning(harness));
    mp6::StepTarget target;
    target.steps = {16000, 16000, 16000, 16000, 16000, 16000};
    target.valid = true;

    // Keep only the host target fresh; controller RX stops arriving.
    for (int i = 0; i < 1700; ++i) {
        harness.session.setTarget(target);
        harness.step();
        if (harness.session.state() != mp6::State::Running) {
            break;
        }
    }
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Stopped));
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::RxStale));
    CHECK(harness.session.diagnostics().detail.find("no controller frame") !=
          std::string::npos);
}

void testEmergencyStopAndWriteFailure() {
    {
        Harness harness;
        CHECK(bringToRunning(harness));
        harness.take();
        CHECK(harness.session.requestEmergencyStop());
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::SafeOff));
        CHECK_EQ(static_cast<int>(harness.session.stopReason()),
                 static_cast<int>(mp6::StopReason::EmergencyStop));

        std::vector<Bytes> writes = harness.take();
        CHECK_EQ(writes.size(), static_cast<std::size_t>(6));
        CHECK_EQ(writes[0][2], 6);   // run mode 1 first
        CHECK_EQ(writes[0][3], 1);
        CHECK_EQ(writes[1][2], 1);   // two zero-rate stop frames
        CHECK_EQ(writes[2][2], 1);
        CHECK_EQ(writes[3][2], 1);   // three motor-disable frames
        CHECK_EQ(writes[3][15], 0x80);
        CHECK_EQ(writes[5][15], 0x80);
        harness.step(1000);
        CHECK(harness.take().empty());
    }
    {
        Harness harness;
        CHECK(bringToRunning(harness));
        harness.take();
        mp6::StepTarget target;
        target.steps = {16000, 16000, 16000, 16000, 16000, 16000};
        target.valid = true;
        harness.session.setTarget(target);
        harness.link.setWriteFailing(true);
        harness.step(17);  // next target slot attempts a write
        harness.step();    // pending failure is consumed here
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
        CHECK_EQ(static_cast<int>(harness.session.stopReason()),
                 static_cast<int>(mp6::StopReason::WriteFailure));
        CHECK(harness.session.txCounters().writeFailures >= 1);
        harness.link.setWriteFailing(false);
    }
}

void testLinkNotOpen() {
    mp6::MemoryLink link;
    mp6::Session session;
    session.attach(&link);
    std::string why;
    CHECK(!session.requestIdentify(&why));
    CHECK(why.find("not open") != std::string::npos);
    CHECK_EQ(static_cast<int>(session.stopReason()),
             static_cast<int>(mp6::StopReason::LinkNotOpen));
}

// ---------------------------------------------------------------------------
// Docked pose vs neutral-ready pose, deterministic ramp endpoints
// ---------------------------------------------------------------------------

void testDockParkEndpoint() {
    Harness harness;
    Plant plant(4500);
    CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Parked));
    CHECK(plant.haveTarget);
    for (int i = 0; i < 6; ++i) {
        CHECK_EQ(plant.lastTarget[static_cast<std::size_t>(i)], 3000);
    }

    std::string why;
    CHECK(harness.session.requestPark(&why));
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Parking));
    CHECK_EQ(harness.session.rampStatus().phase, 0);
    CHECK_EQ(harness.session.rampStatus().goal[0], 5000);

    int maxTarget = plant.lastTarget[0];
    bool reachedDock = false;
    for (int i = 0; i < 40000; ++i) {
        std::vector<Bytes> writes = plant.step(harness);
        for (const Bytes& write : writes) {
            if (write.size() >= 18 && write[2] == 2) {
                maxTarget = std::max(maxTarget, targetWord(write, 0));
            }
        }
        if (harness.session.state() == mp6::State::Docked) {
            reachedDock = true;
            break;
        }
    }
    CHECK(reachedDock);
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::UserPark));
    CHECK_EQ(plant.lastTarget[0], 5000);
    CHECK_EQ(maxTarget, 5000);  // exact endpoint, not the old ~97% tail

    // A Docked pose is the only vent-capable pose. Shutdown from here must not
    // emit a vent until after the observed dock settle (already true).
    harness.take();
    CHECK(harness.session.requestShutdown(&why));
    bool vented = false;
    bool ventBeforeDock = false;
    for (int i = 0; i < 20000; ++i) {
        const mp6::State before = harness.session.state();
        std::vector<Bytes> writes = plant.step(harness);
        for (const Bytes& write : writes) {
            if (write.size() >= 18 && write[2] == 4) {
                if (before != mp6::State::Docked) {
                    ventBeforeDock = true;
                }
                vented = true;
                CHECK_EQ(write[3], 4);  // vent byte
            }
        }
        if (harness.session.state() == mp6::State::SafeOff) {
            break;
        }
    }
    CHECK(vented);
    CHECK(!ventBeforeDock);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::SafeOff));
}

void testShutdownWaitsForDockSettle() {
    Harness harness;
    Plant plant(4500);
    CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
    CHECK(startMotionPlant(harness, plant));
    CHECK(harness.session.requestStopMotion());
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Stopped));
    harness.take();

    // Stopped alone is not proof of the dock pose, and stale feedback cannot
    // authorize the descent.
    harness.step(600);
    std::string why;
    CHECK(!harness.session.requestShutdown(&why));
    CHECK(why.find("fresh") != std::string::npos);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Stopped));

    plant.step(harness);  // refresh feedback
    CHECK(harness.session.requestShutdown(&why));
    // Lifecycle profile transaction, then the 500 ms source pause, then the
    // descent. No descent target may leave before the descent starts.
    int descentTargetsBeforeParking = 0;
    for (int i = 0; i < 2000 && harness.session.state() != mp6::State::Parking;
         ++i) {
        for (const Bytes& write : plant.step(harness)) {
            if (write.size() >= 18 && write[2] == 2) {
                ++descentTargetsBeforeParking;
            }
        }
    }
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Parking));
    // The descent's first target goes out on the tick that enters Parking.
    CHECK(descentTargetsBeforeParking <= 1);
    CHECK_EQ(harness.session.rampStatus().goal[0], 5000);

    int maxTarget = plant.lastTarget[0];
    bool sawDock = false;
    bool ventBeforeDock = false;
    bool vented = false;
    for (int i = 0; i < 60000; ++i) {
        const mp6::State before = harness.session.state();
        std::vector<Bytes> writes = plant.step(harness);
        for (const Bytes& write : writes) {
            if (write.size() >= 18 && write[2] == 2) {
                maxTarget = std::max(maxTarget, targetWord(write, 0));
            }
            if (write.size() >= 18 && write[2] == 4) {
                vented = true;
                if (before != mp6::State::Docked) {
                    ventBeforeDock = true;
                }
            }
        }
        if (harness.session.state() == mp6::State::Docked) {
            sawDock = true;
        }
        if (harness.session.state() == mp6::State::SafeOff) {
            break;
        }
    }
    CHECK(sawDock);
    CHECK(vented);
    CHECK(!ventBeforeDock);
    CHECK_EQ(maxTarget, 5000);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::SafeOff));

    // SafeOff is deliberately terminal until the link is reattached.
    CHECK(!harness.session.requestIdentify(&why));
    CHECK(why.find("terminal") != std::string::npos);
    CHECK(!harness.session.requestConfigure(&why));
    CHECK(why.find("terminal") != std::string::npos);
    CHECK(!harness.session.requestInitialize(&why));
    CHECK(why.find("terminal") != std::string::npos);
    CHECK(!harness.session.requestStartMotion(&why));
    CHECK(why.find("terminal") != std::string::npos);
    harness.session.attach(&harness.link);
    CHECK(harness.session.requestIdentify(&why));
}

// ---------------------------------------------------------------------------
// Failure cleanup, reader terminal failure and link stats
// ---------------------------------------------------------------------------

void testConfigurationWriteFailureCleanup() {
    Harness harness;
    harness.autoAckConfig = false;
    CHECK(identify(harness));
    CHECK(prepareInputs(harness));
    std::string why;
    CHECK(harness.session.requestConfigure(&why));
    harness.step(1);  // first frame succeeds
    CHECK_EQ(harness.session.txCounters().command5, 1ull);
    harness.take();
    const mp6::LinkStats before = harness.link.stats();

    harness.link.setWriteFailing(true);
    harness.step(1);  // second frame write fails
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Configuring));
    const mp6::LinkStats failed = harness.link.stats();
    CHECK_EQ(failed.txWrites, before.txWrites);  // failed writes are not TX
    CHECK(failed.writeFailures > before.writeFailures);
    CHECK_EQ(harness.session.txCounters().command5, 1ull);

    harness.step(1);  // pending failure -> stop, transaction cleared
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Stopped));
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::WriteFailure));
    CHECK(!harness.session.configStatus().configuring);
    CHECK(!harness.session.configured());

    // The link is still usable, so a fresh configuration can start.
    harness.link.setWriteFailing(false);
    CHECK(harness.session.requestConfigure(&why));
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Configuring));
    driveConfigFrames(harness, mp6::kConfigFrameCount);
    harness.feed(makeType43(static_cast<std::uint8_t>(mp6::kConfigParameterCount)));
    harness.step();
    CHECK(harness.session.configured());
}

void testIdentityLossCancelsConfiguration() {
    Harness harness;
    harness.autoAckConfig = false;
    CHECK(identify(harness));
    CHECK(prepareInputs(harness));
    std::string why;
    CHECK(harness.session.requestConfigure(&why));
    harness.step(1);
    CHECK_EQ(harness.session.txCounters().command5, 1ull);
    harness.take();

    // A selector-1 pairing mismatch during the transaction is identity loss.
    Ard pairing = zeroArd();
    pairing[1] = 2;  // configured/expected platform is 1
    pairing[2] = 3;
    harness.feed(makeType48(1, 3, axesAt(16000), pairing));
    harness.step();
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::IdentityLost));
    CHECK(!harness.session.configStatus().configuring);
    CHECK(!harness.session.configured());
    CHECK_EQ(harness.session.configStatus().framesSent, 0);
    CHECK(!harness.session.requestConfigure(&why));
    CHECK(why.find("identity") != std::string::npos);

    // A fresh identify session is the only recovery, and configuration works.
    CHECK(identify(harness));
    CHECK(harness.session.requestConfigure(&why));
    driveConfigFrames(harness, mp6::kConfigFrameCount);
    harness.feed(makeType43(static_cast<std::uint8_t>(mp6::kConfigParameterCount)));
    harness.step();
    CHECK(harness.session.configured());
}

void testWriteFailureBestEffortStop() {
    Harness harness;
    CHECK(bringToRunning(harness));
    harness.take();
    mp6::StepTarget target;
    target.steps = {16000, 16000, 16000, 16000, 16000, 16000};
    target.valid = true;
    harness.session.setTarget(target);

    harness.link.setWriteFailing(true);
    harness.step(17);  // target write fails; failure is pending
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Running));
    harness.link.setWriteFailing(false);

    harness.step(1);  // pending failure: best-effort run-mode 1 is written
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Stopped));
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::WriteFailure));
    std::vector<Bytes> writes = harness.take();
    CHECK_EQ(countCommand(writes, 6), 1);
    CHECK_EQ(countCommand(writes, 1), 0);

    harness.step(1);
    harness.step(50);
    writes = harness.take();
    CHECK_EQ(countCommand(writes, 1), 2);  // exactly the two-frame stop burst
    harness.step(1000);
    for (const Bytes& write : harness.take()) {
        CHECK(write[2] != 1);  // no stop-burst retry storm in Stopped
    }
}

void testReadFailureStopsPromptly() {
    Harness harness;
    CHECK(bringToRunning(harness));
    harness.take();
    harness.link.setReadFailing(true);
    harness.step(1);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Stopped));
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::LinkClosed));
    CHECK(harness.session.diagnostics().detail.find("reader") !=
          std::string::npos);

    harness.take();
    for (int i = 0; i < 100; ++i) {
        harness.step(50);
    }
    CHECK(harness.take().empty());  // no RX, no TX after terminal failure
}

// ---------------------------------------------------------------------------
// Explicit half-even rounding independent of the floating-point environment
// ---------------------------------------------------------------------------

void testHalfEvenUnderRoundingModes() {
    const int original = std::fegetround();
    const int modes[4] = {FE_TONEAREST, FE_UPWARD, FE_DOWNWARD,
                          FE_TOWARDZERO};
    // pOffset alternates +0.5 and -1.5, so both positive and negative exact
    // ties occur. pScale 8 keeps word*scale/8 integral and exact.
    const Axes first = {1, 2, 3, 4, 5, 6};
    const int firstExpected[6] = {2, 0, 4, 2, 6, 4};
    const Axes second = {0, 0, 0, 0, 0, 0};
    const int secondExpected[6] = {0, -2, 0, -2, 0, -2};

    for (int mode : modes) {
        CHECK_EQ(std::fesetround(mode), 0);
        Harness harness;
        CHECK(identify(harness));
        mp6::Calibration calibration = fullCalibration(16000);
        for (int i = 0; i < 6; ++i) {
            calibration.axis[static_cast<std::size_t>(i)].pOffset =
                (i % 2 == 0) ? 0.5 : -1.5;
        }
        std::string why;
        CHECK(harness.session.setControllerConfig(fullControllerConfig(), &why));
        CHECK(supplyPidProfiles(harness));
        CHECK(harness.session.setCalibration(calibration, &why));
        CHECK(harness.session.setNeutralSteps(neutralAt(16000), &why));
        CHECK(harness.session.requestConfigure(&why));
        driveConfigFrames(harness, mp6::kConfigFrameCount);
        harness.feed(
            makeType43(static_cast<std::uint8_t>(mp6::kConfigParameterCount)));
        harness.step();
        CHECK(harness.session.configured());

        harness.feed(makeType48(0, 3, first, zeroArd()));
        harness.step();
        for (int i = 0; i < 6; ++i) {
            CHECK_EQ(harness.session.telemetry().axes[static_cast<std::size_t>(i)]
                         .calibrated,
                     firstExpected[i]);
        }
        harness.feed(makeType48(0, 3, second, zeroArd()));
        harness.step();
        for (int i = 0; i < 6; ++i) {
            CHECK_EQ(harness.session.telemetry().axes[static_cast<std::size_t>(i)]
                         .calibrated,
                     secondExpected[i]);
        }
    }
    CHECK_EQ(std::fesetround(original), 0);
}

// ---------------------------------------------------------------------------
// Review-2 regressions (N1-N6 and the stale-dock audit)
// ---------------------------------------------------------------------------

void testControllerConfigValidatesIncoming() {
    // N1: the validator must inspect the argument, not the previously stored
    // record, and a refused edit must leave the last good configuration intact.
    Harness harness;
    CHECK(identify(harness));
    harness.take();  // discard the identity probe
    std::string why;
    const mp6::ControllerConfig good = fullControllerConfig();
    CHECK(harness.session.setControllerConfig(good, &why));
    CHECK(supplyPidProfiles(harness));

    mp6::ControllerConfig outOfRange = good;
    outOfRange.benchTest = 40000;  // wire range is -32767..32768
    CHECK(!harness.session.setControllerConfig(outOfRange, &why));
    CHECK(why.find("5 BenchTest") != std::string::npos);

    // A configuration built after the refusal must carry the accepted value.
    CHECK(harness.session.requestConfigure(&why));
    const std::vector<Bytes> frames =
        driveConfigFrames(harness, mp6::kConfigFrameCount);
    CHECK(frames.size() >= 2);
    CHECK_EQ(payloadPairId(frames[1], 0), 5);
    CHECK_EQ(payloadPairValue(frames[1], 0), 0);  // 0, not the rejected 40000
    harness.feed(makeType43(static_cast<std::uint8_t>(mp6::kConfigParameterCount)));
    harness.step();
    CHECK(harness.session.configured());

    // A record that becomes incomplete is accepted by the setter (missing is a
    // transaction-time diagnostic) and the named missing parameter comes from
    // the incoming record.
    mp6::ControllerConfig incomplete = fullControllerConfig();
    incomplete.motValFinal.reset();
    CHECK(harness.session.setControllerConfig(incomplete, &why));
    CHECK(!harness.session.requestConfigure(&why));
    CHECK(why.find("30 MotValFinal") != std::string::npos);
    bool named = false;
    for (const std::string& name : harness.session.diagnostics().missing) {
        if (name.find("30 MotValFinal") != std::string::npos) {
            named = true;
        }
    }
    CHECK(named);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Identified));
}

void testDockProofInvalidatedByInputEdits() {
    // N2: a configuration edit invalidates the ack and the Docked proof when the
    // platform is NOT powered (configured, unpowered).
    {
        Harness harness;
        CHECK(bringToConfigured(harness));
        harness.take();
        std::string why;
        mp6::ControllerConfig edited = fullControllerConfig();
        edited.stepRev = 1200;
        CHECK(harness.session.setControllerConfig(edited, &why));
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Identified));
        CHECK(!harness.session.configured());
        // Identity survived, so a fresh configuration is possible.
        CHECK(harness.session.requestConfigure(&why));
        driveConfigFrames(harness, mp6::kConfigFrameCount);
        harness.feed(
            makeType43(static_cast<std::uint8_t>(mp6::kConfigParameterCount)));
        harness.step();
        CHECK(harness.session.configured());
    }
    // A powered (Docked) platform refuses config and calibration edits, so the
    // dock proof cannot be silently dropped under the operator. Shutdown still
    // completes from the same state (no stranded powered platform).
    {
        Harness harness;
        Plant plant(4500);
        CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
        harness.take();
        std::string why;
        mp6::ControllerConfig edited = fullControllerConfig();
        edited.stepRev = 1200;
        CHECK(!harness.session.setControllerConfig(edited, &why));
        CHECK(why.find("powered") != std::string::npos);
        CHECK(!harness.session.setCalibration(fullCalibration(5100), &why));
        CHECK(why.find("powered") != std::string::npos);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Docked));
        CHECK(harness.session.configured());
        CHECK(harness.session.requestShutdown(&why));
        std::vector<Bytes> all;
        for (int i = 0; i < 6000 && harness.session.state() != mp6::State::SafeOff;
             ++i) {
            for (Bytes& write : plant.step(harness)) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::SafeOff));
        int vents = 0;
        for (const Bytes& write : all) {
            if (write.size() >= 4 && write[2] == 4 && write[3] == 4) {
                ++vents;
            }
        }
        CHECK(vents >= 1);
    }
}

void testEmergencyStopDuringConfiguration() {
    // N3: an emergency stop inside a configuration transaction must cancel the
    // transaction before the stop frames are gated, and must actually write.
    {
        Harness harness;
        harness.autoAckConfig = false;
        CHECK(identify(harness));
        CHECK(prepareInputs(harness));
        std::string why;
        CHECK(harness.session.requestConfigure(&why));
        driveConfigFrames(harness, 3);
        CHECK_EQ(harness.session.txCounters().command5, 3ull);
        harness.take();

        CHECK(harness.session.requestEmergencyStop(&why));
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::SafeOff));
        const std::vector<Bytes> writes = harness.take();
        CHECK_EQ(writes.size(), static_cast<std::size_t>(6));
        CHECK_EQ(countCommand(writes, 5), 0);
        CHECK_EQ(writes[0][2], 6);
        CHECK_EQ(writes[0][3], 1);  // run mode 1
        CHECK_EQ(countCommand(writes, 1), 5);  // 2 zero-rate + 3 disable frames
        CHECK(harness.session.diagnostics().detail.find("abandoned") !=
              std::string::npos);
        CHECK(harness.session.diagnostics().detail.find("6 of 6") !=
              std::string::npos);
        CHECK(!harness.session.configStatus().configuring);
        CHECK(!harness.session.configured());
        harness.step(1000);
        CHECK(harness.take().empty());  // the transaction does not resume
    }
    {
        // Pre-identity: the TX policy permits only command 3, so no frame may
        // be sent. SafeOff is still entered, but the detail says so instead of
        // pretending the motors stopped.
        Harness harness;
        harness.autoAckConfig = false;
        CHECK(harness.session.requestIdentify());
        harness.step();
        harness.take();
        CHECK(harness.session.requestEmergencyStop());
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::SafeOff));
        CHECK(harness.take().empty());
        CHECK(harness.session.diagnostics().detail.find("0 of 6") !=
              std::string::npos);
        CHECK(harness.session.diagnostics().detail.find("may still be enabled") !=
              std::string::npos);
    }
}

void testIdentityLossStopsAndResets() {
    // N4: identity loss while a previously authenticated session is moving must
    // send the bounded stop subset (comms loss) but reset every guard.
    {
        Harness harness;
        Plant plant(4500);
        CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
        CHECK(startMotionPlant(harness, plant));
        std::string why;
        CHECK(harness.session.requestPark(&why));
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Parking));
        harness.take();

        // Cut RX while Parking. The RX-stale guard only covers Running, so the
        // 10 s comms TTL ends this session mid-descent.
        std::vector<Bytes> lossWrites;
        bool lost = false;
        for (int i = 0; i <= 12000; ++i) {
            harness.step(1);
            std::vector<Bytes> writes = harness.take();
            if (harness.session.state() == mp6::State::IdentityLost) {
                lossWrites = std::move(writes);
                lost = true;
                break;
            }
        }
        CHECK(lost);
        CHECK_EQ(static_cast<int>(harness.session.stopReason()),
                 static_cast<int>(mp6::StopReason::CommsLost));
        CHECK_EQ(lossWrites.size(), static_cast<std::size_t>(3));
        CHECK_EQ(countCommand(lossWrites, 6), 1);
        CHECK_EQ(lossWrites[0][3], 1);
        CHECK_EQ(countCommand(lossWrites, 1), 2);
        CHECK_EQ(countCommand(lossWrites, 2), 0);
        CHECK(harness.session.diagnostics().detail.find(
                  "best-effort stop: 3 frames written") != std::string::npos);
        CHECK(!harness.session.identityAccepted());
        CHECK(!harness.session.configured());

        // Re-identify is refused while the last known power state is on: identify
        // clears authentication, and a powered platform cannot be shut down from
        // Identified. This is an explicit boundary, not a silent recovery (C).
        std::string idWhy;
        CHECK(!harness.session.requestIdentify(&idWhy));
        CHECK(idWhy.find("powered") != std::string::npos);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::IdentityLost));
    }
    {
        // A pairing mismatch means the responder is a different platform, so
        // this session does not transmit stop frames to it.
        Harness harness;
        CHECK(bringToRunning(harness));
        harness.take();
        Ard pairing = zeroArd();
        pairing[1] = 2;  // paired number is 1
        pairing[2] = 3;
        harness.feed(makeType48(1, 3, axesAt(16000), pairing));
        harness.step();
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::IdentityLost));
        CHECK_EQ(static_cast<int>(harness.session.stopReason()),
                 static_cast<int>(mp6::StopReason::PairingMismatch));
        CHECK(harness.take().empty());
        CHECK(harness.session.diagnostics().detail.find(
                  "not the paired platform") != std::string::npos);
    }
}

void testEmergencyStopClearsRampFollowUps() {
    // N5: follow-up flags must not survive the emergency path.
    Harness harness;
    Plant plant(4500);
    CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
    CHECK(startMotionPlant(harness, plant));
    std::string why;
    CHECK(harness.session.requestPark(&why));
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Parking));
    CHECK(harness.session.rampStatus().active);
    harness.take();

    CHECK(harness.session.requestEmergencyStop());
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::SafeOff));
    CHECK(!harness.session.rampStatus().active);
    CHECK(harness.session.lifecycleStatus().stage == "SafeOff");
    const std::vector<Bytes> writes = harness.take();
    CHECK_EQ(countCommand(writes, 6), 1);
    CHECK_EQ(countCommand(writes, 4), 0);
    // A stale park/shutdown follow-up cannot re-arm anything in SafeOff.
    CHECK(!harness.session.requestShutdown(&why));
    CHECK(!harness.session.requestPark(&why));
    harness.step(2000);
    CHECK(harness.take().empty());
}

void testInputEditGuards() {
    // N6: neutral and pressure inputs are frozen once a lifecycle stage or
    // motion consumes them.
    {
        Harness harness;
        CHECK(bringToRunning(harness));
        std::string why;
        CHECK(!harness.session.setNeutralSteps(neutralAt(16000), &why));
        CHECK(why.find("cannot change") != std::string::npos);
        CHECK(!harness.session.setPressurePlan(mp6::PressurePlan{}, &why));
        CHECK(why.find("cannot change") != std::string::npos);
        CHECK(!harness.session.setCalibration(fullCalibration(16000), &why));
        CHECK(why.find("cannot change") != std::string::npos);
        CHECK(!harness.session.setControllerConfig(fullControllerConfig(), &why));
        CHECK(why.find("cannot change") != std::string::npos);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Running));

        CHECK(harness.session.requestPark(&why));
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Parking));
        CHECK(!harness.session.setNeutralSteps(neutralAt(16000), &why));
        CHECK(why.find("cannot change") != std::string::npos);
        CHECK(!harness.session.setPressurePlan(mp6::PressurePlan{}, &why));
    }
    {
        // A docked platform is powered: idle edits are refused with the
        // shut-down-first reason, not treated as legal, and a shutdown in
        // progress is refused for the same reason (C).
        Harness harness;
        Plant plant(4500);
        CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
        std::string why;
        CHECK(!harness.session.setNeutralSteps(neutralAt(3000), &why));
        CHECK(why.find("powered") != std::string::npos);
        CHECK(!harness.session.setPressurePlan(mp6::PressurePlan{}, &why));
        CHECK(why.find("powered") != std::string::npos);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Docked));
        CHECK(harness.session.requestShutdown(&why));
        CHECK(!harness.session.setPressurePlan(mp6::PressurePlan{}, &why));
        CHECK(why.find("powered") != std::string::npos);
        CHECK(!harness.session.setNeutralSteps(neutralAt(3000), &why));
        CHECK(why.find("powered") != std::string::npos);
    }
    {
        // Before power-on the same edits are legal (initial setup).
        Harness harness;
        CHECK(bringToConfigured(harness));
        std::string why;
        CHECK(harness.session.setNeutralSteps(neutralAt(3000), &why));
        CHECK(harness.session.setPressurePlan(mp6::PressurePlan{}, &why));
    }
}

void testShutdownRechecksDockProof() {
    // Stale-dock audit: Docked is an observation, not a licence to vent.
    Harness harness;
    Plant plant(4500);
    CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
    harness.take();
    std::string why;

    // 1. Waiting makes the feedback stale; the historical Docked state alone
    //    must not authorize a vent.
    harness.step(600);
    harness.take();  // idle polls are not vents
    CHECK(!harness.session.requestShutdown(&why));
    CHECK(why.find("fresh") != std::string::npos);
    CHECK(harness.take().empty());
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Docked));
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::FeedbackStale));

    // 2. Fresh feedback, but the platform was moved externally while idle.
    plant.axes.fill(4200);
    plant.step(harness);
    harness.take();
    CHECK(!harness.session.requestShutdown(&why));
    CHECK(why.find("dock") != std::string::npos);
    CHECK(harness.take().empty());
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Docked));
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::ParkNotReached));

    // 3. requestPark re-verifies too and performs a fresh descent.
    CHECK(harness.session.requestPark(&why));
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Parking));
    CHECK_EQ(harness.session.rampStatus().goal[0], 5000);
    bool reDocked = false;
    for (int i = 0; i < 40000; ++i) {
        plant.step(harness);
        if (harness.session.state() == mp6::State::Docked) {
            reDocked = true;
            break;
        }
    }
    CHECK(reDocked);
    CHECK_EQ(plant.lastTarget[0], 5000);

    // 3b. Fresh feedback at dock, but controller telemetry reports AV sensors not OK (status bit 1 clear).
    harness.feed(makeType48(0, 1, plant.axes, zeroArd())); // status=1: configReceived=true, avOk=false
    harness.step(1);
    harness.take();
    CHECK(!harness.session.requestShutdown(&why));
    CHECK(why.find("AV") != std::string::npos || why.find("sensors") != std::string::npos);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Docked));

    // Restore AV OK
    harness.feed(makeType48(0, 3, plant.axes, zeroArd())); // status=3: configReceived=true, avOk=true
    harness.step(1);

    // 4. Fresh and at the dock with AV OK: the vent is allowed, and only from Docked.
    harness.take();
    CHECK(harness.session.requestShutdown(&why));
    bool vented = false;
    for (int i = 0; i < 20000; ++i) {
        const mp6::State before = harness.session.state();
        for (const Bytes& write : plant.step(harness)) {
            if (write.size() >= 18 && write[2] == 4) {
                vented = true;
                CHECK_EQ(static_cast<int>(before),
                         static_cast<int>(mp6::State::Docked));
                CHECK_EQ(write[3], 4);
            }
        }
        if (harness.session.state() == mp6::State::SafeOff) {
            break;
        }
    }
    CHECK(vented);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::SafeOff));
}

void testCalibratedFeedbackMatchesSourceFloatPrecision() {
    // N7 disposition (no tie epsilon): frm_Main.cs:32233 evaluates
    //   (int)Math.Round((float)word * pScale / 8f + pOffset)
    // in *single* precision. The exact real value of the represented float
    // inputs below is 3.5 - 2^-25, but single-precision evaluation rounds to
    // exactly 3.5, so the source returns 4 (ties to even). A double-precision
    // or epsilon-based rounder would return 3 and diverge from the source.
    const int original = std::fegetround();
    CHECK_EQ(std::fesetround(FE_TONEAREST), 0);

    const float scale = std::nextafter(8.0f, 16.0f);  // 8 + 2^-20
    const float offsetTie = 0.5f - 3.0f * (scale - 8.0f) / 8.0f;
    const float offset = std::nextafter(offsetTie, 0.0f);
    const float product = 3.0f * scale;
    const float quotient = product / 8.0f;
    const float value = quotient + offset;
    const double exact = 3.0 * static_cast<double>(scale) / 8.0 +
                         static_cast<double>(offset);

    CHECK(value == 3.5f);
    CHECK(exact < 3.5);
    CHECK(3.5 - exact == std::ldexp(1.0, -25));
    CHECK_EQ(static_cast<int>(std::nearbyint(static_cast<double>(value))), 4);
    CHECK_EQ(static_cast<int>(std::nearbyint(exact)), 3);

    Harness harness;
    CHECK(identify(harness));
    mp6::Calibration calibration = fullCalibration(16000);
    for (int i = 0; i < 6; ++i) {
        mp6::AxisCalibration& axis =
            calibration.axis[static_cast<std::size_t>(i)];
        axis.pScale = static_cast<double>(scale);
        axis.pOffset = static_cast<double>(offset);
    }
    std::string why;
    CHECK(harness.session.setControllerConfig(fullControllerConfig(), &why));
    CHECK(supplyPidProfiles(harness));
    CHECK(harness.session.setCalibration(calibration, &why));
    CHECK(harness.session.setNeutralSteps(neutralAt(16000), &why));
    CHECK(harness.session.requestConfigure(&why));
    driveConfigFrames(harness, mp6::kConfigFrameCount);
    harness.feed(makeType43(static_cast<std::uint8_t>(mp6::kConfigParameterCount)));
    harness.step();
    CHECK(harness.session.configured());

    harness.feed(makeType48(0, 3, axesAt(3), zeroArd()));
    harness.step();
    for (int i = 0; i < 6; ++i) {
        CHECK_EQ(harness.session.telemetry().axes[static_cast<std::size_t>(i)]
                     .calibrated,
                 4);
    }
    CHECK_EQ(std::fesetround(original), 0);
}

// ---------------------------------------------------------------------------
// Serial documentation guard
// ---------------------------------------------------------------------------

void testSerialDocumentationGuard() {
    if (!IsDocumentationMode()) {
        ++g_skips;
        std::printf("SKIP serial documentation guard (run with "
                    "STEWART_DOCUMENTATION_MODE=1)\n");
        return;
    }
    mp6::SerialLink link;
    CHECK(!link.open("COM1"));
    CHECK(!link.isOpen());
    CHECK(link.lastError().find("documentation mode") != std::string::npos);
    CHECK_EQ(link.stats().openCount, static_cast<std::uint64_t>(0));
}

}  // namespace

// ---------------------------------------------------------------------------
// Sol final-review session fixes: vent authority at transmission (F1),
// identify gating (F5), 60 Hz deadline chain (F13), per-mode lifecycle PID
// profiles and accepted-input invalidation.
// ---------------------------------------------------------------------------

// Shutdown has finished when it reaches a stop or SafeOff; Configuring is a
// transient inside the lifecycle profile transaction.
bool terminalShutdown(mp6::State state) {
    return state == mp6::State::Stopped || state == mp6::State::SafeOff;
}

// Command-4 frames carrying the struts-vent byte (kPneumaticVent = 4).
int countVentFrames(const std::vector<Bytes>& writes) {
    int count = 0;
    for (const Bytes& write : writes) {
        if (write.size() >= 4 && write[2] == 4 && write[3] == 4) {
            ++count;
        }
    }
    return count;
}

// Status nibble 1 clears AV OK; the frame still carries fresh feedback axes.
void injectAvFalse(Harness& harness, const Axes& axes) {
    harness.feed(makeType48(0, 1, axes, zeroArd()));
}

void testVentRechecksDockProofAtTransmission() {
    std::string why;

    // Control: with the dock proof intact the vent proceeds to SafeOff.
    {
        Harness harness;
        Plant plant(5000);
        CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
        harness.take();
        CHECK(harness.session.requestShutdown(&why));
        std::vector<Bytes> all;
        for (int i = 0; i < 6000 && harness.session.state() != mp6::State::SafeOff;
             ++i) {
            for (Bytes& write : plant.step(harness)) {
                all.push_back(std::move(write));
            }
        }
        CHECK(harness.session.state() == mp6::State::SafeOff);
        CHECK(countVentFrames(all) >= 1);
    }

    // F1 probe: AV lost after the request but before the vent is transmitted.
    // The lifecycle profile transaction runs first; the vent must not start.
    {
        Harness harness;
        Plant plant(5000);
        CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
        harness.take();
        CHECK(harness.session.requestShutdown(&why));
        CHECK(!harness.session.requestConfigure(&why));  // profile transaction owns command 5
        injectAvFalse(harness, plant.axes);
        std::vector<Bytes> all;
        for (int i = 0; i < 6000 && !terminalShutdown(harness.session.state());
             ++i) {
            harness.step();
            for (Bytes& write : harness.take()) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(countVentFrames(all), 0);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
        CHECK_EQ(static_cast<int>(harness.session.stopReason()),
                 static_cast<int>(mp6::StopReason::ParkNotReached));
        CHECK(harness.session.diagnostics().detail.find("vent aborted") !=
              std::string::npos);
    }

    // Delayed burst: AV is lost after the first vent frame. The second frame of
    // the two-frame burst must be refused at its own transmission.
    {
        Harness harness;
        Plant plant(5000);
        CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
        harness.take();
        CHECK(harness.session.requestShutdown(&why));
        std::vector<Bytes> all;
        for (int i = 0; i < 6000 && countVentFrames(all) == 0; ++i) {
            for (Bytes& write : plant.step(harness)) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(countVentFrames(all), 1);
        injectAvFalse(harness, plant.axes);
        for (int i = 0; i < 50 && !terminalShutdown(harness.session.state());
             ++i) {
            harness.step();
            for (Bytes& write : harness.take()) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(countVentFrames(all), 1);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
    }

    // Descent completion: Parked -> descent -> dock settle -> lifecycle profile.
    // AV is lost during the profile transaction, after the dock was observed.
    {
        Harness harness;
        Plant plant(5000);
        CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
        harness.take();
        // The profile is applied before the descent (B); AV is lost on the dock
        // settle, before the vent frames are transmitted.
        CHECK(harness.session.requestShutdown(&why));
        for (int i = 0; i < 20000 && harness.session.state() != mp6::State::Docked;
             ++i) {
            plant.step(harness);
        }
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Docked));
        injectAvFalse(harness, plant.axes);
        std::vector<Bytes> all;
        for (int i = 0; i < 6000 && !terminalShutdown(harness.session.state());
             ++i) {
            harness.step();
            for (Bytes& write : harness.take()) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(countVentFrames(all), 0);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
    }
}

void testIdentifyRefusedWhileActive() {
    std::string why;

    // Configuring: a command-5 transaction is in flight (no ack yet).
    {
        Harness harness;
        harness.autoAckConfig = false;
        CHECK(identify(harness));
        CHECK(prepareInputs(harness));
        CHECK(harness.session.requestConfigure(&why));
        CHECK(!harness.session.requestIdentify(&why));
        CHECK(why.find("identify refused") != std::string::npos);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Configuring));
    }

    // Initializing: the lifecycle profile transaction is the first stage.
    {
        Harness harness;
        CHECK(bringToConfigured(harness));
        feedSupply(harness);
        harness.step();
        CHECK(harness.session.requestInitialize(&why));
        CHECK(!harness.session.requestIdentify(&why));
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Initializing));
    }

    // Running, then Parking after a park request.
    {
        Harness harness;
        CHECK(bringToRunning(harness));
        CHECK(!harness.session.requestIdentify(&why));
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Running));
        CHECK(harness.session.requestPark(&why));
        CHECK(!harness.session.requestIdentify(&why));
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Parking));
    }

    // Docked: motors may still be enabled at the dock, so identify is refused.
    {
        Harness harness;
        Plant plant(5000);
        CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
        CHECK(!harness.session.requestIdentify(&why));
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Docked));
    }

    // Stopped with the stop burst drained: motors are still powered (AC on), so
    // identify is refused and the platform keeps its shutdown ability (C).
    {
        Harness harness;
        CHECK(bringToRunning(harness));
        CHECK(harness.session.requestStopMotion(&why));
        for (int i = 0; i < 10; ++i) {
            harness.step(50);  // drain the two-frame stop burst (50 ms spacing)
            harness.take();
        }
        CHECK(!harness.session.requestIdentify(&why));
        CHECK(why.find("powered") != std::string::npos);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
        CHECK(harness.session.identityAccepted());
        CHECK(harness.session.requestShutdown(&why));  // shutdown still possible
    }
}

// One render tick: inputs refreshed, target re-published, then the session tick.
void renderTick(Harness& harness, const mp6::StepTarget& target,
                std::uint64_t at) {
    harness.t = at;
    feedSupply(harness);
    harness.session.setTarget(target);
    harness.session.tick(at);
    harness.take();
}

mp6::StepTarget holdTarget() {
    mp6::StepTarget target;
    target.steps = {16000, 16000, 16000, 16000, 16000, 16000};
    target.valid = true;
    target.generation = 1;
    return target;
}

void testCadenceAveragesAtRenderRates() {
    // F13: the documented 60 Hz cadence must not depend on the render rate.
    // Fixed-integer 1000/hz deadlines measured about 45 Hz at 90 Hz renders.
    const mp6::StepTarget target = holdTarget();
    for (const std::uint64_t fps : {60u, 90u, 120u}) {
        Harness harness;
        CHECK(bringToRunning(harness));
        const std::uint64_t start = harness.t;
        const std::uint64_t before = harness.session.txCounters().command2;
        const std::uint64_t missedBefore =
            harness.session.txCounters().missedCadenceSlots;
        for (std::uint64_t k = 1; k <= fps; ++k) {
            renderTick(harness, target, start + k * 1000u / fps);
        }
        const std::uint64_t sent =
            harness.session.txCounters().command2 - before;
        CHECK(sent >= 59 && sent <= 61);
        CHECK_EQ(harness.session.txCounters().missedCadenceSlots - missedBefore,
                 0);
    }
}

void testCadenceStallResynchronisesWithoutBurst() {
    const mp6::StepTarget target = holdTarget();
    Harness harness;
    CHECK(bringToRunning(harness));
    const std::uint64_t start = harness.t;
    for (std::uint64_t k = 1; k <= 30; ++k) {
        renderTick(harness, target, start + k * 1000u / 60u);
    }
    // A 200 ms render stall: no ticks at all.
    const std::uint64_t resume = start + 500 + 200;
    const std::uint64_t sentBefore = harness.session.txCounters().command2;
    const std::uint64_t missedBefore =
        harness.session.txCounters().missedCadenceSlots;
    renderTick(harness, target, resume);
    // Exactly one frame at resume: stalled deadlines are counted, not burst.
    CHECK_EQ(harness.session.txCounters().command2 - sentBefore, 1);
    const std::uint64_t missed =
        harness.session.txCounters().missedCadenceSlots - missedBefore;
    CHECK(missed >= 10 && missed <= 13);
    renderTick(harness, target, resume + 1);
    CHECK_EQ(harness.session.txCounters().command2 - sentBefore, 1);

    // The chain resynchronises: 60 Hz resumes at its nominal rate.
    const std::uint64_t afterResume = harness.session.txCounters().command2;
    for (std::uint64_t k = 1; k <= 60; ++k) {
        renderTick(harness, target, resume + k * 1000u / 60u);
    }
    const std::uint64_t sent = harness.session.txCounters().command2 - afterResume;
    CHECK(sent >= 59 && sent <= 61);
}

void testLifecyclePidProfilesApplyAtBoundaries() {
    std::string why;

    // Configure requires its own profile; nothing else is sent before it.
    {
        Harness harness;
        CHECK(identify(harness));
        CHECK(harness.session.setControllerConfig(fullControllerConfig(), &why));
        CHECK(harness.session.setCalibration(fullCalibration(16000), &why));
        CHECK(harness.session.setNeutralSteps(neutralAt(16000), &why));
        CHECK(harness.session.setPidProfile(mp6::PidMode::Normal,
                                            pidProfileFor(mp6::PidMode::Normal),
                                            &why));
        CHECK(!harness.session.requestConfigure(&why));
        CHECK(why.find("PID profile (configure)") != std::string::npos);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Identified));
    }

    // Initialize refuses a missing lifecycle profile before any motor frame.
    {
        Harness harness;
        CHECK(identify(harness));
        CHECK(harness.session.setControllerConfig(fullControllerConfig(), &why));
        CHECK(harness.session.setCalibration(fullCalibration(16000), &why));
        CHECK(harness.session.setNeutralSteps(neutralAt(16000), &why));
        CHECK(harness.session.setPidProfile(mp6::PidMode::Configure,
                                            pidProfileFor(mp6::PidMode::Configure),
                                            &why));
        CHECK(harness.session.setPidProfile(mp6::PidMode::Normal,
                                            pidProfileFor(mp6::PidMode::Normal),
                                            &why));
        CHECK(harness.session.requestConfigure(&why));
        driveConfigFrames(harness, mp6::kConfigFrameCount);
        harness.step();
        CHECK(harness.session.configured());
        feedSupply(harness);
        harness.step();
        CHECK(!harness.session.requestInitialize(&why));
        CHECK(why.find("PID profile (lifecycle)") != std::string::npos);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Configured));
    }

    // Full sequence: Configure (P 3.0) and Lifecycle (P 2.0) before power at
    // Initialize; Normal (P 1.0) on an explicit Start, before Running (source:
    // the Start path applies the normal PID values, frm_Main.cs:28031-28060).
    {
        Harness harness;
        CHECK(identify(harness));
        CHECK(prepareInputs(harness));
        std::vector<Bytes> all;
        CHECK(harness.session.requestConfigure(&why));
        for (Bytes& write : driveConfigFrames(harness, mp6::kConfigFrameCount)) {
            all.push_back(std::move(write));
        }
        harness.step();
        CHECK(harness.session.configured());
        feedSupply(harness);
        harness.step();
        CHECK(harness.session.requestInitialize(&why));
        for (int i = 0; i < 25000 && harness.session.state() != mp6::State::Parked;
             ++i) {
            feedSupply(harness);
            harness.step();
            for (Bytes& write : harness.take()) {
                all.push_back(std::move(write));
            }
        }
        CHECK(harness.session.state() == mp6::State::Parked);
        CHECK(harness.session.appliedPidMode() == mp6::PidMode::Lifecycle);

        const std::size_t startIndex = all.size();
        feedSupply(harness);
        harness.step();
        harness.session.setTarget(holdTarget());
        CHECK(harness.session.requestStartMotion(&why));
        CHECK(!harness.session.appliedPidMode().has_value());  // in flight
        for (int i = 0; i < 200 && harness.session.state() != mp6::State::Running;
             ++i) {
            feedSupply(harness);
            harness.step();
            for (Bytes& write : harness.take()) {
                all.push_back(std::move(write));
            }
        }
        CHECK(harness.session.state() == mp6::State::Running);
        CHECK(harness.session.appliedPidMode() == mp6::PidMode::Normal);

        std::vector<int> proportional;  // wire P*10 in transaction order
        int lifecycleIndex = -1;
        int normalIndex = -1;
        int firstPowerIndex = -1;
        int firstTargetIndex = -1;
        for (std::size_t i = 0; i < all.size(); ++i) {
            const Bytes& write = all[i];
            if (write.size() < 18) {
                continue;
            }
            if (write[2] == 5 && payloadPairId(write, 0) == 22) {
                const int value = payloadPairValue(write, 0);
                proportional.push_back(value);
                if (value == 20 && lifecycleIndex < 0) {
                    lifecycleIndex = static_cast<int>(i);
                }
                if (value == 10 && normalIndex < 0) {
                    normalIndex = static_cast<int>(i);
                }
            }
            if (write[2] == 1 && firstPowerIndex < 0) {
                firstPowerIndex = static_cast<int>(i);
            }
            if (write[2] == 2 && i >= startIndex && firstTargetIndex < 0) {
                firstTargetIndex = static_cast<int>(i);
            }
        }
        CHECK_EQ(proportional.size(), static_cast<std::size_t>(3));
        if (proportional.size() == 3) {
            CHECK_EQ(proportional[0], 30);
            CHECK_EQ(proportional[1], 20);
            CHECK_EQ(proportional[2], 10);
        }
        CHECK(lifecycleIndex >= 0);
        CHECK(firstPowerIndex > lifecycleIndex);
        // Normal is applied by the Start transaction, after the Parked pose and
        // before the first motion frame.
        CHECK(normalIndex >= static_cast<int>(startIndex));
        CHECK(firstTargetIndex > normalIndex);
    }
}

void testLifecycleProfileAckFailureStopsBeforePower() {
    Harness harness;
    CHECK(bringToConfigured(harness));
    harness.autoAckConfig = false;
    feedSupply(harness);
    harness.step();
    std::string why;
    CHECK(harness.session.requestInitialize(&why));
    for (int i = 0; i < 5000 && harness.session.state() != mp6::State::Stopped;
         ++i) {
        feedSupply(harness);
        harness.step();
        harness.take();
    }
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Stopped));
    CHECK_EQ(static_cast<int>(harness.session.stopReason()),
             static_cast<int>(mp6::StopReason::ConfigRejected));
    CHECK(!harness.session.powerStatus().acRequested);
    CHECK(!harness.session.configured());
}

void testInvalidateInputsRevokesAuthority() {
    std::string why;
    // Powered (Parked): invalidation is refused and the pose survives.
    {
        Harness harness;
        CHECK(bringToParked(harness));
        CHECK(!harness.session.invalidateInputs(&why));
        CHECK(why.find("powered") != std::string::npos);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Parked));
    }
    // Unpowered (configured, before power-on): invalidation revokes the config
    // ack, the neutral readiness and the observed authority together.
    Harness harness;
    CHECK(bringToConfigured(harness));
    CHECK(harness.session.invalidateInputs(&why));
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Identified));
    CHECK(!harness.session.configured());
    CHECK(!harness.session.requestInitialize(&why));

    // Re-configuring is not enough: initialization needs the neutral again.
    CHECK(harness.session.requestConfigure(&why));
    driveConfigFrames(harness, mp6::kConfigFrameCount);
    harness.step();
    CHECK(harness.session.configured());
    feedSupply(harness);
    harness.step();
    CHECK(!harness.session.requestInitialize(&why));
    CHECK(why.find("neutral") != std::string::npos);

    Harness running;
    CHECK(bringToRunning(running));
    CHECK(!running.session.invalidateInputs(&why));
    CHECK_EQ(static_cast<int>(running.session.state()),
             static_cast<int>(mp6::State::Running));
}

// Stop and Park against a pending shutdown. Stop cancels the whole sequence
// (profile transaction, vent/power follow-ups, descent continuation); Park is
// refused while a shutdown owns the dock. Neither may leave shutdown active
// while reporting success.
void testStopAndParkDuringPendingShutdown() {
    std::string why;

    // Stop before the lifecycle profile is acknowledged: the late ACK must not
    // resume the vent, and the stop is bounded (run mode 1 plus two zero-rate
    // frames) and authenticated.
    {
        Harness harness;
        Plant plant(5000);
        CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
        harness.take();
        harness.autoAckConfig = false;
        CHECK(harness.session.requestShutdown(&why));
        std::vector<Bytes> all;
        for (int i = 0; i < 100 && harness.session.state() != mp6::State::Configuring;
             ++i) {
            harness.step();
            for (Bytes& write : harness.take()) {
                all.push_back(std::move(write));
            }
        }
        CHECK(harness.session.state() == mp6::State::Configuring);
        CHECK(harness.session.requestStopMotion(&why));
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
        // Dock-settle stop-burst frames may still be draining; count from here.
        all.clear();
        for (Bytes& write : harness.take()) {
            all.push_back(std::move(write));
        }
        harness.feed(makeType43(static_cast<std::uint8_t>(mp6::kConfigParameterCount)));
        for (int i = 0; i < 1000; ++i) {
            harness.step();
            for (Bytes& write : harness.take()) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(countVentFrames(all), 0);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
        CHECK(!harness.session.configured());
        CHECK_EQ(countCommand(all, 6), 1);  // run mode 1
        CHECK_EQ(countCommand(all, 1), 2);  // two zero-rate stop frames
    }

    // Stop after the ACK but before the delayed vent: the vent stage has been
    // set by the resume, and a Park request in that window is refused.
    {
        Harness harness;
        Plant plant(5000);
        CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
        harness.take();
        CHECK(harness.session.requestShutdown(&why));
        std::vector<Bytes> all;
        for (int i = 0; i < 6000 && harness.session.lifecycleStatus().stage !=
                                        "Shutdown: vent";
             ++i) {
            for (Bytes& write : plant.step(harness)) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(countVentFrames(all), 0);
        std::string parkWhy;
        CHECK(!harness.session.requestPark(&parkWhy));
        CHECK(parkWhy.find("shutdown") != std::string::npos);
        CHECK(harness.session.requestStopMotion(&why));
        for (int i = 0; i < 1000; ++i) {
            harness.step();
            for (Bytes& write : harness.take()) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(countVentFrames(all), 0);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
    }

    // Stop during the descent to the dock: the descent and the queued shutdown
    // are both abandoned; the plant is left where it stopped and never vents.
    {
        Harness harness;
        Plant plant(5000);
        CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
        harness.take();
        CHECK(harness.session.requestShutdown(&why));
        std::vector<Bytes> all;
        // Profile ACK and the source pause come first; wait for the descent.
        for (int i = 0; i < 2000 && harness.session.state() != mp6::State::Parking;
             ++i) {
            for (Bytes& write : plant.step(harness)) {
                all.push_back(std::move(write));
            }
        }
        CHECK(harness.session.state() == mp6::State::Parking);
        CHECK(harness.session.requestStopMotion(&why));
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
        for (int i = 0; i < 6000; ++i) {
            for (Bytes& write : plant.step(harness)) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(countVentFrames(all), 0);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
    }

    // Park before the ACK: refused while the shutdown is pending, and the
    // shutdown itself is untouched and still vents when the ACK arrives.
    {
        Harness harness;
        Plant plant(5000);
        CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
        harness.take();
        harness.autoAckConfig = false;
        CHECK(harness.session.requestShutdown(&why));
        CHECK(harness.session.requestShutdown(&why));  // idempotent, not restarted
        CHECK(!harness.session.requestPark(&why));
        CHECK(why.find("shutdown") != std::string::npos);
        CHECK(harness.session.state() == mp6::State::Configuring ||
              harness.session.state() == mp6::State::Docked);
        harness.autoAckConfig = true;
        std::vector<Bytes> all;
        for (int i = 0; i < 6000 && harness.session.state() != mp6::State::SafeOff;
             ++i) {
            for (Bytes& write : plant.step(harness)) {
                all.push_back(std::move(write));
            }
        }
        CHECK(harness.session.state() == mp6::State::SafeOff);
        CHECK(countVentFrames(all) >= 1);
    }
}

void testSessionReviewRoundThreeFixes() {
    std::string why;
    // A refused Start must leave every shutdown timing window intact.
    for (const std::uint64_t delay : {0u, 5u, 100u, 520u}) {
        Harness harness;
        Plant plant(3000);
        CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
        harness.take();
        CHECK(harness.session.requestShutdown(&why));
        for (std::uint64_t i = 0; i < delay; ++i) plant.step(harness);
        CHECK(!harness.session.requestStartMotion(&why));
        CHECK(harness.session.shutdownPending());
        std::vector<Bytes> writes;
        for (int i = 0; i < 10000 && harness.session.state() != mp6::State::SafeOff; ++i) {
            for (Bytes& frame : plant.step(harness)) writes.push_back(std::move(frame));
        }
        CHECK(harness.session.state() == mp6::State::SafeOff);
        CHECK(countVentFrames(writes) >= 1);
    }
    // Initialize cannot replace an in-progress shutdown during its parked pause.
    {
        Harness harness;
        Plant plant(3000);
        CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
        CHECK(harness.session.requestShutdown(&why));
        for (int i = 0; i < 520; ++i) plant.step(harness);
        CHECK(harness.session.state() == mp6::State::Parked);
        CHECK(!harness.session.requestInitialize(&why));
    }
    // Stop cancels the initial Configure while preserving only safe identity.
    {
        Harness harness;
        CHECK(identify(harness));
        CHECK(prepareInputs(harness));
        CHECK(harness.session.requestConfigure(&why));
        driveConfigFrames(harness, 5);
        CHECK(harness.session.requestStopMotion(&why));
        CHECK(harness.session.state() == mp6::State::Identified);
        CHECK(!harness.session.configStatus().configuring);
        harness.feed(makeType43(static_cast<std::uint8_t>(mp6::kConfigParameterCount)));
        harness.step(1000);
        CHECK(harness.session.state() == mp6::State::Identified);
        CHECK(!harness.session.configured());
        const std::vector<Bytes> lateWrites = harness.take();
        CHECK(countCommand(lateWrites, 5) == 0);
    }
    // A known powered configuration cannot be resent; ACK-loss recovery remains legal.
    {
        Harness harness;
        Plant plant(3000);
        CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
        CHECK(!harness.session.requestConfigure(&why));
        CHECK(why.find("already acknowledged") != std::string::npos);
    }
    // Failed powered profile clears pose authority; Configure remains the
    // documented recovery route and its refusal explains the lost ACK.
    {
        Harness harness;
        Plant plant(3000);
        CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
        harness.autoAckConfig = false;
        feedSupplyAxes(harness, plant.axes);
        harness.step();
        mp6::StepTarget target;
        target.steps = {3000, 3000, 3000, 3000, 3000, 3000};
        target.valid = true;
        target.generation = 1;
        harness.session.setTarget(target);
        CHECK(harness.session.requestStartMotion(&why));
        for (int i = 0; i < 10000 && harness.session.state() != mp6::State::Stopped; ++i) {
            plant.step(harness);
        }
        CHECK(harness.session.state() == mp6::State::Stopped);
        CHECK(harness.session.powerStatus().acRequested);
        CHECK(!harness.session.configured());
        CHECK(harness.session.requestShutdown(&why));
        // Freshness can still be observed at the failed ACK boundary; once the
        // guard blocks a lifecycle action, its reason must name the lost ACK.
        CHECK(harness.session.requestStopMotion(&why));
        harness.autoAckConfig = true;
        if (!harness.session.requestConfigure(&why)) {
            std::printf("recovery configure refused: %s\n", why.c_str());
        }
        CHECK(harness.session.state() == mp6::State::Configuring);
        for (int i = 0; i < 200 && !harness.session.configured(); ++i) plant.step(harness);
        CHECK(harness.session.configured());
    }
}

// ---------------------------------------------------------------------------
// Sonnet review of the session fixes (sonnet-session-sol-review-1.md):
// A slow telemetry and vent authority, B lifecycle profile before descent,
// C powered-state guard, D cadence normalisation and shortfall, E Normal on
// Start with readiness after its ACK.
// ---------------------------------------------------------------------------

// Stopped with a pending shutdown is not the end: the shutdown still proceeds.
bool isTerminalShutdown(const mp6::Session& session) {
    return !session.shutdownPending();
}

// A: a Docked or Parked shutdown completes with type-48 telemetry at 10, 20, 33
// and 50 ms periods, across several phases of the telemetry clock. The
// calibrated feedback must stay usable across the lifecycle profile ACK.
void testShutdownCompletesWithSlowTelemetry() {
    for (const std::uint64_t period : {10u, 20u, 33u, 50u}) {
        for (const std::uint64_t phase : {std::uint64_t{0}, period / 2}) {
            for (const bool fromParked : {false, true}) {
                Harness harness;
                Plant plant(5000);
                std::string why;
                const bool brought =
                    fromParked ? bringToParkedPlant(harness, plant, 5000, 3000)
                               : bringToDockedPlant(harness, plant, 5000, 3000);
                CHECK(brought);
                harness.take();
                plant.telemetryPeriodMs = period;
                plant.telemetryPhase = phase;

                bool sawConfiguring = false;
                bool calibrationKept = true;
                std::vector<Bytes> all;
                CHECK(harness.session.requestShutdown(&why));
                for (int i = 0; i < 40000 && !isTerminalShutdown(harness.session);
                     ++i) {
                    for (Bytes& write : plant.step(harness)) {
                        all.push_back(std::move(write));
                    }
                    if (harness.session.state() == mp6::State::Configuring) {
                        sawConfiguring = true;
                        for (const mp6::AxisFeedback& axis : harness.session.telemetry().axes) {
                            calibrationKept = calibrationKept && axis.calibratedValid;
                        }
                    }
                }
                CHECK(sawConfiguring);
                // Calibrated feedback is not invalidated by the PID ACK pending.
                CHECK(calibrationKept);
                CHECK_EQ(static_cast<int>(harness.session.state()),
                         static_cast<int>(mp6::State::SafeOff));
                CHECK(countVentFrames(all) >= 1);
            }
        }
    }
}

// A: a dropped frame right after the ACK is bounded. The vent waits for a fresh
// dock observation for at most ventStaleWaitMs, then aborts. An observation that
// arrives in time completes the shutdown.
void testVentWaitIsBoundedForStaleTelemetry() {
    mp6::SessionConfig config;
    config.feedbackFreshMs = 20;  // make the proof go stale between 33 ms frames
    config.statusFreshMs = 20;
    // Recovered (and retried) only where the proof holds: request while fresh.
    {
        Harness harness(config);
        Plant plant(5000);
        CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
        harness.take();
        plant.telemetryPeriodMs = 33;
        std::string why;
        bool requested = false;
        for (int i = 0; i < 200 && !requested; ++i) {
            requested = harness.session.requestShutdown(&why);
            if (!requested) {
                plant.step(harness);
            }
        }
        CHECK(requested);
        std::vector<Bytes> all;
        for (int i = 0; i < 40000 && !isTerminalShutdown(harness.session);
             ++i) {
            for (Bytes& write : plant.step(harness)) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::SafeOff));
        CHECK(countVentFrames(all) >= 1);
        // The wait was used only if the proof was stale at a vent attempt; the
        // shutdown must not abort on that staleness alone.
        CHECK(harness.session.diagnostics().detail.find("vent aborted") ==
              std::string::npos);
    }
    // Telemetry never returns: the wait is bounded and the vent is refused.
    {
        Harness harness(config);
        Plant plant(5000);
        CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
        harness.take();
        std::string why;
        CHECK(harness.session.requestShutdown(&why));
        std::vector<Bytes> all;
        const std::uint64_t requestedAt = harness.session.nowMs();
        for (int i = 0; i < 4000 && !isTerminalShutdown(harness.session);
             ++i) {
            harness.step();  // no telemetry at all
            for (Bytes& write : harness.take()) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
        CHECK_EQ(countVentFrames(all), 0);
        CHECK(harness.session.diagnostics().detail.find(
                  "no fresh dock observation") != std::string::npos);
        CHECK(harness.session.nowMs() - requestedAt < 1500);
    }
    // Explicit AV loss is not stale-only: the vent aborts at its first attempt.
    {
        Harness harness(config);
        Plant plant(5000);
        CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
        harness.take();
        std::string why;
        CHECK(harness.session.requestShutdown(&why));
        injectAvFalse(harness, plant.axes);
        std::vector<Bytes> all;
        const std::uint64_t injectedAt = harness.session.nowMs();
        for (int i = 0; i < 4000 && !isTerminalShutdown(harness.session);
             ++i) {
            harness.step();
            for (Bytes& write : harness.take()) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(countVentFrames(all), 0);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
        CHECK(harness.session.nowMs() - injectedAt < 200);
        CHECK(harness.session.diagnostics().detail.find("vent aborted") !=
              std::string::npos);
    }
    // An axis out of tolerance at the dock is explicit, not stale-only.
    {
        Harness harness(config);
        Plant plant(5000);
        CHECK(bringToDockedPlant(harness, plant, 5000, 3000));
        harness.take();
        std::string why;
        CHECK(harness.session.requestShutdown(&why));
        Axes moved = axesAt(4200);
        harness.feed(makeType48(0, 3, moved, zeroArd()));
        std::vector<Bytes> all;
        for (int i = 0; i < 4000 && !isTerminalShutdown(harness.session);
             ++i) {
            harness.step();
            for (Bytes& write : harness.take()) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(countVentFrames(all), 0);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
    }
}

// B: the lifecycle profile is acknowledged before the first descent frame, then
// the source pause, then the descent. A Stop cancels in each phase.
void testLifecycleProfilePrecedesDescent() {
    // From Parked: profile -> ACK -> 500 ms pause -> first descent frame.
    {
        Harness harness;
        Plant plant(5000);
        CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
        harness.take();
        std::vector<Bytes> all;
        std::vector<std::uint64_t> stamps;  // session time per write
        auto collect = [&](const std::vector<Bytes>& writes) {
            for (const Bytes& write : writes) {
                all.push_back(write);
                stamps.push_back(harness.session.nowMs());
            }
        };
        std::string why;
        CHECK(harness.session.requestShutdown(&why));
        std::uint64_t pauseStartedAt = 0;
        for (int i = 0; i < 40000 && !isTerminalShutdown(harness.session);
             ++i) {
            collect(plant.step(harness));
            if (pauseStartedAt == 0 &&
                harness.session.lifecycleStatus().stage ==
                    "Shutdown: pause after lifecycle profile") {
                pauseStartedAt = harness.session.nowMs();
            }
        }
        CHECK(pauseStartedAt != 0);
        int lifecycleIndex = -1;
        int firstDescent = -1;
        for (std::size_t i = 0; i < all.size(); ++i) {
            if (all[i].size() < 18) {
                continue;
            }
            if (all[i][2] == 5 && payloadPairId(all[i], 0) == 22 &&
                payloadPairValue(all[i], 0) == 20 && lifecycleIndex < 0) {
                lifecycleIndex = static_cast<int>(i);
            }
            if (all[i][2] == 2 && firstDescent < 0) {
                firstDescent = static_cast<int>(i);
            }
        }
        CHECK(lifecycleIndex >= 0);
        CHECK(firstDescent > lifecycleIndex);
        if (firstDescent >= 0) {
            CHECK(stamps[static_cast<std::size_t>(firstDescent)] >= pauseStartedAt + 500);
        }
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::SafeOff));
    }
    // From Running: stop frames first, then the profile, then the descent.
    {
        Harness harness;
        Plant plant(5000);
        CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
        CHECK(startMotionPlant(harness, plant));
        harness.take();
        std::string why;
        CHECK(harness.session.requestShutdown(&why));
        std::vector<Bytes> all;
        for (int i = 0; i < 40000 && !isTerminalShutdown(harness.session);
             ++i) {
            for (Bytes& write : plant.step(harness)) {
                all.push_back(std::move(write));
            }
        }
        int stopIndex = -1;
        int lifecycleIndex = -1;
        int firstDescent = -1;
        for (std::size_t i = 0; i < all.size(); ++i) {
            if (all[i].size() < 18) {
                continue;
            }
            if (all[i][2] == 6 && all[i][3] == 1 && stopIndex < 0) {
                stopIndex = static_cast<int>(i);
            }
            if (all[i][2] == 5 && payloadPairId(all[i], 0) == 22 &&
                payloadPairValue(all[i], 0) == 20 && lifecycleIndex < 0) {
                lifecycleIndex = static_cast<int>(i);
            }
            if (all[i][2] == 2 && lifecycleIndex >= 0 && firstDescent < 0) {
                firstDescent = static_cast<int>(i);
            }
        }
        CHECK(stopIndex >= 0);
        CHECK(lifecycleIndex > stopIndex);
        CHECK(firstDescent > lifecycleIndex);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::SafeOff));
    }
    // Stop during the pause: no descent frame follows, nothing vents.
    {
        Harness harness;
        Plant plant(5000);
        CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
        harness.take();
        std::string why;
        CHECK(harness.session.requestShutdown(&why));
        for (int i = 0; i < 40000 && harness.session.lifecycleStatus().stage !=
                                         "Shutdown: pause after lifecycle profile";
             ++i) {
            plant.step(harness);
        }
        CHECK(harness.session.lifecycleStatus().stage ==
              "Shutdown: pause after lifecycle profile");
        CHECK(harness.session.requestStopMotion(&why));
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
        std::vector<Bytes> all;
        for (int i = 0; i < 2000; ++i) {
            for (Bytes& write : plant.step(harness)) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(countCommand(all, 2), 0);
        CHECK_EQ(countVentFrames(all), 0);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
    }
    // Stop before the profile ACK from Parked: the late ACK does not resume.
    {
        Harness harness;
        Plant plant(5000);
        CHECK(bringToParkedPlant(harness, plant, 5000, 3000));
        harness.take();
        harness.autoAckConfig = false;
        std::string why;
        CHECK(harness.session.requestShutdown(&why));
        for (int i = 0; i < 100 && harness.session.state() != mp6::State::Configuring;
             ++i) {
            plant.step(harness);
        }
        CHECK(harness.session.state() == mp6::State::Configuring);
        CHECK(harness.session.requestStopMotion(&why));
        harness.take();
        harness.feed(makeType43(static_cast<std::uint8_t>(mp6::kConfigParameterCount)));
        std::vector<Bytes> all;
        for (int i = 0; i < 2000; ++i) {
            plant.step(harness);
            for (Bytes& write : harness.take()) {
                all.push_back(std::move(write));
            }
        }
        CHECK_EQ(countCommand(all, 2), 0);
        CHECK_EQ(countVentFrames(all), 0);
        CHECK_EQ(static_cast<int>(harness.session.state()),
                 static_cast<int>(mp6::State::Stopped));
    }
}

// C: a powered platform refuses identify and static edits, so it cannot be
// stranded out of shutdown. Unpowered initial setup is unaffected.
void testPoweredPlatformRefusesIdentifyAndEdits() {
    std::string why;
    Harness harness;
    CHECK(identify(harness));
    // Unpowered: the Normal profile setter works during initial setup.
    CHECK(harness.session.setPidProfile(mp6::PidMode::Normal,
                                        pidProfileFor(mp6::PidMode::Normal), &why));
    CHECK(bringToParked(harness));
    CHECK(harness.session.telemetry().avOk);
    CHECK(!harness.session.requestIdentify(&why));
    CHECK(why.find("powered") != std::string::npos);
    CHECK(!harness.session.setPidProfile(mp6::PidMode::Normal,
                                         pidProfileFor(mp6::PidMode::Normal), &why));
    CHECK(why.find("powered") != std::string::npos);
    CHECK(!harness.session.setControllerConfig(fullControllerConfig(), &why));
    CHECK(why.find("powered") != std::string::npos);
    CHECK(!harness.session.setNeutralSteps(neutralAt(3000), &why));
    CHECK(why.find("powered") != std::string::npos);
    CHECK(!harness.session.invalidateInputs(&why));
    CHECK(why.find("powered") != std::string::npos);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Parked));
    // Shutdown is still available from the same state.
    CHECK(harness.session.requestShutdown(&why));
}

// D: 60 Hz is the only cadence. A requested rate is normalised and reported.
// A sustained under-rate is counted, not reported as zero.
void testCadenceNormalisesAndReportsShortfall() {
    mp6::SessionConfig config;
    config.targetCadenceHz = 90;
    Harness normalised(config);
    CHECK_EQ(normalised.session.config().targetCadenceHz, 60);
    CHECK(normalised.session.cadenceNormalised());
    {
        Harness harness;
        CHECK(!harness.session.cadenceNormalised());
    }

    const mp6::StepTarget target = holdTarget();
    // 50 renders per second for 10 s: 500 frames sent, about 100 slots short.
    {
        Harness harness;
        CHECK(bringToRunning(harness));
        const std::uint64_t start = harness.t;
        const std::uint64_t before = harness.session.txCounters().command2;
        for (std::uint64_t k = 1; k <= 500; ++k) {
            renderTick(harness, target, start + k * 20u);
        }
        const std::uint64_t sent = harness.session.txCounters().command2 - before;
        CHECK(sent >= 495 && sent <= 505);
        CHECK(harness.session.txCounters().missedCadenceSlots >= 95);
        CHECK(harness.session.txCounters().missedCadenceSlots <= 105);
        CHECK_NEAR(harness.session.achievedTargetHz(), 50.0, 1.0);
    }
    // 30 renders per second: about half the slots are short.
    {
        Harness harness;
        CHECK(bringToRunning(harness));
        const std::uint64_t start = harness.t;
        for (std::uint64_t k = 1; k <= 300; ++k) {
            renderTick(harness, target, start + k * 1000u / 30u);
        }
        CHECK(harness.session.txCounters().missedCadenceSlots >= 285);
        CHECK_NEAR(harness.session.achievedTargetHz(), 30.0, 1.0);
    }
}

// E: Normal is applied on an explicit Start while stationary. Running follows
// its ACK and a fresh readiness observation. Stale telemetry at the ACK keeps
// the platform Parked, with the reason stated.
void testStartAppliesNormalAndWaitsForReadiness() {
    Harness harness;
    CHECK(bringToParked(harness));
    CHECK(harness.session.appliedPidMode() == mp6::PidMode::Lifecycle);
    harness.session.setTarget(holdTarget());
    std::string why;
    CHECK(harness.session.requestStartMotion(&why));
    // Applied state and frame count are reset when the transaction starts.
    CHECK(!harness.session.appliedPidMode().has_value());
    CHECK_EQ(harness.session.configStatus().framesSent, 0);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Parked));

    // Telemetry goes quiet across the Normal ACK: no Running.
    for (int i = 0; i < 80; ++i) {
        harness.step(30);
        harness.take();
    }
    CHECK(harness.session.appliedPidMode() == mp6::PidMode::Normal);
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Parked));
    CHECK(harness.session.diagnostics().detail.find("motion not started") !=
          std::string::npos);

    // Fresh telemetry, then a second Start: Running (Normal already applied).
    feedSupply(harness);
    harness.step();
    harness.session.setTarget(holdTarget());
    CHECK(harness.session.requestStartMotion(&why));
    CHECK_EQ(static_cast<int>(harness.session.state()),
             static_cast<int>(mp6::State::Running));
}

// D (E companion): Start on a platform whose Normal profile is missing names it.
void testStartRefusesMissingNormalProfile() {
    Harness harness;
    CHECK(identify(harness));
    std::string why;
    CHECK(harness.session.setControllerConfig(fullControllerConfig(), &why));
    CHECK(harness.session.setCalibration(fullCalibration(16000), &why));
    CHECK(harness.session.setNeutralSteps(neutralAt(16000), &why));
    CHECK(harness.session.setPidProfile(mp6::PidMode::Configure,
                                        pidProfileFor(mp6::PidMode::Configure), &why));
    CHECK(harness.session.setPidProfile(mp6::PidMode::Lifecycle,
                                        pidProfileFor(mp6::PidMode::Lifecycle), &why));
    CHECK(harness.session.requestConfigure(&why));
    driveConfigFrames(harness, mp6::kConfigFrameCount);
    harness.step();
    feedSupply(harness);
    harness.step();
    CHECK(harness.session.requestInitialize(&why));
    for (int i = 0; i < 25000 && harness.session.state() != mp6::State::Parked; ++i) {
        feedSupply(harness);
        harness.step();
    }
    CHECK(harness.session.state() == mp6::State::Parked);
    harness.session.setTarget(holdTarget());
    CHECK(!harness.session.requestStartMotion(&why));
    CHECK(why.find("PID profile (normal)") != std::string::npos);
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::printf("MP6 session tests\n");
    testIdentifyEcho();
    testIdentityMismatchExhaustsProbes();
    testConfigurationTransaction();
    testConfigurationWrongAckAndRetries();
    testSelectorsAndModeContext();
    testFaultLatchBlocksTransmission();
    testConfigurationLossStops();
    testLifecycleAndSupplyGuards();
    testInvalidTargetAndPep();
    testPepOptInEmitsReservedWord();
    testRxStaleStopsMotion();
    testEmergencyStopAndWriteFailure();
    testLinkNotOpen();
    testDockParkEndpoint();
    testShutdownWaitsForDockSettle();
    testConfigurationWriteFailureCleanup();
    testIdentityLossCancelsConfiguration();
    testWriteFailureBestEffortStop();
    testReadFailureStopsPromptly();
    testHalfEvenUnderRoundingModes();
    testControllerConfigValidatesIncoming();
    testDockProofInvalidatedByInputEdits();
    testEmergencyStopDuringConfiguration();
    testIdentityLossStopsAndResets();
    testEmergencyStopClearsRampFollowUps();
    testInputEditGuards();
    testShutdownRechecksDockProof();
    testCalibratedFeedbackMatchesSourceFloatPrecision();
    testVentRechecksDockProofAtTransmission();
    testIdentifyRefusedWhileActive();
    testCadenceAveragesAtRenderRates();
    testCadenceStallResynchronisesWithoutBurst();
    testLifecyclePidProfilesApplyAtBoundaries();
    testLifecycleProfileAckFailureStopsBeforePower();
    testInvalidateInputsRevokesAuthority();
    testStopAndParkDuringPendingShutdown();
    testSessionReviewRoundThreeFixes();
    testShutdownCompletesWithSlowTelemetry();
    testVentWaitIsBoundedForStaleTelemetry();
    testLifecycleProfilePrecedesDescent();
    testPoweredPlatformRefusesIdentifyAndEdits();
    testCadenceNormalisesAndReportsShortfall();
    testStartAppliesNormalAndWaitsForReadiness();
    testStartRefusesMissingNormalProfile();
    testSerialDocumentationGuard();

    std::printf("%d checks, %d failures, %d skips\n", g_checks, g_failures,
                g_skips);
    return g_failures == 0 ? 0 : 1;
}
