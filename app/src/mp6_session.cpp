/*
 * MP6 session implementation.
 *
 * Portable C++17. All hardware access is through mp6::Link.
 *
 * Primary evidence (all line numbers in the recovered decompiled host):
 *   frm_Main.cs:31954-32424  SerialPortRead (frame validation, selector decode)
 *   frm_Main.cs:32203-32217  type 42 echo / type 43 config count
 *   frm_Main.cs:32325-32347  fault word 210/220/230/240/250 latch
 *   frm_Main.cs:32503-32630  TX suppression and gating
 *   frm_Main.cs:32631-32774  frame builder, sequence, forced 127, checksum
 *   frm_Main.cs:33014-33849  Serial_Find_1 (cmd 3 identity, 10 x 200 ms)
 *   frm_Main.cs:31616-31911  Update_OnBoard_Config / SendConfigToBoard
 *   frm_Main.cs:22967-23337  Platform_Initialize_1
 *   frm_Main.cs:24268-24520  Platform_UpDown_CL (timed raise/lower)
 *   frm_Main.cs:25180-25479  Platform_Shutdown (park, vent, power off)
 *   frm_Main.cs:29876-30020  Timer_SwitchBox comms timeout
 *   frm_Main.cs:31230-31373  Update_SwitchBox guards
 *   frm_Main.cs:31532-31540  PowerCheck
 *   frm_Main.cs:34321-34342  StrutAimPressure_Update (aim + 5)
 *   Module2.cs:2240-2280     Motor_Power flag byte and 3 x 50 ms
 *
 * Deliberate departures are marked "DEPARTURE" in comments. They are all
 * conservative: slower config spacing, no automatic power on connect, no TX
 * while a controller fault is latched, and observed settling before success.
 */

#include "mp6_session.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace mp6 {
namespace {

// Payload byte 12 flags come from mp6_protocol.h (kFlagAcPowerRequest,
// kFlagMotorDisable); they are not redeclared here.
constexpr int kWireMin = -32767;
constexpr int kWireMax = 32768;
constexpr int kConfigCount = kConfigParameterCount;

constexpr std::uint8_t kRunModeStop = 1;
constexpr std::uint8_t kRunModeRun = 3;
constexpr std::uint8_t kPneumaticVent = 4;

} // namespace

double roundHalfEven(double value) {
    // Explicit banker's rounding, independent of the process rounding mode.
    // std::nearbyint/rint follow FE_* and would change results if a library or
    // test altered the environment. Math.Round's default is ties-to-even.
    if (!std::isfinite(value)) {
        return value;
    }
    const double base = std::floor(value);
    const double fraction = value - base;
    if (fraction < 0.5) {
        return base;
    }
    if (fraction > 0.5) {
        return base + 1.0;
    }
    // Exact tie. floor() gives the lower integer; choose it when that lower
    // integer is even, otherwise the upper one. fmod is exact for these
    // integer-valued floats and is not affected by the rounding direction.
    const bool baseIsEven = std::fmod(std::fabs(base), 2.0) == 0.0;
    return baseIsEven ? base : base + 1.0;
}

int roundHalfEvenToInt(double value) {
    if (!std::isfinite(value)) {
        return value > 0.0 ? std::numeric_limits<int>::max()
                           : std::numeric_limits<int>::min();
    }
    const double rounded = roundHalfEven(value);
    if (rounded > static_cast<double>(std::numeric_limits<int>::max())) {
        return std::numeric_limits<int>::max();
    }
    if (rounded < static_cast<double>(std::numeric_limits<int>::min())) {
        return std::numeric_limits<int>::min();
    }
    return static_cast<int>(rounded);
}

namespace {

bool inWireRange(long long value) {
    return value >= kWireMin && value <= kWireMax;
}

bool targetWordReserved(std::int32_t value) {
    return value == -32766;  // encodes as raw word 1
}

const char* faultText(std::uint16_t code) {
    switch (code) {
    case 210:
        return "ArdErr 210 Internal Park Mode";
    case 220:
        return "ArdErr 220 AD Pot Error";
    case 230:
        return "ArdErr 230 Low Volts";
    case 240:
        return "EStop (raw code 240)";
    case 250:
        return "ArdErr 250 Comms Drop";
    default:
        return "controller fault word";
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Names
// ---------------------------------------------------------------------------

const char* toString(State state) noexcept {
    switch (state) {
    case State::Closed: return "Closed";
    case State::Opened: return "Opened";
    case State::Identifying: return "Identifying";
    case State::Identified: return "Identified";
    case State::Configuring: return "Configuring";
    case State::Configured: return "Configured";
    case State::Parked: return "Parked";
    case State::Initializing: return "Initializing";
    case State::Running: return "Running";
    case State::Stopped: return "Stopped";
    case State::Parking: return "Parking";
    case State::SafeOff: return "SafeOff";
    case State::Faulted: return "Faulted";
    case State::IdentityLost: return "IdentityLost";
    case State::Docked: return "Docked";
    }
    return "Unknown";
}

const char* toString(Intent intent) noexcept {
    switch (intent) {
    case Intent::Identify: return "Identify";
    case Intent::Configure: return "Configure";
    case Intent::Initialize: return "Initialize";
    case Intent::StartMotion: return "StartMotion";
    case Intent::StopMotion: return "StopMotion";
    case Intent::Park: return "Park";
    case Intent::Shutdown: return "Shutdown";
    case Intent::EmergencyStop: return "EmergencyStop";
    }
    return "Unknown";
}

const char* toString(StopReason reason) noexcept {
    switch (reason) {
    case StopReason::None: return "None";
    case StopReason::UserStop: return "UserStop";
    case StopReason::UserPark: return "UserPark";
    case StopReason::UserShutdown: return "UserShutdown";
    case StopReason::EmergencyStop: return "EmergencyStop";
    case StopReason::InvalidTarget: return "InvalidTarget";
    case StopReason::TargetStale: return "TargetStale";
    case StopReason::RxStale: return "RxStale";
    case StopReason::CommsLost: return "CommsLost";
    case StopReason::WriteFailure: return "WriteFailure";
    case StopReason::LinkClosed: return "LinkClosed";
    case StopReason::SupplyMissing: return "SupplyMissing";
    case StopReason::SupplyLost: return "SupplyLost";
    case StopReason::SupplyStale: return "SupplyStale";
    case StopReason::AirPressureLow: return "AirPressureLow";
    case StopReason::ConfigLost: return "ConfigLost";
    case StopReason::ConfigRejected: return "ConfigRejected";
    case StopReason::PairingMismatch: return "PairingMismatch";
    case StopReason::IdentityMismatch: return "IdentityMismatch";
    case StopReason::IdentityLost: return "IdentityLost";
    case StopReason::ControllerFault: return "ControllerFault";
    case StopReason::FeedbackStale: return "FeedbackStale";
    case StopReason::FeedbackInvalid: return "FeedbackInvalid";
    case StopReason::ParkNotReached: return "ParkNotReached";
    case StopReason::InitializationFailed: return "InitializationFailed";
    case StopReason::LinkNotOpen: return "LinkNotOpen";
    }
    return "Unknown";
}

// ---------------------------------------------------------------------------
// Construction and input validation
// ---------------------------------------------------------------------------

Session::Session() : Session(SessionConfig{}) {}

Session::Session(const SessionConfig& config)
    : config_(config), parser_(1024) {
    // 60 Hz is the only documented cadence. Any other request is normalised
    // here and reported through cadenceNormalised(), never honoured silently.
    cadenceNormalised_ = config_.targetCadenceHz != kSupportedCadenceHz;
    config_.targetCadenceHz = kSupportedCadenceHz;
    config_.identifyProbeTimeoutMs = std::max(1, config_.identifyProbeTimeoutMs);
    config_.identifyMaxProbes = std::max(1, config_.identifyMaxProbes);
    config_.configFrameSpacingMs = std::max(1, config_.configFrameSpacingMs);
    config_.configAckTimeoutMs = std::max(1, config_.configAckTimeoutMs);
    config_.configMaxAttempts = std::max(1, config_.configMaxAttempts);
    config_.rxStaleMs = std::max(1, config_.rxStaleMs);
    config_.commsLostMs = std::max(config_.rxStaleMs + 1, config_.commsLostMs);
    config_.statusFreshMs = std::max(1, config_.statusFreshMs);
    config_.supplyFreshMs = std::max(1, config_.supplyFreshMs);
    config_.pressureFreshMs = std::max(1, config_.pressureFreshMs);
    config_.targetStaleMs = std::max(1, config_.targetStaleMs);
    config_.feedbackFreshMs = std::max(1, config_.feedbackFreshMs);
    config_.parkToleranceSteps = std::max(0, config_.parkToleranceSteps);
    config_.parkSettleMs = std::max(0, config_.parkSettleMs);
    config_.parkTimeoutMs = std::max(0, config_.parkTimeoutMs);
    config_.pressureRefreshMs = std::max(1, config_.pressureRefreshMs);
    config_.shutdownVentTimeoutMs =
        std::max(1, config_.shutdownVentTimeoutMs);
}

Session::~Session() = default;

bool Session::validateControllerConfig(const ControllerConfig& config,
                                       std::string* why) const {
    // Validate the incoming record. Reading the stored member here would leave
    // a newly supplied out-of-range value unchecked until the transaction is
    // built, after it had already replaced the last known-good configuration
    // (review N1).
    auto check = [&](const std::optional<int>& value, const char* name) {
        if (value && !inWireRange(*value)) {
            if (why) {
                *why = std::string("controller parameter ") + name +
                       " is outside the biased wire range";
            }
            return false;
        }
        return true;
    };
    if (!check(config.benchTest, "5 BenchTest") ||
        !check(config.commsTimeoutMs, "6 CommsTimeoutMS") ||
        !check(config.stepRev, "21 StepRev") ||
        !check(config.onboardShutdown, "31 OnBoardShutdown") ||
        !check(config.accelP1Times100, "40 AccelP1") ||
        !check(config.accelP0Times10, "41 AccelP0") ||
        !check(config.decelTimes100, "42 DecelScale") ||
        !check(config.motValFinal, "30 MotValFinal") ||
        !check(config.deadbandEnable, "27 DeadbandEnable") ||
        !check(config.deadbandValue, "28 DeadbandValue") ||
        !check(config.stoppedCountLimit, "29 StoppedCountLimit") ||
        !check(config.avFilterTimes10000, "32 AVFilter") ||
        !check(config.limitsEnable, "33 LimitsEnable") ||
        !check(config.maxPps, "34 MaxPPS") ||
        !check(config.infoMotor, "35 InfoMotor") ||
        !check(config.rotationDirection, "36 RotationDirection")) {
        return false;
    }
    for (int i = 0; i < 6; ++i) {
        const std::size_t index = static_cast<std::size_t>(i);
        if (!check(config.potLo[index], "101..106 PotLo") ||
            !check(config.angLoTimes10[index], "111..116 AngLo") ||
            !check(config.pScaleTimes1000[index], "121..126 pScale") ||
            !check(config.pOffsetTimes10[index], "131..136 pOffset") ||
            !check(config.gearboxTimes100[index], "141..146 Gearbox")) {
            return false;
        }
    }
    return true;
}

bool Session::inputEditable(const char* what, std::string* why) const {
    // Static configuration and calibration are read by the controller while the
    // rails are on. Changing them under a powered platform would strand it: the
    // edit drops the Docked/Parked authority that shutdown needs. The operator
    // shuts down first; lifecycle profile switching is internal and not an edit.
    if (isPowered()) {
        if (why) {
            *why = std::string(what) +
                   " cannot change while motors are powered; run the orderly "
                   "shutdown first";
        }
        return false;
    }
    // Lifecycle stages and motion consume the current input snapshot. Editing
    // them in place would let the dock/neutral goal or pressure aim change
    // between the observation that authorized a step and the step itself.
    // `stage_ != None` also covers the shutdown vent sequence, which runs while
    // the state is Docked.
    if (stage_ != Stage::None) {
        if (why) {
            *why = std::string(what) + " cannot change while '" +
                   lifecycle_.stage + "' is in progress";
        }
        return false;
    }
    switch (state_) {
    case State::Configuring:
    case State::Initializing:
    case State::Running:
    case State::Parking:
        if (why) {
            *why = std::string(what) +
                   " cannot change during configuration or motion";
        }
        return false;
    default:
        break;
    }
    return true;
}

bool Session::setControllerConfig(const ControllerConfig& config,
                                  std::string* why) {
    if (!inputEditable("controller configuration", why)) {
        return false;
    }
    if (!validateControllerConfig(config, why)) {
        return false;
    }
    controllerConfig_ = config;
    configFramesReady_ = false;
    motValFinal_ = config.motValFinal.value_or(-1);
    stepRev_ = config.stepRev.value_or(0);
    // The controller's copy no longer matches the edited values, and any
    // observed neutral/dock pose was established with the previous parameters
    // (StepRev/Gearbox/AngLo feed dockedSteps). This removes the ack and the
    // Docked proof so a later shutdown cannot vent on a historical state (N2).
    dropConfigAuthority(
        "controller configuration changed; controller ack and any dock pose "
        "proof invalidated");
    diagnostics_.missing.clear();
    return true;
}

bool Session::setPidProfile(PidMode mode, const PidProfile& profile,
                            std::string* why) {
    if (!inputEditable("PID profile", why)) {
        return false;
    }
    const char* name = mode == PidMode::Normal      ? "normal"
                       : mode == PidMode::Configure ? "configure"
                                                    : "lifecycle";
    auto check = [&](const std::optional<int>& value, const char* param) {
        if (value && !inWireRange(*value)) {
            if (why) {
                *why = std::string("PID profile (") + name + ") " + param +
                       " is outside the biased wire range";
            }
            return false;
        }
        return true;
    };
    if (!check(profile.proportionalTimes10, "22 P*10") ||
        !check(profile.derivativeTimes10, "23 D*10") ||
        !check(profile.dFilter, "25 DFilter") ||
        !check(profile.tConst, "26 TConst") ||
        !check(profile.stepCorrTimes10000, "39 StepCorrection*10000") ||
        !check(profile.sensorFilterTimes10000, "46 SensorFilter*10000") ||
        !check(profile.historyLen, "47 HistoryLen")) {
        return false;
    }
    pidProfiles_[static_cast<std::size_t>(mode)] = profile;
    dropConfigAuthority(
        "PID profile changed; controller ack and any dock pose proof invalidated");
    return true;
}

bool Session::invalidateInputs(std::string* why) {
    if (!inputEditable("accepted inputs", why)) {
        return false;
    }
    neutralValid_ = false;
    target_.valid = false;
    dropConfigAuthority(
        "accepted inputs invalidated; configuration, neutral and observed pose "
        "must be re-established");
    return true;
}

void Session::dropConfigAuthority(const char* detail) {
    configAckOk_ = false;
    configStatus_.configured = false;
    configStatus_.configuring = false;
    configTransactionActive_ = false;
    configFrameIndex_ = 0;
    configAckDeadlineMs_ = 0;
    appliedPidMode_.reset();
    if (state_ == State::Configured || state_ == State::Parked ||
        state_ == State::Stopped || state_ == State::Docked) {
        ramp_.settled = false;
        ramp_.endpointReached = false;
        state_ = State::Identified;
        diagnostics_.reason = StopReason::None;
        diagnostics_.detail = detail;
    }
}

bool Session::setCalibration(const Calibration& calibration,
                             std::string* why) {
    if (!inputEditable("calibration", why)) {
        return false;
    }
    if (state_ == State::Parked) {
        if (why) *why = "calibration cannot change while parked or moving";
        return false;
    }
    if (!calibration.valid) {
        if (why) *why = "calibration is not marked valid";
        return false;
    }
    for (int i = 0; i < 6; ++i) {
        const AxisCalibration& axis = calibration.axis[static_cast<std::size_t>(i)];
        if (!axis.valid || !std::isfinite(axis.pScale) ||
            !std::isfinite(axis.pOffset) || axis.pScale == 0.0) {
            if (why) {
                *why = "calibration axis " + std::to_string(i + 1) +
                       " is invalid (pScale must be finite and non-zero)";
            }
            return false;
        }
    }
    calibration_ = calibration;
    if (state_ == State::Docked) {
        // dockedSteps and the calibrated feedback that proved the dock both
        // come from this record. Keep the platform identified (so configure /
        // initialize can run again) but drop the Docked proof: shutdown must
        // perform and observe a fresh descent instead of venting at a pose
        // established with the old numbers (review N2).
        ramp_.settled = false;
        ramp_.endpointReached = false;
        state_ = State::Stopped;
        diagnostics_.reason = StopReason::None;
        diagnostics_.detail =
            "calibration replaced; dock pose proof invalidated (re-dock before "
            "shutdown vent)";
    }
    return true;
}

bool Session::setPressurePlan(const PressurePlan& plan, std::string* why) {
    if (!inputEditable("pressure plan", why)) {
        return false;
    }
    if (plan.enabled && plan.aimSupplied &&
        (plan.aimPsi < 0 || plan.aimPsi > 250)) {
        // TxPneu[0] = aim + 5 is a byte.
        if (why) *why = "pressure aim must be 0..250 psi";
        return false;
    }
    pressurePlan_ = plan;
    return true;
}

bool Session::setNeutralSteps(const std::array<std::int32_t, 6>& neutral,
                              std::string* why) {
    if (!inputEditable("neutral step targets", why)) {
        return false;
    }
    for (int i = 0; i < 6; ++i) {
        const std::int32_t value = neutral[static_cast<std::size_t>(i)];
        if (!inWireRange(value) || targetWordReserved(value)) {
            if (why) {
                *why = "neutral step target " + std::to_string(i + 1) +
                       " is not encodable";
            }
            return false;
        }
    }
    neutralSteps_ = neutral;
    neutralValid_ = true;
    return true;
}

void Session::setExpectedPlatformNumber(int platformNumber) {
    expectedPlatformNumber_ = platformNumber;
    if (telemetry_.platformNumber > 0) {
        pairingMatched_ =
            expectedPlatformNumber_ == 0 ||
            telemetry_.platformNumber == expectedPlatformNumber_;
        if (!pairingMatched_) {
            diagnostics_.pairingMismatch = true;
        }
    }
}

void Session::setTarget(const StepTarget& target) {
    target_ = target;
    targetReceivedMs_ = now_;
}

// ---------------------------------------------------------------------------
// Link and session reset
// ---------------------------------------------------------------------------

void Session::attach(Link* link) {
    link_ = link;
    resetSessionForNewLink();
    if (link_ != nullptr && link_->isOpen()) {
        state_ = State::Opened;
    }
}

void Session::detach() {
    link_ = nullptr;
    resetSessionForNewLink();
}

void Session::resetSessionForNewLink() {
    parser_.reset();
    state_ = State::Closed;
    diagnostics_ = Diagnostics{};
    telemetry_ = Telemetry{};
    configStatus_ = ConfigStatus{};
    power_ = PowerStatus{};
    ramp_ = RampStatus{};
    lifecycle_ = LifecycleStatus{};
    tx_ = TxCounters{};
    sequence_ = SequenceCounter{};
    identityAccepted_ = false;
    faultLatched_ = false;
    identifyProbes_ = 0;
    nextProbeMs_ = 0;
    probeDeadlineMs_ = 0;
    probeInFlight_ = false;
    identityEchoMismatches_ = 0;
    pairingObserved_ = false;
    pairingMatched_ = false;
    configAckOk_ = false;
    configFrameIndex_ = 0;
    configAttempts_ = 0;
    nextConfigFrameMs_ = 0;
    configAckDeadlineMs_ = 0;
    configTransactionActive_ = false;
    configTransactionStartMs_ = 0;
    targetReceivedMs_ = 0;
    nextTargetTxMs_ = 0;
    clearMotionGuards();
    writeFailurePending_ = false;
    pepCycles_.fill(0);
    pressureBurstRemaining_ = 0;
    nextPressureBurstMs_ = 0;
    pressureBurstByte_ = 0;
    stopBurstRemaining_ = 0;
    nextStopBurstMs_ = 0;
    stopBurstOrdinary_ = true;
    powerBurstRemaining_ = 0;
    nextPowerBurstMs_ = 0;
    powerObserveAttempts_ = 0;
    stage_ = Stage::None;
    stageDeadlineMs_ = 0;
    pressureAim_ = 0;
    parkAfterRamp_ = false;
    shutdownAfterPark_ = false;
    rampPhase_ = 0;
    settleSinceMs_ = 0;
    rampAccum_.fill(0.0);
    lastRampTickMs_ = 0;
    linkTerminalFailure_ = false;
    commandedRunMode_ = kRunModeStop;
    lastIdlePollMs_ = 0;
    lastPressureSentMs_ = 0;
    lastType48ProcessedMs_ = 0;
    lastTxMs_ = 0;
    lastRxMs_ = 0;
    // lastTickMs is intentionally retained: time is monotonic across links.
    motValFinal_ = controllerConfig_.motValFinal.value_or(-1);
    stepRev_ = controllerConfig_.stepRev.value_or(0);
    configFramesReady_ = false;
    appliedPidMode_.reset();
    pidApplyPending_ = false;
    pidResume_ = PidResume::None;
    pressureBurstVent_ = false;
    targetDueMs_ = 0.0;
}

// ---------------------------------------------------------------------------
// Intents
// ---------------------------------------------------------------------------

bool Session::request(Intent intent, std::string* why) {
    switch (intent) {
    case Intent::Identify: return requestIdentify(why);
    case Intent::Configure: return requestConfigure(why);
    case Intent::Initialize: return requestInitialize(why);
    case Intent::StartMotion: return requestStartMotion(why);
    case Intent::StopMotion: return requestStopMotion(why);
    case Intent::Park: return requestPark(why);
    case Intent::Shutdown: return requestShutdown(why);
    case Intent::EmergencyStop: return requestEmergencyStop(why);
    }
    if (why) *why = "unknown intent";
    return false;
}

bool Session::requestIdentify(std::string* why) {
    if (link_ == nullptr || !link_->isOpen()) {
        if (why) *why = "link is not open";
        diagnostics_.reason = StopReason::LinkNotOpen;
        return false;
    }
    if (faultLatched_) {
        // frm_Main.cs:32503 blocks every transmission while the controller
        // latch is set; the recovered host only clears its own copy after a
        // fresh discovery, which in practice means a new process/session.
        // Never pretend to clear the controller latch from here.
        if (why) {
            *why = "controller fault is latched; close and reopen the link to "
                   "start a new identify session";
        }
        return false;
    }
    if (state_ == State::SafeOff) {
        // Emergency stop is a terminal host state by design: motors are
        // disabled and the recovered host only recovers through a new
        // discovery/session. Do not pretend that identify alone re-arms.
        if (why) {
            *why = "SafeOff is terminal in this session; reconnect the link "
                   "and identify again";
        }
        return false;
    }
    if (link_->terminalFailure()) {
        if (why) {
            *why = "serial link reader failed; reconnect before identifying";
        }
        return false;
    }
    // Identify clears authentication and therefore the host's ability to send
    // stop/power frames. Refuse while the controller may still be executing a
    // command or a lifecycle sequence is in progress (F5).
    const bool activeState =
        state_ == State::Configuring || state_ == State::Initializing ||
        state_ == State::Running || state_ == State::Parking ||
        state_ == State::Docked;
    const bool activeBurst = stopBurstRemaining_ > 0 ||
                             pressureBurstRemaining_ > 0 ||
                             powerBurstRemaining_ > 0;
    if (isPowered()) {
        if (why) {
            *why = "identify refused while motors are powered; run the orderly "
                   "shutdown first";
        }
        return false;
    }
    if (activeState || activeBurst || stage_ != Stage::None) {
        if (why) {
            *why = "identify refused while motion, a lifecycle sequence or a "
                   "stop/power burst is active; let it finish first";
        }
        return false;
    }
    resetFollowUps();
    appliedPidMode_.reset();
    parser_.reset();
    identityAccepted_ = false;
    pairingObserved_ = false;
    pairingMatched_ = false;
    configAckOk_ = false;
    configStatus_.configured = false;
    configStatus_.configuring = false;
    configTransactionActive_ = false;
    identifyProbes_ = 0;
    identityEchoMismatches_ = 0;
    probeInFlight_ = false;
    nextProbeMs_ = now_;
    probeDeadlineMs_ = 0;
    diagnostics_.identityMismatch = false;
    diagnostics_.reason = StopReason::None;
    diagnostics_.detail.clear();
    state_ = State::Identifying;
    return true;
}

bool Session::requestConfigure(std::string* why) {
    if (state_ == State::SafeOff) {
        if (why) {
            *why = "SafeOff is terminal; reconnect the link and identify again "
                   "before configuration";
        }
        return false;
    }
    if (!identityAccepted_ || state_ == State::Faulted ||
        state_ == State::IdentityLost) {
        if (why) *why = "a fresh identity is required before configuration";
        return false;
    }
    if (isPowered() && configAckOk_) {
        if (why) *why = "configuration is already acknowledged while motors are powered";
        return false;
    }
    switch (state_) {
    case State::Configuring:
    case State::Initializing:
    case State::Running:
    case State::Parking:
        if (why) *why = "configuration is not allowed in the current state";
        return false;
    default:
        break;
    }
    if (stage_ != Stage::None) {
        // A pending lifecycle PID profile (for example before a vent) owns the
        // command-5 transaction; a second one must not interleave with it.
        if (why) *why = "configuration cannot start while '" + lifecycle_.stage +
                        "' is in progress";
        return false;
    }
    if (!pairingObserved_ || telemetry_.platformNumber <= 0) {
        if (why) *why = "platform identity (selector 1) has not been observed";
        return false;
    }
    if (!pairingMatched_) {
        if (why) *why = "platform identity does not match the paired number";
        return false;
    }
    if (!buildConfigTransaction(PidMode::Configure, why)) {
        return false;
    }
    configStatus_.wrongAckCount = 0;
    configStatus_.lastAckCount = -1;
    configStatus_.lastAckMs = 0;
    configAttempts_ = 0;
    configAckOk_ = false;
    configStatus_.configured = false;
    state_ = State::Configuring;
    diagnostics_.configRejected = false;
    diagnostics_.configLost = false;
    diagnostics_.reason = StopReason::None;
    startConfiguration(now_);
    return true;
}

bool Session::requestInitialize(std::string* why) {
    if (state_ == State::SafeOff) {
        if (why) {
            *why = "SafeOff is terminal; reconnect the link and identify again "
                   "before initialization";
        }
        return false;
    }
    if (state_ != State::Configured && state_ != State::Parked &&
        state_ != State::Stopped) {
        if (why) *why = "initialization requires a configured controller";
        return false;
    }
    if (stage_ != Stage::None || shutdownAfterPark_ ||
        pidResume_ != PidResume::None) {
        if (why) *why = "initialization refused while a shutdown or profile transaction is pending; stop motion first";
        return false;
    }
    if (!identityAccepted_ || !configAckOk_ || !configStatus_.configured) {
        if (why) *why = "configuration ack (57) is required";
        return false;
    }
    if (!pairingObserved_ || !pairingMatched_) {
        if (why) *why = "platform identity (selector 1) must match";
        return false;
    }
    if (!calibration_.valid) {
        diagnostics_.calibrationRequired = true;
        if (why) *why = "validated calibration is required before initialization";
        return false;
    }
    if (!neutralValid_) {
        diagnostics_.neutralTargetsRequired = true;
        if (why) *why = "neutral step targets are required before initialization";
        return false;
    }
    if (!feedbackFresh()) {
        if (why) *why = "fresh calibrated feedback is required before ramping";
        diagnostics_.reason = StopReason::FeedbackStale;
        return false;
    }
    if (!controllerConfig_.stepRev || !controllerConfig_.motValFinal) {
        if (why) *why = "controller parameters 21 (StepRev) and 30 (MotValFinal) "
                        "are required";
        return false;
    }
    // Initialization applies the Lifecycle profile before power. Normal is
    // applied on an explicit Start while stationary (source: PID_SetValues(0)
    // on the Start path), so it is validated there, not here.
    if (!buildConfigTransaction(PidMode::Lifecycle, why)) {
        return false;
    }
    int aim = 0;
    if (!pressureAimForInitialization(&aim, why)) {
        return false;
    }
    diagnostics_.calibrationRequired = false;
    diagnostics_.neutralTargetsRequired = false;
    diagnostics_.pressureAimRequired = false;
    diagnostics_.reason = StopReason::None;
    diagnostics_.detail.clear();
    pressureAim_ = aim;
    lifecycle_.pressureAimPsi = aim;
    parkAfterRamp_ = false;
    shutdownAfterPark_ = false;
    ramp_.active = false;
    clearMotionGuards();
    state_ = State::Initializing;
    setStage(Stage::InitSetRunMode, "Set run mode 3", 0);
    return true;
}

bool Session::requestStartMotion(std::string* why) {
    if (state_ == State::SafeOff) {
        if (why) {
            *why = "SafeOff is terminal; reconnect the link and identify again "
                   "before motion";
        }
        return false;
    }
    if (state_ != State::Parked) {
        if (why) *why = "motion requires a parked, observed initialization";
        return false;
    }
    if (stage_ != Stage::None || shutdownAfterPark_ ||
        pidResume_ != PidResume::None) {
        if (why) *why = "start refused while a shutdown or profile transaction is pending; stop motion first";
        return false;
    }
    if (!readiness(why)) {
        return false;
    }
    if (appliedPidMode_ && *appliedPidMode_ == PidMode::Normal) {
        return enterRunning(why);
    }
    // Source: the Start path applies the normal PID values (PID_SetValues(0),
    // frm_Main.cs:28031-28060, 34642). Applied while stationary; Running follows
    // the ACK and a fresh readiness check (resumeAfterPidApply, StartMotion).
    std::string profileWhy;
    if (!beginPidApply(PidMode::Normal, PidResume::StartMotion, &profileWhy)) {
        if (why) *why = profileWhy;
        return false;
    }
    diagnostics_.detail = "normal PID profile transaction before motion";
    return true;
}

bool Session::enterRunning(std::string* why) {
    if (!txRunMode(kRunModeRun)) {
        if (why) *why = "run-mode command failed";
        return false;
    }
    clearMotionGuards();
    targetReceivedMs_ = now_;
    nextTargetTxMs_ = now_;
    startCadenceChain();
    lastPressureSentMs_ = now_;
    diagnostics_.reason = StopReason::None;
    diagnostics_.detail.clear();
    state_ = State::Running;
    shutdownAfterPark_ = false;
    return true;
}

bool Session::requestStopMotion(std::string* why) {
    (void)why;
    // Global Stop also abandons the first Configure transaction, whose stage
    // is None. It has not powered the platform, so retain authenticated identity.
    if (state_ == State::Configuring && configTransactionActive_ &&
        stage_ == Stage::None && pidResume_ == PidResume::None) {
        configTransactionActive_ = false;
        configStatus_.configuring = false;
        configFrameIndex_ = 0;
        configAckDeadlineMs_ = 0;
        configStatus_.framesSent = 0;
        resetFollowUps();
        state_ = State::Identified;
        diagnostics_.reason = StopReason::UserStop;
        diagnostics_.detail = "configuration cancelled by user stop";
        return true;
    }
    // Any pending lifecycle sequence is cancelled here, not only motion: a
    // shutdown queued from Docked (PID profile, vent or power-off follow-ups)
    // sits in Docked with a stage set. enterStopped drops every continuation
    // (stage, descent, profile resume, bursts) in one place.
    if (state_ == State::Running || state_ == State::Initializing ||
        state_ == State::Parking || stage_ != Stage::None) {
        // enterStopped owns the single run-mode-1 assertion and the two-frame
        // zero-rate stop burst for a moving or sequencing state.
        enterStopped(StopReason::UserStop, "stop requested by user");
        return true;
    }
    if (state_ == State::Stopped || state_ == State::Parked ||
        state_ == State::Docked || state_ == State::Configured ||
        state_ == State::Identified || state_ == State::SafeOff) {
        return true;
    }
    if (why) *why = "nothing is moving";
    return false;
}

bool Session::requestPark(std::string* why) {
    // A pending or running shutdown owns the dock and the struts. Park must not
    // report success (Docked) or start a second descent while it is in progress;
    // the caller stops motion first to cancel the shutdown.
    if (stage_ != Stage::None || shutdownAfterPark_) {
        if (why) {
            *why = "park refused: a shutdown is in progress; stop motion first "
                   "to cancel it";
        }
        return false;
    }
    // `Parked` is the neutral/ready pose observed at the end of
    // initialization. Park deliberately descends to the calibration dock and
    // only reports `Docked` after an observed settle; the two poses must not be
    // conflated when dockedSteps != neutralSteps.
    if (state_ == State::Docked) {
        std::string at;
        if (atCalibrationDock(&at)) {
            return true;
        }
        // A historical Docked state is not proof of the current pose: if the
        // platform was moved (or feedback is stale) re-descend below instead of
        // reporting success from remembered state.
    }
    if (state_ == State::Parking && parkAfterRamp_) {
        return true;
    }
    if (state_ != State::Running && state_ != State::Stopped &&
        state_ != State::Parked && state_ != State::Docked) {
        if (why) *why = "park requires running, stopped or neutral-ready motion";
        return false;
    }
    if (!calibration_.valid) {
        if (why) *why = "validated calibration is required to park";
        return false;
    }
    if (!feedbackFresh()) {
        if (why) *why = !configAckOk_
                            ? "configuration is not acknowledged; reconfigure or initialize before parking, or use E-STOP"
                            : "fresh calibrated feedback is required to park";
        return false;
    }
    const std::array<int, 6> dock{
        calibration_.axis[0].dockedSteps, calibration_.axis[1].dockedSteps,
        calibration_.axis[2].dockedSteps, calibration_.axis[3].dockedSteps,
        calibration_.axis[4].dockedSteps, calibration_.axis[5].dockedSteps};
    const State previous = state_;
    // The source parks/rams while run mode is still 3 and asserts run mode 1
    // after the platform reaches the base point (frm_Main.cs:25450). Sending 1
    // before the ramp would stop command-2 target execution on the board.
    if (!txRunMode(kRunModeRun)) {
        if (why) *why = "run-mode 3 command failed; park not started";
        return false;
    }
    state_ = State::Parking;
    parkAfterRamp_ = true;
    shutdownAfterPark_ = false;
    if (!startRamp(0, dock, "Parking at calibration dock")) {
        txRunMode(kRunModeStop);
        state_ = previous;
        parkAfterRamp_ = false;
        if (why) *why = "park ramp could not start: " + diagnostics_.detail;
        return false;
    }
    diagnostics_.reason = StopReason::UserPark;
    diagnostics_.detail = "park ramp to calibration dock in progress";
    return true;
}

bool Session::requestShutdown(std::string* why) {
    if (stage_ == Stage::ShutdownVent || stage_ == Stage::ShutdownVentWait ||
        stage_ == Stage::ShutdownPowerOff || stage_ == Stage::ShutdownPause ||
        pidResume_ == PidResume::ShutdownVent ||
        pidResume_ == PidResume::ShutdownPause || shutdownAfterPark_) {
        // Already in progress (queued profile, pause, descent or vent): do not
        // restart it, which would overwrite the state it resumes into.
        return true;
    }
    if (state_ == State::SafeOff) {
        if (why) *why = "motors are already disabled (SafeOff)";
        return false;
    }
    if (state_ != State::Docked && state_ != State::Parked &&
        state_ != State::Stopped && state_ != State::Running) {
        if (why) *why = "orderly shutdown requires a known platform pose";
        return false;
    }
    if (state_ == State::Docked) {
        // The only state from which venting is permitted. It is reached only
        // after the dock ramp's endpoint and settle were observed, but that
        // observation can be historical: the platform may have been moved,
        // the controller restarted, or telemetry may have gone stale. Re-verify
        // the current pose against the current calibration before venting.
        std::string at;
        if (!atCalibrationDock(&at)) {
            const bool stale = !feedbackFresh() || !statusFresh();
            diagnostics_.reason =
                stale ? StopReason::FeedbackStale : StopReason::ParkNotReached;
            diagnostics_.detail = "shutdown refused: " + at;
            if (why) *why = diagnostics_.detail;
            return false;
        }
        std::string profileWhy;
        if (!beginPidApply(PidMode::Lifecycle, PidResume::ShutdownVent,
                           &profileWhy)) {
            if (why) *why = profileWhy;
            return false;
        }
        ramp_.active = false;
        shutdownAfterPark_ = true;
        diagnostics_.reason = StopReason::UserShutdown;
        diagnostics_.detail = "at calibration dock; applying lifecycle PID profile "
                              "before venting struts";
        return true;
    }
    // Parked, Stopped and Running are not docked. Source (frm_Main.cs:25051-25059):
    // Stop_All_Motors -> PID_SetValues(2) -> StopPause(0.5) -> rest ramp. The
    // lifecycle profile transaction therefore precedes the first descent frame.
    if (!calibration_.valid) {
        if (why) *why = "validated calibration is required to reach the dock";
        return false;
    }
    if (!feedbackFresh()) {
        if (why) {
            *why = !configAckOk_
                       ? "configuration is not acknowledged; reconfigure, initialize, then shut down, or use E-STOP"
                       : "fresh calibrated feedback is required to descend to the "
                             "dock before shutdown";
        }
        return false;
    }
    if (!pidProfiles_[static_cast<std::size_t>(PidMode::Lifecycle)]) {
        if (why) *why = "PID profile (lifecycle) is not supplied; shutdown is blocked";
        return false;
    }
    if (state_ == State::Running) {
        // Stop frames first (bounded, authenticated). enterStopped cancels any
        // target stream; the profile then waits for the stop burst to drain.
        enterStopped(StopReason::UserShutdown, "stopping motion before shutdown");
    }
    std::string profileWhy;
    if (!beginPidApply(PidMode::Lifecycle, PidResume::ShutdownPause, &profileWhy)) {
        if (why) *why = profileWhy;
        return false;
    }
    shutdownAfterPark_ = true;
    diagnostics_.reason = StopReason::UserShutdown;
    diagnostics_.detail = "stopping, then lifecycle PID profile, then descent to the "
                          "calibration dock";
    return true;
}

bool Session::requestEmergencyStop(std::string* why) {
    if (state_ == State::Closed || state_ == State::Faulted) {
        if (why) {
            *why = state_ == State::Faulted
                       ? "controller fault is latched; no frame may be sent"
                       : "emergency stop has no writable link";
        }
        return false;
    }
    if (state_ == State::SafeOff) {
        return true;
    }
    // Cancel any in-flight configuration transaction BEFORE the stop frames.
    // canTransmit() permits only command 5 while Configuring, so an emergency
    // stop issued mid-transaction would otherwise be silently gated and this
    // method would report SafeOff after zero writes (review N3). Identity is
    // still accepted in Configuring, so dropping to Identified keeps the
    // normal post-identity TX policy instead of adding a bypass.
    const bool abandonedConfiguration =
        state_ == State::Configuring || configTransactionActive_;
    if (abandonedConfiguration) {
        configTransactionActive_ = false;
        configStatus_.configuring = false;
        configFrameIndex_ = 0;
        configAckDeadlineMs_ = 0;
        configStatus_.framesSent = 0;
        if (state_ == State::Configuring) {
            state_ = State::Identified;
        }
    }
    // Emergency frames are sent immediately and back-to-back rather than
    // waiting for the 50 ms source spacing; stopping takes priority over
    // cadence. The frame set is the recovered stop/off set. Ramp follow-up
    // flags are cleared here as they are in the other stop paths (review N5).
    ramp_.active = false;
    ramp_.settled = false;
    ramp_.endpointReached = false;
    rampAccum_.fill(0.0);
    lastRampTickMs_ = now_;
    settleSinceMs_ = 0;
    resetFollowUps();
    pressureBurstRemaining_ = 0;
    stopBurstRemaining_ = 0;
    powerBurstRemaining_ = 0;
    clearMotionGuards();

    int written = 0;
    if (txRunMode(kRunModeStop)) ++written;
    if (txStopAll()) ++written;
    if (txStopAll()) ++written;
    powerBurstRemaining_ = 0;
    if (txPower(false, true)) ++written;
    if (txPower(false, true)) ++written;
    if (txPower(false, true)) ++written;

    state_ = State::SafeOff;
    lifecycle_.stage = "SafeOff";
    diagnostics_.reason = StopReason::EmergencyStop;
    diagnostics_.detail =
        "emergency stop: run mode 1, zero rate words, motor disable requested";
    if (abandonedConfiguration) {
        diagnostics_.detail += "; in-flight configuration transaction abandoned";
    }
    diagnostics_.detail += "; " + std::to_string(written) +
                          " of 6 frames written";
    if (written == 0) {
        // Honest failure detail: the session is terminal, but no frame reached
        // the controller (pre-identity policy permits only command 3, a fault
        // latch permits nothing, or the link rejected every write).
        diagnostics_.detail +=
            "; no frame could be transmitted, so motors may still be enabled. "
            "Stop the controller at its own controls or reconnect before "
            "retrying";
    }
    return true;
}

bool Session::armPepSpecialTarget(int axis, int cycles, std::string* why) {
    if (!config_.allowPepSpecialTargets) {
        if (why) {
            *why = "PEP word-1 target semantics are unresolved; enable "
                   "SessionConfig::allowPepSpecialTargets only for a deliberate "
                   "source-behaviour port";
        }
        return false;
    }
    if (axis < 0 || axis >= 6 || cycles <= 0) {
        if (why) *why = "axis must be 0..5 and cycles positive";
        return false;
    }
    pepCycles_[static_cast<std::size_t>(axis)] = cycles;
    return true;
}

// ---------------------------------------------------------------------------
// Tick
// ---------------------------------------------------------------------------

void Session::tick(std::uint64_t nowMs) {
    if (nowMs > now_) {
        now_ = nowMs;  // monotonic; a backwards clock is ignored
    }
    if (link_ == nullptr) {
        state_ = State::Closed;
        return;
    }
    if (!link_->isOpen()) {
        if (state_ != State::Closed) {
            state_ = State::Closed;
            diagnostics_.reason = StopReason::LinkClosed;
            diagnostics_.detail = "link closed by owner";
            ramp_.active = false;
            stage_ = Stage::None;
        }
        return;
    }
    if (link_->terminalFailure()) {
        if (!linkTerminalFailure_) {
            linkTerminalFailure_ = true;
            const std::string detail =
                "serial reader terminated" +
                (link_->lastError().empty() ? std::string()
                                            : ": " + link_->lastError());
            if (state_ != State::SafeOff && state_ != State::Faulted &&
                state_ != State::IdentityLost) {
                enterStopped(StopReason::LinkClosed, detail);
            } else {
                diagnostics_.reason = StopReason::LinkClosed;
                diagnostics_.detail = detail;
            }
        }
        // No further RX or TX. The owner closes/joins the link; doing that
        // from the reader's own thread would deadlock.
        return;
    }
    if (state_ == State::Closed) {
        state_ = State::Opened;
    }

    processAvailableRx();

    if (writeFailurePending_) {
        writeFailurePending_ = false;
        if (!faultLatched_ && state_ != State::SafeOff &&
            state_ != State::IdentityLost) {
            enterStopped(StopReason::WriteFailure,
                         "link write failed; motion and bursts stopped");
        }
        return;
    }

    if (faultLatched_) {
        // frm_Main.cs:32503: no frame is transmitted while the raw controller
        // fault latch is set, including command 6 and command 3.
        return;
    }

    updateGuards();
    if (faultLatched_) {
        return;
    }

    driveIdentify();
    driveConfiguration();
    driveIdlePoll();
    drivePressureBurst();
    driveStopBurst();
    drivePowerBurst();
    driveLifecycle();
    driveRamp();
    driveMotion();
}

// ---------------------------------------------------------------------------
// RX
// ---------------------------------------------------------------------------

void Session::processAvailableRx() {
    std::uint8_t buffer[512];
    int reads = 0;
    while (reads < 8) {
        const std::size_t got = link_->read(buffer, sizeof(buffer));
        if (got == 0) {
            break;
        }
        ++reads;
        parser_.feed(buffer, got);
    }
    RxFrame frame;
    int frames = 0;
    while (frames < 128 && parser_.poll(frame)) {
        ++frames;
        lastRxMs_ = now_;
        handleFrame(frame);
    }
}

void Session::handleFrame(const RxFrame& frame) {
    switch (frame.type) {
    case 42:
        handleType42(frame);
        break;
    case 43:
        handleType43(frame);
        break;
    case 48:
        handleType48(frame);
        break;
    default:
        // Types 32/36/38/49 are not needed by this milestone and are ignored
        // after checksum/length validation by the codec.
        break;
    }
}

void Session::handleType42(const RxFrame& frame) {
    const auto data = frame.asType42();
    if (!data) {
        return;
    }
    if (data->byte7 == 3) {
        if (data->byte9 == 253) {
            if (!identityAccepted_ && state_ == State::Identifying) {
                identityAccepted_ = true;
                // frm_Main.cs:33817: a successful discovery clears the host
                // copy of the latch. The controller's own latch is not touched.
                faultLatched_ = false;
                probeInFlight_ = false;
                pairingObserved_ = false;
                pairingMatched_ = false;
                state_ = State::Identified;
                diagnostics_.identityMismatch = false;
                diagnostics_.reason = StopReason::None;
                diagnostics_.detail =
                    "identity echo accepted (type 42 byte7=3 byte9=253)";
            }
        } else {
            ++identityEchoMismatches_;
            diagnostics_.identityMismatch = true;
            diagnostics_.detail =
                "type 42 echo byte9 was " + std::to_string(data->byte9) +
                ", expected 253";
        }
    }
}

void Session::handleType43(const RxFrame& frame) {
    const auto data = frame.asType43();
    if (!data) {
        return;
    }
    const int count = data->configCount;
    configStatus_.lastAckCount = count;
    configStatus_.lastAckMs = now_;
    if (count > 0 && count == kConfigCount) {
        // Only ack the transaction that is actually in flight and only after
        // all 22 frames have been written. A stale/unsolicited 57 ack must not
        // move an unrelated state (for example Running) back to Configured.
        if (state_ == State::Configuring && configTransactionActive_ &&
            configFrameIndex_ == kConfigFrameCount) {
            configAckOk_ = true;
            finishConfiguration(true, "type 43 ack count 57");
        } else {
            configStatus_.lastAckMs = now_;
        }
    } else if (count > 0) {
        ++configStatus_.wrongAckCount;
        diagnostics_.detail =
            "type 43 ack count " + std::to_string(count) + " (expected 57)";
        if (state_ != State::Configuring) {
            // A late/unsolicited wrong ack does not invalidate a good ack.
            diagnostics_.configRejected = true;
        }
    } else {
        ++configStatus_.wrongAckCount;
        diagnostics_.detail = "type 43 ack count 0";
    }
}

void Session::handleType48(const RxFrame& frame) {
    const auto data = frame.asType48();
    if (!data) {
        return;
    }
    telemetry_.lastFrameMs = now_;
    ++telemetry_.type48Frames;
    lastType48ProcessedMs_ = now_;

    // Status nibble is present in every frame and is not selector-dependent.
    telemetry_.configReceived = data->configReceived;
    telemetry_.avOk = data->avOk;
    telemetry_.statusMs = now_;

    if (!data->configReceived) {
        if (configAckOk_ && !configTransactionActive_ &&
            state_ != State::Configuring) {
            if (!configLostActive_) {
                configLostActive_ = true;
                configLostSinceMs_ = now_;
            }
        }
    } else {
        configLostActive_ = false;
    }

    const bool calibrationUsable = calibration_.valid && configAckOk_;
    for (int i = 0; i < 6; ++i) {
        const FeedbackAxis& source = data->axes[static_cast<std::size_t>(i)];
        AxisFeedback& axis = telemetry_.axes[static_cast<std::size_t>(i)];
        axis.seen = true;
        axis.word = source.word;
        axis.raw = source.rawCounts;
        axis.wordMs = now_;
        if (calibrationUsable) {
            const AxisCalibration& cal =
                calibration_.axis[static_cast<std::size_t>(i)];
            // Source parity (frm_Main.cs:32233):
            //   (int)Math.Round((float)num13 * pScale / 8f + pOffset)
            // pScale/pOffset are float[] in Module2.cs:522, so the recovered
            // host rounds the represented *single-precision* value. Evaluate
            // the same expression in float (the /8f is exact) and then resolve
            // ties to even. Do not add a tie epsilon: a value that is genuinely
            // one ulp off a tie must round the way the source does, and a
            // tolerance would also corrupt values that are legitimately close
            // to .5 (review N7).
            const float calibratedFloat =
                static_cast<float>(source.word) *
                    static_cast<float>(cal.pScale) / 8.0f +
                static_cast<float>(cal.pOffset);
            axis.calibrated =
                roundHalfEvenToInt(static_cast<double>(calibratedFloat));
            axis.calibratedValid = true;
        } else {
            axis.calibratedValid = false;
        }
        if (calibration_.axis[static_cast<std::size_t>(i)].pScale != 0.0) {
            axis.potLow = source.rawCounts < 100;
            axis.potHigh = source.rawCounts > 3996;
            if (axis.potLow || axis.potHigh) {
                axis.potAlarmLatched = true;
            }
        }
    }

    const SelectorData selector = decodeSelector(*data);
    if (const auto* s1 = std::get_if<Selector1>(&selector)) {
        telemetry_.platformNumber = s1->platformNumber;
        telemetry_.boardType = s1->boardType;
        telemetry_.platformMs = now_;
        telemetry_.selectorSeenMs[1] = now_;
        ++telemetry_.selectorFrames[1];
        pairingObserved_ = true;
        pairingMatched_ = s1->platformNumber > 0 &&
                          (expectedPlatformNumber_ == 0 ||
                           s1->platformNumber == expectedPlatformNumber_);
        if (!pairingMatched_) {
            diagnostics_.pairingMismatch = true;
            diagnostics_.detail =
                "platform number " + std::to_string(s1->platformNumber) +
                " does not match paired number " +
                std::to_string(expectedPlatformNumber_);
            if (state_ != State::Faulted) {
                enterIdentityLost(StopReason::PairingMismatch,
                                  diagnostics_.detail);
            }
        } else {
            diagnostics_.pairingMismatch = false;
        }
    } else if (const auto* s3 = std::get_if<Selector3>(&selector)) {
        telemetry_.loopCounter = s3->loopCounter;
        telemetry_.pneuSolenoid = s3->pneumaticSolenoid;
        telemetry_.rxDelay = s3->rxDelay;
        telemetry_.auxiliaryMs = now_;
        telemetry_.selectorSeenMs[3] = now_;
        ++telemetry_.selectorFrames[3];
    } else if (const auto* s4 = std::get_if<Selector4>(&selector)) {
        telemetry_.runCounter = s4->runCounter;
        telemetry_.accCount = s4->accCount;
        telemetry_.auxiliaryMs = now_;
        telemetry_.selectorSeenMs[4] = now_;
        ++telemetry_.selectorFrames[4];
    } else if (const auto* s5 = std::get_if<Selector5>(&selector)) {
        telemetry_.onboardPid.proportional = s5->proportional;
        telemetry_.onboardPid.derivative = s5->derivative;
        telemetry_.onboardPid.stepCorrection = s5->stepCorrection;
        telemetry_.onboardPidMs = now_;
        telemetry_.selectorSeenMs[5] = now_;
        ++telemetry_.selectorFrames[5];
    } else if (const auto* s6 = std::get_if<Selector6>(&selector)) {
        telemetry_.pneuAim = static_cast<int>(s6->pneumaticAim);
        telemetry_.avInput10 = static_cast<int>(s6->avInput10);
        telemetry_.avInput11 = static_cast<int>(s6->avInput11);
        telemetry_.avInputMs = now_;
        telemetry_.selectorSeenMs[6] = now_;
        ++telemetry_.selectorFrames[6];
        // frm_Main.cs:32362 and 30633: in run mode 3 these selector-6 values
        // are the source of MainBoardVolts/SolSuppVolts.
        telemetry_.mainBoardVolts =
            static_cast<double>(telemetry_.avInput10) * 2.7843 / 1024.0 * 5.0;
        telemetry_.solenoidVolts =
            static_cast<double>(telemetry_.avInput11) * 2.7843 / 1024.0 * 5.0;
        telemetry_.supplyVoltsMs = now_;
        telemetry_.voltageSource = Telemetry::VoltageSource::Selector6;
        telemetry_.voltageRunMode = commandedRunMode_;
    } else if (const auto* s7 = std::get_if<Selector7>(&selector)) {
        telemetry_.pressure = static_cast<int>(s7->avInput12);
        telemetry_.motorSupplyVolts = s7->supplyVolts;
        telemetry_.pressureMs = now_;
        telemetry_.motorSupplyMs = now_;
        telemetry_.selectorSeenMs[7] = now_;
        ++telemetry_.selectorFrames[7];
    } else if (const auto* s8 = std::get_if<Selector8>(&selector)) {
        telemetry_.runMode = s8->runMode;
        telemetry_.okToRun = s8->okToRun;
        telemetry_.runModeMs = now_;
        telemetry_.selectorSeenMs[8] = now_;
        ++telemetry_.selectorFrames[8];
    } else if (const auto* s9 = std::get_if<Selector9>(&selector)) {
        telemetry_.checksumCurrent = s9->checksumCurrent;
        telemetry_.checksumMs = now_;
        telemetry_.selectorSeenMs[9] = now_;
        ++telemetry_.selectorFrames[9];
    } else if (const auto* s2 = std::get_if<Selector2>(&selector)) {
        (void)s2;  // hour meter is not needed by this milestone
        telemetry_.selectorSeenMs[2] = now_;
        ++telemetry_.selectorFrames[2];
    } else if (std::holds_alternative<std::monostate>(selector)) {
        if (data->selector <= 9) {
            telemetry_.selectorSeenMs[data->selector] = now_;
            ++telemetry_.selectorFrames[data->selector];
        }
    }

    // Mode-dependent words. The host's last commanded command-6 value selects
    // the interpretation, exactly as frm_Main.cs:32347 does with
    // Module2.RunMode. Freeplay records and ADC records are separate; freeplay
    // words can never be read as supplies.
    if (commandedRunMode_ == 3) {
        const auto mode = decodeModeTelemetry(*data, 3);
        if (const auto* freeplay = std::get_if<FreeplayTelemetry>(&mode)) {
            telemetry_.freeplay = *freeplay;
            telemetry_.freeplayMs = now_;
            telemetry_.freeplayRunMode = 3;
        }
    } else {
        const auto mode = decodeModeTelemetry(*data, commandedRunMode_);
        if (const auto* board = std::get_if<NonFreeplayTelemetry>(&mode)) {
            telemetry_.boardAdc = *board;
            telemetry_.boardAdcMs = now_;
            telemetry_.boardAdcRunMode = commandedRunMode_;
            // frm_Main.cs:32362: the non-run-mode branch overwrites
            // AVInput[10]/[11] with ArdTVal[4]/[5].
            telemetry_.mainBoardVolts = board->mainBoardVolts;
            telemetry_.solenoidVolts = board->solenoidSupplyVolts;
            telemetry_.supplyVoltsMs = now_;
            telemetry_.voltageSource = Telemetry::VoltageSource::NonFreeplayAdc;
            telemetry_.voltageRunMode = commandedRunMode_;
        }
    }

    if (const auto fault = decodeFaultCode(*data)) {
        latchControllerFault(static_cast<std::uint16_t>(*fault));
    }
}

void Session::latchControllerFault(std::uint16_t code) {
    const bool first = !faultLatched_ || telemetry_.controllerFaultCode != code;
    faultLatched_ = true;
    telemetry_.controllerFaultLatched = true;
    telemetry_.controllerFaultCode = code;
    telemetry_.controllerFaultMs = now_;
    if (!first) {
        return;
    }
    ramp_.active = false;
    stage_ = Stage::None;
    configTransactionActive_ = false;
    configStatus_.configuring = false;
    configFrameIndex_ = 0;
    configAckDeadlineMs_ = 0;
    configAckOk_ = false;
    configStatus_.configured = false;
    pressureBurstRemaining_ = 0;
    stopBurstRemaining_ = 0;
    powerBurstRemaining_ = 0;
    state_ = State::Faulted;
    diagnostics_.reason = StopReason::ControllerFault;
    diagnostics_.detail = std::string(faultText(code)) +
                          " (raw " + std::to_string(code) +
                          "); all TX blocked until a new identify session";
}

// ---------------------------------------------------------------------------
// Guards and success criteria
// ---------------------------------------------------------------------------

bool Session::fresh(std::uint64_t stamp, int ttlMs) const {
    if (stamp == 0 || now_ < stamp) {
        return false;
    }
    return now_ - stamp <= static_cast<std::uint64_t>(ttlMs);
}

bool Session::feedbackFresh() const {
    for (const AxisFeedback& axis : telemetry_.axes) {
        if (!axis.seen || !axis.calibratedValid ||
            !fresh(axis.wordMs, config_.feedbackFreshMs)) {
            return false;
        }
    }
    return true;
}

bool Session::supplyFresh() const {
    return fresh(telemetry_.supplyVoltsMs, config_.supplyFreshMs) &&
           fresh(telemetry_.motorSupplyMs, config_.supplyFreshMs);
}

bool Session::pressureFresh() const {
    return telemetry_.pressureMs != 0 &&
           fresh(telemetry_.pressureMs, config_.pressureFreshMs);
}

bool Session::statusFresh() const {
    return fresh(telemetry_.statusMs, config_.statusFreshMs);
}

bool Session::targetFresh() const {
    return target_.valid && targetReceivedMs_ != 0 &&
           fresh(targetReceivedMs_, config_.targetStaleMs);
}

bool Session::powerCheck(bool running) const {
    // frm_Main.cs:31532-31540.
    if (!fresh(telemetry_.supplyVoltsMs, config_.supplyFreshMs)) {
        return false;
    }
    if (!(telemetry_.mainBoardVolts > 10.0 && telemetry_.solenoidVolts > 10.0)) {
        return false;
    }
    if (!running) {
        return true;
    }
    if (!fresh(telemetry_.motorSupplyMs, config_.supplyFreshMs)) {
        return false;
    }
    if (config_.motorCount == 63) {
        return telemetry_.motorSupplyVolts[0] > 44 &&
               telemetry_.motorSupplyVolts[1] > 44 &&
               telemetry_.motorSupplyVolts[2] > 44;
    }
    return telemetry_.motorSupplyVolts[0] > 44;
}

bool Session::pressureAimForInitialization(int* aim, std::string* why) const {
    if (aim == nullptr) {
        return false;
    }
    if (!pressurePlan_.enabled) {
        *aim = 0;
        return true;
    }
    if (motValFinal_ == 63) {
        // frm_Main.cs:23091: the aim depends on payload/driver/helper/strut
        // geometry that this session does not own.
        if (!pressurePlan_.aimSupplied) {
            if (why) {
                *why = "MotValFinal 63 requires a supplied strut pressure aim "
                       "(payload/driver/helper/strut inputs are unresolved here)";
            }
            return false;
        }
        *aim = pressurePlan_.aimPsi;
    } else {
        *aim = 30;  // recovered default for other configurations
    }
    return *aim >= 0 && *aim <= 250;
}

bool Session::readiness(std::string* why) const {
    auto fail = [&](const char* text) {
        if (why) *why = text;
        return false;
    };
    if (!identityAccepted_) return fail("identity not accepted");
    if (!pairingObserved_ || !pairingMatched_)
        return fail("platform identity (selector 1) not matched");
    if (!configAckOk_ || !configStatus_.configured)
        return fail("configuration ack 57 not observed");
    if (!statusFresh()) return fail("fresh type-48 status required");
    if (!telemetry_.configReceived)
        return fail("controller reports configuration not received");
    if (!telemetry_.avOk) return fail("controller reports AV sensors not OK");
    if (!calibration_.valid) return fail("validated calibration required");
    if (!feedbackFresh()) return fail("fresh calibrated feedback required");
    if (!targetFresh()) return fail("fresh valid target snapshot required");
    if (!supplyFresh()) return fail("fresh supply telemetry required");
    if (!powerCheck(true))
        return fail("PowerCheck failed (main/solenoid >10 V, motor rails >44 V)");
    if (pressureAim_ > 0) {
        if (!pressureFresh()) return fail("fresh strut pressure required");
        if (telemetry_.pressure < pressureAim_ * 0.5) {
            return fail("strut pressure below 50% of aim");
        }
    }
    return true;
}

void Session::updateGuards() {
    const bool identityState =
        state_ == State::Identified || state_ == State::Configured ||
        state_ == State::Parked || state_ == State::Stopped ||
        state_ == State::Docked || state_ == State::Initializing ||
        state_ == State::Running || state_ == State::Parking;

    if (identityState && identityAccepted_ && lastRxMs_ != 0) {
        const std::uint64_t silence = now_ - lastRxMs_;
        if (state_ == State::Running && !rxStaleHandled_ &&
            silence > static_cast<std::uint64_t>(config_.rxStaleMs)) {
            rxStaleHandled_ = true;
            enterStopped(StopReason::RxStale,
                         "no controller frame for " + std::to_string(silence) +
                             " ms (limit " +
                             std::to_string(config_.rxStaleMs) + " ms)");
            return;
        }
        if (silence > static_cast<std::uint64_t>(config_.commsLostMs)) {
            enterIdentityLost(StopReason::CommsLost,
                              "controller silence exceeded " +
                                  std::to_string(config_.commsLostMs) + " ms");
            return;
        }
    } else if (lastRxMs_ != 0 && identityAccepted_ && !identityState &&
               state_ != State::Faulted && state_ != State::SafeOff) {
        // No additional comms action for transitional states.
    }

    if (configLostActive_ && configAckOk_ && !configTransactionActive_) {
        if (now_ - configLostSinceMs_ >= 200) {
            configLostActive_ = false;
            configAckOk_ = false;
            configStatus_.configured = false;
            diagnostics_.configLost = true;
            enterStopped(StopReason::ConfigLost,
                         "controller reports configuration not received");
            return;
        }
    }

    if (state_ != State::Running) {
        supplyLossActive_ = false;
        pressureLowActive_ = false;
        return;
    }

    if (!supplyFresh()) {
        enterStopped(StopReason::SupplyStale,
                     "supply telemetry exceeded its freshness TTL");
        return;
    }
    if (!powerCheck(true)) {
        if (!supplyLossActive_) {
            supplyLossActive_ = true;
            supplyLossSinceMs_ = now_;
        } else if (now_ - supplyLossSinceMs_ >=
                   static_cast<std::uint64_t>(config_.supplyLossMs)) {
            enterStopped(StopReason::SupplyLost,
                         "PowerCheck failed while running");
            return;
        }
    } else {
        supplyLossActive_ = false;
    }

    if (pressureAim_ > 0 && motValFinal_ == 63) {
        const bool low = !pressureFresh() ||
                         telemetry_.pressure < pressureAim_ * 0.5;
        if (low) {
            if (!pressureLowActive_) {
                pressureLowActive_ = true;
                pressureLowSinceMs_ = now_;
            } else if (now_ - pressureLowSinceMs_ >=
                       static_cast<std::uint64_t>(config_.pressureLowMs)) {
                enterStopped(StopReason::AirPressureLow,
                             "strut pressure below 50% of aim for " +
                                 std::to_string(config_.pressureLowMs) + " ms");
                return;
            }
        } else {
            pressureLowActive_ = false;
        }
    } else {
        pressureLowActive_ = false;
    }

    if (!target_.valid) {
        enterStopped(StopReason::InvalidTarget,
                     "host target snapshot is marked invalid");
        return;
    }
    if (!targetFresh()) {
        enterStopped(StopReason::TargetStale,
                     "host target snapshot exceeded its freshness TTL");
        return;
    }
    if (!feedbackFresh()) {
        enterStopped(StopReason::FeedbackStale,
                     "per-axis feedback exceeded its freshness TTL");
        return;
    }
}

// ---------------------------------------------------------------------------
// State transitions
// ---------------------------------------------------------------------------

void Session::enterStopped(StopReason reason, const std::string& detail) {
    if (state_ == State::Faulted || state_ == State::IdentityLost ||
        state_ == State::SafeOff || state_ == State::Closed) {
        diagnostics_.reason = reason;
        diagnostics_.detail = detail;
        ramp_.active = false;
        return;
    }
    const bool wasMoving = state_ == State::Running ||
                           state_ == State::Initializing ||
                           state_ == State::Parking || stage_ != Stage::None;
    state_ = State::Stopped;
    diagnostics_.reason = reason;
    diagnostics_.detail = detail;
    ramp_.active = false;
    rampAccum_.fill(0.0);
    resetFollowUps();
    pressureBurstRemaining_ = 0;
    powerBurstRemaining_ = 0;
    rxStaleHandled_ = false;
    clearMotionGuards();
    // A stop/rejection abandons the in-flight configuration transaction but
    // does not pretend that a previously acknowledged configuration is gone.
    // Identity loss and controller faults clear the ack separately.
    if (configTransactionActive_) {
        configTransactionActive_ = false;
        configStatus_.configuring = false;
        configFrameIndex_ = 0;
        configAckDeadlineMs_ = 0;
    }
    // Best-effort stop: only when the link is still open, never recursively.
    // A write failure is included because the transport may still accept the
    // emergency stop; driveStopBurst consumes an attempt even on failure, so a
    // permanently failing link cannot cause an unbounded retry loop.
    const bool linkUsable = link_ != nullptr && link_->isOpen();
    if (wasMoving && linkUsable && !faultLatched_) {
        // The recovered host always re-asserts run mode 1 and stops the step
        // outputs on a motion-stopping path. Keep the frames, not the sleeps.
        txRunMode(kRunModeStop);
        startStopBurst(true);
    }
}

void Session::enterIdentityLost(StopReason reason, const std::string& detail) {
    if (state_ == State::Faulted) {
        return;
    }
    const bool wasMoving = state_ == State::Running ||
                           state_ == State::Initializing ||
                           state_ == State::Parking;
    const bool wasAuthenticated = identityAccepted_;
    const bool linkUsable = link_ != nullptr && link_->isOpen() &&
                            !link_->terminalFailure();

    // Bounded best-effort stop, sent while the previous identity is still
    // accepted so the normal post-identity TX gate (not a bypass) permits it.
    //
    // Comms loss: the recovered host reacts with Platform_Shutdown(3, "Comms
    // Lost") (frm_Main.cs:29890, 29958, 30024), which asserts run mode 1 and
    // stops the axes on the way to the dock. That full sequence needs fresh
    // feedback to observe its settle and to vent safely, which a silent link
    // cannot provide, so this path sends only the stop subset that is safe
    // without feedback: run mode 1 plus the two zero-rate stop frames with the
    // current flags. It does not descend, vent or disable the motor rails
    // blind.
    //
    // Pairing mismatch: the responder just identified itself as a different
    // platform. Frames are unaddressed on this serial link, so this path does
    // not transmit at all; stopping a foreign board is not this session's
    // authority. No bypass is introduced in either case.
    int stopped = 0;
    if (wasMoving && wasAuthenticated && reason == StopReason::CommsLost &&
        linkUsable && !faultLatched_) {
        if (txRunMode(kRunModeStop)) ++stopped;
        if (txStopAll()) ++stopped;
        if (txStopAll()) ++stopped;
        writeFailurePending_ = false;  // already stopping; no second stop path
    }

    state_ = State::IdentityLost;
    identityAccepted_ = false;
    pairingObserved_ = false;
    pairingMatched_ = false;
    configAckOk_ = false;
    configStatus_.configured = false;
    configStatus_.configuring = false;
    configTransactionActive_ = false;
    configFrameIndex_ = 0;
    configAckDeadlineMs_ = 0;
    configStatus_.framesSent = 0;
    ramp_.active = false;
    ramp_.settled = false;
    ramp_.endpointReached = false;
    rampAccum_.fill(0.0);
    resetFollowUps();
    pressureBurstRemaining_ = 0;
    stopBurstRemaining_ = 0;
    powerBurstRemaining_ = 0;
    settleSinceMs_ = 0;
    lastRampTickMs_ = now_;
    // Clear every motion guard, not just the config flags: a later identify
    // session must not inherit supply/pressure/RX-stale/config-loss debounce
    // state from the lost session (review N4).
    clearMotionGuards();
    supplyLossSinceMs_ = 0;
    pressureLowSinceMs_ = 0;
    configLostSinceMs_ = 0;
    diagnostics_.reason = reason;
    diagnostics_.detail = detail;
    if (stopped > 0) {
        diagnostics_.detail += " (best-effort stop: " +
                               std::to_string(stopped) + " frames written)";
    } else if (wasMoving && wasAuthenticated &&
               reason == StopReason::PairingMismatch) {
        diagnostics_.detail +=
            " (no stop frame sent: the responder is not the paired platform)";
    }
}

void Session::clearMotionGuards() {
    supplyLossActive_ = false;
    pressureLowActive_ = false;
    configLostActive_ = false;
    rxStaleHandled_ = false;
}

void Session::setStage(Stage stage, const char* name, std::uint64_t deadlineMs) {
    stage_ = stage;
    lifecycle_.stage = name != nullptr ? name : "";
    lifecycle_.stageStartedMs = now_;
    stageDeadlineMs_ = deadlineMs;
}

// ---------------------------------------------------------------------------
// TX
// ---------------------------------------------------------------------------

bool Session::canTransmit(std::uint8_t command) const {
    if (link_ == nullptr || !link_->isOpen()) {
        return false;
    }
    if (faultLatched_) {
        return false;  // frm_Main.cs:32503
    }
    if (state_ == State::Closed) {
        return false;
    }
    if (state_ == State::Configuring) {
        return command == 5;  // frm_Main.cs:32627
    }
    if (!identityAccepted_) {
        return command == 3;  // frm_Main.cs:32620
    }
    if (command == 5) {
        return false;  // only inside a configuration transaction
    }
    switch (command) {
    case 3:
        return true;
    case 6:
        // Run-mode selection is allowed in every non-configuring state after
        // identity, including SafeOff (the recovered stop path re-asserts 1).
        return true;
    case 1:
        // Power/stop frames are legal in every post-identity state except the
        // configuration transaction (handled above); emergency stop must work
        // even from Identified/Configured when nothing has moved yet.
        return true;
    case 2:
        return state_ == State::Running || state_ == State::Initializing ||
               state_ == State::Parking;
    case 4:
        return state_ == State::Initializing || state_ == State::Running ||
               state_ == State::Parking || state_ == State::Stopped ||
               state_ == State::Parked || state_ == State::Docked;
    default:
        return false;
    }
}

void Session::noteTx(std::uint8_t command, std::uint8_t sequence) {
    ++tx_.total;
    tx_.lastSequence = sequence;
    switch (command) {
    case 1: ++tx_.command1; break;
    case 2: ++tx_.command2; break;
    case 3: ++tx_.command3; break;
    case 4: ++tx_.command4; break;
    case 5: ++tx_.command5; break;
    case 6: ++tx_.command6; break;
    default: break;
    }
}

bool Session::transmit(
    std::uint8_t command,
    const std::array<std::uint8_t, kHostPayloadBytes>& payload,
    bool forcedSequence) {
    if (!canTransmit(command)) {
        return false;
    }
    const std::uint8_t sequence =
        forcedSequence ? sequence_.nextForced() : sequence_.nextOrdinary();
    const EncodeResult encoded = encodeFrame(command, payload, sequence);
    if (!encoded) {
        diagnostics_.detail = "frame encode failed";
        return false;
    }
    if (!link_->write(encoded.bytes.data(), encoded.bytes.size())) {
        ++tx_.writeFailures;
        writeFailurePending_ = true;
        return false;
    }
    noteTx(command, sequence);
    lastTxMs_ = now_;
    return true;
}

bool Session::txCommand3() {
    const std::array<std::uint8_t, kHostPayloadBytes> payload{};
    return transmit(3, payload, true);  // RS232_Tx_Motor(3,...) is positive
}

bool Session::txRunMode(std::uint8_t mode) {
    std::array<std::uint8_t, kHostPayloadBytes> payload{};
    payload[0] = mode;
    if (!transmit(6, payload, true)) {
        return false;
    }
    commandedRunMode_ = mode;
    return true;
}

bool Session::txPower(bool acRequest, bool motorDisable) {
    std::array<std::uint8_t, kHostPayloadBytes> payload{};
    payload[12] = static_cast<std::uint8_t>(
        (acRequest ? kFlagAcPowerRequest : 0) |
        (motorDisable ? kFlagMotorDisable : 0));
    if (!transmit(1, payload, true)) {  // Motor_Power passes positive 1
        return false;
    }
    power_.acRequested = acRequest;
    power_.motorDisabled = motorDisable;
    ++power_.framesSent;
    power_.lastFrameMs = now_;
    return true;
}

bool Session::txStopAll() {
    std::array<std::uint8_t, kHostPayloadBytes> payload{};
    payload[12] = static_cast<std::uint8_t>(
        (power_.acRequested ? kFlagAcPowerRequest : 0) |
        (power_.motorDisabled ? kFlagMotorDisable : 0));
    // Stop_All_Motors uses RS232_Tx_Motor(-1,...): ordinary sequence.
    return transmit(1, payload, false);
}

bool Session::txPneumatic(std::uint8_t value) {
    std::array<std::uint8_t, kHostPayloadBytes> payload{};
    payload[0] = value;
    return transmit(4, payload, true);  // StrutAimPressure_Update is positive 4
}

bool Session::txConfigFrame(const ConfigFrame& frame) {
    const EncodeResult encoded = encodeConfigFrame(frame.pairs, true, 0);
    if (!encoded) {
        diagnostics_.detail = "configuration frame is not encodable";
        return false;
    }
    std::array<std::uint8_t, kHostPayloadBytes> payload{};
    for (std::size_t i = 0; i < payload.size(); ++i) {
        payload[i] = encoded.bytes[kHostPayloadOffset + i];
    }
    return transmit(5, payload, true);  // SendConfigToBoard passes positive 5
}

bool Session::txTargets(const std::array<AxisTarget, 6>& targets) {
    std::array<std::uint8_t, kHostPayloadBytes> payload{};
    for (std::size_t i = 0; i < targets.size(); ++i) {
        std::uint16_t word = 0;
        if (targets[i].kind == AxisTarget::Kind::Special) {
            word = 1;  // PEP special value; only reached through opt-in
        } else {
            const std::int32_t value = targets[i].value;
            if (!inWireRange(value) || targetWordReserved(value)) {
                diagnostics_.detail =
                    "target axis " + std::to_string(i + 1) +
                    " is not encodable (range or reserved word 1)";
                return false;
            }
            word = static_cast<std::uint16_t>(value + 32767);
        }
        payload[i * 2] = static_cast<std::uint8_t>(word & 0xFFu);
        payload[i * 2 + 1] = static_cast<std::uint8_t>((word >> 8) & 0xFFu);
    }
    payload[12] = static_cast<std::uint8_t>(
        (power_.acRequested ? kFlagAcPowerRequest : 0) |
        (power_.motorDisabled ? kFlagMotorDisable : 0));
    // Update_Stepper_4 uses RS232_Tx_Motor(-2,...): ordinary sequence.
    return transmit(2, payload, false);
}

// ---------------------------------------------------------------------------
// Configuration transaction
// ---------------------------------------------------------------------------

bool Session::buildConfigTransaction(PidMode mode, std::string* why) {
    std::vector<std::string> missing;
    std::string invalid;
    const char* modeName = mode == PidMode::Normal      ? "normal"
                           : mode == PidMode::Configure ? "configure"
                                                        : "lifecycle";
    const std::optional<PidProfile>& profileSlot =
        pidProfiles_[static_cast<std::size_t>(mode)];
    const PidProfile profile = profileSlot.value_or(PidProfile{});
    auto get = [&](const std::optional<int>& value, const char* name) -> int {
        if (!value) {
            missing.push_back(name);
            return 0;
        }
        if (!inWireRange(*value)) {
            if (invalid.empty()) {
                invalid = std::string(name) + " is outside the biased wire range";
            }
            return 0;
        }
        return *value;
    };
    auto profileValue = [&](const std::optional<int>& value, const char* name) {
        return get(value, (std::string("PID profile (") + modeName + ") " + name)
                              .c_str());
    };
    auto getAxis = [&](const std::array<std::optional<int>, 6>& values,
                       const char* prefix, int index) -> int {
        return get(values[static_cast<std::size_t>(index)],
                   (std::string(prefix) + " axis " +
                    std::to_string(index + 1)).c_str());
    };

    const int bench = get(controllerConfig_.benchTest, "5 BenchTest");
    const int comms = get(controllerConfig_.commsTimeoutMs, "6 CommsTimeoutMS");
    const int stepRev = get(controllerConfig_.stepRev, "21 StepRev");
    const int shutdown = get(controllerConfig_.onboardShutdown, "31 OnBoardShutdown");
    const int accelP1 = get(controllerConfig_.accelP1Times100, "40 AccelP1*100");
    const int accelP0 = get(controllerConfig_.accelP0Times10, "41 AccelP0*10");
    const int decel = get(controllerConfig_.decelTimes100, "42 DecelScale*100");
    if (!profileSlot) {
        missing.push_back(std::string("PID profile (") + modeName + ")");
    }
    const int filter = profileValue(profile.sensorFilterTimes10000,
                                    "46 SensorFilter*10000");
    const int history = profileValue(profile.historyLen, "47 HistoryLen");
    const int motVal = get(controllerConfig_.motValFinal, "30 MotValFinal");
    const int stepCorr = profileValue(profile.stepCorrTimes10000,
                                      "39 StepCorrection*10000");
    const int dFilter = profileValue(profile.dFilter, "25 DFilter");
    const int tConst = profileValue(profile.tConst, "26 TConst");
    const int deadbandEnable = get(controllerConfig_.deadbandEnable, "27 DeadbandEnable");
    const int deadbandValue = get(controllerConfig_.deadbandValue, "28 DeadbandValue");
    const int stoppedCount = get(controllerConfig_.stoppedCountLimit,
                                 "29 StoppedCountLimit");
    const int avFilter = get(controllerConfig_.avFilterTimes10000, "32 AVFilter*10000");
    const int limits = get(controllerConfig_.limitsEnable, "33 LimitsEnable");
    const int maxPps = get(controllerConfig_.maxPps, "34 MaxPPS");
    const int infoMotor = get(controllerConfig_.infoMotor, "35 InfoMotor");
    const int rotation = get(controllerConfig_.rotationDirection, "36 RotationDirection");
    const int proportional = profileValue(profile.proportionalTimes10, "22 P*10");
    const int derivative = profileValue(profile.derivativeTimes10, "23 D*10");

    int potLo[6]{};
    int angLo[6]{};
    int pScale[6]{};
    int pOffset[6]{};
    int gearbox[6]{};
    for (int i = 0; i < 6; ++i) {
        potLo[i] = getAxis(controllerConfig_.potLo, "101..106 PotLo", i);
        angLo[i] = getAxis(controllerConfig_.angLoTimes10, "111..116 AngLo*10", i);
        pScale[i] = getAxis(controllerConfig_.pScaleTimes1000, "121..126 pScale*1000", i);
        pOffset[i] = getAxis(controllerConfig_.pOffsetTimes10, "131..136 pOffset*10", i);
        gearbox[i] = getAxis(controllerConfig_.gearboxTimes100, "141..146 Gearbox*100", i);
    }

    if (!invalid.empty()) {
        if (why) *why = invalid;
        return false;
    }
    if (!missing.empty()) {
        diagnostics_.missing = missing;
        std::string text = "missing controller parameters:";
        for (const std::string& name : missing) {
            text += " " + name + ";";
        }
        if (why) *why = text;
        return false;
    }
    if (stepRev <= 0) {
        if (why) *why = "StepRev must be positive";
        return false;
    }
    if (motVal < 0) {
        if (why) *why = "MotValFinal is invalid";
        return false;
    }

    diagnostics_.missing.clear();
    motValFinal_ = motVal;
    stepRev_ = stepRev;

    auto makeFrame = [](std::initializer_list<ConfigPair> pairs) {
        ConfigFrame frame;
        int index = 0;
        for (const ConfigPair& pair : pairs) {
            if (index < 3) {
                frame.pairs[static_cast<std::size_t>(index)] = pair;
            }
            ++index;
        }
        return frame;
    };
    auto pair = [](int id, int value) {
        ConfigPair p;
        p.id = id;
        p.value = value;
        return p;
    };
    auto empty = pair(0, 0);

    // Exact frame grouping from frm_Main.cs:31709-31911. Unused slots stay
    // (0,0) and do not contribute. The 56 real parameter values plus the
    // (11,57) end marker produce the controller's ack count of 57.
    configFrames_[0] = makeFrame({pair(10, 0), empty, empty});
    configFrames_[1] = makeFrame({pair(5, bench), pair(6, comms), empty});
    configFrames_[2] = makeFrame({pair(21, stepRev),
                                  // SendConfigToBoard sends param 43 = 40.
                                  pair(43, 40),
                                  pair(31, shutdown)});
    configFrames_[3] = makeFrame({pair(40, accelP1), pair(41, accelP0),
                                  pair(42, decel)});
    configFrames_[4] = makeFrame({pair(46, filter), pair(47, history), empty});
    configFrames_[5] = makeFrame({pair(101, potLo[0]), pair(102, potLo[1]),
                                  pair(103, potLo[2])});
    configFrames_[6] = makeFrame({pair(104, potLo[3]), pair(105, potLo[4]),
                                  pair(106, potLo[5])});
    configFrames_[7] = makeFrame({pair(111, angLo[0]), pair(112, angLo[1]),
                                  pair(113, angLo[2])});
    configFrames_[8] = makeFrame({pair(114, angLo[3]), pair(115, angLo[4]),
                                  pair(116, angLo[5])});
    configFrames_[9] = makeFrame({pair(121, pScale[0]), pair(122, pScale[1]),
                                  pair(123, pScale[2])});
    configFrames_[10] = makeFrame({pair(124, pScale[3]), pair(125, pScale[4]),
                                   pair(126, pScale[5])});
    configFrames_[11] = makeFrame({pair(131, pOffset[0]), pair(132, pOffset[1]),
                                   pair(133, pOffset[2])});
    configFrames_[12] = makeFrame({pair(134, pOffset[3]), pair(135, pOffset[4]),
                                   pair(136, pOffset[5])});
    configFrames_[13] = makeFrame({pair(141, gearbox[0]), pair(142, gearbox[1]),
                                   pair(143, gearbox[2])});
    configFrames_[14] = makeFrame({pair(144, gearbox[3]), pair(145, gearbox[4]),
                                   pair(146, gearbox[5])});
    configFrames_[15] = makeFrame({pair(30, motVal), pair(39, stepCorr),
                                   pair(45, 5)});
    configFrames_[16] = makeFrame({empty, pair(25, dFilter), pair(26, tConst)});
    configFrames_[17] = makeFrame({pair(27, deadbandEnable),
                                   pair(28, deadbandValue),
                                   pair(29, stoppedCount)});
    configFrames_[18] = makeFrame({pair(32, avFilter), pair(33, limits),
                                   pair(34, maxPps)});
    configFrames_[19] = makeFrame({pair(35, infoMotor), pair(36, rotation), empty});
    configFrames_[20] = makeFrame({pair(22, proportional), pair(23, derivative),
                                   // Param 20 = 1: onboard command-2 mode.
                                   pair(20, 1)});
    configFrames_[21] = makeFrame({pair(11, kConfigCount), empty, empty});

    configFramesReady_ = true;
    return true;
}

bool Session::startConfiguration(std::uint64_t now) {
    if (!configFramesReady_) {
        return false;
    }
    ++configAttempts_;
    configStatus_.attempts = configAttempts_;
    configStatus_.configuring = true;
    configStatus_.configured = false;
    configStatus_.framesSent = 0;
    configStatus_.framesExpected = kConfigFrameCount;
    configStatus_.parametersExpected = kConfigParameterCount;  // ack value
    configStatus_.parametersTransmitted = kConfigParametersTransmitted;
    configStatus_.transactionStartedMs = now;
    configFrameIndex_ = 0;
    nextConfigFrameMs_ = now;
    configAckDeadlineMs_ = 0;
    configTransactionActive_ = true;
    configTransactionStartMs_ = now;
    state_ = State::Configuring;
    return true;
}

void Session::finishConfiguration(bool success, const std::string& detail) {
    configTransactionActive_ = false;
    configStatus_.configuring = false;
    configFrameIndex_ = 0;
    configAckDeadlineMs_ = 0;
    // A lifecycle PID transaction returns to the state that requested it. The
    // resume step (resumeAfterPidApply) decides the next stage.
    const bool lifecycle = pidResume_ != PidResume::None;
    if (success) {
        configAckOk_ = true;
        configStatus_.configured = true;
        appliedPidMode_ = lifecycle ? pidApplyMode_ : PidMode::Configure;
        diagnostics_.configRejected = false;
        diagnostics_.configLost = false;
        diagnostics_.reason = StopReason::None;
        diagnostics_.detail = detail;
        state_ = lifecycle ? pidResumeState_ : State::Configured;
    } else {
        configAckOk_ = false;
        configStatus_.configured = false;
        appliedPidMode_.reset();
        diagnostics_.configRejected = true;
        diagnostics_.reason = StopReason::ConfigRejected;
        diagnostics_.detail = detail;
        state_ = lifecycle ? pidResumeState_ : State::Identified;
    }
    if (lifecycle) {
        pidApplyPending_ = false;
        pidApplyOk_ = success;
    }
}

bool Session::beginPidApply(PidMode mode, PidResume resume, std::string* why) {
    if (!pidProfiles_[static_cast<std::size_t>(mode)]) {
        if (why) {
            const char* name = mode == PidMode::Normal      ? "normal"
                               : mode == PidMode::Configure ? "configure"
                                                            : "lifecycle";
            *why = std::string("PID profile (") + name +
                   ") is not supplied; motion and venting are blocked until it is";
        }
        return false;
    }
    pidApplyMode_ = mode;
    pidResume_ = resume;
    pidResumeState_ = state_;
    pidApplyPending_ = true;
    pidApplyOk_ = false;
    // Not applied until this transaction is acknowledged (E): the previous mode
    // and frame count must not stay visible while the new one is in flight.
    appliedPidMode_.reset();
    configStatus_.framesSent = 0;
    setStage(Stage::PidApply, "Applying lifecycle PID profile", 0);
    return true;
}

void Session::resumeAfterPidApply() {
    const PidResume resume = pidResume_;
    pidResume_ = PidResume::None;
    if (!pidApplyOk_) {
        const std::string detail =
            "lifecycle PID profile was not acknowledged: " + diagnostics_.detail;
        if (state_ == State::Initializing) {
            failInitialization(StopReason::ConfigRejected, detail);
        } else {
            enterStopped(StopReason::ConfigRejected, detail);
        }
        return;
    }
    switch (resume) {
    case PidResume::InitPower:
        startPowerBurst(true, false);
        setStage(Stage::InitPowerBurst, "Motor power on", 0);
        return;
    case PidResume::ShutdownPause:
        diagnostics_.detail = "lifecycle PID profile applied; pausing before descent";
        setStage(Stage::ShutdownPause, "Shutdown: pause after lifecycle profile",
                 now_ + static_cast<std::uint64_t>(config_.shutdownPauseMs));
        return;
    case PidResume::ShutdownVent:
        diagnostics_.reason = StopReason::UserShutdown;
        diagnostics_.detail = "lifecycle PID profile applied; venting struts before "
                             "power off";
        setStage(Stage::ShutdownVent, "Shutdown: vent", 0);
        return;
    case PidResume::StartMotion: {
        // Running only after the normal ACK and a fresh readiness observation.
        // Otherwise the platform stays Parked and says why. The PidApply stage
        // ends here whichever way it goes.
        stage_ = Stage::None;
        std::string why;
        if (!readiness(&why)) {
            diagnostics_.detail = "motion not started after the normal PID ACK: " + why;
            return;
        }
        std::string runWhy;
        if (!enterRunning(&runWhy)) {
            diagnostics_.detail = "motion not started: " + runWhy;
        }
        return;
    }
    case PidResume::None:
        return;
    }
}

void Session::resetFollowUps() {
    stage_ = Stage::None;
    stageDeadlineMs_ = 0;
    ramp_.active = false;
    rampAccum_.fill(0.0);
    parkAfterRamp_ = false;
    shutdownAfterPark_ = false;
    settleSinceMs_ = 0;
    pressureBurstVent_ = false;
    pidResume_ = PidResume::None;
    pidApplyPending_ = false;
    pidApplyOk_ = false;
}

void Session::startCadenceChain() {
    targetDueMs_ = static_cast<double>(now_);
    nextTargetTxMs_ = now_;
    cadenceStartMs_ = now_;
    cadenceFrames_ = 0;
    tx_.missedCadenceSlots = 0;
}

double Session::achievedTargetHz() const {
    const std::uint64_t elapsed = now_ - cadenceStartMs_;
    if (cadenceFrames_ == 0 || elapsed == 0) {
        return 0.0;
    }
    return static_cast<double>(cadenceFrames_) * 1000.0 /
           static_cast<double>(elapsed);
}

void Session::advanceTargetCadence() {
    // Fractional deadline chain (1000/hz ms). Sending on the first tick at or
    // after each deadline keeps the average at the nominal rate for any render
    // frame rate. A stall that passes several deadlines is resynchronised by
    // advancing past now; no catch-up burst is sent.
    const double period = 1000.0 / static_cast<double>(config_.targetCadenceHz);
    const double nowMs = static_cast<double>(now_);
    targetDueMs_ += period;
    // Resynchronise past now, strictly: a deadline equal to now would otherwise
    // be served on the next tick as well (one extra frame after a stall).
    while (targetDueMs_ <= nowMs) {
        targetDueMs_ += period;
    }
    // The clock has 1 ms resolution: a deadline is due from the start of its
    // millisecond. Rounding up instead would skip a render tick that lands 1 ms
    // early and serve that deadline a whole period late (60 Hz -> 50 frames/s).
    nextTargetTxMs_ = static_cast<std::uint64_t>(std::floor(targetDueMs_));
    // Shortfall since the chain started: deadlines that have passed minus frames
    // sent. A sustained render rate below 60 Hz (50 fps, for example) and a stall
    // both show here; a chain that keeps up reports zero.
    ++cadenceFrames_;
    const std::uint64_t elapsed = now_ - cadenceStartMs_;
    const std::uint64_t passed =
        elapsed * static_cast<std::uint64_t>(config_.targetCadenceHz) / 1000u;
    tx_.missedCadenceSlots = passed > cadenceFrames_ ? passed - cadenceFrames_ : 0;
}

void Session::driveConfiguration() {
    if (state_ != State::Configuring || !configTransactionActive_) {
        return;
    }
    if (configFrameIndex_ < kConfigFrameCount) {
        if (now_ < nextConfigFrameMs_) {
            return;
        }
        if (!txConfigFrame(configFrames_[static_cast<std::size_t>(configFrameIndex_)])) {
            if (writeFailurePending_) {
                return;
            }
            finishConfiguration(false, "configuration frame could not be encoded");
            return;
        }
        ++configFrameIndex_;
        configStatus_.framesSent = configFrameIndex_;
        if (configFrameIndex_ == kConfigFrameCount) {
            configAckDeadlineMs_ =
                now_ + static_cast<std::uint64_t>(config_.configAckTimeoutMs);
        } else {
            nextConfigFrameMs_ =
                now_ + static_cast<std::uint64_t>(config_.configFrameSpacingMs);
        }
        return;
    }

    if (now_ < configAckDeadlineMs_) {
        return;
    }
    if (configAttempts_ < config_.configMaxAttempts) {
        // Update_OnBoard_Config retries the whole fixed transaction.
        startConfiguration(now_);
        return;
    }
    finishConfiguration(
        false, "no valid type-43 ack after " +
                   std::to_string(configAttempts_) + " transactions; expected " +
                   std::to_string(kConfigCount));
}

// ---------------------------------------------------------------------------
// Identification and polling
// ---------------------------------------------------------------------------

void Session::driveIdentify() {
    if (state_ != State::Identifying) {
        return;
    }
    if (probeInFlight_ && now_ >= probeDeadlineMs_) {
        probeInFlight_ = false;
        nextProbeMs_ = now_;
    }
    if (probeInFlight_ || now_ < nextProbeMs_) {
        return;
    }
    if (identifyProbes_ >= config_.identifyMaxProbes) {
        state_ = State::Opened;
        diagnostics_.reason = identityEchoMismatches_ > 0
                                  ? StopReason::IdentityMismatch
                                  : StopReason::IdentityMismatch;
        diagnostics_.detail =
            "no cmd-3 identity echo after " +
            std::to_string(identifyProbes_) + " probes of " +
            std::to_string(config_.identifyProbeTimeoutMs) + " ms";
        return;
    }
    if (!txCommand3()) {
        return;
    }
    ++identifyProbes_;
    probeInFlight_ = true;
    probeDeadlineMs_ =
        now_ + static_cast<std::uint64_t>(config_.identifyProbeTimeoutMs);
    nextProbeMs_ = probeDeadlineMs_;
}

void Session::driveIdlePoll() {
    if (state_ != State::Identified && state_ != State::Configured &&
        state_ != State::Parked && state_ != State::Stopped &&
        state_ != State::Docked) {
        return;
    }
    if (stage_ != Stage::None) {
        return;
    }
    if (now_ - lastIdlePollMs_ <
            static_cast<std::uint64_t>(config_.idlePollMs) ||
        now_ - lastTxMs_ < 200) {
        return;
    }
    if (txCommand3()) {
        lastIdlePollMs_ = now_;
    }
}

// ---------------------------------------------------------------------------
// Bursts
// ---------------------------------------------------------------------------

void Session::startPowerBurst(bool acRequest, bool motorDisable) {
    powerBurstAc_ = acRequest;
    powerBurstDisable_ = motorDisable;
    powerBurstRemaining_ = 3;  // Motor_Power sends three frames
    nextPowerBurstMs_ = now_;
    power_.acRequested = acRequest;
    power_.motorDisabled = motorDisable;
    lifecycle_.powerOnObserved = false;
}

void Session::drivePowerBurst() {
    if (powerBurstRemaining_ <= 0 || now_ < nextPowerBurstMs_) {
        return;
    }
    if (!txPower(powerBurstAc_, powerBurstDisable_)) {
        return;
    }
    --powerBurstRemaining_;
    nextPowerBurstMs_ = now_ + 50;  // StopPause(0.05)
}

void Session::startPressureBurst(std::uint8_t value) {
    pressureBurstRemaining_ = 2;  // StrutAimPressure_Update sends twice
    nextPressureBurstMs_ = now_;
    pressureBurstByte_ = value;
    pressureBurstVent_ = false;
    lastPressureSentMs_ = now_;
}

void Session::startVentBurst() {
    startPressureBurst(kPneumaticVent);
    pressureBurstVent_ = true;
    ventWaitSinceMs_.reset();
}

bool Session::ventTelemetryStaleOnly() const {
    // True only when the stored observations still prove the dock and merely
    // lack freshness. Any explicit failure is not stale-only.
    if (!calibration_.valid || telemetry_.statusMs == 0 || !telemetry_.avOk) {
        return false;
    }
    for (int i = 0; i < 6; ++i) {
        const AxisFeedback& axis = telemetry_.axes[static_cast<std::size_t>(i)];
        if (!axis.seen || !axis.calibratedValid) {
            return false;
        }
        const int error = std::abs(
            axis.calibrated -
            calibration_.axis[static_cast<std::size_t>(i)].dockedSteps);
        if (error > config_.parkToleranceSteps) {
            return false;
        }
    }
    return !statusFresh() || !feedbackFresh();
}

void Session::abortVent(const std::string& detail) {
    pressureBurstRemaining_ = 0;
    pressureBurstVent_ = false;
    ventWaitSinceMs_.reset();
    shutdownAfterPark_ = false;
    const bool stale = !feedbackFresh() || !statusFresh();
    enterStopped(stale ? StopReason::FeedbackStale : StopReason::ParkNotReached,
                 detail);
}

void Session::drivePressureBurst() {
    if (pressureBurstRemaining_ > 0) {
        if (pressureBurstVent_) {
            // Vent authority is evidence at transmission time, not at request
            // time: a delayed burst, the stop-frame gap and the descent-completion
            // path can all outlive the observation that allowed the vent (F1).
            std::string at;
            if (state_ != State::Docked) {
                abortVent("vent aborted before transmission: platform is no longer "
                          "docked");
                return;
            }
            if (!atCalibrationDock(&at)) {
                // A dropped or slow type-48 frame is not an unsafe pose. Wait a
                // bounded time for a fresh observation. Explicit AV loss, an
                // out-of-tolerance axis or an uncalibrated pose abort at once.
                if (!ventTelemetryStaleOnly()) {
                    abortVent("vent aborted before transmission: " + at);
                    return;
                }
                if (!ventWaitSinceMs_) {
                    ventWaitSinceMs_ = now_;
                }
                if (now_ - *ventWaitSinceMs_ >
                    static_cast<std::uint64_t>(config_.ventStaleWaitMs)) {
                    abortVent("vent aborted: no fresh dock observation within " +
                              std::to_string(config_.ventStaleWaitMs) + " ms");
                }
                return;
            }
            ventWaitSinceMs_.reset();
        }
        if (now_ >= nextPressureBurstMs_ && txPneumatic(pressureBurstByte_)) {
            --pressureBurstRemaining_;
            nextPressureBurstMs_ = now_ + 1;  // StopPause(0.001)
        }
        return;
    }
    if (stage_ == Stage::ShutdownVent || stage_ == Stage::ShutdownVentWait ||
        stage_ == Stage::ShutdownPowerOff) {
        return;
    }
    const bool refreshState =
        state_ == State::Running || state_ == State::Parked ||
        (state_ == State::Initializing &&
         (stage_ == Stage::InitRampDock || stage_ == Stage::InitRampPause ||
          stage_ == Stage::InitRampNeutral));
    if (!refreshState || pressureAim_ <= 0) {
        return;
    }
    if (now_ - lastPressureSentMs_ >=
        static_cast<std::uint64_t>(config_.pressureRefreshMs)) {
        startPressureBurst(static_cast<std::uint8_t>(pressureAim_ + 5));
    }
}

void Session::startStopBurst(bool ordinarySequence) {
    stopBurstRemaining_ = 2;  // Stop_All_Motors sends the zero frame twice
    nextStopBurstMs_ = now_;
    stopBurstOrdinary_ = ordinarySequence;
}

void Session::driveStopBurst() {
    if (stopBurstRemaining_ <= 0 || now_ < nextStopBurstMs_) {
        return;
    }
    std::array<std::uint8_t, kHostPayloadBytes> payload{};
    payload[12] = static_cast<std::uint8_t>(
        (power_.acRequested ? kFlagAcPowerRequest : 0) |
        (power_.motorDisabled ? kFlagMotorDisable : 0));
    if (!transmit(1, payload, !stopBurstOrdinary_)) {
        // Consume the attempt even when the write fails. A failing link then
        // produces at most two best-effort stop attempts, never a storm.
        --stopBurstRemaining_;
        nextStopBurstMs_ = now_ + 50;
        return;
    }
    --stopBurstRemaining_;
    nextStopBurstMs_ = now_ + 50;
}

// ---------------------------------------------------------------------------
// Ramps
// ---------------------------------------------------------------------------

std::array<int, 6> Session::currentCalibrated() const {
    std::array<int, 6> out{};
    for (int i = 0; i < 6; ++i) {
        out[static_cast<std::size_t>(i)] =
            telemetry_.axes[static_cast<std::size_t>(i)].calibrated;
    }
    return out;
}

bool Session::atCalibrationDock(std::string* why) const {
    // A Docked state is an observation, not a property of the platform. Prove
    // it again against the current calibration, fresh AV status, and fresh feedback
    // before any action that depends on the dock pose (vent, or "already parked").
    if (!calibration_.valid) {
        if (why) *why = "validated calibration is required to verify the dock pose";
        return false;
    }
    if (!statusFresh()) {
        if (why) *why = "fresh type-48 status is required to verify the dock pose";
        return false;
    }
    if (!telemetry_.avOk) {
        if (why) *why = "controller reports AV sensors not OK";
        return false;
    }
    if (!feedbackFresh()) {
        if (why) {
            *why = "fresh calibrated feedback is required to verify the dock pose";
        }
        return false;
    }
    const std::array<int, 6> feedback = currentCalibrated();
    for (int i = 0; i < 6; ++i) {
        const int error = std::abs(
            feedback[static_cast<std::size_t>(i)] -
            calibration_.axis[static_cast<std::size_t>(i)].dockedSteps);
        if (error > config_.parkToleranceSteps) {
            if (why) {
                *why = "axis " + std::to_string(i + 1) + " is " +
                       std::to_string(error) + " steps from the calibration dock"
                       " (tolerance " +
                       std::to_string(config_.parkToleranceSteps) + " steps)";
            }
            return false;
        }
    }
    return true;
}

bool Session::startRamp(int phase, const std::array<int, 6>& goal,
                        const std::string& stageName) {
    if (!calibration_.valid || !feedbackFresh()) {
        diagnostics_.reason = StopReason::FeedbackStale;
        diagnostics_.detail = "ramp requires fresh calibrated feedback";
        return false;
    }
    if (stepRev_ <= 0) {
        diagnostics_.detail = "StepRev is required to compute a ramp duration";
        return false;
    }
    const int raiseLowerSpeed = roundHalfEvenToInt(stepRev_ * 0.35);
    if (raiseLowerSpeed <= 0) {
        diagnostics_.detail = "RaiseLowerSpeed is not positive";
        return false;
    }
    ramp_.start = currentCalibrated();
    ramp_.goal = goal;
    ramp_.current = ramp_.start;
    ramp_.phase = phase;
    ramp_.active = true;
    ramp_.settled = false;
    ramp_.endpointReached = false;
    ramp_.startMs = now_;
    ramp_.maxError = 0;
    rampAccum_.fill(0.0);
    lastRampTickMs_ = now_;
    startCadenceChain();
    for (int i = 0; i < 6; ++i) {
        const int value = goal[static_cast<std::size_t>(i)];
        if (!inWireRange(value) || targetWordReserved(value)) {
            ramp_.active = false;
            diagnostics_.detail = "ramp goal is not encodable";
            return false;
        }
        ramp_.maxError = std::max(
            ramp_.maxError,
            std::abs(value - ramp_.start[static_cast<std::size_t>(i)]));
    }
    const double seconds =
        std::max(1.0, static_cast<double>(ramp_.maxError) / raiseLowerSpeed);
    ramp_.durationMs = static_cast<std::uint64_t>(roundHalfEven(seconds * 1000.0));
    // The source continues integrating for two seconds after the nominal
    // duration. Our accumulator runs until the exact endpoint is reached, so
    // allow the nominal duration, the tail (about 6% of it) and the existing
    // settle allowance before declaring a timeout.
    const std::uint64_t tailAllowance = std::max<std::uint64_t>(
        static_cast<std::uint64_t>(config_.parkTimeoutMs),
        ramp_.durationMs / 4 + 1000);
    ramp_.timeoutMs = ramp_.durationMs + tailAllowance;
    settleSinceMs_ = 0;
    nextTargetTxMs_ = now_;
    // The caller owns the Stage enum transition; startRamp only publishes the
    // human-readable lifecycle label.
    lifecycle_.stage = stageName;
    lifecycle_.stageStartedMs = now_;
    return true;
}

void Session::driveRamp() {
    if (!ramp_.active) {
        return;
    }
    if (state_ != State::Initializing && state_ != State::Parking) {
        ramp_.active = false;
        return;
    }

    const std::uint64_t elapsed = now_ - ramp_.startMs;
    const double durationSeconds =
        std::max(0.001, static_cast<double>(ramp_.durationMs) / 1000.0);

    if (now_ >= nextTargetTxMs_) {
        // Source shape (frm_Main.cs:24268+, num3 = 2 s): the per-tick
        // increment is InitStepErr/Sequence_Duration, scaled after 93% by a
        // factor that reaches a 0.25 floor. Accumulating the scaled increment
        // (instead of sampling the whole error) preserves the tail. Unlike the
        // source loop, integration continues until the exact endpoint, so a
        // nonzero ramp cannot stop a few percent short; success still requires
        // an observed settle.
        const double progress =
            static_cast<double>(elapsed) / static_cast<double>(ramp_.durationMs);
        const double tailfactor =
            progress <= 0.93
                ? 1.0
                : std::max(0.25, 1.0 - (progress - 0.93) * 6.0);
        const double dtSeconds =
            std::max(0.0, static_cast<double>(now_ - lastRampTickMs_) / 1000.0);
        lastRampTickMs_ = now_;

        bool endpoint = true;
        for (int i = 0; i < 6; ++i) {
            const double start = static_cast<double>(
                ramp_.start[static_cast<std::size_t>(i)]);
            const double goal = static_cast<double>(
                ramp_.goal[static_cast<std::size_t>(i)]);
            const double error = goal - start;
            double& accumulated = rampAccum_[static_cast<std::size_t>(i)];
            if (error == 0.0) {
                accumulated = 0.0;
            } else {
                accumulated += dtSeconds * (error / durationSeconds) * tailfactor;
                if (error > 0.0) {
                    accumulated = std::min(accumulated, error);
                    accumulated = std::max(accumulated, 0.0);
                } else {
                    accumulated = std::max(accumulated, error);
                    accumulated = std::min(accumulated, 0.0);
                }
            }
            if (accumulated != error) {
                endpoint = false;
            }
        }
        if (endpoint) {
            ramp_.endpointReached = true;
        }

        std::array<AxisTarget, 6> targets{};
        for (int i = 0; i < 6; ++i) {
            const int start = ramp_.start[static_cast<std::size_t>(i)];
            const int goal = ramp_.goal[static_cast<std::size_t>(i)];
            int current = ramp_.endpointReached
                              ? goal
                              : start + roundHalfEvenToInt(
                                            rampAccum_[static_cast<std::size_t>(i)]);
            if (goal >= start) {
                current = std::min(current, goal);
                current = std::max(current, start);
            } else {
                current = std::max(current, goal);
                current = std::min(current, start);
            }
            ramp_.current[static_cast<std::size_t>(i)] = current;
            targets[static_cast<std::size_t>(i)] = AxisTarget::ordinary(current);
        }
        if (!txTargets(targets)) {
            if (writeFailurePending_) {
                return;
            }
            if (state_ == State::Parking) {
                enterStopped(StopReason::InvalidTarget, diagnostics_.detail);
            } else {
                failInitialization(StopReason::InvalidTarget,
                                   diagnostics_.detail);
            }
            return;
        }
        advanceTargetCadence();
    }

    if (!ramp_.endpointReached) {
        if (elapsed > ramp_.timeoutMs) {
            const std::string detail =
                "ramp endpoint was not reached before the timeout";
            if (state_ == State::Parking) {
                enterStopped(StopReason::ParkNotReached, detail);
            } else if (state_ == State::Initializing) {
                failInitialization(StopReason::ParkNotReached, detail);
            }
        }
        return;
    }

    const std::array<int, 6> feedback = currentCalibrated();
    bool within = feedbackFresh();
    for (int i = 0; i < 6 && within; ++i) {
        const int error =
            std::abs(feedback[static_cast<std::size_t>(i)] -
                     ramp_.goal[static_cast<std::size_t>(i)]);
        if (error > config_.parkToleranceSteps) {
            within = false;
        }
    }
    if (within) {
        if (settleSinceMs_ == 0) {
            settleSinceMs_ = now_;
        } else if (now_ - settleSinceMs_ >=
                   static_cast<std::uint64_t>(config_.parkSettleMs)) {
            ramp_.active = false;
            ramp_.settled = true;
            ramp_.current = ramp_.goal;
            if (state_ == State::Parking) {
                // Run mode 1 is asserted only after the base-point settle, as
                // in the recovered shutdown (frm_Main.cs:25450).
                txRunMode(kRunModeStop);
                startStopBurst(true);
                state_ = State::Docked;
                stage_ = Stage::None;
                lifecycle_.stage = "Docked";
                if (shutdownAfterPark_) {
                    // Vent only after the dock endpoint was commanded and the
                    // feedback settle window elapsed. The lifecycle profile was
                    // applied before the descent (B); each vent frame re-checks
                    // the dock proof when it is transmitted.
                    diagnostics_.reason = StopReason::UserShutdown;
                    diagnostics_.detail =
                        "calibration dock observed; venting struts before power off";
                    setStage(Stage::ShutdownVent, "Shutdown: vent", 0);
                } else {
                    diagnostics_.reason = StopReason::UserPark;
                    diagnostics_.detail =
                        "docked at calibration dock after observed settle";
                }
                parkAfterRamp_ = false;
            } else if (state_ == State::Initializing) {
                startStopBurst(true);
                if (stage_ == Stage::InitRampDock) {
                    setStage(Stage::InitRampPause,
                             "Platform at base point; pause", now_ + 500);
                } else if (stage_ == Stage::InitRampNeutral) {
                    setStage(Stage::InitComplete,
                             "Initialization complete", 0);
                }
            }
        }
    } else {
        settleSinceMs_ = 0;
        if (elapsed > ramp_.timeoutMs) {
            const std::string detail =
                "ramp did not settle within tolerance " +
                std::to_string(config_.parkToleranceSteps) + " steps";
            if (state_ == State::Parking) {
                enterStopped(StopReason::ParkNotReached, detail);
            } else if (state_ == State::Initializing) {
                failInitialization(StopReason::ParkNotReached, detail);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool Session::startInitialization(std::uint64_t now) {
    (void)now;
    state_ = State::Initializing;
    powerObserveAttempts_ = 0;
    setStage(Stage::InitSetRunMode, "Set run mode 3", 0);
    return true;
}

void Session::failInitialization(StopReason reason, const std::string& detail) {
    ramp_.active = false;
    resetFollowUps();
    txRunMode(kRunModeStop);
    startStopBurst(true);
    pressureAim_ = 0;
    startPressureBurst(5);  // aim 0 encoded as TxPneu[0]=5
    if (power_.acRequested) {
        startPowerBurst(false, true);
    }
    state_ = State::Stopped;
    diagnostics_.reason = reason;
    diagnostics_.detail = detail;
}

void Session::driveLifecycle() {
    switch (stage_) {
    case Stage::None:
    case Stage::InitComplete:
    case Stage::ParkRampNeutral:
        break;
    case Stage::InitSetRunMode: {
        // frm_Main.cs:22986: run mode 3 is asserted before power is requested.
        if (!txRunMode(kRunModeRun)) {
            return;
        }
        // PID_SetValues(2) precedes the power sequence (frm_Main.cs:23019).
        std::string why;
        if (!beginPidApply(PidMode::Lifecycle, PidResume::InitPower, &why)) {
            failInitialization(StopReason::InitializationFailed, why);
        }
        return;
    }
    case Stage::PidApply:
        if (!pidApplyPending_) {
            resumeAfterPidApply();
            return;
        }
        // Start the 22-frame transaction only at a quiet boundary. Stop, pressure
        // and power frames use commands 1 and 4, which Configuring gates out, so
        // starting earlier would silently drop them.
        if (configTransactionActive_ || stopBurstRemaining_ > 0 ||
            pressureBurstRemaining_ > 0 || powerBurstRemaining_ > 0) {
            return;
        }
        {
            std::string why;
            if (!buildConfigTransaction(pidApplyMode_, &why)) {
                pidApplyPending_ = false;
                pidApplyOk_ = false;
                diagnostics_.detail = why;
                resumeAfterPidApply();
                return;
            }
            // configAckOk_ and configured are deliberately kept: the lifecycle
            // profile changes only PID-mode parameters, so the controller still
            // holds the calibration that calibrated feedback depends on (A).
            configAttempts_ = 0;
            configStatus_.wrongAckCount = 0;
            configStatus_.lastAckCount = -1;
            configStatus_.lastAckMs = 0;
            diagnostics_.configRejected = false;
            diagnostics_.configLost = false;
            startConfiguration(now_);
        }
        return;
    case Stage::InitPowerBurst:
        if (powerBurstRemaining_ > 0) {
            return;
        }
        setStage(Stage::InitPowerObserve, "Observe main board volts",
                 now_ + 200);  // StopPause(0.2)
        return;
    case Stage::InitPowerObserve: {
        if (now_ < stageDeadlineMs_) {
            return;
        }
        const bool freshMain =
            fresh(telemetry_.supplyVoltsMs, config_.supplyFreshMs);
        lifecycle_.lastMainBoardVolts = telemetry_.mainBoardVolts;
        if (freshMain && telemetry_.mainBoardVolts > 7.0) {
            lifecycle_.powerOnObserved = true;
            setStage(Stage::InitSettle, "Motor power settle",
                     now_ + 4000);  // StopPause(4.0)
            return;
        }
        ++powerObserveAttempts_;
        if (powerObserveAttempts_ < 3) {
            startPowerBurst(true, false);
            setStage(Stage::InitPowerBurst, "Motor power retry", 0);
            return;
        }
        failInitialization(
            StopReason::SupplyMissing,
            "main board voltage did not exceed 7 V after three power requests"
            " (last " + std::to_string(telemetry_.mainBoardVolts) + " V)");
        return;
    }
    case Stage::InitSettle:
        if (now_ < stageDeadlineMs_) {
            return;
        }
        lifecycle_.supplyObserved = powerCheck(true);
        if (!powerCheck(true)) {
            failInitialization(
                StopReason::SupplyLost,
                "PowerCheck failed during initialization (main/solenoid >10 V,"
                " motor rails >44 V required with fresh telemetry)");
            return;
        }
        {
            int aim = 0;
            std::string why;
            if (!pressureAimForInitialization(&aim, &why)) {
                failInitialization(StopReason::InitializationFailed, why);
                return;
            }
            pressureAim_ = aim;
            lifecycle_.pressureAimPsi = aim;
            // Source sends aim+5; the chkPneuDisable path sends 5 (aim 0).
            startPressureBurst(static_cast<std::uint8_t>(aim + 5));
        }
        setStage(Stage::InitPressure, "Set centre strut pressure",
                 now_ + 2500);  // StopPause(2.5)
        return;
    case Stage::InitPressure:
        if (pressureBurstRemaining_ > 0 || now_ < stageDeadlineMs_) {
            return;
        }
        // Do not declare the pressure step successful from intent alone: the
        // strut sensor must have reported during the pressure window.
        if (!pressureFresh()) {
            failInitialization(StopReason::InitializationFailed,
                               "no fresh strut pressure telemetry after the "
                               "pressure command");
            return;
        }
        lifecycle_.pressureObserved = true;
        if (!startRamp(0,
                       std::array<int, 6>{
                           calibration_.axis[0].dockedSteps,
                           calibration_.axis[1].dockedSteps,
                           calibration_.axis[2].dockedSteps,
                           calibration_.axis[3].dockedSteps,
                           calibration_.axis[4].dockedSteps,
                           calibration_.axis[5].dockedSteps},
                       "Moving Platform To Base Point")) {
            failInitialization(StopReason::InitializationFailed,
                               diagnostics_.detail);
            return;
        }
        setStage(Stage::InitRampDock, "Moving Platform To Base Point", 0);
        return;
    case Stage::InitRampPause:
        if (stopBurstRemaining_ > 0 || now_ < stageDeadlineMs_) {
            return;
        }
        if (!startRamp(1, std::array<int, 6>{neutralSteps_[0], neutralSteps_[1],
                                             neutralSteps_[2], neutralSteps_[3],
                                             neutralSteps_[4], neutralSteps_[5]},
                       "Moving Platform To Zero Point")) {
            failInitialization(StopReason::InitializationFailed,
                               diagnostics_.detail);
            return;
        }
        setStage(Stage::InitRampNeutral, "Moving Platform To Zero Point", 0);
        return;
    case Stage::ShutdownVent:
        if (stopBurstRemaining_ > 0) {
            return;
        }
        startVentBurst();
        setStage(Stage::ShutdownVentWait, "Vent struts; wait for pressure",
                 now_ + static_cast<std::uint64_t>(
                            config_.shutdownVentTimeoutMs));
        return;
    case Stage::ShutdownVentWait:
        if (pressureBurstRemaining_ > 0) {
            return;
        }
        if (motValFinal_ == 63) {
            if (pressureFresh() && std::abs(telemetry_.pressure) < 4) {
                // vented
            } else if (now_ < stageDeadlineMs_) {
                return;
            }
        }
        startPowerBurst(false, true);  // AC off, motor disable on
        setStage(Stage::ShutdownPowerOff, "Motor power off", 0);
        return;
    case Stage::ShutdownPause: {
        if (now_ < stageDeadlineMs_) {
            return;
        }
        // The descent starts only from a fresh observed pose after the profile
        // ACK and the source pause. Explicit AV loss or stale feedback stops here.
        if (!calibration_.valid || !statusFresh() || !telemetry_.avOk ||
            !feedbackFresh()) {
            enterStopped(StopReason::FeedbackStale,
                         "shutdown descent not started: fresh observed feedback and "
                         "AV status are required after the lifecycle profile");
            return;
        }
        if (!txRunMode(kRunModeRun)) {
            enterStopped(StopReason::InvalidTarget,
                         "shutdown descent not started: run-mode command failed");
            return;
        }
        const std::array<int, 6> dock{
            calibration_.axis[0].dockedSteps, calibration_.axis[1].dockedSteps,
            calibration_.axis[2].dockedSteps, calibration_.axis[3].dockedSteps,
            calibration_.axis[4].dockedSteps, calibration_.axis[5].dockedSteps};
        stage_ = Stage::None;
        state_ = State::Parking;
        parkAfterRamp_ = false;
        shutdownAfterPark_ = true;
        if (!startRamp(0, dock, "Shutdown: moving to calibration dock")) {
            txRunMode(kRunModeStop);
            enterStopped(StopReason::FeedbackStale,
                         "shutdown descent could not start: " + diagnostics_.detail);
            return;
        }
        diagnostics_.reason = StopReason::UserShutdown;
        diagnostics_.detail =
            "descending to calibration dock; vent waits for observed settle";
        return;
    }
    case Stage::ShutdownPowerOff:
        if (powerBurstRemaining_ > 0) {
            return;
        }
        state_ = State::SafeOff;
        resetFollowUps();
        diagnostics_.reason = StopReason::UserShutdown;
        diagnostics_.detail = "struts vented; motor rails disabled";
        lifecycle_.stage = "SafeOff";
        return;
    }

    if (state_ == State::Initializing && stage_ == Stage::InitComplete) {
        if (stopBurstRemaining_ > 0) {
            return;
        }
        // Initialization leaves the Lifecycle profile applied. The normal
        // profile is applied on an explicit Start, not at the end of init (E).
        state_ = State::Parked;
        stage_ = Stage::None;
        diagnostics_.reason = StopReason::None;
        diagnostics_.detail = "initialization complete after observed base and "
                             "zero ramps";
        lifecycle_.stage = "Parked";
        lifecycle_.pressureAimPsi = pressureAim_;
    }
}

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------

void Session::driveMotion() {
    if (state_ != State::Running) {
        return;
    }
    if (now_ < nextTargetTxMs_) {
        return;
    }
    if (!target_.valid) {
        enterStopped(StopReason::InvalidTarget,
                     "host target snapshot is marked invalid");
        return;
    }
    if (!targetFresh()) {
        enterStopped(StopReason::TargetStale,
                     "host target snapshot exceeded its freshness TTL");
        return;
    }
    std::array<AxisTarget, 6> targets{};
    for (int i = 0; i < 6; ++i) {
        const std::int32_t value = target_.steps[static_cast<std::size_t>(i)];
        if (!inWireRange(value) || targetWordReserved(value)) {
            enterStopped(StopReason::InvalidTarget,
                         "target axis " + std::to_string(i + 1) +
                             " is outside the encodable range or collides with "
                             "reserved word 1");
            return;
        }
        targets[static_cast<std::size_t>(i)] =
            pepCycles_[static_cast<std::size_t>(i)] > 0
                ? AxisTarget::specialWord()
                : AxisTarget::ordinary(value);
    }
    if (!txTargets(targets)) {
        if (writeFailurePending_) {
            return;
        }
        enterStopped(StopReason::InvalidTarget, diagnostics_.detail);
        return;
    }
    for (int i = 0; i < 6; ++i) {
        if (pepCycles_[static_cast<std::size_t>(i)] > 0) {
            --pepCycles_[static_cast<std::size_t>(i)];
        }
    }
    advanceTargetCadence();
}

}  // namespace mp6
