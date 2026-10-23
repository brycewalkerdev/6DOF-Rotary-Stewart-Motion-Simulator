#pragma once
/*
 * MP6 session: the single owner of MP6 timing, sequencing and transmit gating.
 *
 * Portable C++17. No Win32, no App/Entity/UI types, no filesystem. All I/O goes
 * through an injected mp6::Link, so the same code runs against a real Mega, a
 * recorded byte stream, or an in-memory scripted link.
 *
 * The session implements the host behaviour reconstructed from
 * motum_resources/motum-replacement-handoff/recovered-source/MotSim/frm_Main.cs
 * and Module2.cs. Line citations are repeated at the relevant implementation
 * points in mp6_session.cpp.
 *
 * Scope of this milestone:
 *   - Arduino onboard control only: command 2 step targets (config param 20 = 1).
 *     Host PID command-1 rate streaming is deliberately not implemented.
 *   - Command 1 is emitted only in the recovered power/stop shape (all six rate
 *     words zero, flag byte 6/7 used for AC request and motor disable).
 *   - Identity is the recovered command-3 echo: within 200 ms, a type-42 frame
 *     whose byte 7 is 3 and byte 9 is 253. Platform pairing uses selector 1.
 *     The fixed 253 is source-correct for this firmware's recovered discovery
 *     path; it is not a nonce placeholder.
 *   - Configuration is the fixed 22-frame transaction carrying 56 parameter
 *     values; the end marker (11,57) makes the controller's ack count 57.
 *   - Mode-dependent PID values (PidProfile per PidMode) are carried by that same
 *     transaction, applied at lifecycle boundaries: Configure, Initialize (before
 *     power), explicit Start (Normal, before Running) and shutdown (before
 *     descent/vent). The two-frame Update_OnBoardPID packet is not used on this path.
 *   - Target cadence is 60 Hz only (other requests are normalised), on a
 *     fractional deadline chain; shortfall is reported in
 *     TxCounters::missedCadenceSlots and achievedTargetHz(), never burst.
 *   - `Parked` is the observed neutral/ready pose. `Docked` is the separately
 *     observed calibration-dock pose reached by Park or before shutdown vent.
 *
 * Unresolved semantics are surfaced, never guessed:
 *   - Target word 1 (PEP special value) is not emitted unless the caller
 *     explicitly opts in AND arms an axis. Its firmware meaning is unknown.
 *   - The command-16 firmware park, command-10 hour-meter flush and command-8
 *     torque pulse are not emitted at all.
 *   - Missing calibration/controller/geometry inputs block initialization with
 *     named diagnostics instead of being replaced by invented values.
 */

#include "mp6_link.h"
#include "mp6_protocol.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mp6 {

inline constexpr int kSessionAxes = 6;
inline constexpr int kConfigFrameCount = 22;
// The controller's type-43 acknowledgement counts the 56 real parameters plus
// the end marker (11,57). The begin marker (10,0) is not counted. Keep this name
// for the wire value; the number of parameter values sent is one fewer.
inline constexpr int kConfigParameterCount = 57;
inline constexpr int kConfigParametersTransmitted = 56;
// The only documented target cadence (Control_Update_Target = 1/60). Other
// requests are normalised by the Session constructor.
inline constexpr int kSupportedCadenceHz = 60;

// ---------------------------------------------------------------------------
// State, intents and reasons
// ---------------------------------------------------------------------------

enum class State : std::uint8_t {
    Closed = 0,       // no link attached
    Opened,           // link attached, no identity
    Identifying,      // command-3 probe in flight / retrying
    Identified,       // type-42 echo accepted, pairing not necessarily seen
    Configuring,      // command-5 transaction in flight; only command 5 is sent
    Configured,       // type-43 ack 57 matched (56 params + end marker)
    Parked,           // observed at the neutral/ready target after initialization
    Initializing,     // staged power/pressure/ramp sequence
    Running,          // command-2 targets streaming
    Stopped,          // motion stopped by user or guard; controller still identified
    Parking,          // observed ramp to the calibration dock (park/shutdown)
    SafeOff,          // motors disabled after an orderly or emergency stop
    Faulted,          // controller fault latch: nothing is transmitted
    IdentityLost,     // comms/comms-timeout or pairing/echo mismatch
    Docked,           // observed settle at the calibration dock; may vent
};

enum class Intent : std::uint8_t {
    Identify = 0,
    Configure,
    Initialize,
    StartMotion,
    StopMotion,
    Park,
    Shutdown,
    EmergencyStop,
};

enum class StopReason : std::uint8_t {
    None = 0,
    UserStop,
    UserPark,
    UserShutdown,
    EmergencyStop,
    InvalidTarget,
    TargetStale,
    RxStale,
    CommsLost,
    WriteFailure,
    LinkClosed,
    SupplyMissing,
    SupplyLost,
    SupplyStale,
    AirPressureLow,
    ConfigLost,
    ConfigRejected,
    PairingMismatch,
    IdentityMismatch,
    IdentityLost,
    ControllerFault,
    FeedbackStale,
    FeedbackInvalid,
    ParkNotReached,
    InitializationFailed,
    LinkNotOpen,
};

const char* toString(State state) noexcept;
const char* toString(Intent intent) noexcept;
const char* toString(StopReason reason) noexcept;

double roundHalfEven(double value);
int roundHalfEvenToInt(double value);

// ---------------------------------------------------------------------------
// Tunables (all source timings unless marked)
// ---------------------------------------------------------------------------

struct SessionConfig {
    int targetCadenceHz = 60;             // Control_Update_Target = 1/60
    int idlePollMs = 250;                 // Timer_SwitchBox interval
    int identifyProbeTimeoutMs = 200;     // Serial_Find_1
    int identifyMaxProbes = 10;           // Serial_Find_1 do/while num <= 9
    int configFrameSpacingMs = 1;         // StopPause(0.001) per frame
    int configAckTimeoutMs = 200;         // allowance after the last frame
    int configMaxAttempts = 4;            // Update_OnBoard_Config
    int rxStaleMs = 1500;                 // Timer_SwitchBox comms timeout
    int commsLostMs = 10000;              // running comms-lost shutdown
    int statusFreshMs = 2000;             // config/AV status bits
    int supplyFreshMs = 2000;             // voltage observation TTL
    int supplyLossMs = 500;               // switchbox loss debounce
    int pressureFreshMs = 2000;           // strut pressure TTL
    int pressureLowMs = 8000;             // air pressure-low persistence
    int pressureRefreshMs = 200;          // force update while SimRun
    int targetStaleMs = 500;              // host target snapshot TTL
    int feedbackFreshMs = 500;            // per-axis feedback TTL
    int modeFreshMs = 5000;               // run-mode context TTL
    int parkToleranceSteps = 40;          // docking comparison in source
    int parkSettleMs = 200;               // settle observation (conservative)
    int parkTimeoutMs = 5000;             // extra time after the ramp
    int shutdownVentTimeoutMs = 20000;    // Platform_Shutdown vent wait
    int motorCount = 63;                  // MotValFinal/MotorCount semantics
    // Stop_All_Motors -> PID_SetValues(2) -> StopPause(0.5) -> rest ramp
    // (frm_Main.cs:25051-25059): pause after the lifecycle profile ACK.
    int shutdownPauseMs = 500;
    // Engineering choice (not source): how long a vent may wait for a fresh dock
    // observation when the stored observations still prove the dock.
    int ventStaleWaitMs = 500;
    bool allowPepSpecialTargets = false;  // unresolved word-1 semantics
};

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

// Typed command-5 inputs. Values are the exact wire integers the recovered
// host sends (already scaled). Missing values are reported by name and block
// the configuration transaction; nothing is fabricated.
struct ControllerConfig {
    std::optional<int> benchTest;                 // param 5
    std::optional<int> commsTimeoutMs;            // param 6
    std::optional<int> stepRev;                   // param 21
    std::optional<int> onboardShutdown;           // param 31
    std::optional<int> accelP1Times100;           // param 40
    std::optional<int> accelP0Times10;            // param 41
    std::optional<int> decelTimes100;             // param 42
    std::array<std::optional<int>, 6> potLo;      // params 101..106
    std::array<std::optional<int>, 6> angLoTimes10;    // params 111..116
    std::array<std::optional<int>, 6> pScaleTimes1000; // params 121..126
    std::array<std::optional<int>, 6> pOffsetTimes10;  // params 131..136
    std::array<std::optional<int>, 6> gearboxTimes100; // params 141..146
    std::optional<int> motValFinal;               // param 30
    std::optional<int> deadbandEnable;            // param 27
    std::optional<int> deadbandValue;             // param 28
    std::optional<int> stoppedCountLimit;         // param 29
    std::optional<int> avFilterTimes10000;        // param 32
    std::optional<int> limitsEnable;              // param 33
    std::optional<int> maxPps;                    // param 34
    std::optional<int> infoMotor;                 // param 35
    std::optional<int> rotationDirection;         // param 36
    // Superseded by PidProfile (one per PidMode). The session no longer reads
    // these fields; the desktop must move them into setPidProfile().
    std::optional<int> proportionalTimes10;       // was param 22
    std::optional<int> derivativeTimes10;         // was param 23
    std::optional<int> dFilter;                   // was param 25
    std::optional<int> tConst;                    // was param 26
    std::optional<int> stepCorrTimes10000;        // was param 39
    std::optional<int> sensorFilterTimes10000;    // was param 46
    std::optional<int> historyLen;                // was param 47
    // Param 20 (OnBoardControl) is forced to 1 by this session. Host-PID
    // command-1 streaming is out of scope.
};

// Lifecycle PID modes, matching PID_SetValues(Typ) in frm_Main.cs.
enum class PidMode : std::uint8_t {
    Normal = 0,     // normal motion (frm_Main.cs:28060, 34642)
    Configure = 1,  // coefficient load / configure (frm_Main.cs:18766)
    Lifecycle = 2,  // initialize and shutdown (frm_Main.cs:23019, 25055)
};

// Exact wire integers for the mode-dependent command-5 parameters. Each field is
// what the recovered host sends after its own float/F3 conversion; the desktop
// adapter performs that conversion. Missing fields block configuration by name.
struct PidProfile {
    std::optional<int> proportionalTimes10;      // param 22 (P*10)
    std::optional<int> derivativeTimes10;        // param 23 (D*10)
    std::optional<int> dFilter;                  // param 25
    std::optional<int> tConst;                   // param 26
    std::optional<int> stepCorrTimes10000;       // param 39 (Step Correction*10000)
    std::optional<int> sensorFilterTimes10000;   // param 46 (pneumatic filter*10000)
    std::optional<int> historyLen;               // param 47
};

// One command-5 frame: three biased ID/value pairs.
struct ConfigFrame {
    std::array<ConfigPair, 3> pairs{};
};

// Validated host-side calibration (Load_Coefficients port; computed elsewhere).
// The session only needs pScale/pOffset for calibrated feedback and the docked
// step position for the base-point phase. angleToStep and geometry are not
// duplicated here.
//
// Feedback precision: the recovered host evaluates
//     (int)Math.Round((float)word * pScale / 8f + pOffset)
// (frm_Main.cs:32233) entirely in single precision, so the session narrows
// these double inputs to float for that expression. Ties are resolved on the
// represented float value, exactly as Math.Round(float) does; no epsilon is
// applied. Keep dockedSteps/neutral steps in integer steps.
struct AxisCalibration {
    double pScale = 0.0;       // steps per pot count
    double pOffset = 0.0;      // steps
    int dockedSteps = 0;       // AngLo*Gearbox*StepRev/360 (round-to-even)
    bool valid = false;
};

struct Calibration {
    std::array<AxisCalibration, 6> axis{};
    bool valid = false;
    std::string description;
};

// Strut pressure intent. For MotValFinal==63 the caller must supply the
// payload/driver/helper/strut-derived aim; otherwise the recovered default of
// 30 psi is used. `enabled=false` is the chkPneuDisable case (aim 0).
struct PressurePlan {
    bool enabled = true;
    bool aimSupplied = false;
    int aimPsi = 0;
};

// One pipeline snapshot of six signed step targets. The session rejects
// out-of-range values and the reserved word-1 collision instead of clamping.
struct StepTarget {
    std::array<std::int32_t, 6> steps{};
    bool valid = false;
    std::uint64_t generation = 0;
};

// ---------------------------------------------------------------------------
// Observations
// ---------------------------------------------------------------------------

struct AxisFeedback {
    bool seen = false;
    std::uint16_t word = 0;
    int raw = 0;
    int calibrated = 0;
    bool calibratedValid = false;
    bool potLow = false;
    bool potHigh = false;
    bool potAlarmLatched = false;
    std::uint64_t wordMs = 0;
};

struct OnboardPidTelemetry {
    double proportional = 0.0;
    double derivative = 0.0;
    double stepCorrection = 0.0;
};

struct Telemetry {
    std::uint64_t lastFrameMs = 0;       // any checksum-valid type-48 frame
    std::uint64_t type48Frames = 0;
    std::uint64_t selectorSeenMs[10]{};  // index 0..9, 0 = never
    std::uint64_t selectorFrames[10]{};

    // Status nibble from byte 4 (present in every type-48 frame).
    bool configReceived = false;
    bool avOk = false;
    std::uint64_t statusMs = 0;

    // Selector 1 (pairing).
    int platformNumber = -1;
    int boardType = -1;
    std::uint64_t platformMs = 0;

    // Selector 8 (mode and OK2Run).
    int runMode = -1;
    bool okToRun = false;
    std::uint64_t runModeMs = 0;

    // Selector 6: pneumatic aim and AV inputs 10/11. These are the source of
    // MainBoardVolts/SolSuppVolts in run mode 3 (frm_Main.cs:32362).
    //
    // Freshness dependency: supply freshness is tracked by supplyVoltsMs,
    // which selector 6 refreshes. In run mode 3 the supply guard therefore
    // depends on selector-6 cadence; if the controller stops rotating selector
    // 6 the session treats the retained supply reading as stale and stops
    // motion. Do not substitute a cached selector-6 value for freshness.
    int pneuAim = 0;
    int avInput10 = 0;
    int avInput11 = 0;
    std::uint64_t avInputMs = 0;

    // Selector 7: AV12..14; AV12 is StrutPressure, AV14 packs three 3 V
    // quantised motor supply readings.
    int pressure = 0;
    std::uint64_t pressureMs = 0;
    std::array<int, 3> motorSupplyVolts{};
    std::uint64_t motorSupplyMs = 0;

    // Selector 5 (informational onboard gains actually reported).
    OnboardPidTelemetry onboardPid;
    std::uint64_t onboardPidMs = 0;

    // Selector 9 (informational).
    std::uint16_t checksumCurrent = 0;
    std::uint64_t checksumMs = 0;

    // Selector 3/4 (informational).
    int loopCounter = 0;
    int pneuSolenoid = 0;
    std::uint16_t rxDelay = 0;
    int runCounter = 0;
    int accCount = 0;
    std::uint64_t auxiliaryMs = 0;

    // Raw fault word (ArdTVal[9]) and its latched raw code.
    bool controllerFaultLatched = false;
    std::uint16_t controllerFaultCode = 0;
    std::uint64_t controllerFaultMs = 0;

    // Per-axis feedback. Raw words are always sensor truth; calibrated values
    // require a valid Calibration and a matched configuration ack.
    std::array<AxisFeedback, 6> axes{};

    // Supply volts derived exactly where the Windows host derives them:
    // AVInput[10]/[11], updated by selector 6 or by ArdTVal[4]/[5] when the
    // observed run mode is not 3. Freeplay words never feed these fields.
    double mainBoardVolts = 0.0;
    double solenoidVolts = 0.0;
    std::uint64_t supplyVoltsMs = 0;
    enum class VoltageSource : std::uint8_t { None, Selector6, NonFreeplayAdc };
    VoltageSource voltageSource = VoltageSource::None;
    int voltageRunMode = -1;

    // Mode-dependent words are kept in two separate records, each tagged with
    // the run mode used to interpret it. Run-mode-3 freeplay words cannot
    // overwrite supply/ADC sensor records.
    FreeplayTelemetry freeplay;
    std::uint64_t freeplayMs = 0;
    int freeplayRunMode = -1;
    NonFreeplayTelemetry boardAdc;
    std::uint64_t boardAdcMs = 0;
    int boardAdcRunMode = -1;
};

struct ConfigStatus {
    bool configured = false;
    bool configuring = false;
    int attempts = 0;                    // full transactions started
    int framesSent = 0;                  // frames in the current transaction
    int framesExpected = kConfigFrameCount;
    int parametersExpected = kConfigParameterCount;  // ack value (56 + end)
    int parametersTransmitted = kConfigParametersTransmitted;
    int lastAckCount = -1;
    int wrongAckCount = 0;
    std::uint64_t lastAckMs = 0;
    std::uint64_t transactionStartedMs = 0;
};

struct PowerStatus {
    bool acRequested = false;
    bool motorDisabled = true;
    int framesSent = 0;
    std::uint64_t lastFrameMs = 0;
    bool mainBoardObserved = false;
    double lastMainBoardVolts = 0.0;
};

struct RampStatus {
    bool active = false;
    int phase = 0;  // 0 = base/dock, 1 = neutral/zero
    std::array<int, 6> start{};
    std::array<int, 6> goal{};
    std::array<int, 6> current{};
    std::uint64_t startMs = 0;
    std::uint64_t durationMs = 0;   // nominal source duration
    std::uint64_t timeoutMs = 0;    // nominal + accumulated-tail allowance
    int maxError = 0;
    bool settled = false;
    // True once every axis accumulator has reached its endpoint and the exact
    // goal has been commanded. Observed settle is required after this point.
    bool endpointReached = false;
};

struct LifecycleStatus {
    std::string stage;             // human-readable current internal stage
    std::uint64_t stageStartedMs = 0;
    int attempts = 0;
    int pressureAimPsi = 0;
    double lastMainBoardVolts = 0.0;
    bool powerOnObserved = false;
    bool supplyObserved = false;
    bool pressureObserved = false;
};

struct TxCounters {
    std::uint64_t command1 = 0;
    std::uint64_t command2 = 0;
    std::uint64_t command3 = 0;
    std::uint64_t command4 = 0;
    std::uint64_t command5 = 0;
    std::uint64_t command6 = 0;
    std::uint64_t total = 0;
    std::uint64_t writeFailures = 0;
    // Shortfall of the current target chain: deadlines passed since it started
    // minus frames sent. Reset when a chain starts. Covers stalls and a render
    // rate below 60 Hz (sustained, not only single stalls). Never burst-sent.
    std::uint64_t missedCadenceSlots = 0;
    std::uint8_t lastSequence = 0;
};

struct Diagnostics {
    StopReason reason = StopReason::None;
    std::string detail;
    bool identityMismatch = false;
    bool pairingMismatch = false;
    bool configRejected = false;
    bool configLost = false;
    bool calibrationRequired = false;
    bool neutralTargetsRequired = false;
    bool pressureAimRequired = false;
    std::vector<std::string> missing;  // named missing controller parameters
};

// ---------------------------------------------------------------------------
// Session
// ---------------------------------------------------------------------------

class Session {
public:
    Session();
    explicit Session(const SessionConfig& config);
    ~Session();

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    const SessionConfig& config() const { return config_; }

    // Attaches a link without taking ownership. Attaching always starts a new
    // host-side session: any prior controller-latch latch bit is forgotten,
    // because the recovered host also only clears it through a new discovery
    // after a process restart. The controller latch itself is never "cleared"
    // by a frame from this class.
    void attach(Link* link);
    void detach();
    Link* link() const { return link_; }

    // Validated inputs. Each setter rejects structurally invalid data; missing
    // optional values are allowed here and reported when a transaction or
    // initialization is attempted. All four setters refuse while a lifecycle
    // stage or motion is active: the sequence consumes these inputs at fixed
    // points (neutral at the init pause, pressure aim at the pressure step,
    // calibration during every ramp) and a mid-sequence edit would make the
    // observed pose meaningless. Callers should set them when idle, not
    // re-apply static inputs every frame while Running/Parking/Initializing.
    // A config or calibration edit invalidates any prior Docked proof (see the
    // state commentary in mp6_session.cpp).
    bool setControllerConfig(const ControllerConfig& config, std::string* why = nullptr);
    bool setCalibration(const Calibration& calibration, std::string* why = nullptr);
    bool setPressurePlan(const PressurePlan& plan, std::string* why = nullptr);
    bool setPidProfile(PidMode mode, const PidProfile& profile,
                       std::string* why = nullptr);
    bool setNeutralSteps(const std::array<std::int32_t, 6>& neutral,
                         std::string* why = nullptr);
    void setExpectedPlatformNumber(int platformNumber);
    void setTarget(const StepTarget& target);

    // Revokes the authority derived from accepted inputs: the configuration ack,
    // neutral readiness, the target snapshot and any observed Parked/Docked pose.
    // Refused while a lifecycle stage or motion is active, so an edit cannot
    // change the goal under a running sequence.
    bool invalidateInputs(std::string* why = nullptr);

    // Interfaces in the style of the recovered host; all are explicit.
    bool request(Intent intent, std::string* why = nullptr);
    bool requestIdentify(std::string* why = nullptr);
    bool requestConfigure(std::string* why = nullptr);
    bool requestInitialize(std::string* why = nullptr);
    // Parked -> Running. Applies the normal PID profile first when it is not the
    // applied mode (ACK pending: still Parked, Running follows the ACK and a fresh
    // readiness observation). Returns true when the transaction was started.
    bool requestStartMotion(std::string* why = nullptr);
    // Cancels motion and any pending lifecycle sequence (profile transaction,
    // vent/power follow-ups, descent). Sends the bounded authenticated stop when
    // motors may be enabled. EmergencyStop remains the stronger, terminal path.
    bool requestStopMotion(std::string* why = nullptr);
    // Refused while a shutdown is pending or in progress; stop motion first.
    bool requestPark(std::string* why = nullptr);
    // Orderly shutdown. From Docked it re-verifies calibration, fresh
    // feedback and the observed pose at the current calibration dock before
    // venting; a historical Docked state alone is never sufficient. From
    // Parked/Stopped/Running it descends first and vents only after the same
    // observed settle.
    // Parked/Stopped/Running: stop (Running only), lifecycle PID profile ACK, the
    // source 500 ms pause, then the descent; Docked: profile then vent. Check
    // shutdownPending() for progress; Stop cancels it at any phase.
    bool requestShutdown(std::string* why = nullptr);
    // Emergency frames are the recovered stop set (run mode 1, two zero-rate
    // stop frames, three motor-disable frames), sent immediately and without
    // cadence. An in-flight configuration transaction is abandoned first so
    // the stop frames are not gated. When identity was never accepted (or the
    // list is fault-latched) no frame may legally be sent; the session still
    // enters SafeOff and Diagnostics::detail records "0 of 6 frames written"
    // instead of claiming the motors stopped.
    bool requestEmergencyStop(std::string* why = nullptr);

    // Deliberate, opt-in port of the PEP word-1 override. It is refused unless
    // SessionConfig::allowPepSpecialTargets is true, and only emits the special
    // word for the requested number of command-2 frames. Ordinary targets that
    // collide with word 1 (-32766) are always rejected by the codec.
    bool armPepSpecialTarget(int axis, int cycles, std::string* why = nullptr);

    // Deterministic driver. No sleeps; all deadlines are relative to nowMs.
    void tick(std::uint64_t nowMs);

    State state() const { return state_; }
    StopReason stopReason() const { return diagnostics_.reason; }
    const Diagnostics& diagnostics() const { return diagnostics_; }
    const Telemetry& telemetry() const { return telemetry_; }
    const ConfigStatus& configStatus() const { return configStatus_; }
    const PowerStatus& powerStatus() const { return power_; }
    const RampStatus& rampStatus() const { return ramp_; }
    const LifecycleStatus& lifecycleStatus() const { return lifecycle_; }
    const TxCounters& txCounters() const { return tx_; }
    const StepTarget& target() const { return target_; }
    // Last PID mode acknowledged by the controller; empty while a profile
    // transaction is in flight or after a failed one.
    std::optional<PidMode> appliedPidMode() const { return appliedPidMode_; }
    // Frames sent per second since the current target chain started (0 before).
    double achievedTargetHz() const;
    // True when the constructor normalised a requested cadence to 60 Hz.
    bool cadenceNormalised() const { return cadenceNormalised_; }

    std::uint64_t nowMs() const { return now_; }
    std::uint64_t lastTxMs() const { return lastTxMs_; }
    std::uint64_t lastRxMs() const { return lastRxMs_; }

    bool configured() const { return configStatus_.configured; }
    bool identityAccepted() const { return identityAccepted_; }
    bool controllerFaultLatched() const { return faultLatched_; }
    // True while a shutdown is still proceeding (profile, pause, descent, vent or
    // power-off). A Stopped state with this true is a stop that has not ended the
    // shutdown: a Stop request cancels it, a fault or write failure ends it.
    bool shutdownPending() const {
        return stage_ != Stage::None || pidResume_ != PidResume::None ||
               shutdownAfterPark_;
    }

    // Freshness helpers used by the app UI and by the guards.
    bool feedbackFresh() const;
    bool supplyFresh() const;
    bool pressureFresh() const;
    bool statusFresh() const;
    bool targetFresh() const;

private:
    enum class Stage : std::uint8_t {
        None = 0,
        InitSetRunMode,
        InitPowerBurst,
        InitPowerObserve,
        InitSettle,
        InitPressure,
        InitRampDock,
        InitRampPause,
        InitRampNeutral,
        InitComplete,
        ParkRampNeutral,
        ShutdownVent,
        ShutdownVentWait,
        ShutdownPowerOff,
        ShutdownPause,  // source 0.5 s pause after the lifecycle profile ACK
        PidApply,  // lifecycle profile transaction in flight; see PidResume
    };

    // Where a lifecycle PID transaction returns control once it is acked.
    enum class PidResume : std::uint8_t {
        None = 0,
        InitPower,      // Initializing: continue with motor power
        ShutdownPause,  // Stopped/Parked: lifecycle profile ACK, then pause and descend
        ShutdownVent,   // Docked: lifecycle profile applied, vent may start
        StartMotion,    // Parked: normal profile ACK, then Running after readiness
    };

    // Inputs
    SessionConfig config_;
    ControllerConfig controllerConfig_;
    std::array<ConfigFrame, kConfigFrameCount> configFrames_{};
    bool configFramesReady_ = false;
    Calibration calibration_;
    PressurePlan pressurePlan_;
    std::array<std::int32_t, 6> neutralSteps_{};
    bool neutralValid_ = false;
    std::array<std::optional<PidProfile>, 3> pidProfiles_{};  // by PidMode
    std::optional<PidMode> appliedPidMode_;  // last mode acked by the controller
    int expectedPlatformNumber_ = 0;

    // Link and RX
    Link* link_ = nullptr;
    Parser parser_;
    State state_ = State::Closed;
    Diagnostics diagnostics_;
    Telemetry telemetry_;
    ConfigStatus configStatus_;
    PowerStatus power_;
    RampStatus ramp_;
    LifecycleStatus lifecycle_;
    TxCounters tx_;
    SequenceCounter sequence_;

    // Time
    std::uint64_t now_ = 0;
    std::uint64_t lastTxMs_ = 0;
    std::uint64_t lastRxMs_ = 0;
    std::uint64_t lastIdlePollMs_ = 0;
    std::uint64_t lastType48ProcessedMs_ = 0;
    std::uint64_t lastPressureSentMs_ = 0;

    // Host-commanded run mode. The recovered host interprets ArdTVal[4..8]
    // with Module2.RunMode (the value last sent with command 6), not with the
    // controller-reported selector-8 copy.
    int commandedRunMode_ = 1;
    int motValFinal_ = -1;
    int stepRev_ = 0;

    // Identity
    bool identityAccepted_ = false;
    bool faultLatched_ = false;
    int identifyProbes_ = 0;
    std::uint64_t nextProbeMs_ = 0;
    std::uint64_t probeDeadlineMs_ = 0;
    bool probeInFlight_ = false;
    int identityEchoMismatches_ = 0;

    // Pairing
    bool pairingObserved_ = false;
    bool pairingMatched_ = false;

    // Configuration
    bool configAckOk_ = false;
    int configFrameIndex_ = 0;
    int configAttempts_ = 0;
    std::uint64_t nextConfigFrameMs_ = 0;
    std::uint64_t configAckDeadlineMs_ = 0;
    bool configTransactionActive_ = false;
    std::uint64_t configTransactionStartMs_ = 0;
    // Lifecycle PID transaction (PidApply). While pidApplyPending_ is set the
    // command-5 transaction is part of a lifecycle sequence and returns to
    // pidResumeState_ instead of Configured.
    bool pidApplyPending_ = false;
    bool pidApplyOk_ = false;
    PidMode pidApplyMode_ = PidMode::Normal;
    PidResume pidResume_ = PidResume::None;
    State pidResumeState_ = State::Closed;

    // Motion
    StepTarget target_;
    std::uint64_t targetReceivedMs_ = 0;
    std::uint64_t nextTargetTxMs_ = 0;  // ceil(targetDueMs_)
    double targetDueMs_ = 0.0;          // fractional 1000/60 ms deadline chain
    std::uint64_t cadenceStartMs_ = 0;  // start of the current target chain
    std::uint64_t cadenceFrames_ = 0;   // frames sent on the current chain
    bool cadenceNormalised_ = false;
    std::optional<std::uint64_t> ventWaitSinceMs_;  // bounded stale-only wait
    bool supplyLossActive_ = false;
    std::uint64_t supplyLossSinceMs_ = 0;
    bool pressureLowActive_ = false;
    std::uint64_t pressureLowSinceMs_ = 0;
    bool configLostActive_ = false;
    std::uint64_t configLostSinceMs_ = 0;
    bool rxStaleHandled_ = false;
    bool writeFailurePending_ = false;
    std::array<int, 6> pepCycles_{};

    // Pressure/stop bursts
    int pressureBurstRemaining_ = 0;
    std::uint64_t nextPressureBurstMs_ = 0;
    std::uint8_t pressureBurstByte_ = 0;
    int stopBurstRemaining_ = 0;
    std::uint64_t nextStopBurstMs_ = 0;
    bool stopBurstOrdinary_ = true;
    bool pressureBurstVent_ = false;  // current burst is the struts vent

    // Power burst
    int powerBurstRemaining_ = 0;
    std::uint64_t nextPowerBurstMs_ = 0;
    bool powerBurstAc_ = true;
    bool powerBurstDisable_ = false;
    int powerObserveAttempts_ = 0;
    bool returnToInitPowerBurst_ = false;

    // Init/park/shutdown staging
    Stage stage_ = Stage::None;
    std::uint64_t stageDeadlineMs_ = 0;
    int pressureAim_ = 0;
    bool parkAfterRamp_ = false;
    bool shutdownAfterPark_ = false;
    int rampPhase_ = 0;
    std::uint64_t settleSinceMs_ = 0;
    // Source-shaped ramp integration: accumulate the per-tick increment
    // (scaled by the post-93% tail factor) instead of sampling the whole
    // remaining error. This reaches the exact endpoint deterministically.
    std::array<double, 6> rampAccum_{};
    std::uint64_t lastRampTickMs_ = 0;

    // A terminal reader failure is sticky for this attachment. The session
    // stops but does not call close()/join() (that may be the reader's thread).
    bool linkTerminalFailure_ = false;

    // Helpers
    void resetSessionForNewLink();
    void processAvailableRx();
    void handleFrame(const RxFrame& frame);
    void handleType42(const RxFrame& frame);
    void handleType43(const RxFrame& frame);
    void handleType48(const RxFrame& frame);

    bool transmit(std::uint8_t command,
                  const std::array<std::uint8_t, kHostPayloadBytes>& payload,
                  bool forcedSequence);
    bool txCommand3();
    bool txRunMode(std::uint8_t mode);
    bool txTargets(const std::array<AxisTarget, 6>& targets);
    bool txPower(bool acRequest, bool motorDisable);
    bool txPneumatic(std::uint8_t value);
    bool txConfigFrame(const ConfigFrame& frame);
    bool txStopAll();

    bool canTransmit(std::uint8_t command) const;
    void noteTx(std::uint8_t command, std::uint8_t sequence);

    void driveIdentify();
    void driveIdlePoll();
    void driveConfiguration();
    void driveMotion();
    void drivePressureBurst();
    void driveStopBurst();
    void drivePowerBurst();
    void driveLifecycle();
    void driveRamp();
    void updateGuards();

    bool startConfiguration(std::uint64_t now);
    void finishConfiguration(bool success, const std::string& detail);
    bool beginPidApply(PidMode mode, PidResume resume, std::string* why);
    void resumeAfterPidApply();
    void abortVent(const std::string& detail);
    void advanceTargetCadence();
    void dropConfigAuthority(const char* detail);
    void resetFollowUps();
    bool enterRunning(std::string* why);
    bool ventTelemetryStaleOnly() const;
    void startCadenceChain();
    bool isPowered() const { return power_.acRequested && !power_.motorDisabled; }
    bool startRamp(int phase, const std::array<int, 6>& goal,
                   const std::string& stageName);
    void startPowerBurst(bool acRequest, bool motorDisable);
    void startPressureBurst(std::uint8_t value);
    void startStopBurst(bool ordinarySequence);
    void startVentBurst();

    bool startInitialization(std::uint64_t now);
    void failInitialization(StopReason reason, const std::string& detail);
    void enterStopped(StopReason reason, const std::string& detail);
    void enterIdentityLost(StopReason reason, const std::string& detail);
    void latchControllerFault(std::uint16_t code);

    std::array<int, 6> currentCalibrated() const;
    bool powerCheck(bool running) const;
    bool pressureAimForInitialization(int* aim, std::string* why) const;
    bool readiness(std::string* why) const;
    bool buildConfigTransaction(PidMode mode, std::string* why);
    bool validateControllerConfig(const ControllerConfig& config,
                                  std::string* why) const;
    bool inputEditable(const char* what, std::string* why) const;
    bool atCalibrationDock(std::string* why) const;
    bool fresh(std::uint64_t stamp, int ttlMs) const;
    void setStage(Stage stage, const char* name, std::uint64_t deadlineMs);
    void clearMotionGuards();
};

}  // namespace mp6
