#pragma once

#include <vector>
#include <string>
#include <array>
#include <cstring>
#include <cmath>
#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <unordered_map>

namespace mp6 {
    class Link;
    class Session;
    struct ControllerConfig;
    struct Calibration;
    struct PressurePlan;
    struct PidProfile;
    struct Mp6PlatformConfig;
    struct Mp6DesignParams;
    struct Mp6GeometrySetup;
    struct Mp6Pose;
    struct StepTarget;
}

// History ring buffer size (seconds × expected Hz)
static const int HISTORY_LEN = 600;   // ~10s at 60fps
static const int SPECTRUM_BINS = 128; // FFT/DFT bins

// Global input history (source-agnostic, feeds time series + spectrogram)
static const int INPUT_HISTORY_LEN = 2048;  // ~10s at 200Hz

// Shared C headers from ESP32 firmware
#include "InverseKinematics.h"
#include "AxisScaling.h"
#include "MotionCueing.h"
#include "plugin_manager.h"

// Forward declaration
class ITransport;   // SerialPort or UdpTransport (see transport.h)

// ── Platform Types ──────────────────────────────────────────────────

enum class PlatformType {
    Stepper,    // AASD-15A / closed-loop stepper with gearbox (main platform)
    PWMServo    // Hobby PWM servos (Mini-6DOF)
};

struct ServoConfig {
    int   center_us[6];     // per-servo center pulse width (µs)
    float pulse_per_rad;    // µs per radian of servo rotation
    int   min_pulse_us;     // minimum pulse width (µs)
    int   max_pulse_us;     // maximum pulse width (µs)
    int   pwm_freq_hz;      // PWM frequency (Hz)
    bool  inverted[6];      // per-servo inversion flag

    void initDefaults() {
        for (int i = 0; i < 6; i++) center_us[i] = 1500;
        pulse_per_rad = 450.0f;
        min_pulse_us = 800;
        max_pulse_us = 2200;
        pwm_freq_hz = 50;
        inverted[0] = true;  inverted[1] = false;
        inverted[2] = true;  inverted[3] = false;
        inverted[4] = true;  inverted[5] = false;
    }
};

// ── Entity Types ────────────────────────────────────────────────────

enum class EntityType { SIL, HIL };

struct PipelineConfig {
    StewartConfig       geometry;       // compact form (backward compat)
    PlatformDef         platform;       // generalized per-actuator form
    AxisScaleConfig     axis_scales;    // workspace limits (internal, hidden from user)
    MotionCueingConfig  mca;
    InputFilterConfig   input_filter;   // pre-MCA signal conditioning
    float               occupant[3];
    int                 bit_depth;

    // Platform type — determines which drive train controls to show
    PlatformType        platform_type;
    ServoConfig         servo;          // PWM servo params (only used when platform_type == PWMServo)

    // Phase D: intensity & gain
    float               intensity;      // global motion intensity 0-100%
    float               axis_gain[6];   // per-axis gain multiplier (default 100%)
    bool                axis_invert[6]; // per-axis sign flip (true = ×-1)

    void initDefaults();
    void rebuildPlatform();             // rebuild PlatformDef from geometry
    bool isStepper() const { return platform_type == PlatformType::Stepper; }
    bool isServo()   const { return platform_type == PlatformType::PWMServo; }
};

struct PipelineState {
    float input_pct[6];         // user-facing: -100% to +100%
    float input_physical[6];    // internal: mm / radians (converted from pct)
    float output_angles[6];     // servo angles (radians)
    float output_angles_deg[6]; // servo angles (degrees)
    float output_steps[6];      // motor steps
    float servo_util[6];        // Phase C: per-servo utilization 0-100%
    float max_util;             // Phase C: max utilization across all servos
    int   valid_mask;
    int   ik_seq;
};

struct TransportState {
    bool  usb_connected;
    char  usb_port[32];
    int   usb_tx_count;
    int   usb_rx_count;
    bool  udp_connected;
    char  udp_ip[32];
    bool  ble_connected;
};

// ── Console Log ─────────────────────────────────────────────────────

struct LogEntry {
    double      timestamp;
    int         entity_id;
    char        source[16];   // "sil", "esp32", "hil", "system"
    char        message[256];
};

// ── 3D Visualization Camera ──────────────────────────────────────────

struct VizCamera {
    float azimuth;      // horizontal orbit angle (radians)
    float elevation;    // vertical orbit angle (radians)
    float distance;     // distance from target (mm), 0 = auto
};

// ── Handshake State Machine ─────────────────────────────────────────

enum class HandshakePhase {
    Idle,               // not connected
    WaitFingerprint,    // sent FINGERPRINT?, awaiting response
    WaitConfig,         // sent CONFIG?, awaiting response
    WaitBits,           // sent BITS?, awaiting response
    WaitBitsConfirm,    // sent BITS:N, awaiting firmware echo confirmation
    Validating,         // all responses in — comparing params
    Ready,              // handshake passed — motion enabled
    Failed              // mismatch or timeout — motion blocked
};

// Parameters reported by the device during handshake
struct DeviceParams {
    // Identity (from FINGERPRINT response)
    char fingerprint[16];
    char fw_version[16];
    char platform_id[16];
    int  proto_ver;

    // Geometry (from CONFIG response)
    float RD, PD, L1, L2, height, theta_r, theta_p;
    float theta_s[6];
    int   servo_center[6];
    float pulse_per_rad;
    bool  config_received;

    // Input (from BITS response)
    int   bit_depth;
    float max_raw;
    bool  bits_received;

    void clear() {
        memset(this, 0, sizeof(*this));
    }
};

// ── Entity ──────────────────────────────────────────────────────────


// MP6 (Motum Mega rotary Stewart)

enum class HilBackend {
    ESP32 = 0,
    MP6 = 1
};

enum class Mp6Lifecycle {
    Disconnected,
    Connecting,
    Identifying,
    Identified,
    Configuring,
    Configured,
    Parked,
    Docked,
    Initializing,
    Running,
    Parking,
    Faulted,
    IdentityLost,
    SafeOff,
    Unknown
};
inline const char* mp6LifecycleToString(Mp6Lifecycle lc) {
    switch (lc) {
        case Mp6Lifecycle::Disconnected: return "Disconnected";
        case Mp6Lifecycle::Connecting:   return "Connecting";
        case Mp6Lifecycle::Identifying:  return "Identifying";
        case Mp6Lifecycle::Identified:   return "Stopped / Identified";
        case Mp6Lifecycle::Configuring:  return "Configuring";
        case Mp6Lifecycle::Configured:   return "Configured";
        case Mp6Lifecycle::Parked:       return "Parked";
        case Mp6Lifecycle::Docked:       return "Docked";
        case Mp6Lifecycle::Initializing: return "Initializing";
        case Mp6Lifecycle::Running:      return "Running";
        case Mp6Lifecycle::Parking:      return "Parking";
        case Mp6Lifecycle::Faulted:      return "Faulted";
        case Mp6Lifecycle::IdentityLost: return "IdentityLost";
        case Mp6Lifecycle::SafeOff:      return "SafeOff";
        default:                         return "Unknown";
    }
}


enum class Mp6Intent {
    None,
    Identify,
    Configure,
    Initialize,
    Start,
    Stop,
    Park,
    Shutdown,
    Estop,
    Disconnect
};

struct Mp6MachineSettings {
    // One bit for each explicitly entered field, in mp6MachineSettingNames order.
    static constexpr uint32_t all_fields = (1u << 16) - 1;
    uint32_t explicit_fields = 0;
    bool  supplied = false;
    int   bench_test = 0;
    int   comms_timeout_ms = 1000;
    int   onboard_shutdown = 1;
    int   mot_val_final = 63;
    float sensor_filter = 0.20f;
    int   history_len = 4;
    float sensor_filter_start = 0.15f;
    int   history_len_start = 3;
    int   deadband_enable = 0;
    float av_filter = 0.0f;
    int   limits_enable = 1;
    int   info_motor = 1;
    int   pid_index = 0;
    bool  pressure_enabled = false;
    bool  pressure_aim_supplied = false;
    int   pressure_aim_psi = 0;
};

inline constexpr const char* mp6MachineSettingNames[] = {
    "bench_test", "comms_timeout_ms", "onboard_shutdown", "mot_val_final",
    "sensor_filter", "history_len", "sensor_filter_start", "history_len_start",
    "deadband_enable", "av_filter", "limits_enable", "info_motor", "pid_index",
    "pressure_enabled", "pressure_aim_supplied", "pressure_aim_psi"
};

struct Mp6DesignState {
    char  design_name[64] = "";
    bool  supplied = false;
    float bot_a_mm = 0.0f;
    float bot_b_mm = 0.0f;
    float top_a_mm = 0.0f;
    float top_b_mm = 0.0f;
    float crank_length_mm = 0.0f;
    float rod_length_mm = 0.0f;
    float off_x_mm = 0.0f;
    float pcdx_mm = 0.0f;
    float pcdy_mm = 0.0f;
    float z_off_mm = 0.0f;
    float angle_offset_deg = 0.0f;
    float base_radius_mm = 0.0f;
    float platform_radius_mm = 0.0f;
    float x_offset_mm = 0.0f;
    float point_zero_x_mm = 0.0f;
    float point_zero_z_mm = 0.0f;
    float base_z_mm[6] = {};
    float top_z_mm[6] = {};
    int   pivot_z_provenance = 0; // 0=kUnreviewed, 1=kFixtureOnly, 2=kReviewed
    bool  pivot_z_pending_review = false;
    bool  point_zero_supplied = false;
    bool  stale_older_config = false;
};

struct Mp6VizState {
    bool   solver_ready = false;
    bool   rod_lines_valid = false;
    bool   axis_ok[6] = {};
    float  base_pivot[6][3] = {};
    float  crank_joint[6][3] = {};
    float  rod_top[6][3] = {};
    float  calc_residual_mm[6] = {};
};

enum class Mp6ReviewAction { MarkPivots, ConfirmPivots, ResetPivots, ConfirmPointZeros };
inline bool mp6CanConfirmPivots(const Mp6DesignState& d) {
    return d.pivot_z_pending_review && d.pivot_z_provenance != 2;
}
inline bool mp6ReviewDesign(Mp6DesignState& d, Mp6ReviewAction action) {
    switch (action) {
    case Mp6ReviewAction::MarkPivots:
        if (d.pivot_z_provenance == 2) return false;
        d.pivot_z_pending_review = true; return true;
    case Mp6ReviewAction::ConfirmPivots:
        if (!mp6CanConfirmPivots(d)) return false;
        d.pivot_z_provenance = 2; d.pivot_z_pending_review = false; return true;
    case Mp6ReviewAction::ResetPivots:
        d.pivot_z_provenance = 0; d.pivot_z_pending_review = false; return true;
    case Mp6ReviewAction::ConfirmPointZeros:
        d.point_zero_supplied = true; return true;
    }
    return false;
}

struct Mp6BackendState {
    Mp6Lifecycle       lifecycle = Mp6Lifecycle::Disconnected;
    Mp6Intent          pending_intent = Mp6Intent::None;
    bool               link_open = false;
    char               link_port[32] = "";
    int                link_baud = 115200;
    bool               auto_connect = false;
    bool               link_read_failed = false;
    double             last_connect_attempt = 0.0;
    int                cmd2_hz = 60; // Evidenced 60 Hz contract

    bool               identity_ok = false;
    bool               paired_config = false;
    char               board_type[16] = "";
    int                platform_number_expected = 1;
    int                platform_number_ob = 0;
    int                runmode_ob = 0;
    bool               ok2run_ob = false;

    bool               sending_config = false;
    bool               config_ready = false;
    bool               config_send_ok = false;
    bool               config_received_ob = false;
    int                config_attempts = 0;
    int                config_count_expected = 22;
    int                config_params_transmitted = 0;
    int                config_params_ack = 0;
    char               config_note[256] = "";

    bool               av_ok = false;
    bool               onboard_control = false;
    bool               feedback_valid = false;
    int16_t            feedback_raw[6] = {};
    int16_t            feedback_cal[6] = {};
    bool               pot_alarm[6] = {};
    bool               supplies_valid = false;
    float              supplies[3] = {};
    bool               pressure_valid = false;
    float              pneu_psi = 0.0f;
    float              pneu_aim_psi = 0.0f;

    bool               platform_config_present = false;
    char               platform_config_text[8192] = "";
    float              max_rpm_entry = 0.0f;

    Mp6MachineSettings controller_settings;
    Mp6DesignState     design;
    Mp6VizState        viz;

    bool               step_target_valid = false;
    int32_t            step_target[6] = {};
    bool               target_pose_valid = false;
    float              target_pose[6] = {};
    bool               feedback_pose_valid = false;
    float              feedback_pose[6] = {};

    uint64_t           tel_seq = 0;
    int                tel_hz_est = 0;
    uint64_t           missed_cadence_slots = 0;
    double             achieved_target_hz = 0.0;
    bool               cadence_normalised = false;
    bool               shutdown_pending = false;
    int                applied_pid_mode = -1;

    bool               ramp_active = false;
    int32_t            ramp_start[6] = {};
    int32_t            ramp_current[6] = {};
    int32_t            ramp_goal[6] = {};
    int                ramp_phase = 0;
    int                ramp_duration_ms = 0;
    int                ramp_timeout_ms = 0;
    bool               ramp_endpoint_reached = false;
    bool               ramp_settled = false;

    bool               fault_latched = false;
    uint16_t           fault_code = 0;
    char               fault_text[128] = "";
    bool               estop_latched = false;

    char               last_error[192] = "";
    char               last_action[64] = "";
    char               lifecycle_note[256] = "";
    char               solver_note[128] = "";
};

using Mp6EntityState = Mp6BackendState;

struct Entity {
    int             id;
    char            name[64];
    EntityType      type;
    bool            enabled;
    PipelineConfig  config;
    PipelineState   state;
    TransportState  transport;
    float           color[4];       // RGBA for UI identification
    bool            show_card;      // Entity card window (3D viz + readout)
    bool            show_settings;
    bool            show_platform;  // Platform Setup window
    bool            show_dynamics;  // Dynamics window
    bool            show_console;   // Per-entity I/O monitor window
    VizCamera       viz_cam;
    float           viz_split_ratio;    // 3D viewport height fraction (0.2–0.9, default 0.55)

    // Rates
    float           rate_ik_hz;
    float           rate_tx_hz;
    float           rate_tel_hz;

    // HIL: transport to the ESP32 — SerialPort (USB) or UdpTransport (network bridge)
    std::shared_ptr<ITransport> serial;  // nullptr for SIL entities
    int             hil_tx_hz;           // target motion packet send rate
    double          hil_last_tx_time;    // last binary packet send time
    int             hil_tel_seq;         // last processed telemetry seq
    bool            hil_tel_active;      // true when ESP32 telemetry is flowing
    float           hil_fk_pose[6];      // FK-solved platform pose [tx,ty,tz,roll,pitch,yaw] in IK frame
    // Telemetry interpolation for smooth viz
    float           hil_tel_prev[6];     // previous telemetry angles
    float           hil_tel_curr[6];     // current telemetry angles
    double          hil_tel_prev_time;   // timestamp of previous telemetry
    double          hil_tel_curr_time;   // timestamp of current telemetry
    int             hil_tel_target_hz;   // requested telemetry rate from ESP32
    int             hil_tick_rate_us;    // ISR tick period sent to firmware (4-100 µs)
    char            hil_port[32];        // selected COM port name
    int             hil_baud;            // serial baud rate (saved per-device)
    bool            hil_auto_connect;    // try to reconnect if disconnected
    double          hil_last_reconnect;  // last reconnect attempt time
    uint32_t        hil_tx_raw[6];       // latest baked packet for background TX thread (up to 18-bit)
    float           hil_tx_raw_f[6];     // latest RAW (pre-cueing) input_pct snapshot for CH_DATA_RAW

    // Network HIL transport (Voron bridge). When hil_network, `serial` is a UdpTransport.
    bool            hil_network;         // true = network bridge transport, false = USB serial
    char            hil_host[64];        // bridge host (IP or hostname)
    int             hil_udp_port;        // bridge UDP motion port (default 8767)
    int             hil_tcp_port;        // bridge line-JSON TCP control port (default 8789)

    // Raw-HIL streaming (CH_DATA_RAW). Gated on device capability + user toggle.
    bool            hil_cap_raw;         // device advertises raw-HIL (fingerprint caps=raw / network)
    bool            hil_raw_mode;        // user: stream pre-cueing raw telemetry (ESP cues) vs baked

    // Cue auto-sync (raw-HIL): snapshot of the cue params last pushed to the
    // device, so ANY change path (UI slider, control API, preset) syncs live.
    // 46 floats: intensity + gain[6] + invert[6] + (chgain,hpfc,lpfc)*6 + tilt*3.
    float           cue_pushed[46];      // last pushed to device
    float           cue_prev[46];        // previous frame's snapshot (debounce)
    double          cue_change_time;     // frame_time of last observed change
    bool            cue_sync_valid;      // snapshots initialized
    double          cue_push_time;       // frame_time of last push (auto-persist timer)
    bool            cue_persisted;       // MCA:SAVE queued since last push

    // Device fingerprint / handshake
    char            hil_fingerprint[16]; // stored MAC fingerprint (12 hex chars + NUL)
    char            hil_fw_version[16];  // firmware version string from device
    int             hil_proto_ver;       // protocol version from device
    bool            hil_handshake_ok;    // true after full handshake passed — motion enabled
    bool            hil_handshake_pending; // waiting for any handshake response

    // Multi-step handshake state
    HandshakePhase  hil_handshake_phase; // current phase of handshake sequence
    DeviceParams    hil_device_params;   // parameters reported by device during handshake
    double          hil_handshake_start; // frame_time when handshake began (for timeout)
    char            hil_handshake_msg[128]; // status message for UI display
    char            hil_last_error[192];    // last connection error (persists across disconnect for UI)
    int             hil_hs_attempts;        // FINGERPRINT? send count (reset each connect)
    double          hil_hs_last_send;       // time of last FINGERPRINT? send
    bool            hil_geo_synced;         // true if app geometry matches device after handshake
    std::vector<std::string> hil_cmd_queue; // queued serial commands (sent one per frame)

    // Time-series history ring buffers (for comparison charts)
    float           history_angles[6][HISTORY_LEN];   // servo angles over time (deg)
    float           history_input[6][HISTORY_LEN];     // scaled input over time
    float           history_time[HISTORY_LEN];         // timestamps (seconds)
    int             history_head;                       // write index
    int             history_count;                      // how many valid samples

    // Placeholder: frequency spectrum per axis (not wired yet)
    float           spectrum[6][SPECTRUM_BINS];         // magnitude bins
    float           spectrum_freq_max;                  // max frequency (Hz)

    // Dynamics apply S-curve transition (prevents jarring jumps when settings change mid-motion)
    float           last_scaled_pct[6];                 // last pipeline scaled_pct (updated each frame)
    bool            dyn_transition_active;              // true = crossfade in progress
    double          dyn_transition_start;               // frame_time when transition began
    float           dyn_transition_duration;            // seconds (default 1.5)
    float           dyn_transition_from[6];             // snapshot of scaled_pct before apply

    void            pushHistory(double t);              // record current state
    // MP6 (Motum Mega raw serial backend)
    HilBackend      hil_backend = HilBackend::ESP32;
    Mp6BackendState mp6;
};

// ── Recording / Playback ───────────────────────────────────────────

enum class RecordMode { Idle, Recording };

struct RecordSample {
    double time;        // seconds since record start
    float  input[6];    // 6-DOF input values at this moment
};

struct RecordingState {
    RecordMode                  mode;
    std::vector<RecordSample>   samples;
    double                      start_time;     // wall time when record started
    char                        name[64];       // recording label

    // First-order hold: interpolate between previous and current frame input
    float                       prev_input[6];  // input at previous frame
    float                       curr_input[6];  // input at current frame
    double                      prev_time;      // frame_time of previous update
    double                      curr_time;      // frame_time of current update

    RecordingState() : mode(RecordMode::Idle), start_time(0), prev_time(0), curr_time(0) {
        snprintf(name, sizeof(name), "Recording");
        memset(prev_input, 0, sizeof(prev_input));
        memset(curr_input, 0, sizeof(curr_input));
    }

    double duration() const {
        if (samples.empty()) return 0.0;
        return samples.back().time;
    }
};

// ── Versioned Recording File Format ────────────────────────────────

#define RECORDING_MAGIC   0x53545752   // "STWR"
#define RECORDING_VERSION 1

struct RecordingFileHeader {
    uint32_t magic;              // RECORDING_MAGIC
    uint32_t version;            // format version
    double   sample_rate_hz;     // recording sample rate
    double   duration_sec;       // total duration
    int32_t  sample_count;       // number of RecordSample entries
    int32_t  channels;           // number of channels (6)
    double   created_time;       // unix timestamp when recorded
    char     name[64];           // recording name
    char     source[32];         // input source ("manual", "udp", "capture")
    uint8_t  reserved[128];      // future use — zero-filled
};

// ── Saved Recording Library ────────────────────────────────────────

struct SavedRecording {
    char                        name[64];
    double                      sample_rate_hz;
    double                      created_time;   // unix timestamp
    char                        source[32];
    std::vector<RecordSample>   samples;
    double duration() const {
        if (samples.empty()) return 0.0;
        return samples.back().time;
    }
};

// ── Input Source ────────────────────────────────────────────────────

enum class InputSource { CapturePlayback = 1, Plugin = 2 };


// ── Application State ───────────────────────────────────────────────

struct App {
    std::vector<Entity>   entities;
    int                   next_entity_id;

    // Shared input (from SimTools or manual override)
    float                 shared_input[6];
    std::mutex            input_mutex;       // protects shared_input when UDP writes
    InputSource           input_source;

    // Source-switch ramp-to-home state machine
    bool                  source_switch_active;     // true = ramp-out in progress
    InputSource           source_switch_target;     // source to switch to after ramp completes
    double                source_switch_start;      // frame_time when ramp started
    float                 source_switch_from[6];    // snapshot of input at ramp start
    static constexpr float SOURCE_RAMP_OUT_S = 1.0f;  // seconds to ramp to home
    void                  requestSourceSwitch(InputSource target);  // initiate ramp-to-home then switch


    // Plugin system
    PluginManager         plugin_mgr;
    int                   active_plugin_idx;  // index into plugin_mgr.plugins(), -1 = none

    // Dynamics panel — persisted entity selection and visibility
    int                   selected_dynamics_id = -1;  // entity ID for Dynamics panel, -1 = auto

    // Panel visibility — persisted per workspace
    bool                  show_dynamics     = true;
    bool                  show_console      = true;
    bool                  show_data_streams = true;

    // Console
    std::vector<LogEntry> console_log;
    std::vector<LogEntry> console_log_frozen;   // snapshot when paused
    int                   console_max;
    bool                  console_auto_scroll;
    bool                  console_paused;        // freeze log display
    int                   console_filter;       // -1 = all, else entity_id

    // Global
    bool                  running;
    bool                  motion_started;       // true = motion output active, false = platform at home
    bool                  start_ramp_active;    // S-curve ramp from home to current input on START
    double                start_ramp_begin;     // frame_time when ramp started
    float                 start_ramp_target[6]; // target input_pct at ramp start
    static constexpr float START_RAMP_S = 0.6f; // ramp duration in seconds
    double                frame_time;
    double                fps;
    int                   frame_count;

    // Recording / Playback
    RecordingState        recording;

    // Saved recordings library
    std::vector<SavedRecording> saved_recordings;
    int                   capture_playback_idx;   // index into saved_recordings (-1 = none)
    bool                  capture_playing;         // is capture playback active?
    int                   capture_play_cursor;
    double                capture_start_time;
    bool                  capture_loop;
    float                 capture_speed;           // playback speed multiplier (0.1 = 10%, 2.0 = 200%)

    // Capture ramp state machine (prevents jarring starts/stops/loops)
    enum class CaptureRampPhase { RampIn, Playing, RampOut, HomeHold };
    CaptureRampPhase      capture_ramp_phase;
    double                capture_ramp_start;      // frame_time when current ramp phase started
    bool                  capture_stop_requested;  // true = ramp out then stop (don't loop)
    float                 capture_last_vals[6];    // last data values before ramp-out (for smooth blend)
    static constexpr float CAPTURE_RAMP_IN_S  = 2.0f;
    static constexpr float CAPTURE_RAMP_OUT_S = 2.0f;
    static constexpr float CAPTURE_HOME_HOLD_S = 1.0f;

    // Configurable record/playback rate
    int                   record_rate_hz;          // default 200


    // MCA dynamics presets (user-saved, persisted to disk)
    struct McaDynamicsPreset {
        char                name[64];
        bool                is_builtin;       // true = from PRESET_TABLE, false = user-created
        MotionCueingConfig  mca;              // full MCA config snapshot
        float               intensity;        // global intensity
        float               axis_gain[6];     // per-axis gain trim
    };
    std::vector<McaDynamicsPreset> mca_presets;
    void    saveMcaPreset(const char* name, const MotionCueingConfig& mca, float intensity, const float axis_gain[6]);
    void    deleteMcaPreset(int idx);
    void    loadMcaPreset(int idx, MotionCueingConfig& mca, float& intensity, float axis_gain[6]);
    void    saveMcaPresetsToDisk();
    void    loadMcaPresetsFromDisk();

    // Global input history (source-agnostic ring buffer for time series & spectrogram)
    float                 input_history[6][INPUT_HISTORY_LEN];
    float                 input_history_time[INPUT_HISTORY_LEN];
    int                   input_history_head;
    int                   input_history_count;
    double                input_history_last_push;  // timestamp of last push

    // Input spectrum (DFT of input_history per axis)
    float                 input_spectrum[6][SPECTRUM_BINS];
    float                 input_spectrum_freq_max;

    // Console input logging rate (user-controlled)
    double                last_input_log_time;
    int                   console_log_rate;    // 0=off, 1=1Hz, 2=10Hz, 3=30Hz, 4=60Hz, 5=every frame

    App();
    ~App();

    // MP6 runtime state (main update thread only)
    struct Mp6Runtime;
    std::unordered_map<int, std::unique_ptr<Mp6Runtime>> mp6_runtimes;

    Entity& addEntity(const char* name, EntityType type, HilBackend backend = HilBackend::ESP32);
    bool    mp6Connect(Entity& e, std::string* error);
    bool    mp6ConnectWithLink(Entity& e, std::unique_ptr<mp6::Link> link, std::string* error);
    void    mp6Disconnect(Entity& e);
    bool    mp6Command(Entity& e, Mp6Intent intent, std::string* error);
    bool    mp6ImportPlatformConfigText(Entity& e, const char* text, std::string* error);
    bool    mp6RefreshDesignGeometry(Entity& e, std::string* error);
    bool    mp6AcceptDesign(Entity& e, const Mp6DesignState& draft, std::string* error);
    bool    mp6RecoverKnownState(Entity& e, std::string* error);
    bool    mp6SetControllerSettings(Entity& e, const Mp6MachineSettings& settings, std::string* error);
    bool    mp6ValidateMachineSettings(const Mp6MachineSettings& settings, std::string* error);
    bool    mp6IsMotorPowerRequested(const Entity& e) const;
    bool    mp6InvalidateAuthority(Entity& e, bool invalidate_config, bool invalidate_geometry);
    void    mp6UpdateEntity(Entity& e);
    void    startMotion();
    void    stopMotion();
    void    requestEstop();
    void    removeEntity(int id);
    Entity* findEntity(int id);
    void    log(int entity_id, const char* source, const char* fmt, ...);
    void    handleHilLine(int entity_id, const char* line);  // parse ESP32 serial responses
    void    update();   // called every frame


    // Recording
    void    startRecording();
    void    stopRecording();
    // Capture library
    void    saveRecordingToLibrary(const char* name);
    void    deleteRecordingFromLibrary(int idx);
    void    startCapturePlayback(int idx);
    void    stopCapturePlayback();
    void    updateCapturePlayback();
    void    saveRecordingsToDisk();
    void    loadRecordingsFromDisk();

    // Settings persistence (auto-save on change)
    void    saveSettings();
    void    loadSettings();
    bool    settings_dirty;    // set true when any setting changes, checked each frame

    // Push the entity's cue settings (intensity/gain/invert/washout/tilt) to a
    // raw-HIL device as MCA:* commands so device feel tracks app tuning live.
    void    pushCueSettingsToDevice(Entity& e);
    // Per-frame change watcher: auto-push when any cue param changes (debounced).
    void    syncCueSettings(Entity& e);

    // Background HIL TX thread (runs independently of UI frame rate)
    std::thread       hil_tx_thread;
    std::atomic<bool> hil_tx_stop{false};
    void              hilTxLoop();
};

// Global app instance
extern App g_app;
