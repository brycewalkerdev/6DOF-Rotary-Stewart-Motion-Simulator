#include "app.h"
#include "automation.h"
#include "serial_port.h"
#include "dev_log.h"
#include "mp6_link.h"
#include "mp6_session.h"
#include "mp6_config.h"
#include "mp6_geometry.h"
#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <chrono>
#include <algorithm>
#include "cJSON.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket close
typedef int SOCKET;
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Global instance
App g_app;

// ── MP6 session runtime (app-owned, never persisted) ────────────────
//
// One live runtime per MP6 entity: the raw Win32 serial link plus the
// single-writer mp6 session engine. All access happens on the main update
// thread; the UI and the control API only stage intents on the entity.
struct App::Mp6Runtime {
    std::unique_ptr<mp6::Link>    link;
    std::unique_ptr<mp6::Session> session;
    std::uint64_t                 target_gen = 0;  // StepTarget generation
    // Inputs derived from the imported .pltc, applied to the session on the
    // main thread only (never from the UI/network thread that imports).
    bool inputs_dirty = false;
    mp6::ControllerConfig pending_cfg;
    mp6::Calibration pending_cal;
    bool pending_cal_valid = false;
    mp6::PressurePlan pending_pressure;
    bool pending_pressure_valid = false;
    mp6::PidProfile pending_pid_configure;
    mp6::PidProfile pending_pid_lifecycle;
    mp6::PidProfile pending_pid_normal;
    bool pending_pid_valid = false;
    std::array<std::int32_t, 6> neutral_steps{};
    std::array<double, 6> neutral_crank_rad{};
    bool neutral_valid = false;
    bool neutral_dirty = false;
    bool start_blend_pending = false;
    bool start_blend_active = false;
    double start_blend_begin = 0.0;
    std::array<std::int32_t, 6> start_blend_start{};
    bool axis_test_returning = false;
    bool axis_test_entering = false;
    bool axis_test_entry_waiting_tx = false;
    std::array<std::int32_t, 6> axis_test_entry_steps{};
    std::array<double, 6> axis_test_entry_position{};
    std::array<std::int32_t, 6> axis_test_last_sent_steps{};
    bool axis_test_last_sent_valid = false;
    double axis_test_last_sent_time = 0.0;
    bool axis_test_return_waiting_tx = false;
    double axis_test_last_time = 0.0;
    // Typed parse of the stored pltc, kept for the crank-geometry rebuild
    // (rod/crank overrides, travel limits, angle_to_step calibration).
    mp6::Mp6PlatformConfig parsed_cfg;
    // Crank-geometry staging: rebuilt on the main thread from the design
    // inputs + parsed pltc (never pre-filled from a reference rig).
    bool        geometry_dirty = false;
    mp6::Mp6DesignParams design;                 // table numbers as entered
    mp6::Mp6GeometrySetup geometry;              // pivots/attachments (z explicit)
    double      min_rad = 0.0, max_rad = 0.0;
    double      angle_zero_rad[6] = {};
    double      angle_to_step[6] = {};
    double      x_offset_mm = 0.0, z_offset_mm = 0.0;
    mp6::Mp6CrankSolver solver;
    bool        solver_ready = false;
    bool        solver_fresh = false;            // a solve produced results
    mp6::Mp6Pose staged_pose;                    // last pipeline pose (see below)
    bool        staged_pose_valid = false;
    // Telemetry/TX rate estimation state (app-side bookkeeping only).
    std::uint64_t tel_frames_prev = 0;
    std::uint64_t tx_total_prev = 0;
    double        rate_sample_time = 0.0;
};

// App-side view of a session state; a rather coarser set than mp6::State with
// honest "nothing known" whenever the app cannot claim a controller fact.
static Mp6Lifecycle mp6MapLifecycle(mp6::State st) {
    switch (st) {
        case mp6::State::Closed:       return Mp6Lifecycle::Disconnected;
        case mp6::State::Opened:       return Mp6Lifecycle::Connecting;
        case mp6::State::Identifying:  return Mp6Lifecycle::Identifying;
        case mp6::State::Identified:   return Mp6Lifecycle::Identified;
        case mp6::State::Configuring:  return Mp6Lifecycle::Configuring;
        case mp6::State::Configured:   return Mp6Lifecycle::Configured;
        case mp6::State::Parked:       return Mp6Lifecycle::Parked;
        // Docked: observed settle at the calibration dock (park/shutdown
        // destination). Source-backed distinct state — never relabelled as
        // neutral Parked or as a readiness state.
        case mp6::State::Docked:       return Mp6Lifecycle::Docked;
        case mp6::State::Initializing: return Mp6Lifecycle::Initializing;
        case mp6::State::Running:      return Mp6Lifecycle::Running;
        case mp6::State::Parking:      return Mp6Lifecycle::Parking;
        case mp6::State::Faulted:      return Mp6Lifecycle::Faulted;
        case mp6::State::IdentityLost: return Mp6Lifecycle::IdentityLost;
        // Stopped: the controller is still identified but not streaming;
        // report Identified with the reason in the note — no invented alias.
        case mp6::State::Stopped:
            return Mp6Lifecycle::Identified;
        // SafeOff: motors disabled. It is a distinct source-backed state, not
        // a neutral idle flavour of "Identified": nothing resumes from here
        // without a full reconnect + fresh Identify (session review F6).
        case mp6::State::SafeOff:
            return Mp6Lifecycle::SafeOff;
    }
    return Mp6Lifecycle::Unknown;
}

// ── .pltc → session inputs (mp6_config integration) ─────────────────
//
// The app-side translation is deliberately limited to associations whose
// names agree on both sides (the session's ControllerConfig field names carry
// the wire scale: pScale*1000, pOffset*10, gearbox*100, AngLo*10, AccelP1*100
// ...). Fields the snapshot cannot supply (BenchTest, CommsTimeoutMS,
// OnBoardShutdown, sensor filter, history length, MotValFinal, step
// correction, PID rows for params 22/23, deadband row mapping, AVFilter,
// LimitsEnable, InfoMotor) are left unset — the session collects and reports
// them by name when a configuration transaction is attempted. No value is
// invented anywhere; nothing from the recovered reference rig is copied in.
static int mp6WireRound(double v) {
    return (int)mp6::mp6_net_round(v);
}

// Parity with frm_Main.cs:31570 txtStErrCorr.Text = StepErrCorr.ToString("F3")
// and frm_Main.cs:31844 (int)Math.Round(Val(txtStErrCorr.Text) * 10000.0)
static int mp6QuantizeStepCorrection(float sec) {
    // Framework Single formatting first makes a seven-significant-digit
    // decimal NumberBuffer, then rounds fixed decimals away from zero.
    // Integer decimal arithmetic avoids treating the promoted binary float as F3.
    char buf[40];
    snprintf(buf, sizeof(buf), "%.6e", std::fabs(static_cast<double>(sec)));
    long long digits = 0;
    const char* c = buf;
    for (; *c && *c != 'e'; ++c)
        if (*c >= '0' && *c <= '9') digits = digits * 10 + (*c - '0');
    int exponent = std::atoi(c + 1) - 3; // seven digits -> thousandths
    if (exponent >= 0) {
        for (int i = 0; i < exponent; ++i) digits *= 10;
    } else {
        long long divisor = 1;
        for (int i = 0; i < -exponent && i < 18; ++i) divisor *= 10;
        digits = -exponent > 18 ? 0 : (digits + divisor / 2) / divisor;
    }
    return static_cast<int>((sec < 0 ? -digits : digits) * 10);
}

// Parity with Module2 float intermediate and (int)Math.Round()
static int mp6QuantizeFloatParam(float val) {
    return mp6::roundHalfEvenToInt(static_cast<double>(val));
}

static int mp6QuantizeFloatProduct(float value, float scale) {
    return mp6QuantizeFloatParam(value * scale);
}

static bool mp6BuildSessionInputs(const mp6::Mp6PlatformConfig& cfg,
                                  const Mp6MachineSettings& ms,
                                  mp6::ControllerConfig& cc,
                                  mp6::Calibration& cal,
                                  mp6::PressurePlan& plan,
                                  mp6::PidProfile& pid_configure,
                                  mp6::PidProfile& pid_lifecycle,
                                  mp6::PidProfile& pid_normal,
                                  std::string* why) {
    auto invalid = [&](const char* field) {
        if (why) *why = std::string("missing or out-of-range consumed input: ") + field;
        return false;
    };
    for (int i = 0; i < 3; ++i) {
        const auto& pid = cfg.pid[i];
        if (!pid.resolved) return invalid("PID profile");
        for (double value : {pid.p, pid.d, pid.integral, pid.dfilter, pid.tconst})
            if (!std::isfinite(value) || std::abs(value) > 1000000) return invalid("PID coefficient");
        if (!cfg.step_err_corr[i].present || std::abs(cfg.step_err_corr[i].value) > 1000)
            return invalid("Step Correction");
    }
    for (const auto* value : {&cfg.step_rev, &cfg.rotate_direction, &cfg.accel_scale,
                             &cfg.accel_p1, &cfg.accel_p0, &cfg.decel_scale,
                             &cfg.deadband_pid, &cfg.stopcount_limit})
        if (!value->present || !std::isfinite(value->value) || std::abs(value->value) > 1000000)
            return invalid("controller scalar");
    for (const auto& axis : cfg.axis)
        for (double value : {axis.pot_lo, axis.angle_docked_deg, axis.pot_scale, axis.gearbox})
            if (!std::isfinite(value) || std::abs(value) > 1000000) return invalid("Pot calibration");
    mp6::Mp6PlatformConfig normalized_cfg = cfg;
    if (normalized_cfg.step_rev.present)
        normalized_cfg.step_rev.value = mp6::mp6_normalize_step_rev(cfg.step_rev.value);
    for (int i = 0; i < mp6::kAxes; ++i)
        if (normalized_cfg.axis[i].present[0])
            normalized_cfg.axis[i].pot_lo = mp6::mp6_normalize_pot_lo(cfg.axis[i].pot_lo);
    mp6::Mp6AxisCalibration ac[mp6::kAxes];
    std::string err;
    if (!mp6::mp6_config_derive_calibration(normalized_cfg, ac, &err)) {
        if (why) *why = err;
        return false;
    }
    for (const auto& axis : ac)
        for (double value : {double(axis.p_scale) * 1000, double(axis.p_offset) * .1,
                             double(axis.ang_lo_deg) * 10, double(axis.docked_steps)})
            if (!std::isfinite(value) || value < -2147483647 || value > 2147483647)
                return invalid("derived calibration");
    if (std::abs(cfg.accel_scale.value * cfg.accel_p1.value * 100.0) > 2147483647)
        return invalid("AccelP1");
    // Fields available from PlatformConfig.pltc
    if (normalized_cfg.step_rev.present) cc.stepRev = static_cast<int>(normalized_cfg.step_rev.value);
    if (cfg.rotate_direction.present)  cc.rotationDirection = mp6WireRound(cfg.rotate_direction.value);
    if (cfg.max_pps != 0)              cc.maxPps = cfg.max_pps;   // derived (L18644: MaxRPM*StepRev/60)

    // Param 40: (int)Math.Round(AccelScale * Conversion.Val(txtAccelP1.Text) * 100.0)
    if (cfg.accel_p1.present) {
        double ascale = cfg.accel_scale.present ? cfg.accel_scale.value : 1.0;
        cc.accelP1Times100 = mp6WireRound(ascale * cfg.accel_p1.value * 100.0);
    }
    // Param 41: (int)Math.Round(Conversion.Val(txtAccelP0.Text) * 0.1)  [Source parity: * 0.1]
    if (cfg.accel_p0.present)          cc.accelP0Times10    = mp6WireRound(cfg.accel_p0.value * 0.1);
    // Param 42: (int)Math.Round(Module2.DecelScale * 100f)
    if (cfg.decel_scale.present)       cc.decelTimes100     = mp6QuantizeFloatProduct(static_cast<float>(cfg.decel_scale.value), 100.0f);
    // Param 28: Conversions.ToInteger(txtDeadbandPID.Text)
    if (cfg.deadband_pid.present)      cc.deadbandValue     = mp6WireRound(cfg.deadband_pid.value);
    // Param 29: Module2.StoppedCountLimit
    if (cfg.stopcount_limit.present)   cc.stoppedCountLimit = mp6WireRound(cfg.stopcount_limit.value);

    // Build the 3 PID profiles matching frm_Main.cs PID_SetValues
    auto build_pid = [&](int typ_mode, float pneu_filt, int pneu_hist) -> mp6::PidProfile {
        mp6::PidProfile prof;
        int pltc_idx = (typ_mode >= 0 && typ_mode < mp6::kPidRows) ? typ_mode : 0;
        if (cfg.pid[pltc_idx].resolved) {
            float p = (float)cfg.pid[pltc_idx].p;
            float d = (float)cfg.pid[pltc_idx].d;
            float df = (float)cfg.pid[pltc_idx].dfilter;
            float tc = (float)cfg.pid[pltc_idx].tconst;
            prof.proportionalTimes10 = mp6::roundHalfEvenToInt(static_cast<double>(p * 10.0f));
            prof.derivativeTimes10   = mp6::roundHalfEvenToInt(static_cast<double>(d * 10.0f));
            prof.dFilter             = mp6QuantizeFloatParam(df);
            prof.tConst              = mp6QuantizeFloatParam(tc);
        }
        if (cfg.step_err_corr[pltc_idx].present) {
            prof.stepCorrTimes10000 = mp6QuantizeStepCorrection((float)cfg.step_err_corr[pltc_idx].value);
        }
        prof.sensorFilterTimes10000 = mp6::roundHalfEvenToInt(static_cast<double>(pneu_filt * 10000.0f));
        prof.historyLen = pneu_hist;
        return prof;
    };

    // Mode 1: Configure (coefficient load) - uses Start pneumatic filter/history
    pid_configure = build_pid(1, ms.sensor_filter_start, ms.history_len_start);
    // Mode 2: Lifecycle (init / shutdown) - uses Start pneumatic filter/history
    pid_lifecycle = build_pid(2, ms.sensor_filter_start, ms.history_len_start);
    // Mode 0: Normal (run / motion) - uses Run pneumatic filter/history
    pid_normal    = build_pid(0, ms.sensor_filter, ms.history_len);

    // Per-axis: potLo is column 0 exactly as recorded. AngLo/pScale/pOffset
    // triple is taken solely from the module's derived calibration struct.
    static_assert(mp6::kAxes == 6, "app assumes 6 MP6 axes");
    for (int i = 0; i < mp6::kAxes; i++) {
        const mp6::Mp6AxisPots& pots = cfg.axis[i];
        if (pots.present[0]) cc.potLo[i] = static_cast<int>(normalized_cfg.axis[i].pot_lo);
        if (pots.present[5]) cc.gearboxTimes100[i] = mp6QuantizeFloatProduct(static_cast<float>(pots.gearbox), 100.0f);
        cc.angLoTimes10[i]    = mp6QuantizeFloatProduct(ac[i].ang_lo_deg, 10.0f);
        cc.pScaleTimes1000[i] = mp6QuantizeFloatProduct(ac[i].p_scale, 1000.0f);
        // Param 131..136: (int)Math.Round((double)Module2.pOffset * 0.1)  [Source parity: * 0.1]
        cc.pOffsetTimes10[i]  = mp6WireRound((double)ac[i].p_offset * 0.1);
        cal.axis[i].pScale       = (double)ac[i].p_scale;
        cal.axis[i].pOffset      = (double)ac[i].p_offset;
        cal.axis[i].dockedSteps  = mp6WireRound(ac[i].docked_steps);
        cal.axis[i].valid        = true;
    }
    cal.valid = true;
    cal.description = "derived from imported PlatformConfig.pltc (Load_Coefficients float parity)";

    // Explicitly supplied machine controller settings (BenchTest, CommsTimeoutMS,
    // OnBoardShutdown, MotValFinal, SensorFilter, HistoryLen, AVFilter, LimitsEnable, InfoMotor, etc.)
    if (ms.supplied) {
        cc.benchTest = ms.bench_test;
        cc.commsTimeoutMs = ms.comms_timeout_ms;
        cc.onboardShutdown = ms.onboard_shutdown;
        cc.motValFinal = ms.mot_val_final;
        cc.sensorFilterTimes10000 = mp6WireRound((double)ms.sensor_filter * 10000.0);
        cc.historyLen = ms.history_len;
        cc.deadbandEnable = ms.deadband_enable;
        cc.avFilterTimes10000 = mp6WireRound((double)ms.av_filter * 10000.0);
        cc.limitsEnable = ms.limits_enable;
        cc.infoMotor = ms.info_motor;

        plan.enabled = ms.pressure_enabled;
        plan.aimSupplied = ms.pressure_aim_supplied;
        plan.aimPsi = ms.pressure_aim_psi;
    }
    return true;
}

static bool mp6SetSnapshot(mp6::Session& session, const App::Mp6Runtime& rt,
                           std::string* why) {
    return session.setControllerConfig(rt.pending_cfg, why)
        && rt.pending_cal_valid && session.setCalibration(rt.pending_cal, why)
        && rt.pending_pid_valid
        && session.setPidProfile(mp6::PidMode::Configure, rt.pending_pid_configure, why)
        && session.setPidProfile(mp6::PidMode::Lifecycle, rt.pending_pid_lifecycle, why)
        && session.setPidProfile(mp6::PidMode::Normal, rt.pending_pid_normal, why)
        && rt.pending_pressure_valid && session.setPressurePlan(rt.pending_pressure, why);
}

// Reparse + derive + stage the stored pltc text. Callers (import, settings
// load, connect) may be off the update thread; this never touches a live
// session — staged inputs are applied in mp6UpdateEntity on the main thread.
static void mp6ApplyPlatformConfig(App& app, Entity& e) {
    app.mp6InvalidateAuthority(e, true, true);
    auto reset_pending = [&]() {
        auto it = app.mp6_runtimes.find(e.id);
        if (it != app.mp6_runtimes.end()) {
            it->second->inputs_dirty = false;
            it->second->pending_cfg = mp6::ControllerConfig{};
            it->second->pending_cal_valid = false;
            it->second->pending_cal = mp6::Calibration{};
        }
    };
    if (!e.mp6.platform_config_present || e.mp6.platform_config_text[0] == '\0') {
        e.mp6.config_ready = false;
        snprintf(e.mp6.config_note, sizeof(e.mp6.config_note), "no platform config imported");
        reset_pending();
        return;
    }
    mp6::Mp6PlatformConfig parsed;
    std::string err;
    if (!mp6::Mp6PlatformConfig::from_pltc(e.mp6.platform_config_text, parsed)) {
        e.mp6.config_ready = false;
        snprintf(e.mp6.config_note, sizeof(e.mp6.config_note),
                 "structural .pltc parse failure: %s", err.c_str());
        app.log(e.id, "mp6", "Platform config parse failed: %s", err.c_str());
        reset_pending();
        return;
    }
    // ── MaxPPS materialization (engine duty of the caller, N3) ──────────
    // The cmd-5 MaxPPS parameter must be materialized as
    // round(Max Motor RPM * Step/Rev / 60) — snapshot 230*3200/60 → 12267 —
    // or sourced explicitly; the engine refuses "derivable but not computed".
    // The .pltc "Max Motor RPM" row is machine data and wins; the explicit
    // MaxRPM entry is used only when the row is absent. The product range is
    // checked BEFORE rounding/integer conversion, exactly like the engine's
    // own plausibility check — absurd inputs leave max_pps = 0 so readiness
    // blocks with the named error instead of silently wrapping an int. No
    // clamping happens later either: the session's param-34 wire range is the
    // real check and its refusal is reported verbatim.
    int  max_pps_materialized = 0;
    char max_pps_src[64] = {};
    {
        bool have_rpm = false;
        double rpm = 0.0;
        if (parsed.max_motor_rpm.present && parsed.max_motor_rpm.value > 0.0) {
            have_rpm = true; rpm = parsed.max_motor_rpm.value;
            snprintf(max_pps_src, sizeof(max_pps_src), "pltc Max Motor RPM row");
        } else if (e.mp6.max_rpm_entry > 0.0f) {
            have_rpm = true; rpm = (double)e.mp6.max_rpm_entry;
            snprintf(max_pps_src, sizeof(max_pps_src), "explicit MaxRPM entry");
        }
        if (have_rpm && parsed.step_rev.present &&
            std::isfinite(parsed.step_rev.value) && parsed.step_rev.value > 0.0 &&
            mp6::mp6_net_round(parsed.step_rev.value) >= 1.0 &&
            mp6::mp6_net_round(parsed.step_rev.value) <= INT_MAX) {
            const int step_rev = mp6::mp6_normalize_step_rev(parsed.step_rev.value);
            double exact = rpm * step_rev / 60.0;
            if (std::isfinite(exact) && exact >= 1.0 && exact <= 2147483647.0)
                max_pps_materialized = mp6WireRound(exact);
        }
    }
    parsed.max_pps = max_pps_materialized;

    int n_warn = 0, n_err = 0;
    for (const auto& d : parsed.diagnostics) {
        if (d.severity == mp6::ConfigDiagnostic::Severity::kError) n_err++;
        else if (d.severity == mp6::ConfigDiagnostic::Severity::kWarning) n_warn++;
    }
    if (n_err != 0) {
        snprintf(e.mp6.config_note, sizeof(e.mp6.config_note),
                 "platform config has %d parse errors; Configure blocked", n_err);
        reset_pending();
        return;
    }
    if (!app.mp6ValidateMachineSettings(e.mp6.controller_settings, &err)) {
        snprintf(e.mp6.config_note, sizeof(e.mp6.config_note), "%s", err.c_str());
        reset_pending();
        return;
    }
    mp6::ControllerConfig cc;
    mp6::Calibration cal;
    mp6::PressurePlan plan;
    mp6::PidProfile pid_cfg, pid_life, pid_norm;
    std::string build_err;
    if (!mp6BuildSessionInputs(parsed, e.mp6.controller_settings, cc, cal, plan,
                               pid_cfg, pid_life, pid_norm, &build_err)) {
        app.mp6InvalidateAuthority(e, true, true);
        snprintf(e.mp6.config_note, sizeof(e.mp6.config_note),
                 "calibration derivation failed: %s", build_err.c_str());
        app.log(e.id, "mp6", "Calibration derivation failed: %s", build_err.c_str());
        reset_pending();
        return;
    }
    auto& rt = app.mp6_runtimes[e.id];
    if (!rt) rt = std::make_unique<App::Mp6Runtime>();
    rt->pending_cfg = cc;
    rt->pending_cal = cal;
    rt->pending_cal_valid = true;
    rt->pending_pressure = plan;
    rt->pending_pressure_valid = true;
    rt->pending_pid_configure = pid_cfg;
    rt->pending_pid_lifecycle = pid_life;
    rt->pending_pid_normal    = pid_norm;
    rt->pending_pid_valid     = true;
    rt->parsed_cfg = parsed;
    mp6::Session validator;
    if (!mp6SetSnapshot(validator, *rt, &err)) {
        app.mp6InvalidateAuthority(e, true, true);
        snprintf(e.mp6.config_note, sizeof(e.mp6.config_note), "snapshot rejected: %s", err.c_str());
        reset_pending();
        return;
    }
    rt->inputs_dirty = true;
    rt->geometry_dirty = true;
    e.mp6.config_ready = true;
    if (!max_pps_materialized) {
        snprintf(e.mp6.config_note, sizeof(e.mp6.config_note),
                 "parsed (%d err, %d warn); inputs staged — MaxPPS NOT materialized "
                 "(supply Max Motor RPM); remaining params reported at Configure",
                 n_err, n_warn);
    } else if (n_err > 0 || n_warn > 0)
        snprintf(e.mp6.config_note, sizeof(e.mp6.config_note),
                 "parsed (%d err, %d warn); MaxPPS %d (%s); inputs staged — the session "
                 "reports anything else missing by name at Configure",
                 n_err, n_warn, max_pps_materialized, max_pps_src);
    else
        snprintf(e.mp6.config_note, sizeof(e.mp6.config_note),
                 "parsed cleanly; MaxPPS %d (%s); inputs staged — anything else "
                 "missing is reported by name at Configure",
                 max_pps_materialized, max_pps_src);
    app.log(e.id, "mp6", "Platform config parsed (errors %d, warnings %d); session inputs staged",
        n_err, n_warn);
}

// ── Design geometry → crank solver staging ────────────────────────────
//
// The mp6_geometry module contract: design-table numbers come from
// mp6_design(typ) or hand entry, pivot z coordinates are explicit inputs (the
// recovered Other\<Platform>.txt file is unrecovered), and Module2
// overrides/validation are the caller's job. This staging step does exactly
// that bookkeeping and never invents a machine constant.
static bool mp6ValidateDesignGeometry(const Mp6DesignState& d, std::string* error) {
    if (d.design_name[0] == '\0') {
        if (error) *error = "no design entered (pick a MOT design or \"Manual\" and fill the fields)";
        return false;
    }
    if (mp6::mp6_design_from_name(d.design_name) < 0 && strcmp(d.design_name, "Manual") != 0) {
        if (error) *error = "unknown design name";
        return false;
    }
    for (float value : {d.bot_a_mm, d.bot_b_mm, d.top_a_mm, d.top_b_mm,
                       d.crank_length_mm, d.rod_length_mm, d.off_x_mm, d.pcdx_mm,
                       d.pcdy_mm, d.z_off_mm, d.angle_offset_deg,
                       d.point_zero_x_mm, d.point_zero_z_mm}) {
        if (!std::isfinite(value)) {
            if (error) *error = "design inputs must be finite";
            return false;
        }
    }
    if (d.bot_b_mm <= 1.0f || d.top_b_mm <= 1.0f
        || d.bot_a_mm <= 1.0f || d.top_a_mm <= 1.0f) {
        if (error) *error = "BotA/BotB/TopA/TopB radii must all be positive";
        return false;
    }
    if (d.crank_length_mm <= 0.0f || d.rod_length_mm <= 0.0f) {
        if (error) *error = "crank and rod lengths must be positive";
        return false;
    }
    return true;
}

static void mp6StageDesignInputs(App& app, Entity& e) {
    std::string why;
    if (!mp6ValidateDesignGeometry(e.mp6.design, &why)) {
        e.mp6.design.supplied = false;
        app.mp6InvalidateAuthority(e, true, true);
        snprintf(e.mp6.config_note, sizeof(e.mp6.config_note), "design geometry rejected: %s", why.c_str());
        snprintf(e.mp6.solver_note, sizeof(e.mp6.solver_note), "design geometry rejected: %s", why.c_str());
        app.log(e.id, "mp6", "Design geometry rejected during staging: %s", why.c_str());
        return;
    }
    auto& rt = app.mp6_runtimes[e.id];
    if (!rt) rt = std::make_unique<App::Mp6Runtime>();
    rt->design.bot_a_mm = (double)e.mp6.design.bot_a_mm;
    rt->design.bot_b_mm = (double)e.mp6.design.bot_b_mm;
    rt->design.top_a_mm = (double)e.mp6.design.top_a_mm;
    rt->design.top_b_mm = (double)e.mp6.design.top_b_mm;
    rt->design.crank_length_mm = (double)e.mp6.design.crank_length_mm;
    rt->design.rod_length_mm = (double)e.mp6.design.rod_length_mm;
    rt->design.off_x_mm = (double)e.mp6.design.off_x_mm;
    rt->design.pcdx_mm = (double)e.mp6.design.pcdx_mm;
    rt->design.pcdy_mm = (double)e.mp6.design.pcdy_mm;
    rt->design.z_off_mm = (double)e.mp6.design.z_off_mm;
    rt->geometry_dirty = true;
    app.settings_dirty = true;
}

bool App::mp6RefreshDesignGeometry(Entity& e, std::string* error) {
    if (e.mp6.pending_intent != Mp6Intent::None) {
        if (error) *error = "dispatch the pending MP6 intent before editing accepted inputs";
        return false;
    }
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6) {
        if (error) *error = "entity is not on the MP6 backend";
        return false;
    }
    if (mp6IsMotorPowerRequested(e)) {
        if (error) *error = "cannot modify design geometry while motors are enabled; shut down first";
        return false;
    }
    if (e.mp6.lifecycle == Mp6Lifecycle::Configuring ||
        e.mp6.lifecycle == Mp6Lifecycle::Initializing ||
        e.mp6.lifecycle == Mp6Lifecycle::Running ||
        e.mp6.lifecycle == Mp6Lifecycle::Parking) {
        if (error) *error = "cannot modify design geometry while session is actively moving or configuring; shut down first";
        return false;
    }
    mp6InvalidateAuthority(e, false, true);
    e.mp6.design.supplied = false;
    if (!mp6ValidateDesignGeometry(e.mp6.design, error)) return false;
    // Mirror the legacy radius fields consumed by the settings snapshot and
    // the viz fallback path.
    e.mp6.design.base_radius_mm     = e.mp6.design.bot_b_mm;
    e.mp6.design.platform_radius_mm = e.mp6.design.top_b_mm;
    e.mp6.design.x_offset_mm        = e.mp6.design.off_x_mm;
    e.mp6.design.supplied = true;
    mp6StageDesignInputs(*this, e);
    log(e.id, "mp6", "Design geometry staged (%s)", e.mp6.design.design_name);
    return true;
}

bool App::mp6AcceptDesign(Entity& e, const Mp6DesignState& draft, std::string* error) {
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6) {
        if (error) *error = "entity is not an MP6 HIL entity";
        return false;
    }
    if (e.mp6.pending_intent != Mp6Intent::None) {
        if (error) *error = "dispatch the pending MP6 intent before editing accepted inputs";
        return false;
    }
    if (mp6IsMotorPowerRequested(e) || e.mp6.sending_config || e.mp6.shutdown_pending) {
        if (error) *error = "cannot modify design while powered or configuring; shut down first";
        return false;
    }
    mp6InvalidateAuthority(e, false, true);
    e.mp6.design = draft;
    return mp6RefreshDesignGeometry(e, error);
}

bool App::mp6RecoverKnownState(Entity& e, std::string* error) {
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6) {
        if (error) *error = "entity is not an MP6 HIL entity";
        return false;
    }
    // Explicit operator action only: independent controls have established a
    // known physical state. Auto-connect never clears this desktop latch.
    mp6Disconnect(e);
    mp6InvalidateAuthority(e, true, true);
    e.mp6.estop_latched = false;
    e.mp6.fault_latched = false;
    e.mp6.auto_connect = false;
    return true;
}

bool App::mp6SetControllerSettings(Entity& e, const Mp6MachineSettings& settings, std::string* error) {
    if (e.mp6.pending_intent != Mp6Intent::None) {
        if (error) *error = "dispatch the pending MP6 intent before editing accepted inputs";
        return false;
    }
    auto err = [&](const char* m) {
        snprintf(e.mp6.last_error, sizeof(e.mp6.last_error), "%s", m);
        if (error) *error = m;
        return false;
    };
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6)
        return err("entity is not an MP6 HIL entity");
    if (mp6IsMotorPowerRequested(e)) {
        return err("cannot modify controller settings while motors are enabled; shut down first");
    }
    if (e.mp6.lifecycle == Mp6Lifecycle::Configuring ||
        e.mp6.lifecycle == Mp6Lifecycle::Initializing ||
        e.mp6.lifecycle == Mp6Lifecycle::Running ||
        e.mp6.lifecycle == Mp6Lifecycle::Parking) {
        return err("cannot modify controller settings while session is actively moving or configuring; shut down first");
    }

    mp6InvalidateAuthority(e, true, true);
    std::string val_err;
    if (!mp6ValidateMachineSettings(settings, &val_err))
        return err(val_err.c_str());

    e.mp6.controller_settings = settings;
    e.mp6.controller_settings.supplied = true;
    mp6ApplyPlatformConfig(*this, e);
    settings_dirty = true;
    log(e.id, "mp6", "Controller settings applied (PID index %d, timeout %d ms, pressure aim %d psi)",
        settings.pid_index, settings.comms_timeout_ms, settings.pressure_aim_psi);
    return true;
}

static inline int clampBitDepth(int bits) {
    if (bits < 8) return 8;
    if (bits > 18) return 18;
    return bits;
}

static inline float txMaxRawForBitDepth(int bits) {
    bits = clampBitDepth(bits);
    return (float)((1u << bits) - 2u);
}

// ── PipelineConfig ──────────────────────────────────────────────────

void PipelineConfig::initDefaults() {
    initDefaultStewartConfig(&geometry);
    computeStepsPerDegree(&geometry);
    rebuildPlatform();
    computeAxisScalesFromGeometry(&axis_scales, &geometry, 0.9f);
    memset(&mca, 0, sizeof(mca));
    mca.enabled = 0;  // MCA off by default for manual sliders
    initInputFilter(&input_filter, 60.0f);
    occupant[0] = 0.0f;
    occupant[1] = 0.0f;
    occupant[2] = 800.0f;
    bit_depth = 12;
    platform_type = PlatformType::Stepper;  // default to stepper (main platform)
    servo.initDefaults();
    // Phase D: default intensity and per-axis gains
    intensity = 100.0f;
    for (int i = 0; i < 6; i++) axis_gain[i] = 100.0f;
    for (int i = 0; i < 6; i++) axis_invert[i] = false;
}

void PipelineConfig::rebuildPlatform() {
    buildPlatformFromConfig(&geometry, &platform);
}

// ── App ─────────────────────────────────────────────────────────────

static float g_entity_colors[][4] = {
    {0.38f, 0.65f, 0.96f, 1.0f},  // blue
    {0.20f, 0.83f, 0.60f, 1.0f},  // green
    {0.98f, 0.74f, 0.18f, 1.0f},  // amber
    {0.75f, 0.50f, 0.95f, 1.0f},  // purple
    {0.97f, 0.44f, 0.44f, 1.0f},  // red
    {0.13f, 0.83f, 0.87f, 1.0f},  // cyan
};
static const int g_num_colors = sizeof(g_entity_colors) / sizeof(g_entity_colors[0]);

App::App()
    : next_entity_id(0)
    , input_source(InputSource::Plugin)
    , console_max(500)
    , console_auto_scroll(true)
    , console_paused(false)
    , console_filter(-1)
    , running(true)
    , motion_started(false)
    , start_ramp_active(false)
    , start_ramp_begin(0.0)
    , frame_time(0.0)
    , fps(0.0)
    , frame_count(0)
    , last_input_log_time(0.0)
    , console_log_rate(2)  // default: 10 Hz
    , capture_playback_idx(-1)
    , capture_playing(false)
    , capture_play_cursor(0)
    , capture_start_time(0.0)
    , capture_loop(false)
    , capture_speed(1.0f)
    , capture_ramp_phase(CaptureRampPhase::Playing)
    , capture_ramp_start(0.0)
    , capture_stop_requested(false)
    , record_rate_hz(200)
    , input_history_head(0)
    , input_history_count(0)
    , input_history_last_push(0.0)
    , input_spectrum_freq_max(100.0f)
    , settings_dirty(false)
    , source_switch_active(false)
    , source_switch_target(InputSource::Plugin)
    , source_switch_start(0.0)
    , active_plugin_idx(-1)
{
    memset(shared_input, 0, sizeof(shared_input));
    memset(capture_last_vals, 0, sizeof(capture_last_vals));
    memset(input_history, 0, sizeof(input_history));
    memset(input_history_time, 0, sizeof(input_history_time));
    memset(input_spectrum, 0, sizeof(input_spectrum));

    // Start background HIL TX thread
    hil_tx_stop.store(false);
    if (!IsDocumentationMode()) hil_tx_thread = std::thread(&App::hilTxLoop, this);

#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

    // Scan for plugins in the plugins/ directory next to the executable
    if (!IsDocumentationMode()) plugin_mgr.scanDirectory("plugins");
}

App::~App() {
    plugin_mgr.deactivateActive();
    plugin_mgr.unloadAll();
    hil_tx_stop.store(true);
    if (hil_tx_thread.joinable()) hil_tx_thread.join();
#ifdef _WIN32
    WSACleanup();
#endif
}

void App::hilTxLoop() {
    using clock = std::chrono::steady_clock;
    double last_send[8] = {};
    auto now_sec = []() -> double {
        return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
    };

    double last_motion_true = 0.0;
    while (!hil_tx_stop.load()) {
        // Gate the stream on motion: incoming frames auto-switch the device
        // DEMO->LIVE, so streaming while stopped locks out on-device demo
        // playback. 3s grace after STOP lets the ramp-home frames land; the
        // device cue washout settles the rest.
        {
            double now = now_sec();
            if (motion_started) last_motion_true = now;
            if (!motion_started && now - last_motion_true > 3.0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
        }
        int ei = 0;
        for (auto& e : entities) {
            if (ei >= 8) break;
            // MP6 TX is exclusively owned by the mp6 session engine at its own
            // cadence through its own raw link. The COBS loop must never send
            // MP6 frames or touch MP6 entity fields.
            if (e.hil_backend != HilBackend::ESP32) { ei++; continue; }
            if (e.type != EntityType::HIL || !e.serial || !e.serial->isOpen() || !e.hil_handshake_ok) { ei++; continue; }

            double interval = 1.0 / (double)e.hil_tx_hz;
            double now = now_sec();
            if (now - last_send[ei] >= interval) {
                last_send[ei] = now;
                if (e.hil_raw_mode && e.hil_cap_raw) {
                    // RAW HIL: stream pre-cueing input_pct (app axis order, no swap).
                    // The ESP runs the single cue engine and swaps axes after cueing.
                    float f[6];
                    memcpy(f, e.hil_tx_raw_f, sizeof(f));
                    e.serial->sendCobsDataRaw(f);
                } else {
                    // BAKED: hil_tx_raw already carries MCA dynamics, tilt
                    // coordination, intensity, axis gain, and the surge<->sway swap.
                    uint32_t raw[6];
                    memcpy(raw, e.hil_tx_raw, sizeof(raw));
                    e.serial->sendCobsData(raw, e.config.bit_depth);
                }
            }
            ei++;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}

static const char* SETTINGS_FILE = "stewart_settings.json";

// Forward declarations (defined in MCA Dynamics Presets section below)
static cJSON* mcaConfigToJSON(const MotionCueingConfig& mca, float intensity, const float axis_gain[6]);
static void mcaConfigFromJSON(cJSON* obj, MotionCueingConfig& mca, float& intensity, float axis_gain[6]);

static void saveEntityToJSON(cJSON* ej, const Entity& e) {
    cJSON_AddStringToObject(ej, "name", e.name);
    cJSON_AddNumberToObject(ej, "type", (int)e.type);
    cJSON_AddBoolToObject(ej, "enabled", e.enabled);
    cJSON_AddNumberToObject(ej, "intensity", e.config.intensity);
    cJSON_AddNumberToObject(ej, "bit_depth", e.config.bit_depth);

    // Color
    cJSON* col = cJSON_AddArrayToObject(ej, "color");
    for (int i = 0; i < 4; i++) cJSON_AddItemToArray(col, cJSON_CreateNumber(e.color[i]));

    // Per-axis gain + inversion
    cJSON* gains = cJSON_AddArrayToObject(ej, "axis_gain");
    for (int i = 0; i < 6; i++) cJSON_AddItemToArray(gains, cJSON_CreateNumber(e.config.axis_gain[i]));
    cJSON* inverts = cJSON_AddArrayToObject(ej, "axis_invert");
    for (int i = 0; i < 6; i++) cJSON_AddItemToArray(inverts, cJSON_CreateBool(e.config.axis_invert[i]));

    // Geometry
    cJSON* geo = cJSON_AddObjectToObject(ej, "geometry");
    cJSON_AddNumberToObject(geo, "RD", e.config.geometry.RD);
    cJSON_AddNumberToObject(geo, "PD", e.config.geometry.PD);
    cJSON_AddNumberToObject(geo, "L1", e.config.geometry.ServoArmLengthL1);
    cJSON_AddNumberToObject(geo, "L2", e.config.geometry.ConnectingArmLengthL2);
    cJSON_AddNumberToObject(geo, "z_home", e.config.geometry.platformHeight);
    cJSON_AddNumberToObject(geo, "theta_r", e.config.geometry.theta_r);
    cJSON_AddNumberToObject(geo, "theta_p", e.config.geometry.theta_p);
    cJSON_AddNumberToObject(geo, "encoder_ppr", e.config.geometry.encoder_ppr);
    cJSON_AddNumberToObject(geo, "virtual_gear", e.config.geometry.virtual_gear);
    cJSON_AddNumberToObject(geo, "planetary_ratio", e.config.geometry.planetary_ratio);

    // Full MCA dynamics config (replaces old mca_preset-only save)
    cJSON* mca_obj = mcaConfigToJSON(e.config.mca, e.config.intensity, e.config.axis_gain);
    cJSON_AddItemToObject(ej, "mca_config", mca_obj);

    // Input filter config
    cJSON* iflt_obj = cJSON_AddObjectToObject(ej, "input_filter");
    cJSON_AddBoolToObject(iflt_obj, "enabled", e.config.input_filter.enabled != 0);
    cJSON* iflt_axes = cJSON_AddArrayToObject(iflt_obj, "axes");
    for (int i = 0; i < 6; i++) {
        const InputAxisFilter& ax = e.config.input_filter.axes[i];
        cJSON* a = cJSON_CreateObject();
        cJSON_AddBoolToObject(a, "lp_on", ax.lp_enabled != 0);
        cJSON_AddNumberToObject(a, "lp_fc", ax.lp.fc);
        cJSON_AddNumberToObject(a, "lp_Q", ax.lp.Q);
        cJSON_AddBoolToObject(a, "notch_on", ax.notch_enabled != 0);
        cJSON_AddNumberToObject(a, "notch_fc", ax.notch.fc);
        cJSON_AddNumberToObject(a, "notch_Q", ax.notch.Q);
        cJSON_AddItemToArray(iflt_axes, a);
    }

    // Occupant offset
    cJSON* occ = cJSON_AddArrayToObject(ej, "occupant");
    for (int i = 0; i < 3; i++) cJSON_AddItemToArray(occ, cJSON_CreateNumber(e.config.occupant[i]));

    // Platform type + servo config
    cJSON_AddNumberToObject(ej, "platform_type", (int)e.config.platform_type);
    if (e.config.isServo()) {
        cJSON* sv = cJSON_AddObjectToObject(ej, "servo");
        cJSON* centers = cJSON_AddArrayToObject(sv, "center_us");
        for (int i = 0; i < 6; i++) cJSON_AddItemToArray(centers, cJSON_CreateNumber(e.config.servo.center_us[i]));
        cJSON_AddNumberToObject(sv, "pulse_per_rad", e.config.servo.pulse_per_rad);
        cJSON_AddNumberToObject(sv, "min_pulse_us", e.config.servo.min_pulse_us);
        cJSON_AddNumberToObject(sv, "max_pulse_us", e.config.servo.max_pulse_us);
        cJSON_AddNumberToObject(sv, "pwm_freq_hz", e.config.servo.pwm_freq_hz);
        cJSON* inv = cJSON_AddArrayToObject(sv, "inverted");
        for (int i = 0; i < 6; i++) cJSON_AddItemToArray(inv, cJSON_CreateBool(e.config.servo.inverted[i]));
    }

    // ── Backend selection + MP6 config (HIL only) ──
    if (e.type == EntityType::HIL) {
        cJSON_AddStringToObject(ej, "backend",
            e.hil_backend == HilBackend::MP6 ? "mp6" : "esp32");
        if (e.hil_backend == HilBackend::MP6) {
            cJSON* m6 = cJSON_AddObjectToObject(ej, "mp6");
            cJSON_AddStringToObject(m6, "link_port", e.mp6.link_port);
            cJSON_AddBoolToObject(m6, "auto_connect", e.mp6.auto_connect);
            cJSON_AddBoolToObject(m6, "onboard_control", e.mp6.onboard_control);
            cJSON_AddNumberToObject(m6, "cmd2_hz", e.mp6.cmd2_hz);
            cJSON_AddNumberToObject(m6, "link_baud", e.mp6.link_baud);
            cJSON_AddNumberToObject(m6, "platform_number", e.mp6.platform_number_expected);
            cJSON_AddBoolToObject(m6, "platform_config_present", e.mp6.platform_config_present);
            cJSON_AddNumberToObject(m6, "max_rpm_entry", e.mp6.max_rpm_entry);
            if (e.mp6.platform_config_present)
                cJSON_AddStringToObject(m6, "platform_config_text", e.mp6.platform_config_text);
            cJSON* dsn = cJSON_AddObjectToObject(m6, "design");
            cJSON_AddBoolToObject(dsn, "supplied", e.mp6.design.supplied);
            cJSON_AddStringToObject(dsn, "design_name", e.mp6.design.design_name);
            auto addf = [&](const char* k, float v) { cJSON_AddNumberToObject(dsn, k, v); };
            addf("bot_a_mm", e.mp6.design.bot_a_mm);
            addf("bot_b_mm", e.mp6.design.bot_b_mm);
            addf("top_a_mm", e.mp6.design.top_a_mm);
            addf("top_b_mm", e.mp6.design.top_b_mm);
            addf("crank_length_mm", e.mp6.design.crank_length_mm);
            addf("off_x_mm", e.mp6.design.off_x_mm);
            addf("pcdx_mm", e.mp6.design.pcdx_mm);
            addf("pcdy_mm", e.mp6.design.pcdy_mm);
            addf("rod_length_mm", e.mp6.design.rod_length_mm);
            addf("z_off_mm", e.mp6.design.z_off_mm);
            addf("point_zero_x_mm", e.mp6.design.point_zero_x_mm);
            addf("point_zero_z_mm", e.mp6.design.point_zero_z_mm);
            addf("angle_offset_deg", e.mp6.design.angle_offset_deg);
            addf("base_radius_mm", e.mp6.design.base_radius_mm);
            addf("platform_radius_mm", e.mp6.design.platform_radius_mm);
            addf("x_offset_mm", e.mp6.design.x_offset_mm);
            cJSON_AddNumberToObject(dsn, "pivot_z_provenance",
                                    (double)e.mp6.design.pivot_z_provenance);
            cJSON_AddBoolToObject(dsn, "point_zero_supplied", e.mp6.design.point_zero_supplied);
            {
                double bzs[6], tzs[6];
                for (int i = 0; i < 6; i++) {
                    bzs[i] = e.mp6.design.base_z_mm[i];
                    tzs[i] = e.mp6.design.top_z_mm[i];
                }
                cJSON_AddItemToObject(dsn, "base_z_mm", cJSON_CreateDoubleArray(bzs, 6));
                cJSON_AddItemToObject(dsn, "top_z_mm", cJSON_CreateDoubleArray(tzs, 6));
            }
            cJSON* cs = cJSON_AddObjectToObject(m6, "controller_settings");
            cJSON_AddBoolToObject(cs, "supplied", e.mp6.controller_settings.supplied);
            cJSON_AddNumberToObject(cs, "explicit_fields", e.mp6.controller_settings.explicit_fields);
            cJSON_AddNumberToObject(cs, "sensor_filter_start", e.mp6.controller_settings.sensor_filter_start);
            cJSON_AddNumberToObject(cs, "history_len_start", e.mp6.controller_settings.history_len_start);
            cJSON_AddNumberToObject(cs, "bench_test", e.mp6.controller_settings.bench_test);
            cJSON_AddNumberToObject(cs, "comms_timeout_ms", e.mp6.controller_settings.comms_timeout_ms);
            cJSON_AddNumberToObject(cs, "onboard_shutdown", e.mp6.controller_settings.onboard_shutdown);
            cJSON_AddNumberToObject(cs, "mot_val_final", e.mp6.controller_settings.mot_val_final);
            cJSON_AddNumberToObject(cs, "sensor_filter", e.mp6.controller_settings.sensor_filter);
            cJSON_AddNumberToObject(cs, "history_len", e.mp6.controller_settings.history_len);
            cJSON_AddNumberToObject(cs, "deadband_enable", e.mp6.controller_settings.deadband_enable);
            cJSON_AddNumberToObject(cs, "av_filter", e.mp6.controller_settings.av_filter);
            cJSON_AddNumberToObject(cs, "limits_enable", e.mp6.controller_settings.limits_enable);
            cJSON_AddNumberToObject(cs, "info_motor", e.mp6.controller_settings.info_motor);
            cJSON_AddNumberToObject(cs, "pid_index", e.mp6.controller_settings.pid_index);
            cJSON_AddBoolToObject(cs, "pressure_enabled", e.mp6.controller_settings.pressure_enabled);
            cJSON_AddBoolToObject(cs, "pressure_aim_supplied", e.mp6.controller_settings.pressure_aim_supplied);
            cJSON_AddNumberToObject(cs, "pressure_aim_psi", e.mp6.controller_settings.pressure_aim_psi);
        }
    }

    // HIL-specific
    if (e.type == EntityType::HIL) {
        cJSON* hil = cJSON_AddObjectToObject(ej, "hil");
        cJSON_AddStringToObject(hil, "port", e.hil_port);
        cJSON_AddNumberToObject(hil, "baud", e.hil_baud);
        cJSON_AddNumberToObject(hil, "tx_hz", e.hil_tx_hz);
        cJSON_AddNumberToObject(hil, "tick_rate_us", e.hil_tick_rate_us);
        cJSON_AddBoolToObject(hil, "auto_connect", e.hil_auto_connect);
        // Network bridge transport
        cJSON_AddBoolToObject(hil, "network", e.hil_network);
        cJSON_AddStringToObject(hil, "host", e.hil_host);
        cJSON_AddNumberToObject(hil, "udp_port", e.hil_udp_port);
        cJSON_AddNumberToObject(hil, "tcp_port", e.hil_tcp_port);
        cJSON_AddBoolToObject(hil, "raw_mode", e.hil_raw_mode);
        if (e.hil_fingerprint[0] != '\0')
            cJSON_AddStringToObject(hil, "fingerprint", e.hil_fingerprint);
    }
}

static void loadEntityFromJSON(Entity& e, cJSON* ej) {
    cJSON* val;
    if ((val = cJSON_GetObjectItem(ej, "name")))      snprintf(e.name, sizeof(e.name), "%s", val->valuestring);
    if ((val = cJSON_GetObjectItem(ej, "enabled")))    e.enabled = cJSON_IsTrue(val);
    if ((val = cJSON_GetObjectItem(ej, "intensity")))  e.config.intensity = (float)val->valuedouble;
    if ((val = cJSON_GetObjectItem(ej, "bit_depth")))
        e.config.bit_depth = clampBitDepth(val->valueint);

    // Color
    cJSON* col = cJSON_GetObjectItem(ej, "color");
    if (col && cJSON_IsArray(col)) {
        for (int i = 0; i < 4 && i < cJSON_GetArraySize(col); i++)
            e.color[i] = (float)cJSON_GetArrayItem(col, i)->valuedouble;
    }

    // Per-axis gain + inversion
    cJSON* gains = cJSON_GetObjectItem(ej, "axis_gain");
    if (gains && cJSON_IsArray(gains)) {
        for (int i = 0; i < 6 && i < cJSON_GetArraySize(gains); i++)
            e.config.axis_gain[i] = (float)cJSON_GetArrayItem(gains, i)->valuedouble;
    }
    cJSON* inverts = cJSON_GetObjectItem(ej, "axis_invert");
    if (inverts && cJSON_IsArray(inverts)) {
        for (int i = 0; i < 6 && i < cJSON_GetArraySize(inverts); i++)
            e.config.axis_invert[i] = cJSON_IsTrue(cJSON_GetArrayItem(inverts, i));
    }

    // Geometry
    cJSON* geo = cJSON_GetObjectItem(ej, "geometry");
    if (geo) {
        if ((val = cJSON_GetObjectItem(geo, "RD")))                e.config.geometry.RD = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "PD")))                e.config.geometry.PD = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "L1")))                e.config.geometry.ServoArmLengthL1 = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "L2")))                e.config.geometry.ConnectingArmLengthL2 = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "z_home")))            e.config.geometry.platformHeight = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "theta_r")))           e.config.geometry.theta_r = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "theta_p")))           e.config.geometry.theta_p = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "encoder_ppr")))       e.config.geometry.encoder_ppr = val->valueint;
        if ((val = cJSON_GetObjectItem(geo, "virtual_gear")))      e.config.geometry.virtual_gear = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "planetary_ratio")))   e.config.geometry.planetary_ratio = (float)val->valuedouble;
        e.config.rebuildPlatform();
    }

    // Full MCA dynamics config
    cJSON* mca_obj = cJSON_GetObjectItem(ej, "mca_config");
    if (mca_obj && cJSON_IsObject(mca_obj)) {
        initMotionCueing(&e.config.mca, 60.0f);
        mcaConfigFromJSON(mca_obj, e.config.mca, e.config.intensity, e.config.axis_gain);
        // Recalculate biquad coefficients at default sample rate
        float sr = e.config.mca.sample_rate > 0 ? e.config.mca.sample_rate : 60.0f;
        for (int i = 0; i < 6; i++) {
            if (e.config.mca.channels[i].hp_enabled && e.config.mca.channels[i].hp.fc > 0)
                biquadSetHighpass(&e.config.mca.channels[i].hp, e.config.mca.channels[i].hp.fc, sr, e.config.mca.channels[i].hp.Q);
            if (e.config.mca.channels[i].lp_enabled && e.config.mca.channels[i].lp.fc > 0)
                biquadSetLowpass(&e.config.mca.channels[i].lp, e.config.mca.channels[i].lp.fc, sr, e.config.mca.channels[i].lp.Q);
        }
        if (e.config.mca.tilt.enabled && e.config.mca.tilt.fc > 0) {
            biquadSetLowpass(&e.config.mca.tilt.surge_lp, e.config.mca.tilt.fc, sr, e.config.mca.tilt.Q);
            biquadSetLowpass(&e.config.mca.tilt.sway_lp,  e.config.mca.tilt.fc, sr, e.config.mca.tilt.Q);
        }
        if (e.config.mca.tilt.surge_hp_enabled && e.config.mca.tilt.hp_fc > 0) {
            float hq = e.config.mca.tilt.hp_Q > 0 ? e.config.mca.tilt.hp_Q : 0.707f;
            biquadSetHighpass(&e.config.mca.tilt.surge_hp, e.config.mca.tilt.hp_fc, sr, hq);
        }
        if (e.config.mca.tilt.sway_hp_enabled && e.config.mca.tilt.sway_hp_fc > 0) {
            float hq = e.config.mca.tilt.sway_hp_Q > 0 ? e.config.mca.tilt.sway_hp_Q : 0.707f;
            biquadSetHighpass(&e.config.mca.tilt.sway_hp, e.config.mca.tilt.sway_hp_fc, sr, hq);
        }
    } else if ((val = cJSON_GetObjectItem(ej, "mca_preset"))) {
        // Legacy fallback: old settings with just preset index
        int preset = val->valueint;
        if (preset > 0 && preset < MCA_PRESET_COUNT) {
            initMotionCueing(&e.config.mca, 60.0f);
            setMotionCueingPreset(&e.config.mca, preset);
        }
    }

    // Input filter config
    cJSON* iflt_obj = cJSON_GetObjectItem(ej, "input_filter");
    if (iflt_obj && cJSON_IsObject(iflt_obj)) {
        cJSON* val_if;
        if ((val_if = cJSON_GetObjectItem(iflt_obj, "enabled"))) e.config.input_filter.enabled = cJSON_IsTrue(val_if) ? 1 : 0;
        cJSON* iflt_axes = cJSON_GetObjectItem(iflt_obj, "axes");
        if (iflt_axes && cJSON_IsArray(iflt_axes)) {
            float sr = e.config.input_filter.sample_rate > 0 ? e.config.input_filter.sample_rate : 60.0f;
            for (int i = 0; i < 6 && i < cJSON_GetArraySize(iflt_axes); i++) {
                cJSON* a = cJSON_GetArrayItem(iflt_axes, i);
                InputAxisFilter& ax = e.config.input_filter.axes[i];
                if ((val_if = cJSON_GetObjectItem(a, "lp_on")))    ax.lp_enabled = cJSON_IsTrue(val_if) ? 1 : 0;
                if ((val_if = cJSON_GetObjectItem(a, "lp_fc")))    ax.lp.fc = (float)val_if->valuedouble;
                if ((val_if = cJSON_GetObjectItem(a, "lp_Q")))     ax.lp.Q = (float)val_if->valuedouble;
                if ((val_if = cJSON_GetObjectItem(a, "notch_on"))) ax.notch_enabled = cJSON_IsTrue(val_if) ? 1 : 0;
                if ((val_if = cJSON_GetObjectItem(a, "notch_fc"))) ax.notch.fc = (float)val_if->valuedouble;
                if ((val_if = cJSON_GetObjectItem(a, "notch_Q")))  ax.notch.Q = (float)val_if->valuedouble;
                if (ax.lp_enabled && ax.lp.fc > 0)
                    biquadSetLowpass(&ax.lp, ax.lp.fc, sr, ax.lp.Q > 0 ? ax.lp.Q : 0.707f);
                if (ax.notch_enabled && ax.notch.fc > 0)
                    biquadSetNotch(&ax.notch, ax.notch.fc, sr, ax.notch.Q > 0 ? ax.notch.Q : 5.0f);
            }
        }
    }

    // Occupant offset
    cJSON* occ = cJSON_GetObjectItem(ej, "occupant");
    if (occ && cJSON_IsArray(occ)) {
        for (int i = 0; i < 3 && i < cJSON_GetArraySize(occ); i++)
            e.config.occupant[i] = (float)cJSON_GetArrayItem(occ, i)->valuedouble;
    }

    // Platform type + servo config
    if ((val = cJSON_GetObjectItem(ej, "platform_type")))
        e.config.platform_type = (PlatformType)val->valueint;
    cJSON* sv = cJSON_GetObjectItem(ej, "servo");
    if (sv && cJSON_IsObject(sv)) {
        cJSON* centers = cJSON_GetObjectItem(sv, "center_us");
        if (centers && cJSON_IsArray(centers)) {
            for (int i = 0; i < 6 && i < cJSON_GetArraySize(centers); i++)
                e.config.servo.center_us[i] = cJSON_GetArrayItem(centers, i)->valueint;
        }
        if ((val = cJSON_GetObjectItem(sv, "pulse_per_rad"))) e.config.servo.pulse_per_rad = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(sv, "min_pulse_us")))  e.config.servo.min_pulse_us = val->valueint;
        if ((val = cJSON_GetObjectItem(sv, "max_pulse_us")))  e.config.servo.max_pulse_us = val->valueint;
        if ((val = cJSON_GetObjectItem(sv, "pwm_freq_hz")))   e.config.servo.pwm_freq_hz = val->valueint;
        cJSON* inv = cJSON_GetObjectItem(sv, "inverted");
        if (inv && cJSON_IsArray(inv)) {
            for (int i = 0; i < 6 && i < cJSON_GetArraySize(inv); i++)
                e.config.servo.inverted[i] = cJSON_IsTrue(cJSON_GetArrayItem(inv, i));
        }
    }
    // Backend selection first (defaults apply when absent: pre-MP6 files are
    // ESP32 entities and need no new keys to round-trip).
    if ((val = cJSON_GetObjectItem(ej, "backend"))) {
        if (val->valuestring && strcmp(val->valuestring, "mp6") == 0)
            e.hil_backend = HilBackend::MP6;
        else
            e.hil_backend = HilBackend::ESP32;
    }
    cJSON* m6 = cJSON_GetObjectItem(ej, "mp6");
    if (e.hil_backend == HilBackend::MP6 && m6 && cJSON_IsObject(m6)) {
        if ((val = cJSON_GetObjectItem(m6, "link_port")) && cJSON_IsString(val))
            snprintf(e.mp6.link_port, sizeof(e.mp6.link_port), "%s", val->valuestring);
        if ((val = cJSON_GetObjectItem(m6, "auto_connect"))) e.mp6.auto_connect = cJSON_IsTrue(val);
        if ((val = cJSON_GetObjectItem(m6, "onboard_control"))) e.mp6.onboard_control = cJSON_IsTrue(val);
        if ((val = cJSON_GetObjectItem(m6, "cmd2_hz"))) {
            int hz = val->valueint;
            e.mp6.cmd2_hz = (hz >= 10 && hz <= 120) ? hz : 60;
        }
        // MP6 desktop transport is documented and commissioned at 115200 8N1.
        // Ignore historical/arbitrary saved baud values rather than reopening
        // the link at an unreviewed rate.
        e.mp6.link_baud = 115200;
        if ((val = cJSON_GetObjectItem(m6, "platform_number")))
            e.mp6.platform_number_expected = val->valueint;
        if ((val = cJSON_GetObjectItem(m6, "platform_config_present")))
            e.mp6.platform_config_present = cJSON_IsTrue(val);
        // MaxRPM entry precedes the import: mp6ImportPlatformConfigText
        // materializes MaxPPS from the .pltc row or, when that row is absent,
        // from this explicit entry.
        if ((val = cJSON_GetObjectItem(m6, "max_rpm_entry")))
            e.mp6.max_rpm_entry = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(m6, "platform_config_text")) && val->valuestring)
            g_app.mp6ImportPlatformConfigText(e, val->valuestring, nullptr);
        cJSON* dsn = cJSON_GetObjectItem(m6, "design");
        if (dsn && cJSON_IsObject(dsn)) {
            bool design_data_valid = true;
            if ((val = cJSON_GetObjectItem(dsn, "supplied")))
                e.mp6.design.supplied = cJSON_IsTrue(val);
            if ((val = cJSON_GetObjectItem(dsn, "design_name")) && cJSON_IsString(val))
                snprintf(e.mp6.design.design_name, sizeof(e.mp6.design.design_name), "%s", val->valuestring ? val->valuestring : "");
            auto getf = [&](const char* k, float* out) {
                cJSON* v = cJSON_GetObjectItem(dsn, k);
                if (!cJSON_IsNumber(v) || !std::isfinite(v->valuedouble)
                    || !std::isfinite(static_cast<float>(v->valuedouble))) {
                    design_data_valid = false;
                    return;
                }
                *out = static_cast<float>(v->valuedouble);
            };
            getf("bot_a_mm", &e.mp6.design.bot_a_mm);
            getf("bot_b_mm", &e.mp6.design.bot_b_mm);
            getf("top_a_mm", &e.mp6.design.top_a_mm);
            getf("top_b_mm", &e.mp6.design.top_b_mm);
            getf("crank_length_mm", &e.mp6.design.crank_length_mm);
            getf("off_x_mm", &e.mp6.design.off_x_mm);
            getf("pcdx_mm", &e.mp6.design.pcdx_mm);
            getf("pcdy_mm", &e.mp6.design.pcdy_mm);
            getf("rod_length_mm", &e.mp6.design.rod_length_mm);
            getf("z_off_mm", &e.mp6.design.z_off_mm);
            getf("point_zero_x_mm", &e.mp6.design.point_zero_x_mm);
            getf("point_zero_z_mm", &e.mp6.design.point_zero_z_mm);
            getf("angle_offset_deg", &e.mp6.design.angle_offset_deg);
            getf("base_radius_mm", &e.mp6.design.base_radius_mm);
            getf("platform_radius_mm", &e.mp6.design.platform_radius_mm);
            getf("x_offset_mm", &e.mp6.design.x_offset_mm);
            {   // Per-axis pivot z arrays are explicit source data.
                auto get_z = [&](const char* key, float (&out)[6]) {
                    cJSON* array = cJSON_GetObjectItem(dsn, key);
                    if (!cJSON_IsArray(array) || cJSON_GetArraySize(array) != 6) {
                        design_data_valid = false;
                        return;
                    }
                    for (int i = 0; i < 6; ++i) {
                        cJSON* item = cJSON_GetArrayItem(array, i);
                        if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble)
                            || !std::isfinite(static_cast<float>(item->valuedouble))) {
                            design_data_valid = false;
                            return;
                        }
                        out[i] = static_cast<float>(item->valuedouble);
                    }
                };
                get_z("base_z_mm", e.mp6.design.base_z_mm);
                get_z("top_z_mm", e.mp6.design.top_z_mm);
                // Pivot provenance: persisted only for the new three-state
                // gate. Older settings files (pre-provenance) carry the
                // old `pivot_z_supplied` bool under values written before
                // the calibration-column correction — those are treated as
                // Unreviewed regardless of the bool, and the design is
                // flagged so the UI asks for an explicit source re-import
                // + review before kReviewed can be granted. Nothing is
                // silently promoted.
                if ((val = cJSON_GetObjectItem(dsn, "pivot_z_provenance"))) {
                    int p = val->valueint;
                    // Clamp: 0 Unreviewed, 1 FixtureOnly (never live), 2 Reviewed.
                    e.mp6.design.pivot_z_provenance = (p == 0 || p == 1 || p == 2) ? p : 0;
                } else if ((val = cJSON_GetObjectItem(dsn, "pivot_z_supplied"))) {
                    if (cJSON_IsTrue(val)) {
                        e.mp6.design.pivot_z_provenance = 0;   // Unreviewed, review required
                        e.mp6.design.stale_older_config = true;
                    }
                }
                if ((val = cJSON_GetObjectItem(dsn, "point_zero_supplied")))
                    e.mp6.design.point_zero_supplied = cJSON_IsTrue(val);
            }
            // Legacy and malformed records remain explicitly unreviewed. A
            // saved review flag cannot vouch for missing or nonnumeric data.
            if (!cJSON_IsString(cJSON_GetObjectItem(dsn, "design_name"))
                || !cJSON_GetObjectItem(dsn, "pivot_z_provenance")
                || !cJSON_GetObjectItem(dsn, "point_zero_supplied"))
                design_data_valid = false;
            if (!design_data_valid) {
                e.mp6.design.supplied = false;
                e.mp6.design.pivot_z_provenance = 0;
                e.mp6.design.point_zero_supplied = false;
                g_app.mp6InvalidateAuthority(e, true, true);
                g_app.log(e.id, "mp6", "Loaded design is incomplete or contains nonnumeric/nonfinite values; geometry and Configure/Initialize authority cleared for review");
            }
            // Stage a solver rebuild for the loaded inputs (applied on the next
            // main-thread update; readiness gates decide whether it succeeds).
            if (design_data_valid)
                mp6StageDesignInputs(g_app, e);
        }
        cJSON* cs = cJSON_GetObjectItem(m6, "controller_settings");
        if (cs && cJSON_IsObject(cs)) {
            auto& s = e.mp6.controller_settings;
            if ((val = cJSON_GetObjectItem(cs, "supplied"))) s.supplied = cJSON_IsTrue(val);
            if ((val = cJSON_GetObjectItem(cs, "bench_test"))) s.bench_test = val->valueint;
            if ((val = cJSON_GetObjectItem(cs, "comms_timeout_ms"))) s.comms_timeout_ms = val->valueint;
            if ((val = cJSON_GetObjectItem(cs, "onboard_shutdown"))) s.onboard_shutdown = val->valueint;
            if ((val = cJSON_GetObjectItem(cs, "mot_val_final"))) s.mot_val_final = val->valueint;
            if ((val = cJSON_GetObjectItem(cs, "sensor_filter"))) s.sensor_filter = (float)val->valuedouble;
            if ((val = cJSON_GetObjectItem(cs, "history_len"))) s.history_len = val->valueint;
            if ((val = cJSON_GetObjectItem(cs, "deadband_enable"))) s.deadband_enable = val->valueint;
            if ((val = cJSON_GetObjectItem(cs, "av_filter"))) s.av_filter = (float)val->valuedouble;
            if ((val = cJSON_GetObjectItem(cs, "limits_enable"))) s.limits_enable = val->valueint;
            if ((val = cJSON_GetObjectItem(cs, "info_motor"))) s.info_motor = val->valueint;
            if ((val = cJSON_GetObjectItem(cs, "pid_index"))) s.pid_index = val->valueint;
            if ((val = cJSON_GetObjectItem(cs, "pressure_enabled"))) s.pressure_enabled = cJSON_IsTrue(val);
            if ((val = cJSON_GetObjectItem(cs, "pressure_aim_supplied"))) s.pressure_aim_supplied = cJSON_IsTrue(val);
            if ((val = cJSON_GetObjectItem(cs, "pressure_aim_psi"))) s.pressure_aim_psi = val->valueint;
            s.explicit_fields = 0;
            for (unsigned i = 0; i < 16; ++i) {
                cJSON* field = cJSON_GetObjectItem(cs, mp6MachineSettingNames[i]);
                bool boolean = i == 13 || i == 14;
                if (field && (boolean ? cJSON_IsBool(field) : cJSON_IsNumber(field))
                    && (boolean || (std::isfinite(field->valuedouble)
                        && ((i == 4 || i == 6 || i == 9) || field->valuedouble == field->valueint))))
                    s.explicit_fields |= 1u << i;
            }
            cJSON* supplied_mask = cJSON_GetObjectItem(cs, "explicit_fields");
            if (!cJSON_IsNumber(supplied_mask) || supplied_mask->valuedouble != Mp6MachineSettings::all_fields)
                s.explicit_fields = 0;
            if ((val = cJSON_GetObjectItem(cs, "sensor_filter_start"))) s.sensor_filter_start = (float)val->valuedouble;
            if ((val = cJSON_GetObjectItem(cs, "history_len_start"))) s.history_len_start = val->valueint;
            if (s.supplied) {
                std::string why;
                if (!g_app.mp6ValidateMachineSettings(s, &why)) {
                    s.supplied = false;
                    g_app.log(e.id, "mp6", "Restored controller settings invalid (%s); marked unsupplied", why.c_str());
                }
            }
        }
    }
    cJSON* hil = cJSON_GetObjectItem(ej, "hil");
    if (hil) {
        if ((val = cJSON_GetObjectItem(hil, "port"))) snprintf(e.hil_port, sizeof(e.hil_port), "%s", val->valuestring);
        if ((val = cJSON_GetObjectItem(hil, "baud"))) e.hil_baud = val->valueint;
        if ((val = cJSON_GetObjectItem(hil, "tx_hz"))) e.hil_tx_hz = val->valueint;
        if ((val = cJSON_GetObjectItem(hil, "tick_rate_us"))) {
            int v = val->valueint;
            if (v >= 4 && v <= 100) e.hil_tick_rate_us = v;
        }
        if ((val = cJSON_GetObjectItem(hil, "auto_connect"))) e.hil_auto_connect = cJSON_IsTrue(val);
        if ((val = cJSON_GetObjectItem(hil, "network"))) e.hil_network = cJSON_IsTrue(val);
        if ((val = cJSON_GetObjectItem(hil, "host")) && cJSON_IsString(val)) snprintf(e.hil_host, sizeof(e.hil_host), "%s", val->valuestring);
        if ((val = cJSON_GetObjectItem(hil, "udp_port"))) e.hil_udp_port = val->valueint;
        if ((val = cJSON_GetObjectItem(hil, "tcp_port"))) e.hil_tcp_port = val->valueint;
        if ((val = cJSON_GetObjectItem(hil, "raw_mode"))) e.hil_raw_mode = cJSON_IsTrue(val);
        if ((val = cJSON_GetObjectItem(hil, "fingerprint"))) snprintf(e.hil_fingerprint, sizeof(e.hil_fingerprint), "%s", val->valuestring);
    }
}

void App::saveSettings() {
    if (IsDocumentationMode()) { settings_dirty = false; return; }
    cJSON* root = cJSON_CreateObject();

    // Console
    cJSON_AddNumberToObject(root, "console_log_rate", console_log_rate);
    cJSON_AddNumberToObject(root, "console_max", console_max);
    cJSON_AddBoolToObject(root, "console_auto_scroll", console_auto_scroll);

    // Recording
    cJSON_AddNumberToObject(root, "record_rate_hz", record_rate_hz);

    // Input source
    cJSON_AddNumberToObject(root, "input_source", (int)input_source);

    // Active plugin name (saved by name so it survives plugin reorder)
    if (active_plugin_idx >= 0 && active_plugin_idx < plugin_mgr.pluginCount()) {
        const char* pname = plugin_mgr.pluginName(active_plugin_idx);
        if (pname) cJSON_AddStringToObject(root, "active_plugin_name", pname);
    }

    // Dynamics panel selection + panel visibility
    cJSON_AddNumberToObject(root, "selected_dynamics_id", selected_dynamics_id);
    cJSON_AddBoolToObject(root, "show_dynamics",      show_dynamics);
    cJSON_AddBoolToObject(root, "show_console",       show_console);
    cJSON_AddBoolToObject(root, "show_data_streams",  show_data_streams);

    // Entities — full serialization
    cJSON* ents = cJSON_AddArrayToObject(root, "entities");
    for (auto& e : entities) {
        cJSON* ej = cJSON_CreateObject();
        saveEntityToJSON(ej, e);
        cJSON_AddItemToArray(ents, ej);
    }

    char* str = cJSON_Print(root);
    if (str) {
        FILE* f = fopen(SETTINGS_FILE, "w");
        if (f) {
            fputs(str, f);
            fclose(f);
        }
        cJSON_free(str);
    }
    cJSON_Delete(root);
    settings_dirty = false;
}

void App::loadSettings() {
    if (IsDocumentationMode()) return;
    FILE* f = fopen(SETTINGS_FILE, "r");
    if (!f) return;

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return; }

    char* buf = (char*)malloc(sz + 1);
    fread(buf, 1, sz, f);
    buf[sz] = 0;
    fclose(f);

    cJSON* root = cJSON_Parse(buf);
    free(buf);
    if (!root) return;

    cJSON* val;

    // Console
    if ((val = cJSON_GetObjectItem(root, "console_log_rate")))    console_log_rate = val->valueint;
    if ((val = cJSON_GetObjectItem(root, "console_max")))         console_max = val->valueint;
    if ((val = cJSON_GetObjectItem(root, "console_auto_scroll"))) console_auto_scroll = cJSON_IsTrue(val);

    // Recording
    if ((val = cJSON_GetObjectItem(root, "record_rate_hz")))    record_rate_hz = val->valueint;

    // Input source (old Manual=0 maps to Plugin; only CapturePlayback/Plugin are valid now)
    if ((val = cJSON_GetObjectItem(root, "input_source"))) {
        int src = val->valueint;
        if (src == (int)InputSource::CapturePlayback)
            input_source = InputSource::CapturePlayback;
        else
            input_source = InputSource::Plugin;
    }

    // Active plugin name (resolve index after plugins are already scanned)
    if ((val = cJSON_GetObjectItem(root, "active_plugin_name"))) {
        const char* pname = val->valuestring;
        if (pname) {
            for (int i = 0; i < plugin_mgr.pluginCount(); i++) {
                const char* n = plugin_mgr.pluginName(i);
                if (n && strcmp(n, pname) == 0) { active_plugin_idx = i; break; }
            }
        }
    }
    // Default to Manual Sliders plugin if no plugin was saved or found
    if (active_plugin_idx < 0 && input_source == InputSource::Plugin) {
        for (int i = 0; i < plugin_mgr.pluginCount(); i++) {
            const char* n = plugin_mgr.pluginName(i);
            if (n && strcmp(n, "Manual Sliders") == 0) { active_plugin_idx = i; break; }
        }
        if (active_plugin_idx < 0 && plugin_mgr.pluginCount() > 0)
            active_plugin_idx = 0;
    }

    // Dynamics panel selection + panel visibility
    if ((val = cJSON_GetObjectItem(root, "selected_dynamics_id")))
        selected_dynamics_id = val->valueint;
    if ((val = cJSON_GetObjectItem(root, "show_dynamics")))
        show_dynamics = cJSON_IsTrue(val);
    if ((val = cJSON_GetObjectItem(root, "show_console")))
        show_console = cJSON_IsTrue(val);
    if ((val = cJSON_GetObjectItem(root, "show_data_streams")))
        show_data_streams = cJSON_IsTrue(val);

    // Entities — clear and recreate from saved data
    cJSON* ents = cJSON_GetObjectItem(root, "entities");
    if (ents && cJSON_IsArray(ents) && cJSON_GetArraySize(ents) > 0) {
        // Close any existing HIL serial connections before clearing
        for (auto& e : entities) {
            if (e.serial) { e.serial->close(); e.serial.reset(); }
        }
        entities.clear();

        cJSON* ej;
        cJSON_ArrayForEach(ej, ents) {
            // Determine type
            EntityType type = EntityType::SIL;
            if ((val = cJSON_GetObjectItem(ej, "type")) && val->valueint == (int)EntityType::HIL)
                type = EntityType::HIL;

            // Get name
            const char* name = "Entity";
            if ((val = cJSON_GetObjectItem(ej, "name"))) name = val->valuestring;

            Entity& e = addEntity(name, type);
            loadEntityFromJSON(e, ej);
        }
    }

    cJSON_Delete(root);
    log(-1, "system", "Settings loaded from %s (%d entities)", SETTINGS_FILE, (int)entities.size());
}

Entity& App::addEntity(const char* name, EntityType type, HilBackend backend) {
    Entity e = {};
    e.id = next_entity_id++;
    snprintf(e.name, sizeof(e.name), "%s", name);
    e.type = type;
    e.hil_backend = backend;
    e.enabled = true;
    e.config.initDefaults();
    memset(&e.state, 0, sizeof(e.state));
    memset(&e.transport, 0, sizeof(e.transport));
    int ci = e.id % g_num_colors;
    memcpy(e.color, g_entity_colors[ci], sizeof(e.color));
    e.show_card = true;
    e.show_settings = false;
    e.show_platform = false;
    e.show_dynamics = false;
    e.show_console = false;
    e.viz_cam = {0.8f, 0.45f, 0.0f}; // azimuth ~45°, elevation ~25°, auto distance
    e.viz_split_ratio = 0.55f;
    e.rate_ik_hz = 0.0f;
    e.rate_tx_hz = 0.0f;
    e.rate_tel_hz = 0.0f;
    // MP6 backend defaults (plain fields; link/session modules fill the rest).
    // auto_connect is deliberately false: opening a COM port resets a Mega
    // (DTR) and there is no auto-recovery path for MP6 in this build.
    e.mp6 = Mp6EntityState{};
    e.mp6.link_baud = 115200;              // documented 115200 8N1, RTS/DTR low
    e.mp6.onboard_control = true;          // onboard loop; host sends cmd 2
    e.mp6.cmd2_hz = 60;                    // documented 60 Hz cadence
    e.mp6.lifecycle = Mp6Lifecycle::Disconnected;
    snprintf(e.mp6.lifecycle_note, sizeof(e.mp6.lifecycle_note),
             "no MP6 session attached in this build");
    e.mp6.auto_connect = false;
    // HIL fields
    e.serial = nullptr;
    e.hil_tx_hz = 60;
    e.hil_last_tx_time = 0.0;
    e.hil_tel_seq = 0;
    memset(e.hil_fk_pose, 0, sizeof(e.hil_fk_pose));
    memset(e.hil_tel_prev, 0, sizeof(e.hil_tel_prev));
    memset(e.hil_tel_curr, 0, sizeof(e.hil_tel_curr));
    e.hil_tel_prev_time = 0.0;
    e.hil_tel_curr_time = 0.0;
    e.hil_tel_target_hz = 30;
    e.hil_tick_rate_us = 4;
    memset(e.hil_port, 0, sizeof(e.hil_port));
    e.hil_baud = 921600;
    e.hil_auto_connect = !IsDocumentationMode();
    e.hil_last_reconnect = 0.0;
    memset(e.hil_tx_raw_f, 0, sizeof(e.hil_tx_raw_f));
    e.hil_network = false;
    snprintf(e.hil_host, sizeof(e.hil_host), "%s", "192.168.1.168");  // Voron default
    e.hil_udp_port = 8767;
    e.hil_tcp_port = 8789;
    e.hil_cap_raw = false;
    e.hil_raw_mode = false;
    memset(e.hil_fingerprint, 0, sizeof(e.hil_fingerprint));
    memset(e.hil_fw_version, 0, sizeof(e.hil_fw_version));
    e.hil_proto_ver = 0;
    e.hil_handshake_ok = false;
    e.hil_handshake_pending = false;
    e.hil_handshake_phase = HandshakePhase::Idle;
    e.hil_device_params.clear();
    e.hil_handshake_start = 0.0;
    memset(e.hil_handshake_msg, 0, sizeof(e.hil_handshake_msg));
    e.hil_hs_attempts = 0;
    e.hil_hs_last_send = 0.0;
    // Init TX raw to center (home) so platform doesn't jerk on connect
    {
        float max_raw = txMaxRawForBitDepth(e.config.bit_depth);
        for (int i = 0; i < 6; i++) e.hil_tx_raw[i] = (uint32_t)(max_raw * 0.5f);
    }
    // Init history ring buffers
    memset(e.history_angles, 0, sizeof(e.history_angles));
    memset(e.history_input, 0, sizeof(e.history_input));
    memset(e.history_time, 0, sizeof(e.history_time));
    e.history_head = 0;
    e.history_count = 0;
    // Init placeholder spectrum
    memset(e.spectrum, 0, sizeof(e.spectrum));
    e.spectrum_freq_max = 30.0f;  // placeholder max freq
    // Init dynamics transition
    memset(e.last_scaled_pct, 0, sizeof(e.last_scaled_pct));
    e.dyn_transition_active = false;
    e.dyn_transition_start = 0.0;
    e.dyn_transition_duration = 1.5f;
    // Init cue auto-sync snapshots
    memset(e.cue_pushed, 0, sizeof(e.cue_pushed));
    memset(e.cue_prev, 0, sizeof(e.cue_prev));
    e.cue_change_time = 0.0;
    e.cue_sync_valid = false;
    e.cue_push_time = 0.0;
    e.cue_persisted = true;
    entities.push_back(e);
    log(e.id, "system", "Entity '%s' created (type: %s%s)", name,
        type == EntityType::SIL ? "SIL" : "HIL",
        type == EntityType::SIL ? "" :
        (backend == HilBackend::MP6 ? " [MP6 Motum Mega]" : " [ESP32]"));
    return entities.back();
}

void App::removeEntity(int id) {
    for (auto it = entities.begin(); it != entities.end(); ++it) {
        if (it->id == id) {
            log(id, "system", "Entity '%s' removed", it->name);
            // Drop any live MP6 session/link first: nothing may outlive the
            // entity state it mirrors.
            mp6_runtimes.erase(id);
            entities.erase(it);
            return;
        }
    }
}

Entity* App::findEntity(int id) {
    for (auto& e : entities) {
        if (e.id == id) return &e;
    }
    return nullptr;
}

void App::log(int entity_id, const char* source, const char* fmt, ...) {
    LogEntry entry = {};
    entry.timestamp = frame_time;
    entry.entity_id = entity_id;
    snprintf(entry.source, sizeof(entry.source), "%s", source);

    va_list args;
    va_start(args, fmt);
    vsnprintf(entry.message, sizeof(entry.message), fmt, args);
    va_end(args);

    console_log.push_back(entry);
    if ((int)console_log.size() > console_max) {
        console_log.erase(console_log.begin(),
                          console_log.begin() + ((int)console_log.size() - console_max));
    }

    // Also print to stdout
    printf("[E%d/%s] %s\n", entity_id, source, entry.message);
}

// ── HIL Serial Line Handler (multi-step handshake + logging) ─────────
//
// Handshake sequence:
//   1. FINGERPRINT? → verify device identity (MAC, fw version, protocol)
//   2. CONFIG?      → read geometry (RD, PD, L1, L2, height, theta_r, theta_p, servo config)
//   3. BITS?        → read input bit depth
//   4. Validate     → compare device params against app expectations
//   5. Sync         → push any corrections (BITS:N) then enable motion
//
// Motion packets are BLOCKED until phase reaches Ready.

static void advanceHandshake(App& app, Entity& e) {
    if (!e.serial) return;
    switch (e.hil_handshake_phase) {
        case HandshakePhase::WaitFingerprint:
            // Fingerprint received — query geometry next
            e.hil_handshake_phase = HandshakePhase::WaitConfig;
            e.hil_hs_last_send = app.frame_time;
            e.hil_hs_attempts = 0;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Querying geometry...");
            e.serial->write((const uint8_t*)"\0\0\0\0", 4);
            e.serial->sendCommand("CONFIG?");
            break;

        case HandshakePhase::WaitConfig:
            // Geometry received — query bit depth next
            e.hil_handshake_phase = HandshakePhase::WaitBits;
            e.hil_hs_last_send = app.frame_time;
            e.hil_hs_attempts = 0;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Querying bit depth...");
            e.serial->write((const uint8_t*)"\0\0\0\0", 4);
            e.serial->sendCommand("BITS?");
            break;

        case HandshakePhase::WaitBits: {
            // BITS? response received — now push our desired bit depth and wait for echo.
            // Motion stays BLOCKED until firmware confirms the correct BITS:N back.
            auto& dp = e.hil_device_params;

            // Check protocol version compatibility
            if (dp.proto_ver != 1) {
                app.log(e.id, "hil", "WARNING: Unknown protocol version %d (expected 1)", dp.proto_ver);
            }

            // Send BITS:N immediately — firmware will echo it back as "BITS:N,max_raw=..."
            {
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "BITS:%d", e.config.bit_depth);
                e.serial->sendCommand(cmd);
                app.log(e.id, "hil", "BITS:%d sent — waiting for firmware echo...", e.config.bit_depth);
            }
            e.hil_handshake_phase = HandshakePhase::WaitBitsConfirm;
            e.hil_hs_last_send = app.frame_time;
            e.hil_hs_attempts = 0;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Confirming bit depth...");
            break;
        }

        case HandshakePhase::WaitBitsConfirm: {
            // Firmware echoed BITS:N confirmation — now validate and finalize.
            e.hil_handshake_phase = HandshakePhase::Validating;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Validating parameters...");

            auto& dp = e.hil_device_params;

            // Check geometry sync between app settings and device
            e.hil_geo_synced = true;
            if (dp.config_received) {
                const auto& geo = e.config.geometry;
                float diffs[] = {
                    fabsf(geo.RD - dp.RD),
                    fabsf(geo.PD - dp.PD),
                    fabsf(geo.ServoArmLengthL1 - dp.L1),
                    fabsf(geo.ConnectingArmLengthL2 - dp.L2),
                    fabsf(geo.platformHeight - dp.height),
                    fabsf(geo.theta_r - dp.theta_r),
                    fabsf(geo.theta_p - dp.theta_p),
                };
                for (int i = 0; i < 7; i++) {
                    if (diffs[i] > 0.05f) { e.hil_geo_synced = false; break; }
                }

                if (!e.hil_geo_synced) {
                    app.log(e.id, "hil", "Geometry out of sync — pushing app settings to device.");
                    app.log(e.id, "hil", "  App:    RD=%.2f PD=%.2f L1=%.2f L2=%.2f H=%.2f theta_r=%.2f theta_p=%.2f",
                        geo.RD, geo.PD, geo.ServoArmLengthL1, geo.ConnectingArmLengthL2,
                        geo.platformHeight, geo.theta_r, geo.theta_p);
                    app.log(e.id, "hil", "  Device: RD=%.2f PD=%.2f L1=%.2f L2=%.2f H=%.2f theta_r=%.2f theta_p=%.2f",
                        dp.RD, dp.PD, dp.L1, dp.L2, dp.height, dp.theta_r, dp.theta_p);
                    // Push app geometry to device (app is authoritative)
                    char cmd[128];
                    snprintf(cmd, sizeof(cmd), "CONFIG:RD=%.4f", geo.RD);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:PD=%.4f", geo.PD);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:L1=%.4f", geo.ServoArmLengthL1);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:L2=%.4f", geo.ConnectingArmLengthL2);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:height=%.4f", geo.platformHeight);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:theta_r=%.4f", geo.theta_r);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:theta_p=%.4f", geo.theta_p);
                    e.hil_cmd_queue.push_back(cmd);
                } else {
                    app.log(e.id, "hil", "Geometry in sync with device.");
                }
                // Import theta_s from device (not configurable via app UI, firmware-authoritative)
                if (dp.config_received) {
                    bool has_theta_s = false;
                    for (int i = 0; i < 6; i++) if (dp.theta_s[i] != 0.0f) { has_theta_s = true; break; }
                    if (has_theta_s) {
                        for (int i = 0; i < 6; i++) e.config.geometry.theta_s[i] = dp.theta_s[i];
                        e.config.rebuildPlatform();
                    }
                }
            }

            {
                e.hil_handshake_phase = HandshakePhase::Ready;
                e.hil_handshake_ok = true;
                // Reset FK pose so stale pose from prior connection doesn't
                // corrupt the visualization on the new connection.
                memset(e.hil_fk_pose, 0, sizeof(e.hil_fk_pose));
                e.hil_tel_seq = -1;
                e.state.ik_seq = 0;
                if (e.hil_geo_synced) {
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                        "Ready — fw %s, proto %d", dp.fw_version, dp.proto_ver);
                } else {
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                        "Ready — GEOMETRY OUT OF SYNC");
                }

                // Auto-detect platform type from firmware platform_id
                if (strstr(dp.platform_id, "mini") != nullptr) {
                    e.config.platform_type = PlatformType::PWMServo;
                    // Import servo params from device if available
                    if (dp.config_received) {
                        for (int i = 0; i < 6; i++)
                            e.config.servo.center_us[i] = dp.servo_center[i];
                        if (dp.pulse_per_rad > 0.0f)
                            e.config.servo.pulse_per_rad = dp.pulse_per_rad;
                    }
                    app.log(e.id, "hil", "Platform type: PWM Servo (Mini-6DOF)");
                } else {
                    e.config.platform_type = PlatformType::Stepper;
                    app.log(e.id, "hil", "Platform type: Stepper");
                }

                // Send entity color to RGB LED on controller
                {
                    int r = (int)(e.color[0] * 255.0f);
                    int g = (int)(e.color[1] * 255.0f);
                    int b = (int)(e.color[2] * 255.0f);
                    char ledcmd[32];
                    snprintf(ledcmd, sizeof(ledcmd), "LED:%d,%d,%d", r, g, b);
                    e.hil_cmd_queue.push_back(ledcmd);
                }

                // Lock input source to serial, query stats, set telemetry rate + tick rate
                e.hil_cmd_queue.push_back("INPUT:SERIAL");
                e.hil_cmd_queue.push_back("INPUT_STAT");
                {
                    char telcmd[32];
                    snprintf(telcmd, sizeof(telcmd), "TELRATE:%d", e.hil_tel_target_hz);
                    e.hil_cmd_queue.push_back(telcmd);
                }
                {
                    char tickcmd[32];
                    snprintf(tickcmd, sizeof(tickcmd), "TICKRATE:%d", e.hil_tick_rate_us);
                    e.hil_cmd_queue.push_back(tickcmd);
                }

                app.log(e.id, "hil", "Handshake complete — motion enabled (fw %s, proto %d, bits %d, tel %dHz, tick %dµs)",
                    dp.fw_version, dp.proto_ver,
                    dp.bits_received ? dp.bit_depth : e.config.bit_depth,
                    e.hil_tel_target_hz, e.hil_tick_rate_us);
            }
            e.hil_handshake_pending = false;
            break;
        }
        default:
            break;
    }
}

void App::handleHilLine(int entity_id, const char* line) {
    // Always log the line to console
    log(entity_id, "esp32", "%s", line);

    Entity* e = findEntity(entity_id);
    if (!e) return;

    // ── Phase 1: FINGERPRINT response ────────────────────────────────
    // Format: "FINGERPRINT:AABBCCDDEEFF,fw=1.0.0,proto=1,platform=mini6dof"
    if (strncmp(line, "FINGERPRINT:", 12) == 0) {
        const char* payload = line + 12;

        // Extract MAC (12 hex chars before first comma)
        char mac[16] = {};
        const char* comma1 = strchr(payload, ',');
        if (comma1) {
            int len = (int)(comma1 - payload);
            if (len > 0 && len < (int)sizeof(mac)) {
                memcpy(mac, payload, len);
                mac[len] = '\0';
            }
        }

        // Extract fw version
        char fw[16] = {};
        const char* fw_start = strstr(payload, "fw=");
        if (fw_start) {
            fw_start += 3;
            const char* fw_end = strchr(fw_start, ',');
            int len = fw_end ? (int)(fw_end - fw_start) : (int)strlen(fw_start);
            if (len > 0 && len < (int)sizeof(fw)) {
                memcpy(fw, fw_start, len);
                fw[len] = '\0';
            }
        }

        // Extract protocol version
        int proto = 0;
        const char* proto_start = strstr(payload, "proto=");
        if (proto_start) proto = atoi(proto_start + 6);

        // Extract platform ID
        char plat[16] = {};
        const char* plat_start = strstr(payload, "platform=");
        if (plat_start) {
            plat_start += 9;
            const char* plat_end = strchr(plat_start, ',');
            int len = plat_end ? (int)(plat_end - plat_start) : (int)strlen(plat_start);
            if (len > 0 && len < (int)sizeof(plat)) {
                memcpy(plat, plat_start, len);
                plat[len] = '\0';
            }
        }

        // Store in device params
        auto& dp = e->hil_device_params;
        snprintf(dp.fingerprint, sizeof(dp.fingerprint), "%s", mac);
        snprintf(dp.fw_version, sizeof(dp.fw_version), "%s", fw);
        snprintf(dp.platform_id, sizeof(dp.platform_id), "%s", plat);
        dp.proto_ver = proto;

        // Also store in legacy fields for UI/save compat
        snprintf(e->hil_fw_version, sizeof(e->hil_fw_version), "%s", fw);
        e->hil_proto_ver = proto;

        // Raw-HIL capability gate: newer firmware advertises "rawhil=1" or a
        // caps token containing "raw" (e.g. caps=raw). Old firmware omits it →
        // hil_cap_raw stays false → baked path unchanged.
        {
            bool cap = false;
            if (strstr(payload, "rawhil=1")) cap = true;
            else { const char* caps = strstr(payload, "caps=");
                   if (caps && strstr(caps, "raw")) cap = true; }
            e->hil_cap_raw = cap;
            if (!cap) e->hil_raw_mode = false;  // can't stream raw to a baked-only device
        }

        // Fingerprint verification — ONLY advance handshake during WaitFingerprint.
        // Stale FINGERPRINT responses (from manual button clicks or previous sessions)
        // must not corrupt the state machine when we're at a different phase.
        bool during_fp_phase = (e->hil_handshake_phase == HandshakePhase::WaitFingerprint);

        if (e->hil_fingerprint[0] == '\0') {
            // First connect — store fingerprint
            snprintf(e->hil_fingerprint, sizeof(e->hil_fingerprint), "%s", mac);
            log(e->id, "hil", "Device paired: %s (fw %s, proto %d, platform %s)", mac, fw, proto, plat);
            settings_dirty = true;
            if (during_fp_phase) advanceHandshake(*this, *e);
        } else if (strcmp(e->hil_fingerprint, mac) == 0) {
            // Same device
            if (during_fp_phase) {
                log(e->id, "hil", "Device verified: %s (fw %s, proto %d, platform %s)", mac, fw, proto, plat);
                advanceHandshake(*this, *e);
            }
            // else: stale duplicate — ignore silently
        } else {
            // DIFFERENT device — reject!
            e->hil_handshake_ok = false;
            e->hil_handshake_phase = HandshakePhase::Failed;
            snprintf(e->hil_handshake_msg, sizeof(e->hil_handshake_msg),
                "FINGERPRINT MISMATCH — expected %s", e->hil_fingerprint);
            snprintf(e->hil_last_error, sizeof(e->hil_last_error),
                "FINGERPRINT MISMATCH\nConnected device: %s\nExpected: %s\nUse \"Clear Fingerprint\" to pair a new device.",
                mac, e->hil_fingerprint);
            log(e->id, "hil", "FINGERPRINT MISMATCH! Expected %s, got %s — disconnecting",
                e->hil_fingerprint, mac);
            log(e->id, "hil", "Clear fingerprint in Settings to pair a new device");
            if (e->serial) {
                e->serial->close();
                e->serial.reset();
            }
            e->transport.usb_connected = false;
            e->hil_auto_connect = false;
        }
        return;
    }

    // ── Phase 2: CONFIG response ─────────────────────────────────────
    // Format: "CONFIG:RD=15.75,PD=16.00,L1=7.25,L2=28.50,height=25.52,theta_r=10.00,theta_p=30.00"
    // Also accept "CONFIG:OK key=value" confirmations after push
    if (strncmp(line, "CONFIG:", 7) == 0) {
        // Skip CONFIG:OK and CONFIG:ERR confirmations (just log them)
        if (strncmp(line + 7, "OK ", 3) == 0 || strncmp(line + 7, "ERR", 3) == 0) {
            // Already logged via general serial output
            return;
        }

        // Accept CONFIG response during handshake or after (for re-query)
        bool during_handshake = (e->hil_handshake_phase == HandshakePhase::WaitConfig);
        auto& dp = e->hil_device_params;
        const char* p = line + 7;

        // Parse key=value pairs
        const char* v;
        if ((v = strstr(p, "RD=")))    dp.RD = (float)atof(v + 3);
        if ((v = strstr(p, "PD=")))    dp.PD = (float)atof(v + 3);
        if ((v = strstr(p, "L1=")))    dp.L1 = (float)atof(v + 3);
        if ((v = strstr(p, "L2=")))    dp.L2 = (float)atof(v + 3);
        if ((v = strstr(p, "height="))) dp.height = (float)atof(v + 7);
        if ((v = strstr(p, "theta_r="))) dp.theta_r = (float)atof(v + 8);
        if ((v = strstr(p, "theta_p="))) dp.theta_p = (float)atof(v + 8);
        if ((v = strstr(p, "theta_s="))) {
            sscanf(v + 8, "%f,%f,%f,%f,%f,%f",
                &dp.theta_s[0], &dp.theta_s[1], &dp.theta_s[2],
                &dp.theta_s[3], &dp.theta_s[4], &dp.theta_s[5]);
        }
        dp.config_received = true;

        // Re-check geometry sync after any CONFIG update
        if (!during_handshake) {
            const auto& geo = e->config.geometry;
            e->hil_geo_synced = true;
            float diffs[] = {
                fabsf(geo.RD - dp.RD), fabsf(geo.PD - dp.PD),
                fabsf(geo.ServoArmLengthL1 - dp.L1), fabsf(geo.ConnectingArmLengthL2 - dp.L2),
                fabsf(geo.platformHeight - dp.height),
                fabsf(geo.theta_r - dp.theta_r), fabsf(geo.theta_p - dp.theta_p),
            };
            for (int i = 0; i < 7; i++) {
                if (diffs[i] > 0.05f) { e->hil_geo_synced = false; break; }
            }
            if (e->hil_geo_synced) {
                log(e->id, "hil", "Geometry sync confirmed — device matches app settings.");
                snprintf(e->hil_handshake_msg, sizeof(e->hil_handshake_msg),
                    "Ready — fw %s, proto %d", dp.fw_version, dp.proto_ver);
            } else {
                log(e->id, "hil", "Geometry still out of sync after push.");
            }
        }
        // Advance handshake after CONFIG (stepper platforms don't send SERVO:)
        if (during_handshake) {
            advanceHandshake(*this, *e);
        }
        return;
    }

    // Format: "SERVO:center=1500,1500,1500,1500,1500,1500,pulse_per_rad=1018.6"
    if (strncmp(line, "SERVO:", 6) == 0 &&
        (e->hil_handshake_phase == HandshakePhase::WaitConfig || e->hil_handshake_phase == HandshakePhase::Ready)) {
        auto& dp = e->hil_device_params;
        const char* p = line + 6;

        const char* center = strstr(p, "center=");
        if (center) {
            sscanf(center + 7, "%d,%d,%d,%d,%d,%d",
                &dp.servo_center[0], &dp.servo_center[1], &dp.servo_center[2],
                &dp.servo_center[3], &dp.servo_center[4], &dp.servo_center[5]);
        }
        const char* ppr = strstr(p, "pulse_per_rad=");
        if (ppr) dp.pulse_per_rad = (float)atof(ppr + 14);

        // Both CONFIG and SERVO received — advance
        advanceHandshake(*this, *e);
        return;
    }

    // ── Phase 3: BITS? response ─────────────────────────────────────
    // Format: "BITS:12,max_raw=4095" — firmware reports current bit depth
    if (strncmp(line, "BITS:", 5) == 0 &&
        e->hil_handshake_phase == HandshakePhase::WaitBits) {
        auto& dp = e->hil_device_params;
        dp.bit_depth = atoi(line + 5);
        const char* mr = strstr(line, "max_raw=");
        if (mr) dp.max_raw = (float)atof(mr + 8);
        dp.bits_received = true;
        // Sends BITS:N and transitions to WaitBitsConfirm
        advanceHandshake(*this, *e);
        return;
    }

    // ── Phase 3b: BITS:N echo confirmation ──────────────────────────
    // Firmware echoes "BITS:N,max_raw=..." after we send BITS:N — confirms
    // the correct bit depth is active before motion TX begins.
    if (strncmp(line, "BITS:", 5) == 0 &&
        e->hil_handshake_phase == HandshakePhase::WaitBitsConfirm) {
        auto& dp = e->hil_device_params;
        int confirmed = atoi(line + 5);
        const char* mr = strstr(line, "max_raw=");
        if (mr) dp.max_raw = (float)atof(mr + 8);
        dp.bit_depth = confirmed;
        dp.bits_received = true;
        log(e->id, "hil", "BITS:%d confirmed by firmware (max_raw=%.0f) — motion enabled",
            confirmed, dp.max_raw);
        advanceHandshake(*this, *e);
        return;
    }
}

// ── MP6 backend integration (app-owned glue) ────────────────────────
//
// Separation rules encoded here:
//   * MP6 never uses ITransport/COBS. Its FE FC host frames and 80 81 82
//     telemetry are handled by the mp6_protocol codec through its own raw
//     serial link (mp6_link/mp6_session modules when present).
//   * The background hilTxLoop() never sends MP6 frames and never races entity
//     state; it is gate-guarded to ESP32 entities below.
//   * User intent alone never establishes initialized/parked success: state
//     transitions are reported from observed telemetry only.

bool App::mp6ConnectWithLink(Entity& e, std::unique_ptr<mp6::Link> link, std::string* error) {
    auto err = [&](const char* m) {
        snprintf(e.mp6.last_error, sizeof(e.mp6.last_error), "%s", m);
        if (error) *error = m;
        return false;
    };
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6)
        return err("entity is not an MP6 HIL entity");
    if (!link || !link->isOpen())
        return err("injected link is null or not open");
    if (e.mp6.link_open) return true;
    e.mp6.last_connect_attempt = frame_time;

    auto& rt = mp6_runtimes[e.id];
    if (!rt) rt = std::make_unique<Mp6Runtime>();
    rt->link = std::move(link);

    mp6::SessionConfig cfg;
    cfg.targetCadenceHz = 60; // Evidenced 60 Hz contract
    e.mp6.cmd2_hz = 60;
    rt->session = std::make_unique<mp6::Session>(cfg);
    rt->session->attach(rt->link.get());
    rt->session->setExpectedPlatformNumber(e.mp6.platform_number_expected);
    e.mp6.link_open = true;
    e.mp6.identity_ok = false;
    e.mp6.paired_config = false;
    e.mp6.feedback_valid = false;
    e.mp6.supplies_valid = false;
    e.mp6.pressure_valid = false;
    e.mp6.viz.solver_ready = false;
    e.mp6.viz.rod_lines_valid = false;
    e.mp6.step_target_valid = false;
    e.mp6.config_send_ok = false;
    e.mp6.pending_intent = Mp6Intent::None;
    e.mp6.tel_seq = 0;
    e.mp6.tel_hz_est = 0;
    memset(e.mp6.last_error, 0, sizeof(e.mp6.last_error));
    // Validate and stage both snapshots; invalid geometry clears authority
    // after config parsing so the parser cannot restore Configure readiness.
    mp6ApplyPlatformConfig(*this, e);
    mp6StageDesignInputs(*this, e);
    snprintf(e.mp6.lifecycle_note, sizeof(e.mp6.lifecycle_note),
             "link open; no identity echo observed yet");
    snprintf(e.mp6.last_action, sizeof(e.mp6.last_action), "connect");
    log(e.id, "mp6", "Connected with link (raw MP6 link; mp6::Session single writer)");
    settings_dirty = true;
    return true;
}

bool App::mp6Connect(Entity& e, std::string* error) {
    auto err = [&](const char* m) {
        snprintf(e.mp6.last_error, sizeof(e.mp6.last_error), "%s", m);
        if (error) *error = m;
        return false;
    };
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6)
        return err("entity is not an MP6 HIL entity");
    if (IsDocumentationMode())
        return err("documentation mode: real COM connections are disabled");
    if (e.mp6.link_open) return true;
    if (e.mp6.link_port[0] == '\0')
        return err("no COM port configured");
    e.mp6.last_connect_attempt = frame_time;

    auto slink = std::make_unique<mp6::SerialLink>();
    if (!slink->open(e.mp6.link_port, e.mp6.link_baud)) {
        std::string why = slink->lastError();
        snprintf(e.mp6.last_error, sizeof(e.mp6.last_error),
                 "COM open failed (%s): %s", e.mp6.link_port,
                 why.empty() ? "driver or OS error" : why.c_str());
        log(e.id, "mp6", "Connect refused: %s", e.mp6.last_error);
        e.mp6.link_open = false;
        return err(e.mp6.last_error);
    }
    return mp6ConnectWithLink(e, std::move(slink), error);
}

void App::mp6Disconnect(Entity& e) {
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6) return;
    e.mp6.axis_test_enabled = false;
    e.mp6.axis_test_entering = false;
    e.mp6.axis_test_returning = false;
    e.mp6.viz.axis_test_active = false;
    e.mp6.viz.axis_test_geometry_valid = false;
    e.mp6.viz.rod_lines_valid = false;
    for (int i = 0; i < 6; ++i) e.mp6.viz.measured_crank_valid[i] = false;
    if (auto it = mp6_runtimes.find(e.id); it != mp6_runtimes.end() && it->second) {
        it->second->axis_test_entering = false;
        it->second->axis_test_entry_waiting_tx = false;
        it->second->axis_test_return_waiting_tx = false;
    }
    for (int i = 0; i < 6; ++i) e.mp6.axis_test_setpoint_deg[i] = e.mp6.axis_test_offset_deg[i] = 0.0f,
        e.mp6.axis_test_offset_steps[i] = e.mp6.axis_test_target_steps[i] = 0;
    auto it = mp6_runtimes.find(e.id);
    if (it != mp6_runtimes.end()) {
        // Detach first so the session can never transmit through a link we are
        // about to destroy; then drop both halves. The controller is left
        // exactly as it was — app-side disconnect is not a shutdown intent and
        // it is never presented as one.
        Mp6Runtime& rt = *it->second;
        if (rt.session) {
            if (rt.session->state() != mp6::State::Closed)
                log(e.id, "mp6",
                    "Disconnecting %s at session state %s — controller left as-is, its state is unverified",
                    e.mp6.link_port, mp6::toString(rt.session->state()));
            rt.session->detach();
        }
        mp6_runtimes.erase(it);
    }
    e.mp6.link_open = false;
    e.rate_tx_hz = 0.0f;
    e.mp6.tel_hz_est = 0;
    e.mp6.identity_ok = false;
    e.mp6.paired_config = false;
    e.mp6.sending_config = false;
    e.mp6.pending_intent = Mp6Intent::None;
    e.mp6.lifecycle = Mp6Lifecycle::Disconnected;
    snprintf(e.mp6.lifecycle_note, sizeof(e.mp6.lifecycle_note),
             "link closed; controller state unverified — re-pair on the next connect");
    // link_port, link_baud and auto_connect are deliberately kept: disconnect
    // should not silently forget the user's connection setup.
    settings_dirty = true;
}

bool App::mp6Command(Entity& e, Mp6Intent intent, std::string* error) {
    auto err = [&](const char* m) {
        snprintf(e.mp6.last_error, sizeof(e.mp6.last_error), "%s", m);
        if (error) *error = m;
        return false;
    };
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6)
        return err("entity is not an MP6 HIL entity");
    if (intent == Mp6Intent::None) return err("no intent given");
    if (intent == Mp6Intent::Stop || intent == Mp6Intent::Estop ||
        intent == Mp6Intent::Park || intent == Mp6Intent::Shutdown ||
        intent == Mp6Intent::Disconnect) {
        e.mp6.axis_test_enabled = false;
        e.mp6.axis_test_entering = false;
        e.mp6.axis_test_returning = false;
        e.mp6.axis_test_setpoint_deg[0] = e.mp6.axis_test_setpoint_deg[1] =
        e.mp6.axis_test_setpoint_deg[2] = e.mp6.axis_test_setpoint_deg[3] =
        e.mp6.axis_test_setpoint_deg[4] = e.mp6.axis_test_setpoint_deg[5] = 0.0f;
        e.mp6.axis_test_offset_deg[0] = e.mp6.axis_test_offset_deg[1] =
        e.mp6.axis_test_offset_deg[2] = e.mp6.axis_test_offset_deg[3] =
        e.mp6.axis_test_offset_deg[4] = e.mp6.axis_test_offset_deg[5] = 0.0f;
        e.mp6.axis_test_offset_steps[0] = e.mp6.axis_test_offset_steps[1] =
        e.mp6.axis_test_offset_steps[2] = e.mp6.axis_test_offset_steps[3] =
        e.mp6.axis_test_offset_steps[4] = e.mp6.axis_test_offset_steps[5] = 0;
        for (int i = 0; i < 6; ++i) e.mp6.axis_test_target_steps[i] = 0;
        auto rt = mp6_runtimes.find(e.id);
        if (rt != mp6_runtimes.end() && rt->second) {
            rt->second->axis_test_entering = false;
            rt->second->axis_test_entry_waiting_tx = false;
            rt->second->axis_test_returning = false;
            rt->second->axis_test_return_waiting_tx = false;
        }
    }
    bool is_disconnect = (intent == Mp6Intent::Disconnect);
    auto runtime = mp6_runtimes.find(e.id);
    bool attached_safety = (intent == Mp6Intent::Stop || intent == Mp6Intent::Estop)
        && runtime != mp6_runtimes.end() && runtime->second && runtime->second->session;
    if (!is_disconnect && !e.mp6.link_open && !attached_safety)
        return err("MP6 link is not open — connect explicitly first");
    // The onboard fault latch refuses everything except disconnect: TX is
    // suppressed wholesale by the session; recovery is re-pair, not a clear.
    if (!is_disconnect && intent != Mp6Intent::Estop && e.mp6.fault_latched) {
        char m[192];
        snprintf(m, sizeof(m),
                 "onboard fault latch active (code %u): TX suppressed; recovery is re-pair, not a manual clear",
                 e.mp6.fault_code);
        return err(m);
    }
    auto priority = [](Mp6Intent in) {
        return in == Mp6Intent::Estop ? 3 : in == Mp6Intent::Disconnect ? 2 : in == Mp6Intent::Stop ? 1 : 0;
    };
    if (priority(e.mp6.pending_intent) > priority(intent))
        return err("a stronger safety intent is already pending");
    if (intent == Mp6Intent::Start && e.mp6.estop_latched)
        return err("E-STOP latch requires explicit known-state recovery");
    switch (intent) {
        case Mp6Intent::Identify:
            if (mp6IsMotorPowerRequested(e)) {
                if (error) *error = "cannot identify while motors are enabled; shut down first";
                return false;
            }
            // nothing beyond an open link required; the session probes for the
            // type-42 echo (byte 7 == 3, byte 9 == 253) on its own cadence
            break;
        case Mp6Intent::Configure:
            if (!e.mp6.identity_ok)
                return err("controller identity not established (no verified type-42 echo); run Identify first");
            if (!e.mp6.config_ready)
                return err(e.mp6.config_note[0] ? e.mp6.config_note
                                                : "platform configuration not imported/validated; cannot send configs");
            if (e.mp6.design.supplied == false)
                return err("design attachment radii are not supplied; geometry is incomplete — motion stays blocked");
            break;
        case Mp6Intent::Initialize:
            if (!e.mp6.paired_config)
                return err("platform pairing not established (no observed selector-1 platform number)");
            if (!e.mp6.config_send_ok)
                return err("configuration transaction not verified (no observed ConfigSendOK); Initialize refused");
            if (auto it = mp6_runtimes.find(e.id); it == mp6_runtimes.end() || !it->second || !it->second->neutral_valid || !it->second->solver_ready || it->second->geometry_dirty || !e.mp6.design.supplied || e.mp6.design.pivot_z_provenance != 2 || !e.mp6.design.point_zero_supplied)
                return err("neutral zero-pose steps could not be solved by rotary crank solver; Initialize refused");
            break;
        case Mp6Intent::Start:
            if (!e.enabled) return err("enable the entity before Start");
            if (!e.mp6.config_send_ok)
                return err("Invalid Config Checksum — config transaction not verified; streaming refused");
            if (!e.mp6.av_ok)
                return err("AV_OK not observed from controller telemetry; streaming refused");
            if (e.mp6.pending_intent != Mp6Intent::None)
                return err("another MP6 intent is still staged and not yet dispatched; dispatch it first");
            break;
        case Mp6Intent::Stop:
        case Mp6Intent::Park:
        case Mp6Intent::Shutdown:
            break;
        case Mp6Intent::Estop:
            // Staged only by requestEstop; refusals are not differentiating here.
            break;
        default:
            break;
    }
    e.mp6.pending_intent = intent;
    snprintf(e.mp6.last_action, sizeof(e.mp6.last_action), "intent %d staged", (int)intent);
    snprintf(e.mp6.lifecycle_note, sizeof(e.mp6.lifecycle_note),
             "intent %d staged; dispatched and observed on the next update", (int)intent);
    log(e.id, "mp6", "Intent %d staged (dispatched by mp6UpdateEntity)", (int)intent);
    return true;
}

bool App::mp6AxisTestAllowed(const Entity& e, std::string* reason) const {
    auto fail = [&](const char* why) { if (reason) *reason = why; return false; };
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6) return fail("entity is not an MP6 HIL entity");
    if (IsDocumentationMode()) return fail("axis test is unavailable in documentation mode");
    auto it = mp6_runtimes.find(e.id);
    if (it == mp6_runtimes.end() || !it->second || !it->second->session ||
        it->second->session->state() != mp6::State::Running ||
        it->second->start_blend_pending || it->second->start_blend_active)
        return fail("axis test requires Running after the start blend completes");
    if (!it->second->axis_test_last_sent_valid) return fail("axis test requires a transmitted command-2 target");
    if (e.mp6.estop_latched || e.mp6.fault_latched) return fail("axis test blocked by E-STOP or fault latch");
    if (!e.enabled) return fail("enable the MP6 entity first");
    return true;
}

bool App::mp6AxisTest(Entity& e, bool enable, int axis, double offset_deg,
                      double cap_deg, double slew_deg_s, bool neutral_all,
                      std::string* error) {
    auto err = [&](const char* why) { if (error) *error = why; return false; };
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6) return err("entity is not an MP6 HIL entity");
    if (!std::isfinite(cap_deg) || (cap_deg >= 0.0 && (cap_deg < 0.1 || cap_deg > 15.0))) return err("cap_deg must be finite and between 0.1 and 15");
    if (!std::isfinite(slew_deg_s) || slew_deg_s < 0.1 || slew_deg_s > 30.0) return err("slew_deg_s must be finite and between 0.1 and 30");
    if (axis < -1 || axis > 5) return err("axis must be 1..6");
    const double effective_cap = cap_deg >= 0.0 ? cap_deg : axis >= 0 ? e.mp6.axis_test_cap_deg[axis] : 15.0;
    if (axis >= 0 && (!std::isfinite(offset_deg) || std::abs(offset_deg) > effective_cap)) return err("offset_deg exceeds the configured cap");
    if (e.mp6.axis_test_entering && axis >= 0 && offset_deg != 0.0)
        return err("axis test is entering neutral; nonzero setpoints are unavailable");
    if (enable) {
        std::string why;
        if (!mp6AxisTestAllowed(e, &why)) { if (error) *error = why; return false; }
        if (axis >= 0 && offset_deg != 0.0 && !e.mp6.axis_test_enabled &&
            mp6_runtimes.at(e.id)->axis_test_last_sent_steps != mp6_runtimes.at(e.id)->neutral_steps)
            return err("axis test is entering neutral; nonzero setpoints are unavailable");
    }
    if (e.mp6.axis_test_single_only && axis >= 0 && offset_deg != 0.0) {
        for (int i = 0; i < 6; ++i) if (i != axis &&
            (e.mp6.axis_test_setpoint_deg[i] != 0.0f || e.mp6.axis_test_offset_deg[i] != 0.0f))
            return err("single-axis-only is enabled; return the other axis to neutral first");
    }
    if (cap_deg >= 0.0) {
        if (axis >= 0) {
            e.mp6.axis_test_cap_deg[axis] = (float)cap_deg;
            e.mp6.axis_test_setpoint_deg[axis] = std::clamp(e.mp6.axis_test_setpoint_deg[axis], -(float)cap_deg, (float)cap_deg);
        } else for (int i = 0; i < 6; ++i) {
            e.mp6.axis_test_cap_deg[i] = (float)cap_deg;
            e.mp6.axis_test_setpoint_deg[i] = std::clamp(e.mp6.axis_test_setpoint_deg[i], -(float)cap_deg, (float)cap_deg);
        }
    }
    e.mp6.axis_test_slew_deg_s = (float)slew_deg_s;
    if (neutral_all) {
        for (float& v : e.mp6.axis_test_setpoint_deg) v = 0.0f;
    } else if (axis >= 0) e.mp6.axis_test_setpoint_deg[axis] = (float)offset_deg;
    if (!enable && e.mp6.axis_test_enabled) {
        e.mp6.axis_test_enabled = false;
        e.mp6.axis_test_entering = false;
        e.mp6.axis_test_returning = true;
        for (int i = 0; i < 6; ++i) e.mp6.axis_test_setpoint_deg[i] = 0.0f;
        auto it = mp6_runtimes.find(e.id);
        if (it != mp6_runtimes.end() && it->second) {
            it->second->axis_test_entering = false;
            it->second->axis_test_entry_waiting_tx = false;
            it->second->axis_test_returning = true;
            it->second->axis_test_return_waiting_tx = false;
            if (it->second->axis_test_last_sent_valid) for (int i = 0; i < 6; ++i) {
                const auto delta = it->second->axis_test_last_sent_steps[i] - it->second->neutral_steps[i];
                e.mp6.axis_test_offset_steps[i] = delta;
                e.mp6.axis_test_offset_deg[i] = static_cast<float>(delta / it->second->angle_to_step[i] * 180.0 / 3.14159265358979323846);
            }
        }
    } else if (enable) {
        const bool was_enabled = e.mp6.axis_test_enabled;
        e.mp6.axis_test_enabled = true;
        e.mp6.axis_test_returning = false;
        if (auto it = mp6_runtimes.find(e.id); it != mp6_runtimes.end() && it->second) {
            if (!was_enabled) {
                auto& rt = *it->second;
                const auto& sent = rt.axis_test_last_sent_steps;
                rt.axis_test_returning = false;
                rt.axis_test_return_waiting_tx = false;
                rt.start_blend_pending = false;
                rt.start_blend_active = false;
                rt.axis_test_entering = false;
                rt.axis_test_entry_waiting_tx = false;
                e.mp6.axis_test_entering = false;
                bool at_neutral = true;
                for (int i = 0; i < 6; ++i) {
                    rt.axis_test_entry_steps[i] = sent[i];
                    rt.axis_test_entry_position[i] = sent[i];
                    at_neutral = at_neutral && sent[i] == rt.neutral_steps[i];
                }
                rt.axis_test_entering = !at_neutral;
                rt.axis_test_entry_waiting_tx = false;
                rt.axis_test_last_sent_time = frame_time;
                e.mp6.axis_test_entering = !at_neutral;
            }
            it->second->axis_test_last_time = frame_time;
        }
    }
    return true;
}

// Import the raw platform configuration text. Typed parsing/validation and
// machine-constant checks belong to the mp6_config module; until it lands this
// stores the material and reports readiness as false (never guessed).
bool App::mp6ImportPlatformConfigText(Entity& e, const char* text, std::string* error) {
    if (e.mp6.pending_intent != Mp6Intent::None) {
        if (error) *error = "dispatch the pending MP6 intent before editing accepted inputs";
        return false;
    }
    auto err = [&](const char* m) { if (error) { *error = m; } return false; };
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6)
        return err("entity is not an MP6 HIL entity");
    if (mp6IsMotorPowerRequested(e)) {
        return err("cannot change platform configuration while motors are enabled; shut down first");
    }
    if (e.mp6.lifecycle == Mp6Lifecycle::Configuring ||
        e.mp6.lifecycle == Mp6Lifecycle::Initializing ||
        e.mp6.lifecycle == Mp6Lifecycle::Running ||
        e.mp6.lifecycle == Mp6Lifecycle::Parking) {
        return err("cannot import platform config while session is actively moving or configuring; shut down first");
    }
    if (!text || text[0] == '\0') {
        e.mp6.platform_config_present = false;
        e.mp6.platform_config_text[0] = '\0';
        mp6InvalidateAuthority(e, true, true);
        snprintf(e.mp6.config_note, sizeof(e.mp6.config_note), "no platform config imported");
        return true;   // clearing the import is a valid explicit action
    }
    mp6InvalidateAuthority(e, true, true);
    size_t n = strlen(text);
    if (n >= sizeof(e.mp6.platform_config_text))
        return err("platform config text exceeds 8191 bytes");
    memcpy(e.mp6.platform_config_text, text, n + 1);
    e.mp6.platform_config_present = true;
    // mp6_config is in the build: parse + derive + stage session inputs now.
    // config_ready means "text parsed and calibration derivable"; the session
    // collects anything still missing by name when a transaction is attempted.
    mp6ApplyPlatformConfig(*this, e);
    log(e.id, "mp6", "Platform config imported (%d bytes) — %s",
        (int)n, e.mp6.config_note[0] ? e.mp6.config_note : "no note");
    settings_dirty = true;
    return true;
}

// App-wide E-STOP: every path (toolbar, control API) funnels here so MP6
// entities always receive the staged stop/shutdown intents as well.

bool App::mp6IsMotorPowerRequested(const Entity& e) const {
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6) return false;
    auto it = mp6_runtimes.find(e.id);
    if (it != mp6_runtimes.end() && it->second && it->second->session) {
        const auto& p = it->second->session->powerStatus();
        if (p.acRequested && !p.motorDisabled) return true;
        mp6::State st = it->second->session->state();
        if (st == mp6::State::Initializing || st == mp6::State::Running ||
            st == mp6::State::Parking || st == mp6::State::Parked ||
            st == mp6::State::Docked || st == mp6::State::Stopped) {
            return true;
        }
    }
    if (e.mp6.lifecycle == Mp6Lifecycle::Initializing ||
        e.mp6.lifecycle == Mp6Lifecycle::Running ||
        e.mp6.lifecycle == Mp6Lifecycle::Parking ||
        e.mp6.lifecycle == Mp6Lifecycle::Parked ||
        e.mp6.lifecycle == Mp6Lifecycle::Docked) {
        return true;
    }
    return false;
}

bool App::mp6ValidateMachineSettings(const Mp6MachineSettings& settings, std::string* error) {
    auto err = [&](const char* m) {
        if (error) *error = m;
        return false;
    };
    if (!std::isfinite(settings.sensor_filter) || !std::isfinite(settings.av_filter) ||
        !std::isfinite(settings.sensor_filter_start)) {
        return err("non-finite floating point settings value");
    }
    if (settings.bench_test != 0 && settings.bench_test != 1)
        return err("bench_test must be 0 or 1");
    if (settings.comms_timeout_ms < 100 || settings.comms_timeout_ms > 10000)
        return err("comms_timeout_ms must be between 100 and 10000 ms");
    if (settings.onboard_shutdown != 0 && settings.onboard_shutdown != 1)
        return err("onboard_shutdown must be 0 or 1");
    if (settings.mot_val_final != 63 && settings.mot_val_final != 100)
        return err("mot_val_final must be 63 or 100");
    if (settings.sensor_filter < 0.0f || settings.sensor_filter > 1.0f)
        return err("sensor_filter must be between 0.0 and 1.0");
    if (settings.history_len < 1 || settings.history_len > 10)
        return err("history_len must be between 1 and 10");
    if (settings.sensor_filter_start < 0.0f || settings.sensor_filter_start > 1.0f)
        return err("sensor_filter_start must be between 0.0 and 1.0");
    if (settings.history_len_start < 1 || settings.history_len_start > 10)
        return err("history_len_start must be between 1 and 10");
    if (settings.deadband_enable != 0 && settings.deadband_enable != 1)
        return err("deadband_enable must be 0 or 1");
    if (settings.av_filter < 0.0f || settings.av_filter > 1.0f)
        return err("av_filter must be between 0.0 and 1.0");
    if (settings.limits_enable != 0 && settings.limits_enable != 1)
        return err("limits_enable must be 0 or 1");
    if (settings.info_motor < 1 || settings.info_motor > 6)
        return err("info_motor must select a motor between 1 and 6");
    if (settings.pid_index < 0 || settings.pid_index > 2)
        return err("pid_index must be 0 (run), 1 (dock), or 2 (startup)");
    if (settings.pressure_enabled && settings.pressure_aim_supplied) {
        if (settings.pressure_aim_psi < 0 || settings.pressure_aim_psi > 150)
            return err("pressure_aim_psi must be between 0 and 150 psi");
    }
    if (settings.pressure_enabled && !settings.pressure_aim_supplied)
        return err("enabled pressure requires an explicitly supplied aim");
    if (settings.pressure_aim_psi < 0 || settings.pressure_aim_psi > 150)
        return err("pressure_aim_psi must be between 0 and 150 psi");
    if (settings.explicit_fields != Mp6MachineSettings::all_fields) {
        if (error) {
            *error = "missing explicit machine settings:";
            for (unsigned i = 0; i < 16; ++i)
                if (!(settings.explicit_fields & (1u << i))) *error += std::string(" ") + mp6MachineSettingNames[i];
        }
        return false;
    }
    return true;
}

bool App::mp6InvalidateAuthority(Entity& e, bool invalidate_config, bool invalidate_geometry) {
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6) return false;
    if (mp6IsMotorPowerRequested(e)) return false;
    auto runtime = mp6_runtimes.find(e.id);
    if (runtime != mp6_runtimes.end() && runtime->second && runtime->second->session) {
        std::string why;
        if (!runtime->second->session->invalidateInputs(&why)) return false;
    }
    if (invalidate_config) {
        e.mp6.config_ready = false;
        e.mp6.config_send_ok = false;
        e.mp6.av_ok = false;
    }
    if (invalidate_geometry) {
        e.mp6.viz.solver_ready = false;
        e.mp6.viz.rod_lines_valid = false;
    }
    e.mp6.config_send_ok = false;
    e.mp6.step_target_valid = false;
    e.mp6.target_pose_valid = false;
    if (e.mp6.pending_intent != Mp6Intent::Estop && e.mp6.pending_intent != Mp6Intent::Disconnect
        && e.mp6.pending_intent != Mp6Intent::Stop) e.mp6.pending_intent = Mp6Intent::None;
    auto it = mp6_runtimes.find(e.id);
    if (it != mp6_runtimes.end()) {
        auto& rt = *it->second;
        if (invalidate_config) {
            rt.inputs_dirty = false;
            rt.parsed_cfg = {};
            rt.pending_cfg = {};
            rt.pending_cal_valid = false;
            rt.pending_pid_valid = false;
            rt.pending_pressure_valid = false;
            rt.pending_pressure.enabled = false;
            rt.pending_pressure.aimPsi = 0;
        }
        if (invalidate_geometry) {
            rt.solver_ready = false;
            rt.neutral_valid = false;
            rt.neutral_dirty = false;
            rt.staged_pose_valid = false;
            rt.start_blend_pending = rt.start_blend_active = false;
            rt.neutral_steps = {};
            rt.geometry_dirty = false;
        }
    }
    return true;
}

void App::startMotion() {
    motion_started = true;
    start_ramp_active = true;
    start_ramp_begin = frame_time;
    for (int i = 0; i < 6; i++) {
        start_ramp_target[i] = shared_input[i];
    }
    for (auto& e : entities) {
        if (e.type == EntityType::HIL && e.hil_backend == HilBackend::MP6) {
            if (e.mp6.lifecycle == Mp6Lifecycle::Parked) {
                std::string why;
                mp6Command(e, Mp6Intent::Start, &why);
            }
        }
    }
}

void App::stopMotion() {
    motion_started = false;
    start_ramp_active = false;
    if (capture_playing) stopCapturePlayback();
    if (plugin_mgr.activeIndex() >= 0) plugin_mgr.deactivateActive();
    { std::lock_guard<std::mutex> lock(input_mutex); memset(shared_input, 0, sizeof(shared_input)); }
    for (auto& e : entities) {
        memset(e.state.input_pct, 0, sizeof(e.state.input_pct));
        if (e.type == EntityType::HIL && e.hil_backend == HilBackend::MP6) {
            auto runtime = mp6_runtimes.find(e.id);
            if (runtime != mp6_runtimes.end() && runtime->second && runtime->second->session) {
                std::string why;
                mp6Command(e, Mp6Intent::Stop, &why);
            }
        }
    }
}

void App::requestEstop() {
    motion_started = false;
    start_ramp_active = false;
    if (capture_playing) stopCapturePlayback();
    if (plugin_mgr.activeIndex() >= 0) plugin_mgr.deactivateActive();
    for (auto& e : entities) {
        memset(e.state.input_pct, 0, sizeof(e.state.input_pct));
        if (e.type == EntityType::HIL && e.hil_backend == HilBackend::MP6) {
            e.mp6.axis_test_enabled = false;
            e.mp6.axis_test_entering = false;
            e.mp6.axis_test_returning = false;
            if (auto it = mp6_runtimes.find(e.id); it != mp6_runtimes.end() && it->second) {
                it->second->axis_test_entering = false;
                it->second->axis_test_entry_waiting_tx = false;
                it->second->axis_test_returning = false;
                it->second->axis_test_return_waiting_tx = false;
            }
            for (int i = 0; i < 6; ++i) e.mp6.axis_test_setpoint_deg[i] = e.mp6.axis_test_offset_deg[i] = 0.0f,
                e.mp6.axis_test_offset_steps[i] = e.mp6.axis_test_target_steps[i] = 0;
            e.mp6.pending_intent = e.mp6.link_open ? Mp6Intent::Estop : Mp6Intent::None;
            e.mp6.estop_latched = true;
            snprintf(e.mp6.last_action, sizeof(e.mp6.last_action), "E-STOP staged");
            mp6UpdateEntity(e);
            if (e.mp6.link_open && e.mp6.fault_latched) {
                // Onboard fault latch: TX is suppressed wholesale; nothing to
                // send, nothing this app may clear.
                log(e.id, "mp6", "E-STOP: onboard fault latch active (code %u) — TX suppressed",
                    (unsigned)e.mp6.fault_code);
            } else {
                log(e.id, "mp6", "E-STOP staged: %s",
                    e.mp6.link_open ? "stop targets, then shutdown session-side" : "offline");
            }
            snprintf(e.mp6.last_error, sizeof(e.mp6.last_error), "E-STOP");
        }
    }
    {
        std::lock_guard<std::mutex> lock(input_mutex);
        memset(shared_input, 0, sizeof(shared_input));
    }
    log(-1, "system", "E-STOP");
}

// ── Single-threaded MP6 session driver ──────────────────────────────
//
// Runs on the main update thread only. Staged intents (from the UI, the
// control API or requestEstop) are dispatched here and nowhere else — the
// mp6::Session is not thread-safe and the network thread must never touch it.
// After dispatch the session engine ticks; it enforces its own TX cadence
// internally, so the app frame rate only samples the driver. Everything the
// UI, persistence and the control API read is mirrored into the entity's
// plain-data fields from observed session/telemetry state; nothing is asserted
// from a staging click alone.
void App::mp6UpdateEntity(Entity& e) {
    if (e.type != EntityType::HIL || e.hil_backend != HilBackend::MP6) return;
    auto it = mp6_runtimes.find(e.id);
    if (it == mp6_runtimes.end() || !it->second) return;
    Mp6Runtime& rt = *it->second;
    if (!rt.session) return;
    mp6::Session& s = *rt.session;
    // Safety dispatch precedes snapshot/geometry work, including rejection paths.
    const auto pending = e.mp6.pending_intent;
    const bool safety = pending == Mp6Intent::Stop || pending == Mp6Intent::Estop
                     || pending == Mp6Intent::Disconnect;
    if (pending == Mp6Intent::Disconnect) { mp6Disconnect(e); return; }
    if (safety) {
        e.mp6.pending_intent = Mp6Intent::None;
        rt.start_blend_pending = rt.start_blend_active = false;
        std::string why;
        bool accepted = pending == Mp6Intent::Estop ? s.requestEmergencyStop(&why) : s.requestStopMotion(&why);
        if (!accepted) snprintf(e.mp6.last_error, sizeof(e.mp6.last_error), "%s", why.c_str());
    }


    // Apply config/inputs staged by an import (or by settings load) before
    // anything else: a staged Configure must act on the inputs it will need.
    if (!safety && rt.inputs_dirty && e.mp6.link_open) {
        rt.inputs_dirty = false;
        std::string why;
        mp6::Session validator;
        if (!mp6SetSnapshot(validator, rt, &why) || !s.invalidateInputs(&why)
            || !mp6SetSnapshot(s, rt, &why)) {
            e.mp6.config_ready = false;
            e.mp6.config_send_ok = false;
            rt.neutral_valid = rt.solver_ready = false;
            snprintf(e.mp6.last_error, sizeof(e.mp6.last_error), "snapshot rejected: %s", why.c_str());
            snprintf(e.mp6.config_note, sizeof(e.mp6.config_note), "%s", e.mp6.last_error);
            return;
        }
        log(e.id, "mp6", "Validated session snapshot applied");
    }

    // Rebuild crank geometry when design inputs (or a reparse) changed. The
    // solver is built on this thread only; init() refuses unusable numbers
    // with a named reason and nothing is guessed.
    if (!safety && rt.geometry_dirty) {
        std::string validation_error;
        if (!mp6ValidateDesignGeometry(e.mp6.design, &validation_error)) {
            e.mp6.design.supplied = false;
            mp6InvalidateAuthority(e, true, true);
            snprintf(e.mp6.config_note, sizeof(e.mp6.config_note), "design geometry rejected: %s", validation_error.c_str());
            snprintf(e.mp6.solver_note, sizeof(e.mp6.solver_note), "design geometry rejected: %s", validation_error.c_str());
            log(e.id, "mp6", "Design geometry rejected during rebuild: %s", validation_error.c_str());
            return;
        }
        rt.geometry_dirty = false;
        rt.solver_ready = false;
        rt.neutral_valid = false;
        rt.neutral_steps = {};
        e.mp6.step_target_valid = false;
        e.mp6.viz.solver_ready = false;
        e.mp6.viz.rod_lines_valid = false;
        e.mp6.solver_note[0] = '\0';
        std::string why;

        // F2: Populate rt.design completely from e.mp6.design before attachments
        rt.design.bot_a_mm = (double)e.mp6.design.bot_a_mm;
        rt.design.bot_b_mm = (double)e.mp6.design.bot_b_mm;
        rt.design.top_a_mm = (double)e.mp6.design.top_a_mm;
        rt.design.top_b_mm = (double)e.mp6.design.top_b_mm;
        rt.design.crank_length_mm = (double)e.mp6.design.crank_length_mm;
        rt.design.rod_length_mm = (double)e.mp6.design.rod_length_mm;
        rt.design.off_x_mm = (double)e.mp6.design.off_x_mm;
        rt.design.pcdx_mm = (double)e.mp6.design.pcdx_mm;
        rt.design.pcdy_mm = (double)e.mp6.design.pcdy_mm;
        rt.design.z_off_mm = (double)e.mp6.design.z_off_mm;

        double rod = rt.design.rod_length_mm;
        double crank = rt.design.crank_length_mm;
        double x_off = (double)e.mp6.design.off_x_mm;
        double ang_off = (double)e.mp6.design.angle_offset_deg;
        double tmin = 0.0, tmax = 0.0;
        bool have_limits = false;
        if (e.mp6.platform_config_present && e.mp6.platform_config_text[0]) {
            mp6::Mp6PlatformConfig pc;
            if (mp6::Mp6PlatformConfig::from_pltc(e.mp6.platform_config_text, pc)) {
                if (pc.rod_length_mm.present)   rod = pc.rod_length_mm.value;
                if (pc.crank_length_mm.present) crank = pc.crank_length_mm.value;
                if (pc.x_offset_mm.present)     x_off = pc.x_offset_mm.value;
                if (pc.angle_offset_deg.present) ang_off = pc.angle_offset_deg.value;
                if (pc.travel_min_rad.present && pc.travel_max_rad.present) {
                    tmin = pc.travel_min_rad.value;
                    tmax = pc.travel_max_rad.value;
                    have_limits = true;
                }
            }
        }
        mp6::Mp6PlanarAttachments planar = mp6::mp6_planar_attachments(rt.design);
        mp6::Mp6GeometrySetup geo;
        std::array<double, 6> bz{}, tz{};
        for (int i = 0; i < 6; i++) {
            bz[i] = (double)e.mp6.design.base_z_mm[i];
            tz[i] = (double)e.mp6.design.top_z_mm[i];
        }
        geo = mp6::mp6_geometry_setup(planar, bz, tz);
        mp6::Mp6OffsetsInputs off;
        off.x_offset_cfg_mm = x_off;
        off.angle_offset_deg = ang_off;
        off.crank_length_mm = crank;
        // F8: Primary frm_Main.cs:16610 uses Module2.ZOff from frm_Design.cs:686 (MOT155-D2 = 550 -> 530 mm).
        // Pltc "Z Offset" row 506 is unconsumed in recovered source.
        off.design_z_off_mm = (double)e.mp6.design.z_off_mm;
        off.point_zero_x_mm = (double)e.mp6.design.point_zero_x_mm;
        off.point_zero_z_mm = (double)e.mp6.design.point_zero_z_mm;
        mp6::Mp6Offsets offs = mp6::mp6_derive_offsets(off);
        rt.x_offset_mm = offs.x_offset_mm;
        rt.z_offset_mm = offs.z_offset_mm;
        // Calibration-dependent values from the last derived calibration.
        mp6::Mp6AxisCalibration ac[6];
        std::string cal_err;
        bool cal_ok = mp6::mp6_config_derive_calibration(rt.parsed_cfg, ac, &cal_err);
        // Real data the recovery could not pin down stays unresolved until the
        // user explicitly supplies it: pivot z coordinates (unrecovered
        // Other\<Platform>.txt) and the MotConv point zeros. Zeros are never
        // passed through as machine values, and nothing from the geometry
        // module's test fixtures is consulted here. Pivot provenance mirrors
        // the engine's three-state gate (F4): kUnreviewed blocks, kFixtureOnly
        // is the OFFLINE-TEST-FIXTURE state and never valid on this live path,
        // kReviewed is granted only by the explicit UI review action.
        if (e.mp6.design.pivot_z_provenance == 1) {
            snprintf(e.mp6.solver_note, sizeof(e.mp6.solver_note),
                     "geometry staged; pivot z provenance is the OFFLINE TEST FIXTURE "
                     "state — never valid for a real rig; re-enter the real points-file "
                     "values and review them");
            log(e.id, "mp6",
                "Crank solver not initialized: pivot z provenance is fixture-only (blocked)");
        } else if (!e.mp6.design.supplied || e.mp6.design.pivot_z_provenance != 2) {
            snprintf(e.mp6.solver_note, sizeof(e.mp6.solver_note),
                     "geometry staged; pivot z provenance unreviewed — enter the real "
                     "base/top pivot z (points-file data) and use the review action; "
                     "no fixture or zero default is assumed");
            log(e.id, "mp6",
                "Crank solver not initialized: pivot z provenance unreviewed");
        } else if (!e.mp6.design.point_zero_supplied) {
            snprintf(e.mp6.solver_note, sizeof(e.mp6.solver_note),
                     "geometry staged; MotConv point zeros unresolved — enter the "
                     "real \"X Point Zero\"/\"Z Point Zero\" and confirm; none invented");
            log(e.id, "mp6", "Crank solver not initialized: MotConv point zeros unresolved");
        } else {
            // Defensive provenance gate: entered-but-degenerate z data must not
            // pass silently. All six base/top z must be finite, and at least
            // one of each set must be nonzero (all-zero pivots are the shape of
            // the "nothing entered yet" state, not machine data).
            bool finite_z = true, all_zero_z = true;
            for (int i = 0; i < 6; i++) {
                double b = (double)e.mp6.design.base_z_mm[i], t = (double)e.mp6.design.top_z_mm[i];
                if (!std::isfinite(b) || !std::isfinite(t)) finite_z = false;
                if (b != 0.0 || t != 0.0) all_zero_z = false;
            }
            if (!finite_z || all_zero_z) {
                snprintf(e.mp6.solver_note, sizeof(e.mp6.solver_note),
                         "geometry staged; pivot z entry is degenerate (%s) — flagged, "
                         "not treated as machine data",
                         all_zero_z ? "all zeros" : "non-finite value");
                log(e.id, "mp6", "Crank solver not initialized: pivot z entry degenerate (%s)",
                    all_zero_z ? "all zeros" : "non-finite");
            } else if (!cal_ok) {
                snprintf(e.mp6.solver_note, sizeof(e.mp6.solver_note),
                         "geometry staged; crank solver needs a parsed .pltc calibration (%s)",
                         cal_err.c_str());
                log(e.id, "mp6", "Crank solver not initialized: calibration unavailable (%s)",
                    cal_err.c_str());
            } else if (!have_limits) {
                snprintf(e.mp6.solver_note, sizeof(e.mp6.solver_note),
                         "geometry staged; crank solver needs the .pltc travel limits "
                         "(\"Max Travel (rad)\" row) — none invented");
                log(e.id, "mp6", "Crank solver not initialized: travel limits missing");
            } else {
                std::array<double, 6> rods{}, zeros{}, ats{};
                for (int i = 0; i < 6; i++) {
                    rods[i] = rod;
                    zeros[i] = ac[i].rod_angle_zero_rad;
                    ats[i] = ac[i].angle_to_step;
                }
                mp6::Mp6CrankSolver nv;
                if (!nv.init(geo, rods, crank, tmin, tmax, zeros, &why)) {
                    snprintf(e.mp6.solver_note, sizeof(e.mp6.solver_note),
                             "crank solver refused the inputs: %s", why.c_str());
                    log(e.id, "mp6", "Crank solver init refused: %s", why.c_str());
                } else {
                    rt.solver = std::move(nv);
                    rt.design.rod_length_mm = rod;
                    rt.design.crank_length_mm = crank;
                    rt.geometry = geo;
                    rt.min_rad = tmin;
                    rt.max_rad = tmax;
                    for (int i = 0; i < 6; i++) {
                        rt.angle_to_step[i] = ats[i];
                        rt.angle_zero_rad[i] = zeros[i];
                    }
                    rt.solver_ready = true;
                    e.mp6.viz.solver_ready = true;

                    // Solve neutral steps at zero pose for session initialization
                    std::array<mp6::Mp6Point3, 6> neutral_targets = rt.geometry.top_attachment_mm;
                    mp6::Mp6Pose zero_pose{};
                    mp6::mp6_transform_points(neutral_targets, zero_pose, rt.x_offset_mm, rt.z_offset_mm);
                    mp6::Mp6SolveResult zero_res;
                    rt.solver.solve(neutral_targets, ats, /*docking=*/false, /*test_mode=*/false, &zero_res);
                    bool neutral_ok = true;
                    for (int i = 0; i < 6; i++) {
                        if (!zero_res.axis[i].ok || mp6::mp6_step_word_reserved(zero_res.axis[i].step_target)) {
                            neutral_ok = false;
                            break;
                        }
                        rt.neutral_steps[i] = zero_res.axis[i].step_target;
                        rt.neutral_crank_rad[i] = zero_res.axis[i].angle_aim_rad + rt.angle_zero_rad[i];
                    }
                    rt.neutral_valid = neutral_ok;
                    if (neutral_ok) {
                        rt.neutral_valid = s.setNeutralSteps(rt.neutral_steps, &why);
                        if (!rt.neutral_valid) {
                            rt.solver_ready = false;
                            e.mp6.viz.solver_ready = false;
                            snprintf(e.mp6.last_error, sizeof(e.mp6.last_error), "neutral rejected: %s", why.c_str());
                        }
                    }
                    // A real pivot z of exactly 0 is a live AtanKD hazard
                    // (zero-collapse remaps that stage — geometry review F2);
                    // it stays a named warning, never silently accepted.
                    bool any_zero_z = false;
                    for (int i = 0; i < 6; i++)
                        if (e.mp6.design.base_z_mm[i] == 0.0f || e.mp6.design.top_z_mm[i] == 0.0f)
                            any_zero_z = true;
                    snprintf(e.mp6.solver_note, sizeof(e.mp6.solver_note),
                             "crank solver ready (X_Offset %.1f, Z_Offset %.1f, rod %.1f, "
                             "crank %.1f mm)%s",
                             offs.x_offset_mm, offs.z_offset_mm, rod, crank,
                             any_zero_z
                                 ? " | WARNING: a pivot z is exactly 0 — AtanKD zero-collapse "
                                   "applies; verify the value against the points file"
                                 : "");
                    log(e.id, "mp6", "Crank solver initialized (%s)", e.mp6.solver_note);
                }
            }
        }
    }

    // Dispatch one staged intent. A refusal is recorded verbatim and the
    // stage is cleared: the caller re-issues it; we never pretend success.
    if (e.mp6.pending_intent != Mp6Intent::None) {
        Mp6Intent staged = e.mp6.pending_intent;
        e.mp6.pending_intent = Mp6Intent::None;
        if (staged == Mp6Intent::Disconnect) {
            mp6Disconnect(e);
            return;   // runtime is gone; mirror update would read freed state
        }
        mp6::Intent si;
        switch (staged) {
            case Mp6Intent::Identify:   si = mp6::Intent::Identify; break;
            case Mp6Intent::Configure:  si = mp6::Intent::Configure; break;
            case Mp6Intent::Initialize: si = mp6::Intent::Initialize; break;
            case Mp6Intent::Start:      si = mp6::Intent::StartMotion; break;
            case Mp6Intent::Stop:       si = mp6::Intent::StopMotion; break;
            case Mp6Intent::Park:       si = mp6::Intent::Park; break;
            case Mp6Intent::Shutdown:   si = mp6::Intent::Shutdown; break;
            case Mp6Intent::None:       si = mp6::Intent::EmergencyStop; break;
            default:                    si = mp6::Intent::EmergencyStop; break; // Estop
        }
        std::string why;
        if (staged == Mp6Intent::Stop || staged == Mp6Intent::Estop || staged == Mp6Intent::Disconnect)
            rt.start_blend_pending = rt.start_blend_active = false;
        if (staged == Mp6Intent::Start) rt.start_blend_pending = true;
        if (s.request(si, &why)) {
            if (staged == Mp6Intent::Start) motion_started = true;
            snprintf(e.mp6.last_action, sizeof(e.mp6.last_action),
                     "intent %d dispatched", (int)staged);
            log(e.id, "mp6", "Intent %d dispatched (session state %s)",
                (int)staged, mp6::toString(s.state()));
        } else {
            if (staged == Mp6Intent::Start) rt.start_blend_pending = false;
            snprintf(e.mp6.last_error, sizeof(e.mp6.last_error),
                     "session refused intent %d: %s", (int)staged, why.c_str());
            log(e.id, "mp6", "Intent %d refused: %s", (int)staged, why.c_str());
        }
    }

    // Command-2 onboard control: stream the fresh crank-solver step aims while
    // the session is Running or Parked. The session owns cadence/freshness guards; a
    // fault or Estop latch below stops feeding without any extra claim here.
    if ((e.mp6.axis_test_enabled || rt.axis_test_entering) && (s.state() != mp6::State::Running || e.mp6.estop_latched || e.mp6.fault_latched)) {
        e.mp6.axis_test_enabled = false;
        e.mp6.axis_test_entering = false;
        e.mp6.axis_test_returning = false;
        rt.axis_test_entering = false;
        rt.axis_test_entry_waiting_tx = false;
        rt.axis_test_returning = false;
        rt.axis_test_return_waiting_tx = false;
        for (int i = 0; i < 6; ++i) e.mp6.axis_test_setpoint_deg[i] = e.mp6.axis_test_offset_deg[i] = 0.0f;
    }
    const bool axis_override = (e.mp6.axis_test_enabled || rt.axis_test_returning || rt.axis_test_entering) && s.state() == mp6::State::Running;
    if (axis_override) {
        const double dt = std::clamp(frame_time - rt.axis_test_last_time, 0.0, 0.1);
        rt.axis_test_last_time = frame_time;
        bool returned = true;
        bool entered = true;
        for (int i = 0; i < 6; ++i) {
            if (rt.axis_test_entering) {
                const double tx_dt = std::clamp(frame_time - rt.axis_test_last_sent_time, 0.0, 0.1);
                const double max_steps = e.mp6.axis_test_slew_deg_s * tx_dt * 3.14159265358979323846 / 180.0 * std::abs(rt.angle_to_step[i]);
                const double frame_steps = e.mp6.axis_test_slew_deg_s * dt * 3.14159265358979323846 / 180.0 * std::abs(rt.angle_to_step[i]);
                rt.axis_test_entry_position[i] += std::clamp((double)rt.neutral_steps[i] - rt.axis_test_entry_position[i], -frame_steps, frame_steps);
                const int max_sent_delta = (std::max)(1, (int)std::ceil(max_steps));
                const int rounded_entry = mp6::mp6_net_round(rt.axis_test_entry_position[i]);
                rt.axis_test_entry_steps[i] = std::clamp(rounded_entry,
                    rt.axis_test_last_sent_steps[i] - max_sent_delta,
                    rt.axis_test_last_sent_steps[i] + max_sent_delta);
                e.mp6.axis_test_target_steps[i] = rt.axis_test_entry_steps[i];
                e.mp6.axis_test_offset_steps[i] = rt.axis_test_entry_steps[i] - rt.neutral_steps[i];
                e.mp6.axis_test_offset_deg[i] = static_cast<float>(e.mp6.axis_test_offset_steps[i] / rt.angle_to_step[i] * 180.0 / 3.14159265358979323846);
                entered = entered && rt.axis_test_entry_steps[i] == rt.neutral_steps[i];
                continue;
            }
            const double goal = e.mp6.axis_test_enabled
                ? std::clamp((double)e.mp6.axis_test_setpoint_deg[i], -(double)e.mp6.axis_test_cap_deg[i], (double)e.mp6.axis_test_cap_deg[i])
                : 0.0;
            const double step = e.mp6.axis_test_slew_deg_s * dt;
            double current = e.mp6.axis_test_offset_deg[i];
            current += std::clamp(goal - current, -step, step);
            const double requested_abs = rt.neutral_crank_rad[i] + current * 3.14159265358979323846 / 180.0;
            const double limited_abs = std::clamp(requested_abs, rt.min_rad, rt.max_rad);
            current = (limited_abs - rt.neutral_crank_rad[i]) * 180.0 / 3.14159265358979323846;
            e.mp6.axis_test_offset_deg[i] = (float)current;
            e.mp6.axis_test_offset_steps[i] = mp6::mp6_net_round(
                current * 3.14159265358979323846 / 180.0 * rt.angle_to_step[i]);
            e.mp6.axis_test_target_steps[i] = rt.neutral_steps[i] + e.mp6.axis_test_offset_steps[i];
            if (std::abs(current) > 1e-6 || std::abs(goal - current) > 1e-6) returned = false;
        }
        if (rt.axis_test_entering && entered) rt.axis_test_entry_waiting_tx = true;
        if (rt.axis_test_returning && returned) rt.axis_test_return_waiting_tx = true;
    }
    if ((s.state() == mp6::State::Running || s.state() == mp6::State::Parked)
        && (axis_override || e.mp6.step_target_valid)
        && !e.mp6.estop_latched && !e.mp6.fault_latched) {
        mp6::StepTarget t;
        double blend = 1.0;
        const bool blending = rt.start_blend_active;
        if (rt.start_blend_pending) blend = 0.0;
        else if (rt.start_blend_active) {
            blend = std::clamp((frame_time - rt.start_blend_begin) / START_RAMP_S, 0.0, 1.0);
            blend = blend * blend * (3.0 - 2.0 * blend);
            if (blend >= 1.0) rt.start_blend_active = false;
        }
        for (int i = 0; i < 6; i++)
            t.steps[i] = axis_override ? e.mp6.axis_test_target_steps[i]
                : mp6::roundHalfEvenToInt((blending ? rt.start_blend_start[i] : rt.neutral_steps[i]) +
                    blend * (e.mp6.step_target[i] - (blending ? rt.start_blend_start[i] : rt.neutral_steps[i])));
        for (int i = 0; i < 6; ++i) e.mp6.axis_test_target_steps[i] = t.steps[i];
        t.valid = true;
        t.generation = ++rt.target_gen;
        s.setTarget(t);
    }

    const auto command2_before = s.txCounters().command2;
    s.tick((std::uint64_t)(frame_time * 1000.0));
    if (s.txCounters().command2 != command2_before) {
        rt.axis_test_last_sent_steps = s.target().steps;
        rt.axis_test_last_sent_valid = true;
        rt.axis_test_last_sent_time = frame_time;
    }
    if (rt.axis_test_entering && rt.axis_test_entry_waiting_tx && rt.axis_test_last_sent_steps == rt.neutral_steps) {
        rt.axis_test_entering = false;
        rt.axis_test_entry_waiting_tx = false;
        e.mp6.axis_test_entering = false;
        for (int i = 0; i < 6; ++i) e.mp6.axis_test_offset_deg[i] = 0.0f,
            e.mp6.axis_test_offset_steps[i] = 0;
    }
    if (rt.axis_test_returning && rt.axis_test_return_waiting_tx && rt.axis_test_last_sent_steps == rt.neutral_steps) {
        rt.axis_test_returning = false;
        rt.axis_test_return_waiting_tx = false;
        e.mp6.axis_test_returning = false;
        rt.start_blend_start = rt.axis_test_last_sent_steps;
        rt.start_blend_active = true;
        rt.start_blend_begin = frame_time;
    }
    if (s.state() != mp6::State::Running && (e.mp6.axis_test_enabled || rt.axis_test_returning || rt.axis_test_entering)) {
        e.mp6.axis_test_enabled = false;
        e.mp6.axis_test_entering = false;
        rt.axis_test_entering = false;
        rt.axis_test_entry_waiting_tx = false;
        rt.axis_test_returning = false;
        rt.axis_test_return_waiting_tx = false;
        e.mp6.axis_test_returning = false;
        for (int i = 0; i < 6; ++i) {
            e.mp6.axis_test_setpoint_deg[i] = e.mp6.axis_test_offset_deg[i] = 0.0f;
            e.mp6.axis_test_offset_steps[i] = 0;
        }
    }
    if (rt.start_blend_pending && s.state() == mp6::State::Running) {
        rt.start_blend_pending = false;
        rt.start_blend_active = true;
        rt.start_blend_start = rt.neutral_steps;
        rt.start_blend_begin = frame_time;
    }

    // ── Mirror observed state ─────────────────────────────────────────
    const mp6::Telemetry& tel = s.telemetry();
    bool open = s.link() != nullptr && s.link()->isOpen();
    e.mp6.link_open = open;
    e.mp6.lifecycle = mp6MapLifecycle(s.state());

    const mp6::Diagnostics& dg = s.diagnostics();
    if (dg.reason != mp6::StopReason::None)
        snprintf(e.mp6.lifecycle_note, sizeof(e.mp6.lifecycle_note), "%s (stop: %s)",
                 mp6::toString(s.state()), mp6::toString(dg.reason));
    else if (dg.detail[0] != '\0')
        snprintf(e.mp6.lifecycle_note, sizeof(e.mp6.lifecycle_note), "%s (%s)",
                 mp6::toString(s.state()), dg.detail.c_str());
    else
        snprintf(e.mp6.lifecycle_note, sizeof(e.mp6.lifecycle_note), "%s",
                 mp6::toString(s.state()));

    e.mp6.identity_ok = s.identityAccepted();
    e.mp6.platform_number_ob = tel.platformNumber;
    if (tel.boardType >= 0)
        snprintf(e.mp6.board_type, sizeof(e.mp6.board_type), "%d", tel.boardType);
    else
        e.mp6.board_type[0] = '\0';
    e.mp6.paired_config = (e.mp6.platform_number_expected != 0
                           && tel.platformNumber == e.mp6.platform_number_expected);

    e.mp6.runmode_ob = tel.runMode;
    e.mp6.ok2run_ob = tel.okToRun;
    e.mp6.config_received_ob = tel.configReceived;
    e.mp6.av_ok = tel.avOk;
    e.mp6.tel_seq++;
    e.mp6.missed_cadence_slots = s.txCounters().missedCadenceSlots;
    e.mp6.achieved_target_hz = s.achievedTargetHz();
    e.mp6.cadence_normalised = s.cadenceNormalised();
    e.mp6.shutdown_pending = s.shutdownPending();
    auto apm = s.appliedPidMode();
    e.mp6.applied_pid_mode = apm.has_value() ? static_cast<int>(*apm) : -1;

    const mp6::ConfigStatus& cs = s.configStatus();
    e.mp6.sending_config = cs.configuring;
    e.mp6.config_count_expected = cs.framesExpected;
    e.mp6.config_params_transmitted = cs.parametersTransmitted;
    e.mp6.config_params_ack = cs.parametersExpected;
    e.mp6.config_send_ok = cs.configured;
    e.mp6.config_attempts = cs.attempts;

    // Ramp (park/shutdown descent) observation — dock semantics: the descent
    // is the only path to the vent stage; Parked alone is the neutral-ready
    // hold and never implies a dock settle.
    {
        const mp6::RampStatus& rp = s.rampStatus();
        e.mp6.ramp_active = rp.active;
        e.mp6.ramp_phase = rp.phase;
        e.mp6.ramp_duration_ms = (unsigned)rp.durationMs;
        e.mp6.ramp_timeout_ms = (unsigned)rp.timeoutMs;
        e.mp6.ramp_endpoint_reached = rp.endpointReached;
        e.mp6.ramp_settled = rp.settled;
        for (int i = 0; i < 6; i++) {
            e.mp6.ramp_start[i] = rp.start[i];
            e.mp6.ramp_goal[i] = rp.goal[i];
            e.mp6.ramp_current[i] = rp.current[i];
        }
    }
    e.mp6.link_read_failed =
        (s.link() != nullptr && s.link()->terminalFailure());

    e.mp6.fault_latched = s.controllerFaultLatched();
    if (e.mp6.fault_latched || tel.controllerFaultCode != 0) {
        e.mp6.fault_code = tel.controllerFaultCode;
        snprintf(e.mp6.fault_text, sizeof(e.mp6.fault_text),
                 "onboard shutdown code %u", (unsigned)tel.controllerFaultCode);
    } else {
        e.mp6.fault_code = 0;
        e.mp6.fault_text[0] = '\0';
    }

    e.mp6.supplies_valid = (open && s.supplyFresh());
    if (e.mp6.supplies_valid)
        for (int i = 0; i < 3; i++)
            e.mp6.supplies[i] = (float)tel.motorSupplyVolts[i];

    e.mp6.pneu_aim_psi = (float)tel.pneuAim;
    e.mp6.pressure_valid = (open && s.pressureFresh());
    if (e.mp6.pressure_valid)
        e.mp6.pneu_psi = (float)tel.pressure;

    e.mp6.feedback_valid = (open && s.feedbackFresh());
    if (e.mp6.feedback_valid) {
        for (int i = 0; i < 6; i++) {
            const mp6::AxisFeedback& af = tel.axes[i];
            e.mp6.feedback_raw[i] = (short)af.raw;
            // Calibrated feedback exists only with a valid Calibration and a
            // matched config ack; the absence stays visible, never zeroed.
            e.mp6.feedback_cal[i] = af.calibratedValid ? (short)af.calibrated : SHRT_MIN;
            e.mp6.pot_alarm[i] = af.potAlarmLatched;
        }
    }

    // Rate estimates (RX type-48 telemetry and session TX), sampled ~1 Hz.
    if (frame_time - rt.rate_sample_time >= 1.0) {
        if (rt.rate_sample_time > 0.0 && open) {
            double dt = frame_time - rt.rate_sample_time;
            std::uint64_t tel_delta = tel.type48Frames - rt.tel_frames_prev;
            std::uint64_t tx_delta = s.txCounters().total - rt.tx_total_prev;
            e.mp6.tel_hz_est = (int)((double)tel_delta / dt + 0.5);
            e.rate_tx_hz = (float)((double)tx_delta / dt);
        }
        rt.tel_frames_prev = tel.type48Frames;
        rt.tx_total_prev = s.txCounters().total;
        rt.rate_sample_time = frame_time;
        if (open) e.rate_tel_hz = (float)e.mp6.tel_hz_est;
    }
}

// ── MP6 pipeline conversion ─────────────────────────────────────────
//
// single mapping table (do NOT silently reuse the linear-actuator IK swap):
//   app scaled_pct axis    → MP6 pose component
//   [0] Surge  (mm)        → x = GLon (longitudinal)
//   [1] Sway   (mm)        → y = GLat (lateral)
//   [2] Heave  (mm)        → z = GVert (vertical)
//   [3] Roll   (rad)       → roll
//   [4] Pitch  (rad)       → pitch
//   [5] Yaw    (rad)       → yaw
// Evidence: MotSim CalcPlatform_3 translates attachment points by GLon (X),
// GVert (Z), GLat (Y) after roll/pitch/yaw polar rotations about pivot 1; the
// app surge axis is longitudinal and sway is lateral (source reports are
// advisory and were cross-checked; the SIL/ESP32 surge<->sway swap belongs to
// the linear-actuator IK frame and is deliberately not applied here).
// VERIFY-BEFORE-HARDWARE: rotation signs must be confirmed against the
// recovered CalcPlatform_3/MotorPos_6DOF_C once the geometry module lands;
// no wire capture exists yet, so no sign is baked in beyond this table.
static void mp6UpdatePipeline(App& app, Entity& e, const float scaled_pct[6]) {
    // Post-cue percent → physical MP6 pose (mm / rad).
    float pose[6];
    for (int i = 0; i < 6; i++) {
        float phys = (scaled_pct[i] / 100.0f) * e.config.axis_scales.scale[i];
        if (e.config.axis_scales.is_angle[i] && i >= 3)
            phys *= (float)(M_PI / 180.0);   // axis_scales angles are degrees
        pose[i] = phys;
    }
    // input_physical keeps the same quantities (app/MP6 orders agree here).
    memcpy(e.state.input_physical, pose, sizeof(pose));

    // Modelled TARGET pose for the viz; never confused with feedback.
    memcpy(e.mp6.target_pose, pose, sizeof(pose));
    e.mp6.target_pose_valid = true;

    // ── Rotary crank solve (mp6_geometry) when a solver is staged ─────────
    // Runs on the update thread; the solver instance lives in the runtime.
    // Pose convention (mp6::Mp6Pose): Surge→lon(+X), Sway→lat(+Y),
    // Heave→vert(+Z), Roll/Pitch/Yaw unchanged — identical to the table above.
    if (auto it = g_app.mp6_runtimes.find(e.id); it != g_app.mp6_runtimes.end()) {
        App::Mp6Runtime& rt = *it->second;
        if (rt.solver_ready && rt.session) {
            mp6::Mp6Pose p;
            p.lon_mm  = pose[0];
            p.lat_mm  = pose[1];
            p.vert_mm = pose[2];
            p.roll    = pose[3];
            p.pitch   = pose[4];
            p.yaw     = pose[5];
            rt.staged_pose = p;
            rt.staged_pose_valid = true;

            // Transform the top attachments by the pose + offsets and solve
            // the rotary cranks (MotorPos_6DOF_C parity).
            std::array<mp6::Mp6Point3, 6> targets = rt.geometry.top_attachment_mm;
            mp6::mp6_transform_points(targets, p, rt.x_offset_mm, rt.z_offset_mm);
            std::array<double, 6> ats{};
            for (int i = 0; i < 6; i++) ats[i] = rt.angle_to_step[i];
            // Docking bypasses the travel limits exactly like the recovered
            // dock/base-point descent (session fix F1/F2): docked is a distinct
            // observed state now, produced by the Park/Shutdown dock descent —
            // never the neutral-ready Parked hold.
            bool docking = (e.mp6.lifecycle == Mp6Lifecycle::Parking
                         || e.mp6.lifecycle == Mp6Lifecycle::Docked);
            mp6::Mp6SolveResult res;
            rt.solver.solve(targets, ats, docking, /*test_mode=*/false, &res);
            rt.solver_fresh = true;

            bool all_ok = true;
            for (int i = 0; i < 6; i++) {
                all_ok = all_ok && res.axis[i].ok;
                if (!mp6::mp6_step_word_reserved(res.axis[i].step_target))
                    e.mp6.step_target[i] = res.axis[i].step_target;
                else
                    all_ok = false;   // reserved word 1 collision: refuse, never clamp
                e.mp6.viz.base_pivot[i][0] = (float)rt.geometry.base_pivot_mm[i][0];
                e.mp6.viz.base_pivot[i][1] = (float)rt.geometry.base_pivot_mm[i][1];
                e.mp6.viz.base_pivot[i][2] = (float)rt.geometry.base_pivot_mm[i][2];
                e.mp6.viz.crank_joint[i][0] = (float)res.axis[i].crank_joint_mm[0];
                e.mp6.viz.crank_joint[i][1] = (float)res.axis[i].crank_joint_mm[1];
                e.mp6.viz.crank_joint[i][2] = (float)res.axis[i].crank_joint_mm[2];
                e.mp6.viz.rod_top[i][0] = (float)targets[i][0];
                e.mp6.viz.rod_top[i][1] = (float)targets[i][1];
                e.mp6.viz.rod_top[i][2] = (float)targets[i][2];
                e.mp6.viz.calc_residual_mm[i] = (float)res.axis[i].residual_mm;
                e.mp6.viz.axis_ok[i] = res.axis[i].ok;
                e.mp6.viz.measured_crank_valid[i] = false;
            }
            e.mp6.viz.rod_lines_valid = all_ok;
            // A target chain is only claimed fresh when every axis solved and
            // the post-cue pose was produced this very frame.
            e.mp6.step_target_valid = all_ok;
        } else {
            e.mp6.viz.axis_test_active = false;
            e.mp6.viz.axis_test_geometry_valid = false;
            e.mp6.viz.rod_lines_valid = false;
            e.mp6.step_target_valid = false;
            memset(e.mp6.step_target, 0, sizeof(e.mp6.step_target));
        }
    } else {
        e.mp6.viz.axis_test_active = false;
        e.mp6.viz.axis_test_geometry_valid = false;
        e.mp6.viz.rod_lines_valid = false;
    }
}

static void mp6UpdateAxisTestView(Entity& e) {
    e.mp6.viz.axis_test_active = false;
    e.mp6.viz.axis_test_geometry_valid = false;
    for (int i = 0; i < 6; ++i) e.mp6.viz.measured_crank_valid[i] = false;
    auto it = g_app.mp6_runtimes.find(e.id);
    if (it == g_app.mp6_runtimes.end()) return;
    App::Mp6Runtime& rt = *it->second;
    const bool active = e.mp6.axis_test_enabled || e.mp6.axis_test_entering
                     || e.mp6.axis_test_returning || rt.axis_test_entering
                     || rt.axis_test_returning;
    if (!active || !rt.session || rt.session->state() != mp6::State::Running
        || !e.mp6.link_open || e.mp6.estop_latched || e.mp6.fault_latched) return;

    e.mp6.viz.axis_test_active = true;
    if (!rt.neutral_valid || !rt.solver_ready || rt.design.crank_length_mm <= 0.0
        || rt.design.rod_length_mm <= 0.0) return;
    auto neutral_tops = rt.geometry.top_attachment_mm;
    mp6::mp6_transform_points(neutral_tops, mp6::Mp6Pose{}, rt.x_offset_mm, rt.z_offset_mm);
    bool angle_data_valid = true;
    for (int i = 0; i < 6; ++i) {
        if (rt.angle_to_step[i] == 0.0) { angle_data_valid = false; break; }
        const int32_t commanded_steps = rt.axis_test_last_sent_valid
            ? rt.axis_test_last_sent_steps[i] : e.mp6.axis_test_target_steps[i];
        const double commanded_angle = rt.neutral_crank_rad[i]
            + (commanded_steps - rt.neutral_steps[i]) / rt.angle_to_step[i];
        const auto commanded = mp6::Mp6CrankSolver::crank_joint_from_angle_aim(
            rt.geometry, i, rt.design.crank_length_mm, commanded_angle);
        double dx = commanded[0] - neutral_tops[i][0];
        double dy = commanded[1] - neutral_tops[i][1];
        double dz = commanded[2] - neutral_tops[i][2];
        e.mp6.viz.axis_ok[i] = std::abs(std::sqrt(dx * dx + dy * dy + dz * dz)
            - rt.design.rod_length_mm) <= 1.0;
        for (int c = 0; c < 3; ++c) {
            e.mp6.viz.crank_joint[i][c] = (float)commanded[c];
            e.mp6.viz.rod_top[i][c] = (float)neutral_tops[i][c];
            e.mp6.viz.axis_test_plate[i][c] = (float)neutral_tops[i][c];
        }
        if (e.mp6.feedback_valid && e.mp6.feedback_cal[i] != SHRT_MIN) {
            const double measured_angle = rt.neutral_crank_rad[i]
                + (e.mp6.feedback_cal[i] - rt.neutral_steps[i]) / rt.angle_to_step[i];
            const auto measured = mp6::Mp6CrankSolver::crank_joint_from_angle_aim(
                rt.geometry, i, rt.design.crank_length_mm, measured_angle);
            for (int c = 0; c < 3; ++c)
                e.mp6.viz.measured_crank_joint[i][c] = (float)measured[c];
            e.mp6.viz.measured_crank_valid[i] = true;
        }
    }
    e.mp6.viz.axis_test_geometry_valid = angle_data_valid;
    e.mp6.viz.rod_lines_valid = angle_data_valid;
}

// ── Recording / Playback ────────────────────────────────────────────

void App::startRecording() {
    recording.samples.clear();
    recording.start_time = frame_time;
    recording.mode = RecordMode::Recording;
    // Initialize first-order hold state
    {
        std::lock_guard<std::mutex> lock(input_mutex);
        memcpy(recording.prev_input, shared_input, sizeof(recording.prev_input));
        memcpy(recording.curr_input, shared_input, sizeof(recording.curr_input));
    }
    recording.prev_time = frame_time;
    recording.curr_time = frame_time;
    const char* src = "plugin";
    if (input_source == InputSource::CapturePlayback) src = "capture";
    else if (active_plugin_idx >= 0) src = plugin_mgr.pluginName(active_plugin_idx);
    log(-1, "record", "Recording started (source: %s, rate: %d Hz)", src, record_rate_hz);
}

void App::stopRecording() {
    recording.mode = RecordMode::Idle;
    // Compute data range for diagnostic
    float rmin = 1e9f, rmax = -1e9f;
    for (auto& s : recording.samples) {
        for (int i = 0; i < 6; i++) {
            if (s.input[i] < rmin) rmin = s.input[i];
            if (s.input[i] > rmax) rmax = s.input[i];
        }
    }
    bool flat = !recording.samples.empty() && (fabsf(rmax - rmin) < 0.01f);
    log(-1, "record", "Recording stopped — %d samples (%.1fs) range: %.1f..%.1f%s",
        (int)recording.samples.size(), recording.duration(), rmin, rmax,
        flat ? " [FLAT — input was static during recording]" : "");

    // Auto-save with date/time + source name
    if (!recording.samples.empty()) {
        time_t now = time(nullptr);
        struct tm lt;
#ifdef _WIN32
        localtime_s(&lt, &now);
#else
        localtime_r(&now, &lt);
#endif
        const char* src = "Plugin";
        if (input_source == InputSource::CapturePlayback) src = "Capture";
        else if (active_plugin_idx >= 0) {
            const char* pn = plugin_mgr.pluginName(active_plugin_idx);
            if (pn) src = pn;
        }
        char auto_name[64];
        snprintf(auto_name, sizeof(auto_name), "%04d-%02d-%02d %02d:%02d:%02d %s",
                 lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
                 lt.tm_hour, lt.tm_min, lt.tm_sec, src);
        saveRecordingToLibrary(auto_name);
        // Auto-select the newly saved recording
        capture_playback_idx = (int)saved_recordings.size() - 1;
    }
}


// ── Capture Library ─────────────────────────────────────────────────

void App::saveRecordingToLibrary(const char* name) {
    if (recording.samples.empty()) {
        log(-1, "capture", "Nothing to save — no samples");
        return;
    }
    SavedRecording sr;
    snprintf(sr.name, sizeof(sr.name), "%s", name);
    // With fixed-rate oversampling, the configured rate is exact by construction.
    sr.sample_rate_hz = (double)record_rate_hz;
    sr.created_time = (double)time(nullptr);
    const char* src = "plugin";
    if (input_source == InputSource::CapturePlayback) src = "capture";
    snprintf(sr.source, sizeof(sr.source), "%s", src);
    sr.samples = recording.samples;
    saved_recordings.push_back(std::move(sr));
    log(-1, "capture", "Saved \"%s\" (%d samples, %.1fs, %.0f Hz actual)",
        name, (int)saved_recordings.back().samples.size(),
        saved_recordings.back().duration(), saved_recordings.back().sample_rate_hz);
    saveRecordingsToDisk();
}

void App::deleteRecordingFromLibrary(int idx) {
    if (idx < 0 || idx >= (int)saved_recordings.size()) return;
    if (capture_playing && capture_playback_idx == idx) stopCapturePlayback();
    log(-1, "capture", "Deleted \"%s\"", saved_recordings[idx].name);
    saved_recordings.erase(saved_recordings.begin() + idx);
    if (capture_playback_idx >= (int)saved_recordings.size())
        capture_playback_idx = (int)saved_recordings.size() - 1;
    saveRecordingsToDisk();
}

void App::startCapturePlayback(int idx) {
    if (idx < 0 || idx >= (int)saved_recordings.size()) return;
    capture_playback_idx = idx;
    capture_playing = true;
    capture_play_cursor = 0;
    capture_start_time = frame_time;
    capture_stop_requested = false;
    input_source = InputSource::CapturePlayback;

    // Start with ramp-in phase
    capture_ramp_phase = CaptureRampPhase::RampIn;
    capture_ramp_start = frame_time;
    memset(capture_last_vals, 0, sizeof(capture_last_vals));

    // Diagnostic: compute data range to detect flat recordings
    auto& sr = saved_recordings[idx];
    float vmin[6], vmax[6];
    for (int i = 0; i < 6; i++) { vmin[i] = 1e9f; vmax[i] = -1e9f; }
    for (auto& s : sr.samples) {
        for (int i = 0; i < 6; i++) {
            if (s.input[i] < vmin[i]) vmin[i] = s.input[i];
            if (s.input[i] > vmax[i]) vmax[i] = s.input[i];
        }
    }
    bool is_flat = true;
    for (int i = 0; i < 6; i++) {
        if (fabsf(vmax[i] - vmin[i]) > 0.01f) { is_flat = false; break; }
    }

    log(-1, "capture", "Playing \"%s\" (%d samples, %.1fs, %s)",
        sr.name, (int)sr.samples.size(), sr.duration(),
        capture_loop ? "loop" : "once");
    log(-1, "capture", "  Range: %.1f..%.1f  %.1f..%.1f  %.1f..%.1f  %.1f..%.1f  %.1f..%.1f  %.1f..%.1f",
        vmin[0], vmax[0], vmin[1], vmax[1], vmin[2], vmax[2],
        vmin[3], vmax[3], vmin[4], vmax[4], vmin[5], vmax[5]);
    if (is_flat)
        log(-1, "capture", "  WARNING: Recording data is flat (no variation). Record while moving sliders or use Test Signal.");
}

void App::stopCapturePlayback() {
    if (!capture_playing) return;
    // If currently playing data, initiate ramp-out instead of instant stop
    if (capture_ramp_phase == CaptureRampPhase::RampIn ||
        capture_ramp_phase == CaptureRampPhase::Playing) {
        capture_stop_requested = true;
        capture_ramp_phase = CaptureRampPhase::RampOut;
        capture_ramp_start = frame_time;
        // Snapshot current values for smooth blend to home
        if (!entities.empty())
            memcpy(capture_last_vals, entities[0].state.input_pct, sizeof(capture_last_vals));
        log(-1, "capture", "Ramping down to home...");
        return;
    }
    // Already ramping out or at home — force stop
    capture_playing = false;
    // Zero all entities to home
    for (auto& e : entities)
        memset(e.state.input_pct, 0, sizeof(e.state.input_pct));
    {
        std::lock_guard<std::mutex> lock(input_mutex);
        memset(shared_input, 0, sizeof(shared_input));
    }
    log(-1, "capture", "Playback stopped");
}

// Smoothstep: 0→1 for t in [0,1]
static float smoothstep01(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

// Sample the recording at a given elapsed time, interpolating between samples.
// Uses O(1) direct index lookup via sample_rate_hz (audio-style fixed-rate).
// The cursor parameter is updated for compatibility but not used for lookup.
static void sampleRecording(SavedRecording& sr, double elapsed, int& cursor, float out[6]) {
    auto& samples = sr.samples;
    if (samples.empty()) { memset(out, 0, 6 * sizeof(float)); return; }

    double dur = sr.duration();
    if (elapsed < 0.0) elapsed = 0.0;
    if (elapsed > dur) elapsed = dur;

    // Direct index from sample rate: O(1) lookup (no linear scan)
    double fidx = elapsed * sr.sample_rate_hz;
    int idx = (int)fidx;
    if (idx >= (int)samples.size() - 1) {
        memcpy(out, samples.back().input, 6 * sizeof(float));
        cursor = (int)samples.size() - 1;
        return;
    }
    cursor = idx;
    float alpha = (float)(fidx - (double)idx);
    for (int i = 0; i < 6; i++) {
        out[i] = samples[idx].input[i] * (1.0f - alpha) +
                 samples[idx + 1].input[i] * alpha;
    }
}

void App::updateCapturePlayback() {
    if (!capture_playing) return;
    if (capture_playback_idx < 0 || capture_playback_idx >= (int)saved_recordings.size()) {
        capture_playing = false;
        return;
    }

    auto& sr = saved_recordings[capture_playback_idx];
    if (sr.samples.empty()) { capture_playing = false; return; }

    float output[6] = {};
    double phase_elapsed = frame_time - capture_ramp_start;

    switch (capture_ramp_phase) {
        case CaptureRampPhase::RampIn: {
            // Ramp envelope from 0→1 over CAPTURE_RAMP_IN_S
            float env = smoothstep01((float)(phase_elapsed / CAPTURE_RAMP_IN_S));

            // Sample recording data (data clock runs during ramp-in)
            double data_elapsed = (frame_time - capture_start_time) * (double)capture_speed;
            float data[6];
            sampleRecording(sr, data_elapsed, capture_play_cursor, data);

            for (int i = 0; i < 6; i++)
                output[i] = data[i] * env;

            // Transition to Playing when ramp complete
            if (phase_elapsed >= CAPTURE_RAMP_IN_S)
                capture_ramp_phase = CaptureRampPhase::Playing;
            break;
        }

        case CaptureRampPhase::Playing: {
            double data_elapsed = (frame_time - capture_start_time) * (double)capture_speed;
            double dur = sr.duration();

            if (data_elapsed >= dur) {
                // End of data — start ramp-out
                // Snapshot last values for smooth blend
                if (!entities.empty())
                    memcpy(capture_last_vals, entities[0].state.input_pct, sizeof(capture_last_vals));
                capture_ramp_phase = CaptureRampPhase::RampOut;
                capture_ramp_start = frame_time;
                capture_stop_requested = !capture_loop;
                // Output the last values this frame
                memcpy(output, capture_last_vals, sizeof(output));
            } else {
                sampleRecording(sr, data_elapsed, capture_play_cursor, output);
            }
            break;
        }

        case CaptureRampPhase::RampOut: {
            // Ramp envelope from 1→0 over CAPTURE_RAMP_OUT_S
            float env = 1.0f - smoothstep01((float)(phase_elapsed / CAPTURE_RAMP_OUT_S));

            // Blend last data values toward home (0)
            for (int i = 0; i < 6; i++)
                output[i] = capture_last_vals[i] * env;

            if (phase_elapsed >= CAPTURE_RAMP_OUT_S) {
                if (capture_stop_requested) {
                    // Done — full stop
                    capture_playing = false;
                    for (auto& e : entities)
                        memset(e.state.input_pct, 0, sizeof(e.state.input_pct));
                    {
                        std::lock_guard<std::mutex> lock(input_mutex);
                        memset(shared_input, 0, sizeof(shared_input));
                    }
                    log(-1, "capture", "Playback stopped");
                    return;
                } else {
                    // Loop: hold at home briefly
                    capture_ramp_phase = CaptureRampPhase::HomeHold;
                    capture_ramp_start = frame_time;
                    memset(output, 0, sizeof(output));
                }
            }
            break;
        }

        case CaptureRampPhase::HomeHold: {
            // Hold at home for CAPTURE_HOME_HOLD_S
            memset(output, 0, sizeof(output));

            if (phase_elapsed >= CAPTURE_HOME_HOLD_S) {
                // Restart: reset data clock and cursor, ramp back in
                capture_start_time = frame_time;
                capture_play_cursor = 0;
                capture_ramp_phase = CaptureRampPhase::RampIn;
                capture_ramp_start = frame_time;
                log(-1, "capture", "Looping playback...");
            }
            break;
        }
    }

    // Feed into all entities and shared_input
    for (auto& e : entities) {
        memcpy(e.state.input_pct, output, sizeof(output));
    }
    {
        std::lock_guard<std::mutex> lock(input_mutex);
        memcpy(shared_input, output, sizeof(output));
    }
}

static const char* RECORDINGS_DIR = "recordings";

void App::saveRecordingsToDisk() {
    if (IsDocumentationMode()) return;
#ifdef _WIN32
    CreateDirectoryA(RECORDINGS_DIR, NULL);
#else
    mkdir(RECORDINGS_DIR, 0755);
#endif

    // Remove stale .stwr/.bin files beyond current recording count
    for (int ri = (int)saved_recordings.size(); ri < 1000; ri++) {
        char old_path[256];
        bool found = false;
        snprintf(old_path, sizeof(old_path), "%s/%03d.stwr", RECORDINGS_DIR, ri);
        if (remove(old_path) == 0) found = true;
        snprintf(old_path, sizeof(old_path), "%s/%03d.bin", RECORDINGS_DIR, ri);
        if (remove(old_path) == 0) found = true;
        if (!found) break;  // no more files at this index, stop scanning
    }

    // Write manifest (JSON array with metadata per recording)
    cJSON* root = cJSON_CreateArray();
    for (int ri = 0; ri < (int)saved_recordings.size(); ri++) {
        auto& sr = saved_recordings[ri];

        // Manifest entry
        cJSON* entry = cJSON_CreateObject();
        cJSON_AddStringToObject(entry, "name", sr.name);
        cJSON_AddNumberToObject(entry, "sample_rate_hz", sr.sample_rate_hz);
        cJSON_AddNumberToObject(entry, "created_time", sr.created_time);
        cJSON_AddStringToObject(entry, "source", sr.source);
        cJSON_AddItemToArray(root, entry);

        // Write versioned binary file: recordings/000.stwr
        char path[256];
        snprintf(path, sizeof(path), "%s/%03d.stwr", RECORDINGS_DIR, ri);
        FILE* f = fopen(path, "wb");
        if (f) {
            RecordingFileHeader hdr = {};
            hdr.magic = RECORDING_MAGIC;
            hdr.version = RECORDING_VERSION;
            hdr.sample_rate_hz = sr.sample_rate_hz;
            hdr.duration_sec = sr.duration();
            hdr.sample_count = (int32_t)sr.samples.size();
            hdr.channels = 6;
            hdr.created_time = sr.created_time;
            snprintf(hdr.name, sizeof(hdr.name), "%s", sr.name);
            snprintf(hdr.source, sizeof(hdr.source), "%s", sr.source);
            memset(hdr.reserved, 0, sizeof(hdr.reserved));

            fwrite(&hdr, sizeof(RecordingFileHeader), 1, f);
            fwrite(sr.samples.data(), sizeof(RecordSample), hdr.sample_count, f);
            fclose(f);
        }
    }

    char* str = cJSON_Print(root);
    if (str) {
        char path[256];
        snprintf(path, sizeof(path), "%s/manifest.json", RECORDINGS_DIR);
        FILE* f = fopen(path, "w");
        if (f) { fputs(str, f); fclose(f); }
        cJSON_free(str);
    }
    cJSON_Delete(root);
}

void App::loadRecordingsFromDisk() {
    if (IsDocumentationMode()) return;
    char path[256];
    snprintf(path, sizeof(path), "%s/manifest.json", RECORDINGS_DIR);
    FILE* f = fopen(path, "r");
    if (!f) return;

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return; }

    char* buf = (char*)malloc(sz + 1);
    fread(buf, 1, sz, f);
    buf[sz] = 0;
    fclose(f);

    cJSON* root = cJSON_Parse(buf);
    free(buf);
    if (!root || !cJSON_IsArray(root)) { cJSON_Delete(root); return; }

    saved_recordings.clear();
    int ri = 0;
    cJSON* item;
    cJSON_ArrayForEach(item, root) {
        SavedRecording sr = {};
        sr.sample_rate_hz = 200.0;  // default
        sr.created_time = 0.0;
        snprintf(sr.source, sizeof(sr.source), "unknown");

        // New format: manifest entries are objects with metadata
        if (cJSON_IsObject(item)) {
            cJSON* v;
            if ((v = cJSON_GetObjectItem(item, "name")))
                snprintf(sr.name, sizeof(sr.name), "%s", v->valuestring ? v->valuestring : "Untitled");
            if ((v = cJSON_GetObjectItem(item, "sample_rate_hz")))
                sr.sample_rate_hz = v->valuedouble;
            if ((v = cJSON_GetObjectItem(item, "created_time")))
                sr.created_time = v->valuedouble;
            if ((v = cJSON_GetObjectItem(item, "source")))
                snprintf(sr.source, sizeof(sr.source), "%s", v->valuestring ? v->valuestring : "unknown");
        } else if (cJSON_IsString(item)) {
            // Legacy format: manifest entries are plain name strings
            snprintf(sr.name, sizeof(sr.name), "%s", item->valuestring ? item->valuestring : "Untitled");
        }

        // Try versioned .stwr file first, fall back to legacy .bin
        bool loaded = false;
        char binpath[256];
        snprintf(binpath, sizeof(binpath), "%s/%03d.stwr", RECORDINGS_DIR, ri);
        FILE* bf = fopen(binpath, "rb");
        if (bf) {
            RecordingFileHeader hdr = {};
            if (fread(&hdr, sizeof(RecordingFileHeader), 1, bf) == 1 &&
                hdr.magic == RECORDING_MAGIC && hdr.version >= 1 &&
                hdr.sample_count > 0 && hdr.sample_count < 10000000) {
                sr.sample_rate_hz = hdr.sample_rate_hz;
                sr.created_time = hdr.created_time;
                snprintf(sr.name, sizeof(sr.name), "%s", hdr.name);
                snprintf(sr.source, sizeof(sr.source), "%s", hdr.source);
                sr.samples.resize(hdr.sample_count);
                fread(sr.samples.data(), sizeof(RecordSample), hdr.sample_count, bf);
                loaded = true;
            }
            fclose(bf);
        }

        if (!loaded) {
            // Legacy .bin fallback (v0 format: int count + raw samples)
            snprintf(binpath, sizeof(binpath), "%s/%03d.bin", RECORDINGS_DIR, ri);
            bf = fopen(binpath, "rb");
            if (bf) {
                int count = 0;
                fread(&count, sizeof(int), 1, bf);
                if (count > 0 && count < 10000000) {
                    sr.samples.resize(count);
                    fread(sr.samples.data(), sizeof(RecordSample), count, bf);
                }
                fclose(bf);
            }
        }

        // Recompute actual sample rate from timestamps (fixes stale values
        // saved when configured rate exceeded achievable frame rate)
        if (sr.samples.size() > 1) {
            double dur = sr.samples.back().time;
            if (dur > 0.0)
                sr.sample_rate_hz = (double)(sr.samples.size() - 1) / dur;
        }

        saved_recordings.push_back(std::move(sr));
        ri++;
    }

    cJSON_Delete(root);
    if (!saved_recordings.empty()) {
        log(-1, "capture", "Loaded %d saved recording(s) from disk", (int)saved_recordings.size());
    }
}

// ── Device cue sync (raw-HIL) ────────────────────────────────────────
// In RAW mode the ESP is the single cue engine, so app-side tuning changes
// must be forwarded as MCA:* commands or the device feel never updates.
// Queued (one per frame) to interleave safely with the COBS data stream.

// Build the cue snapshot used for change detection (must list every parameter
// pushCueSettingsToDevice sends). 34 floats used; array is 46 with zeroed
// tail — keep the memset or memcmp compares stack garbage.
static void cueSnapshot(const Entity& e, float s[46]) {
    memset(s, 0, 46 * sizeof(float));
    int k = 0;
    s[k++] = e.config.intensity;
    for (int i = 0; i < 6; i++) s[k++] = e.config.axis_gain[i];
    for (int i = 0; i < 6; i++) s[k++] = e.config.axis_invert[i] ? 1.0f : 0.0f;
    for (int i = 0; i < 6; i++) {
        s[k++] = e.config.mca.channels[i].gain;
        s[k++] = e.config.mca.channels[i].hp_enabled ? e.config.mca.channels[i].hp.fc : 0.0f;
        s[k++] = e.config.mca.channels[i].lp_enabled ? e.config.mca.channels[i].lp.fc : 0.0f;
    }
    s[k++] = e.config.mca.tilt.enabled ? 1.0f : 0.0f;
    s[k++] = e.config.mca.tilt.surge_gain;
    s[k++] = e.config.mca.tilt.sway_gain;
}

void App::pushCueSettingsToDevice(Entity& e) {
    if (e.type != EntityType::HIL || !e.serial || !e.serial->isOpen()) return;
    if (!e.hil_handshake_ok || !e.hil_cap_raw || !e.hil_raw_mode) return;

    char c[64];
    snprintf(c, sizeof(c), "MCA:INTENSITY=%.4f", e.config.intensity / 100.0f);
    e.hil_cmd_queue.push_back(c);
    for (int i = 0; i < 6; i++) {
        snprintf(c, sizeof(c), "MCA:GAIN=%d,%.4f", i, e.config.axis_gain[i] / 100.0f);
        e.hil_cmd_queue.push_back(c);
        snprintf(c, sizeof(c), "MCA:INVERT=%d,%d", i, e.config.axis_invert[i] ? 1 : 0);
        e.hil_cmd_queue.push_back(c);
        snprintf(c, sizeof(c), "MCA:CHGAIN=%d,%.4f", i, e.config.mca.channels[i].gain);
        e.hil_cmd_queue.push_back(c);
        snprintf(c, sizeof(c), "MCA:HPFC=%d,%.4f", i,
                 e.config.mca.channels[i].hp_enabled ? e.config.mca.channels[i].hp.fc : 0.0f);
        e.hil_cmd_queue.push_back(c);
        snprintf(c, sizeof(c), "MCA:LPFC=%d,%.4f", i,
                 e.config.mca.channels[i].lp_enabled ? e.config.mca.channels[i].lp.fc : 0.0f);
        e.hil_cmd_queue.push_back(c);
    }
    snprintf(c, sizeof(c), "MCA:TILT=%.4f,%.4f",
             e.config.mca.tilt.enabled ? e.config.mca.tilt.surge_gain : 0.0f,
             e.config.mca.tilt.enabled ? e.config.mca.tilt.sway_gain  : 0.0f);
    e.hil_cmd_queue.push_back(c);
    cueSnapshot(e, e.cue_pushed);
    memcpy(e.cue_prev, e.cue_pushed, sizeof(e.cue_prev));
    e.cue_sync_valid = true;
    e.cue_push_time = frame_time;
    e.cue_persisted = false;             // arm the auto-persist timer
    log(e.id, "hil", "Cue settings pushed to device (%d cmds queued)", 32);
}

// Per-frame cue auto-sync: detect ANY cue-parameter change (UI slider, API,
// preset load) on a raw-HIL entity and push it once the value settles for
// 0.3s (debounce, so slider drags don't spam 32-command bursts).
void App::syncCueSettings(Entity& e) {
    if (e.type != EntityType::HIL || !e.serial || !e.serial->isOpen()) return;
    if (!e.hil_handshake_ok || !e.hil_cap_raw || !e.hil_raw_mode) return;

    float snap[46];
    cueSnapshot(e, snap);
    if (!e.cue_sync_valid) {
        // First sight of a connected device: push so device == app from the start.
        pushCueSettingsToDevice(e);
        return;
    }
    if (memcmp(snap, e.cue_prev, sizeof(snap)) != 0) {
        e.cue_change_time = frame_time;               // still moving
        memcpy(e.cue_prev, snap, sizeof(e.cue_prev));
        return;
    }
    if (memcmp(snap, e.cue_pushed, sizeof(snap)) != 0 &&
        frame_time - e.cue_change_time > 0.3) {
        for (int i = 0; i < 46; i++) {
            if (snap[i] != e.cue_pushed[i]) {
                DEV_LOG("hil", "cue-sync diff [%d]: pushed=%.6f now=%.6f",
                        i, e.cue_pushed[i], snap[i]);
                break;
            }
        }
        pushCueSettingsToDevice(e);
        return;
    }

    // Auto-persist: once tuning has settled for 5s after a push, save to the
    // device's NVS so a power-cycle boots with EXACTLY the feel that was last
    // heard/felt. Without this, boot reverts to the last explicit save — the
    // "cue resets on boot" surprise (dangerous if that save was stronger).
    if (!e.cue_persisted && frame_time - e.cue_push_time > 5.0) {
        e.hil_cmd_queue.push_back("MCA:SAVE");
        e.cue_persisted = true;
        saveSettings();
        log(e.id, "hil", "Cue settings auto-persisted (device NVS + app settings)");
    }
}

// ── MCA Dynamics Presets ─────────────────────────────────────────────

static const char* MCA_PRESETS_FILE = "mca_dynamics_presets.json";

// Helper: serialize a BiquadFilter's tuning params (fc, Q) to JSON
static void biquadToJSON(cJSON* obj, const char* prefix, const BiquadFilter& f) {
    char key[32];
    snprintf(key, sizeof(key), "%s_fc", prefix);  cJSON_AddNumberToObject(obj, key, f.fc);
    snprintf(key, sizeof(key), "%s_Q", prefix);   cJSON_AddNumberToObject(obj, key, f.Q);
}

static void biquadFromJSON(cJSON* obj, const char* prefix, BiquadFilter& f) {
    char key[32]; cJSON* val;
    snprintf(key, sizeof(key), "%s_fc", prefix);
    if ((val = cJSON_GetObjectItem(obj, key))) f.fc = (float)val->valuedouble;
    snprintf(key, sizeof(key), "%s_Q", prefix);
    if ((val = cJSON_GetObjectItem(obj, key))) f.Q = (float)val->valuedouble;
}

static cJSON* mcaConfigToJSON(const MotionCueingConfig& mca, float intensity, const float axis_gain[6]) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(obj, "enabled", mca.enabled);
    cJSON_AddNumberToObject(obj, "preset", mca.preset);
    cJSON_AddNumberToObject(obj, "intensity", intensity);

    cJSON* gains = cJSON_AddArrayToObject(obj, "axis_gain");
    for (int i = 0; i < 6; i++) cJSON_AddItemToArray(gains, cJSON_CreateNumber(axis_gain[i]));

    // Per-axis channel filters
    cJSON* channels = cJSON_AddArrayToObject(obj, "channels");
    for (int i = 0; i < 6; i++) {
        const AxisChannelFilter& ch = mca.channels[i];
        cJSON* cj = cJSON_CreateObject();
        cJSON_AddNumberToObject(cj, "hp_enabled", ch.hp_enabled);
        biquadToJSON(cj, "hp", ch.hp);
        cJSON_AddNumberToObject(cj, "lp_enabled", ch.lp_enabled);
        biquadToJSON(cj, "lp", ch.lp);
        cJSON_AddNumberToObject(cj, "gain", ch.gain);
        cJSON_AddNumberToObject(cj, "rate_limit", ch.rate_limit);
        cJSON_AddItemToArray(channels, cj);
    }

    // Tilt coordination
    cJSON* tilt = cJSON_AddObjectToObject(obj, "tilt");
    cJSON_AddNumberToObject(tilt, "enabled", mca.tilt.enabled);
    cJSON_AddNumberToObject(tilt, "surge_gain", mca.tilt.surge_gain);
    cJSON_AddNumberToObject(tilt, "sway_gain", mca.tilt.sway_gain);
    cJSON_AddNumberToObject(tilt, "fc", mca.tilt.fc);
    cJSON_AddNumberToObject(tilt, "Q", mca.tilt.Q);
    cJSON_AddNumberToObject(tilt, "hp_enabled", mca.tilt.hp_enabled);
    cJSON_AddNumberToObject(tilt, "hp_fc", mca.tilt.hp_fc);
    cJSON_AddNumberToObject(tilt, "hp_Q", mca.tilt.hp_Q);
    cJSON_AddNumberToObject(tilt, "surge_hp_enabled", mca.tilt.surge_hp_enabled);
    cJSON_AddNumberToObject(tilt, "sway_hp_enabled", mca.tilt.sway_hp_enabled);
    cJSON_AddNumberToObject(tilt, "sway_hp_fc", mca.tilt.sway_hp_fc);
    cJSON_AddNumberToObject(tilt, "sway_hp_Q", mca.tilt.sway_hp_Q);
    cJSON_AddNumberToObject(tilt, "hp_linked", mca.tilt.hp_linked);

    return obj;
}

static void mcaConfigFromJSON(cJSON* obj, MotionCueingConfig& mca, float& intensity, float axis_gain[6]) {
    cJSON* val;
    if ((val = cJSON_GetObjectItem(obj, "enabled")))   mca.enabled = val->valueint;
    if ((val = cJSON_GetObjectItem(obj, "preset")))     mca.preset = val->valueint;
    if ((val = cJSON_GetObjectItem(obj, "intensity")))  intensity = (float)val->valuedouble;

    cJSON* gains = cJSON_GetObjectItem(obj, "axis_gain");
    if (gains && cJSON_IsArray(gains)) {
        for (int i = 0; i < 6 && i < cJSON_GetArraySize(gains); i++)
            axis_gain[i] = (float)cJSON_GetArrayItem(gains, i)->valuedouble;
    }

    cJSON* channels = cJSON_GetObjectItem(obj, "channels");
    if (channels && cJSON_IsArray(channels)) {
        for (int i = 0; i < 6 && i < cJSON_GetArraySize(channels); i++) {
            cJSON* cj = cJSON_GetArrayItem(channels, i);
            AxisChannelFilter& ch = mca.channels[i];
            if ((val = cJSON_GetObjectItem(cj, "hp_enabled"))) ch.hp_enabled = val->valueint;
            biquadFromJSON(cj, "hp", ch.hp);
            if ((val = cJSON_GetObjectItem(cj, "lp_enabled"))) ch.lp_enabled = val->valueint;
            biquadFromJSON(cj, "lp", ch.lp);
            if ((val = cJSON_GetObjectItem(cj, "gain")))       ch.gain = (float)val->valuedouble;
            if ((val = cJSON_GetObjectItem(cj, "rate_limit"))) ch.rate_limit = (float)val->valuedouble;
        }
    }

    cJSON* tilt = cJSON_GetObjectItem(obj, "tilt");
    if (tilt && cJSON_IsObject(tilt)) {
        if ((val = cJSON_GetObjectItem(tilt, "enabled")))    mca.tilt.enabled = val->valueint;
        if ((val = cJSON_GetObjectItem(tilt, "surge_gain"))) mca.tilt.surge_gain = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "sway_gain")))  mca.tilt.sway_gain = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "fc")))         mca.tilt.fc = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "Q")))          mca.tilt.Q = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "hp_enabled"))) mca.tilt.hp_enabled = val->valueint;
        if ((val = cJSON_GetObjectItem(tilt, "hp_fc")))      mca.tilt.hp_fc = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "hp_Q")))       mca.tilt.hp_Q = (float)val->valuedouble;

        // v5 independent surge/sway HP fields
        if ((val = cJSON_GetObjectItem(tilt, "surge_hp_enabled"))) mca.tilt.surge_hp_enabled = val->valueint;
        if ((val = cJSON_GetObjectItem(tilt, "sway_hp_enabled")))  mca.tilt.sway_hp_enabled = val->valueint;
        if ((val = cJSON_GetObjectItem(tilt, "sway_hp_fc")))       mca.tilt.sway_hp_fc = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "sway_hp_Q")))        mca.tilt.sway_hp_Q = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "hp_linked")))        mca.tilt.hp_linked = val->valueint;

        // Backward compat: old configs without per-channel fields → copy from shared
        if (!cJSON_GetObjectItem(tilt, "surge_hp_enabled")) {
            mca.tilt.surge_hp_enabled = mca.tilt.hp_enabled;
            mca.tilt.sway_hp_enabled  = mca.tilt.hp_enabled;
            mca.tilt.sway_hp_fc = mca.tilt.hp_fc;
            mca.tilt.sway_hp_Q  = mca.tilt.hp_Q;
            mca.tilt.hp_linked = 1;
        }
    }
}

void App::saveMcaPreset(const char* name, const MotionCueingConfig& mca, float intensity, const float axis_gain[6]) {
    // Overwrite if same name exists
    for (auto& p : mca_presets) {
        if (strcmp(p.name, name) == 0) {
            p.mca = mca;
            p.intensity = intensity;
            memcpy(p.axis_gain, axis_gain, sizeof(p.axis_gain));
            saveMcaPresetsToDisk();
            log(-1, "dynamics", "Preset '%s' updated", name);
            return;
        }
    }
    McaDynamicsPreset p;
    memset(&p, 0, sizeof(p));
    snprintf(p.name, sizeof(p.name), "%s", name);
    p.is_builtin = false;
    p.mca = mca;
    p.intensity = intensity;
    memcpy(p.axis_gain, axis_gain, sizeof(p.axis_gain));
    mca_presets.push_back(p);
    saveMcaPresetsToDisk();
    log(-1, "dynamics", "Preset '%s' saved", name);
}

void App::deleteMcaPreset(int idx) {
    if (idx >= 0 && idx < (int)mca_presets.size() && !mca_presets[idx].is_builtin) {
        log(-1, "dynamics", "Preset '%s' deleted", mca_presets[idx].name);
        mca_presets.erase(mca_presets.begin() + idx);
        saveMcaPresetsToDisk();
    }
}

void App::loadMcaPreset(int idx, MotionCueingConfig& mca, float& intensity, float axis_gain[6]) {
    if (idx < 0 || idx >= (int)mca_presets.size()) return;
    const McaDynamicsPreset& p = mca_presets[idx];
    float sr = mca.sample_rate;  // preserve current sample rate
    mca = p.mca;
    mca.sample_rate = sr;
    intensity = p.intensity;
    memcpy(axis_gain, p.axis_gain, sizeof(float) * 6);
    // Recalculate biquad coefficients at current sample rate
    for (int i = 0; i < 6; i++) {
        if (mca.channels[i].hp_enabled && mca.channels[i].hp.fc > 0)
            biquadSetHighpass(&mca.channels[i].hp, mca.channels[i].hp.fc, sr, mca.channels[i].hp.Q);
        if (mca.channels[i].lp_enabled && mca.channels[i].lp.fc > 0)
            biquadSetLowpass(&mca.channels[i].lp, mca.channels[i].lp.fc, sr, mca.channels[i].lp.Q);
    }
    if (mca.tilt.enabled && mca.tilt.fc > 0) {
        biquadSetLowpass(&mca.tilt.surge_lp, mca.tilt.fc, sr, mca.tilt.Q);
        biquadSetLowpass(&mca.tilt.sway_lp,  mca.tilt.fc, sr, mca.tilt.Q);
    }
    resetMotionCueing(&mca);
    log(-1, "dynamics", "Loaded preset '%s'", p.name);
}

void App::saveMcaPresetsToDisk() {
    if (IsDocumentationMode()) return;
    cJSON* root = cJSON_CreateArray();
    for (auto& p : mca_presets) {
        cJSON* pj = mcaConfigToJSON(p.mca, p.intensity, p.axis_gain);
        cJSON_AddStringToObject(pj, "name", p.name);
        cJSON_AddBoolToObject(pj, "is_builtin", p.is_builtin);
        cJSON_AddItemToArray(root, pj);
    }
    char* str = cJSON_Print(root);
    if (str) {
        FILE* f = fopen(MCA_PRESETS_FILE, "w");
        if (f) { fputs(str, f); fclose(f); }
        cJSON_free(str);
    }
    cJSON_Delete(root);
}

void App::loadMcaPresetsFromDisk() {
    mca_presets.clear();

    // Always seed with built-in presets
    const char* builtin_names[] = {"Off", "Gentle", "Moderate", "Aggressive", "Race Pro"};
    for (int i = 0; i < MCA_PRESET_COUNT; i++) {
        McaDynamicsPreset p;
        memset(&p, 0, sizeof(p));
        snprintf(p.name, sizeof(p.name), "%s", builtin_names[i]);
        p.is_builtin = true;
        initMotionCueing(&p.mca, 60.0f);
        setMotionCueingPreset(&p.mca, i);
        p.intensity = 100.0f;
        for (int j = 0; j < 6; j++) p.axis_gain[j] = 100.0f;
        mca_presets.push_back(p);
    }

    if (IsDocumentationMode()) return;  // built-ins only; no user preset file access

    // Load user presets from file (may also override built-in presets if user saved over them)
    FILE* f = fopen(MCA_PRESETS_FILE, "r");
    if (!f) return;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return; }
    char* buf = (char*)malloc(sz + 1);
    fread(buf, 1, sz, f);
    buf[sz] = 0;
    fclose(f);

    cJSON* root = cJSON_Parse(buf);
    free(buf);
    if (!root || !cJSON_IsArray(root)) { cJSON_Delete(root); return; }

    cJSON* pj;
    int user_count = 0;
    cJSON_ArrayForEach(pj, root) {
        cJSON* name_val = cJSON_GetObjectItem(pj, "name");
        cJSON* builtin_val = cJSON_GetObjectItem(pj, "is_builtin");
        if (!name_val || !name_val->valuestring) continue;

        bool is_builtin = builtin_val && cJSON_IsTrue(builtin_val);

        if (is_builtin) {
            // Override the matching built-in preset with saved values
            for (auto& bp : mca_presets) {
                if (bp.is_builtin && strcmp(bp.name, name_val->valuestring) == 0) {
                    mcaConfigFromJSON(pj, bp.mca, bp.intensity, bp.axis_gain);
                    break;
                }
            }
        } else {
            // Add user preset
            McaDynamicsPreset p;
            memset(&p, 0, sizeof(p));
            snprintf(p.name, sizeof(p.name), "%s", name_val->valuestring);
            p.is_builtin = false;
            initMotionCueing(&p.mca, 60.0f);
            p.intensity = 100.0f;
            for (int j = 0; j < 6; j++) p.axis_gain[j] = 100.0f;
            mcaConfigFromJSON(pj, p.mca, p.intensity, p.axis_gain);
            mca_presets.push_back(p);
            user_count++;
        }
    }
    cJSON_Delete(root);
    if (user_count > 0)
        log(-1, "dynamics", "Loaded %d user preset(s)", user_count);
}

// ── UDP Listener ────────────────────────────────────────────────────

static double GetTimeSeconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}


// ── Source Switch Ramp-to-Home ───────────────────────────────────────

void App::requestSourceSwitch(InputSource target) {
    if (target == input_source && !source_switch_active) return;  // already there
    if (source_switch_active && target == source_switch_target) return;  // already ramping to this

    // Snapshot current input values for smooth blend
    {
        std::lock_guard<std::mutex> lock(input_mutex);
        memcpy(source_switch_from, shared_input, sizeof(source_switch_from));
    }
    source_switch_target = target;
    source_switch_start = frame_time;
    source_switch_active = true;
    log(-1, "input", "Ramping to home before switching to %s...",
        target == InputSource::CapturePlayback ? "Capture" :
        target == InputSource::Plugin ? "Plugin" : "?");
}

// ── Update ──────────────────────────────────────────────────────────

void App::update() {
    // ── Source-switch ramp-to-home ──
    if (source_switch_active) {
        double elapsed = frame_time - source_switch_start;
        float t = (SOURCE_RAMP_OUT_S > 0.0f) ? (float)(elapsed / SOURCE_RAMP_OUT_S) : 1.0f;
        if (t >= 1.0f) t = 1.0f;
        // Smoothstep ease-out
        float alpha = 1.0f - t * t * (3.0f - 2.0f * t);

        // Blend from snapshot toward zero
        float ramped[6];
        for (int i = 0; i < 6; i++)
            ramped[i] = source_switch_from[i] * alpha;

        // Write to all entities and shared_input
        for (auto& e : entities)
            memcpy(e.state.input_pct, ramped, sizeof(ramped));
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            memcpy(shared_input, ramped, sizeof(ramped));
        }

        if (t >= 1.0f) {
            // Ramp complete — stop old source, switch to new
            if (capture_playing)        stopCapturePlayback();
            if (plugin_mgr.activeIndex() >= 0) {
                plugin_mgr.deactivateActive();
                active_plugin_idx = -1;
            }

            input_source = source_switch_target;
            source_switch_active = false;

            // Zero everything
            memset(shared_input, 0, sizeof(shared_input));
            for (auto& e : entities)
                memset(e.state.input_pct, 0, sizeof(e.state.input_pct));

            // Activate plugin if switching to Plugin source AND motion is started
            if (motion_started && input_source == InputSource::Plugin && active_plugin_idx >= 0) {
                float sr = (fps > 1.0) ? (float)fps : 60.0f;
                if (!plugin_mgr.activatePlugin(active_plugin_idx, sr)) {
                    log(-1, "plugin", "Failed to activate plugin");
                    active_plugin_idx = -1;
                }
            }

            log(-1, "input", "Source switched, rig at home");
        }
        // Skip normal input processing during ramp
        goto skip_input_processing;
    }

    // ── Motion gate: when stopped, force everything to zero ──
    if (!motion_started) {
        if (capture_playing) {
            capture_playing = false;
            log(-1, "capture", "Playback stopped (motion off)");
        }
        for (auto& e : entities)
            memset(e.state.input_pct, 0, sizeof(e.state.input_pct));
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            memset(shared_input, 0, sizeof(shared_input));
        }
        goto skip_input_processing;
    }

    // ── S-curve ramp from home to current input on START ──
    if (start_ramp_active) {
        float t = (float)(frame_time - start_ramp_begin) / START_RAMP_S;
        if (t >= 1.0f) {
            t = 1.0f;
            start_ramp_active = false;
        }
        // Smoothstep S-curve: 3t² - 2t³
        float s = t * t * (3.0f - 2.0f * t);
        for (auto& e : entities) {
            for (int i = 0; i < 6; i++)
                e.state.input_pct[i] = start_ramp_target[i] * s;
        }
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            for (int i = 0; i < 6; i++)
                shared_input[i] = start_ramp_target[i] * s;
        }
    }

    // Handle capture playback — writes directly to entity input_pct
    if (capture_playing) {
        updateCapturePlayback();
    }

    // ── Plugin input processing ─────────────────────────────────────
    if (input_source == InputSource::Plugin && plugin_mgr.activeIndex() >= 0) {
        float plugin_out[6] = {};
        static double last_plugin_time = 0.0;
        double dt = (last_plugin_time > 0.0) ? (frame_time - last_plugin_time) : 0.0;
        if (dt < 0.0 || dt > 0.1) dt = 0.0;
        last_plugin_time = frame_time;
        float sr = (fps > 1.0) ? (float)fps : 60.0f;
        if (plugin_mgr.processActive(frame_time, dt, frame_count, sr, plugin_out)) {
            std::lock_guard<std::mutex> lock(input_mutex);
            memcpy(shared_input, plugin_out, sizeof(shared_input));
        }
    }

    // ── Input bus sync ────────────────────────────────────────────────
    // Ensure shared_input and entity.input_pct are always consistent.
    // Plugin writes to shared_input → push to entities.
    // Capture writes to entity.input_pct → push to shared_input.
    if (input_source == InputSource::Plugin && plugin_mgr.activeIndex() >= 0) {
        // External source → entities
        float snap[6];
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            memcpy(snap, shared_input, sizeof(snap));
        }
        for (auto& e : entities) {
            memcpy(e.state.input_pct, snap, sizeof(snap));
        }
    } else {
        // Manual / Capture → shared_input
        if (!entities.empty()) {
            std::lock_guard<std::mutex> lock(input_mutex);
            memcpy(shared_input, entities[0].state.input_pct, sizeof(shared_input));
        }
    }

    // Record at exact configured sample rate via oversampling (audio-style).
    // Each frame, fill ALL samples that should exist up to wall-clock time.
    // First-order hold: linearly interpolate between previous and current frame
    // input values for smooth waveforms (eliminates staircase artifacts).
    // Result: exactly rate × duration samples with perfectly spaced timestamps.
    if (recording.mode == RecordMode::Recording) {
        // Advance first-order hold: prev ← old curr, curr ← new input
        memcpy(recording.prev_input, recording.curr_input, sizeof(recording.prev_input));
        recording.prev_time = recording.curr_time;
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            memcpy(recording.curr_input, shared_input, sizeof(recording.curr_input));
        }
        recording.curr_time = frame_time;

        double elapsed = frame_time - recording.start_time;
        int target_count = (int)(elapsed * (double)record_rate_hz) + 1;
        if (target_count > (int)recording.samples.size()) {
            double frame_dt = recording.curr_time - recording.prev_time;
            while ((int)recording.samples.size() < target_count) {
                RecordSample s;
                s.time = (double)recording.samples.size() / (double)record_rate_hz;
                // Interpolate: where does this sample fall between prev and curr frame?
                double sample_wall = recording.start_time + s.time;
                float alpha = (frame_dt > 1e-9)
                    ? (float)((sample_wall - recording.prev_time) / frame_dt)
                    : 1.0f;
                if (alpha < 0.0f) alpha = 0.0f;
                if (alpha > 1.0f) alpha = 1.0f;
                for (int i = 0; i < 6; i++) {
                    s.input[i] = recording.prev_input[i] * (1.0f - alpha)
                               + recording.curr_input[i] * alpha;
                }
                recording.samples.push_back(s);
            }
        }
    }

    // Grab shared input (thread-safe)
    float current_input[6];
    {
        std::lock_guard<std::mutex> lock(input_mutex);
        memcpy(current_input, shared_input, sizeof(current_input));
    }

    // Block scope: everything here can be safely skipped by goto skip_input_processing
    {
    // Console input logging (user-controlled rate via console_log_rate)
    // Only log when the selected source is actually active/connected
    bool source_active = true;
    if (input_source == InputSource::Plugin && plugin_mgr.activeIndex() < 0) source_active = false;

    if (console_log_rate > 0 && source_active) {
        static const double rate_intervals[] = {0.0, 1.0, 0.1, 1.0/30.0, 1.0/60.0, 0.0};
        double interval = rate_intervals[console_log_rate];
        bool should_log = (interval <= 0.0) || (frame_time - last_input_log_time >= interval);
        if (should_log) {
            last_input_log_time = frame_time;
            const char* src_str = "plugin";
            if (input_source == InputSource::CapturePlayback) src_str = "capture";
            log(-1, src_str, "IN: %+.0f %+.0f %+.0f %+.0f %+.0f %+.0f",
                current_input[0], current_input[1], current_input[2],
                current_input[3], current_input[4], current_input[5]);
        }
    }

    // Push to global input history ring buffer (at record_rate_hz)
    {
        double hist_interval = 1.0 / (double)record_rate_hz;
        if (frame_time - input_history_last_push >= hist_interval) {
            input_history_last_push = frame_time;
            int idx = input_history_head;

            // shared_input is always current (bus sync above keeps it in sync)
            float vals[6];
            {
                std::lock_guard<std::mutex> lock(input_mutex);
                memcpy(vals, shared_input, sizeof(vals));
            }

            input_history_time[idx] = (float)frame_time;
            for (int i = 0; i < 6; i++)
                input_history[i][idx] = vals[i];
            input_history_head = (input_history_head + 1) % INPUT_HISTORY_LEN;
            if (input_history_count < INPUT_HISTORY_LEN) input_history_count++;
        }
    }

    // Compute input spectrum (simple DFT, updated every ~100ms)
    {
        static double last_dft_time = 0.0;
        if (frame_time - last_dft_time >= 0.1 && input_history_count >= 64) {
            last_dft_time = frame_time;
            input_spectrum_freq_max = (float)record_rate_hz * 0.5f;  // Nyquist

            // Use ~1 second rolling window for responsive spectrum display
            int max_window = record_rate_hz;  // 1 second of samples
            int N = input_history_count;
            if (N > max_window) N = max_window;
            if (N > INPUT_HISTORY_LEN) N = INPUT_HISTORY_LEN;
            int start = (input_history_head - N + INPUT_HISTORY_LEN) % INPUT_HISTORY_LEN;

            for (int axis = 0; axis < 6; axis++) {
                for (int k = 0; k < SPECTRUM_BINS; k++) {
                    double freq = (double)k / (double)SPECTRUM_BINS * input_spectrum_freq_max;
                    double re = 0.0, im = 0.0;
                    for (int n = 0; n < N; n++) {
                        int si = (start + n) % INPUT_HISTORY_LEN;
                        double angle = -2.0 * M_PI * freq * (double)n / (double)record_rate_hz;
                        re += input_history[axis][si] * cos(angle);
                        im += input_history[axis][si] * sin(angle);
                    }
                    input_spectrum[axis][k] = (float)(sqrt(re * re + im * im) / (double)N);
                }
            }
        }
    }
    } // end block scope (skippable by goto)

skip_input_processing:
    // F4: Process lifecycle, freshness guards, and safety intents for DISABLED MP6 entities
    for (auto& e : entities) {
        if (!e.enabled && e.type == EntityType::HIL && e.hil_backend == HilBackend::MP6) {
            // Disabling a moving entity must take an explicit stop path
            if (e.mp6.lifecycle == Mp6Lifecycle::Running ||
                e.mp6.lifecycle == Mp6Lifecycle::Initializing ||
                e.mp6.lifecycle == Mp6Lifecycle::Parking || e.mp6.sending_config || e.mp6.shutdown_pending ||
                e.mp6.pending_intent == Mp6Intent::Start || e.mp6.pending_intent == Mp6Intent::Initialize ||
                e.mp6.pending_intent == Mp6Intent::Configure || e.mp6.pending_intent == Mp6Intent::Shutdown) {
                std::string why;
                mp6Command(e, Mp6Intent::Stop, &why);
            }
            e.mp6.step_target_valid = false;
            mp6UpdateEntity(e);
            e.mp6.viz.axis_test_active = false;
            e.mp6.viz.axis_test_geometry_valid = false;
            e.mp6.viz.rod_lines_valid = false;
            for (int i = 0; i < 6; ++i) e.mp6.viz.measured_crank_valid[i] = false;
        }
    }

    // Run pipeline for each enabled entity
    for (auto& e : entities) {
        if (!e.enabled) continue;

        // HIL entities: block the ESP32-conditioned pipeline until handshake
        // completes. Before Ready, we don't know geometry/bit-depth/scales —
        // all output would be garbage. Zero everything so the UI shows idle/
        // home state. MP6 entities are NOT gated this way: their display-side
        // pipeline (model pose) is independent of device parameters, and all
        // device-side gating (config ack, AV_OK, identity, fault latch) lives
        // in the MP6 session/TX path.
        bool hil_blocked = (e.type == EntityType::HIL
                            && e.hil_backend == HilBackend::ESP32
                            && !e.hil_handshake_ok);

        float scaled_pct[6] = {};

        if (!hil_blocked) {
            // Common: read input, apply input filter, MCA dynamics, then intensity/gain
            float pct[6];
            memcpy(pct, e.state.input_pct, sizeof(pct));

            // Pre-MCA input filtering (signal conditioning)
            if (e.config.input_filter.enabled) {
                float sr = (fps > 1.0) ? (float)fps : 60.0f;
                if (fabsf(e.config.input_filter.sample_rate - sr) > 5.0f)
                    inputFilterUpdateSampleRate(&e.config.input_filter, sr);
                float filt_out[6];
                processInputFilter(&e.config.input_filter, pct, filt_out);
                memcpy(pct, filt_out, sizeof(pct));
            }

            // MCA: HP washout + LP smoothing (if enabled via Dynamics panel)
            if (e.config.mca.enabled) {
                // Keep MCA sample rate in sync with actual frame rate
                float sr = (fps > 1.0) ? (float)fps : 60.0f;
                if (fabsf(e.config.mca.sample_rate - sr) > 5.0f)
                    mcaUpdateSampleRate(&e.config.mca, sr);
                float mca_out[6];
                processMotionCueing(&e.config.mca, pct, mca_out);
                memcpy(pct, mca_out, sizeof(pct));
            }

            for (int i = 0; i < 6; i++) {
                float inv = e.config.axis_invert[i] ? -1.0f : 1.0f;
                scaled_pct[i] = pct[i] * (e.config.intensity / 100.0f) * (e.config.axis_gain[i] / 100.0f) * inv;
            }

            // Dynamics apply S-curve transition: blend from old output to new
            if (e.dyn_transition_active) {
                float elapsed = (float)(frame_time - e.dyn_transition_start);
                float dur = e.dyn_transition_duration;
                if (elapsed >= dur) {
                    e.dyn_transition_active = false;
                } else {
                    float t = elapsed / dur;
                    t = t * t * (3.0f - 2.0f * t); // smoothstep
                    for (int i = 0; i < 6; i++)
                        scaled_pct[i] = e.dyn_transition_from[i] + (scaled_pct[i] - e.dyn_transition_from[i]) * t;
                }
            }
        } else {
            // HIL blocked: zero pipeline INPUT state only.
            // Do NOT zero output_angles / servo_util / max_util here —
            // those are owned by the telemetry update section and only
            // change when a new TEL line arrives.  Zeroing them every
            // frame causes flicker (zeroed on non-TEL frames, restored
            // on TEL frames → constant oscillation).
            memset(e.state.input_physical, 0, sizeof(e.state.input_physical));
        }
        memcpy(e.last_scaled_pct, scaled_pct, sizeof(scaled_pct));

        if (e.type == EntityType::SIL) {
            // ── SIL Pipeline: local IK computation ──
            float physical[6];
            for (int i = 0; i < 6; i++) {
                physical[i] = (scaled_pct[i] / 100.0f) * e.config.axis_scales.scale[i];
                if (e.config.axis_scales.is_angle[i]) {
                    physical[i] *= (float)(M_PI / 180.0);
                }
            }
            // IK X-axis = platform lateral, Y-axis = platform longitudinal
            { float tmp = physical[0]; physical[0] = physical[1]; physical[1] = tmp; }
            float angles[6];
            calcAllActuatorAngles(physical, &e.config.platform, angles);
            int valid_mask = validatePositionV2(physical, &e.config.platform);
            // Swap back so input_physical stays in app order for display
            { float tmp = physical[0]; physical[0] = physical[1]; physical[1] = tmp; }
            memcpy(e.state.input_physical, physical, sizeof(physical));
            memcpy(e.state.output_angles, angles, sizeof(angles));

            float max_util = 0.0f;
            float range = e.config.platform.servo_max_rad - e.config.platform.servo_min_rad;
            for (int i = 0; i < 6; i++) {
                e.state.output_angles_deg[i] = angles[i] * (180.0f / (float)M_PI);
                e.state.output_steps[i] = e.state.output_angles_deg[i] * e.config.platform.steps_per_degree;
                float util = fabsf(angles[i]) / (range * 0.5f) * 100.0f;
                if (util > 100.0f) util = 100.0f;
                e.state.servo_util[i] = util;
                if (util > max_util) max_util = util;
            }
            e.state.max_util = max_util;
            e.state.valid_mask = valid_mask;
            e.state.ik_seq++;
            e.rate_ik_hz = (float)fps;

        } else if (e.type == EntityType::HIL && e.hil_backend == HilBackend::MP6) {
            // ── MP6 pipeline: post-cue pose → session targets (see
            // mp6UpdatePipeline for the single mapping table). Device-side
            // lifecycle/TX belongs to the mp6 session engine, driven below.
            if (e.mp6.auto_connect && !e.mp6.link_open
                && !IsDocumentationMode()
                && e.mp6.link_port[0] != '\0'
                && frame_time - e.mp6.last_connect_attempt > 2.0) {
                // Auto-connect is a stored user choice and defaults OFF.
                // Failures are logged inside mp6Connect and retried every 2 s.
                std::string why;
                mp6Connect(e, &why);
            }
            mp6UpdatePipeline(*this, e, scaled_pct);
            e.rate_ik_hz = 0.0f;
            e.rate_tel_hz = 0.0f;
            mp6UpdateEntity(e);   // dispatch staged intents + tick + mirror
            mp6UpdateAxisTestView(e);
        } else if (e.type == EntityType::HIL) {
            // ── HIL auto-reconnect ──
            // Detect lost connection
            if (e.serial && !e.serial->isOpen()) {
                log(e.id, "hil", "Connection lost on %s", e.hil_port);
                e.serial->close();
                e.serial.reset();
                e.transport.usb_connected = false;
            }
            // Auto-reconnect: retry every 3 seconds if auto_connect is enabled.
            // Serial-only — the network transport is (re)built from the device
            // card's Network Connect button, not auto-enumerated here.
            if (!e.hil_network && !e.serial && e.hil_auto_connect
                && frame_time - e.hil_last_reconnect >= 3.0) {
                e.hil_last_reconnect = frame_time;

                // If no port configured, try to auto-detect first available
                if (e.hil_port[0] == '\0') {
                    auto ports = SerialPort::enumerate();
                    if (!ports.empty()) {
                        snprintf(e.hil_port, sizeof(e.hil_port), "%s", ports[0].port.c_str());
                        log(e.id, "hil", "Auto-detected port: %s", e.hil_port);
                    }
                }

                if (e.hil_port[0] != '\0') {
                    auto sp = std::make_shared<SerialPort>();
                    DEV_LOG("hil", "Opening serial port %s at %d baud", e.hil_port, e.hil_baud);
                    if (sp->open(e.hil_port, e.hil_baud)) {
                        sp->setCobsMode(true);
                        e.serial = sp;
                        e.transport.usb_connected = true;
                        snprintf(e.transport.usb_port, sizeof(e.transport.usb_port), "%s", e.hil_port);
                        e.hil_tel_seq = 0;
                        e.hil_handshake_ok = false;
                        e.cue_sync_valid = false;   // re-push cue settings after handshake
                        e.hil_handshake_pending = true;
                        e.hil_handshake_phase = HandshakePhase::WaitFingerprint;
                        e.hil_device_params.clear();
                        e.hil_handshake_start = frame_time;
                        e.hil_hs_attempts = 1;
                        e.hil_hs_last_send = frame_time;
                        snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                                 "Requesting fingerprint...");
                        log(e.id, "hil", "Connected to %s — handshaking...", e.hil_port);
                        DEV_LOG("hil", "Connected to %s, sending FINGERPRINT? immediately", e.hil_port);
                        // Flush decoder state with COBS delimiters so the next
                        // command frame starts cleanly.
                        sp->write((const uint8_t*)"\0\0\0\0", 4);
                        sp->sendCommand("FINGERPRINT?");
                    } else {
                        DEV_WARN("hil", "Failed to open %s", e.hil_port);
                        log(e.id, "hil", "Auto-connect failed: %s (retrying...)", e.hil_port);
                    }
                }
            }

            // ── Drain serial line queue on main thread (thread-safe) ──
            if (e.serial && e.serial->isOpen()) {
                auto lines = e.serial->drainLines();
                for (const auto& line : lines) {
                    DEV_LOG("serial", "[E%d] RX: %s", e.id, line.c_str());
                    handleHilLine(e.id, line.c_str());
                }
            }

            // ── Proof-of-life handshake ──────────────────────────────────
            // Instead of blindly sending FINGERPRINT? on connect, we wait
            // for the first TEL line (proof the ESP32 is alive and its
            // serial task is running). Then we send FINGERPRINT? and retry
            // every 1s until we get a response. This is robust against:
            //   - ESP32 boot delay (DTR/RTS reset or power cycle)
            //   - Stale ASCII buffer on ESP32 (binary packet residue)
            //   - OS serial buffer latency
            if (e.serial && e.serial->isOpen()
                && e.hil_handshake_phase == HandshakePhase::WaitFingerprint
                && e.hil_handshake_pending) {

                double since_last = frame_time - e.hil_hs_last_send;

                // Retry every 0.5s — first attempt was sent on connect
                bool should_try = since_last > 0.5;

                if (should_try && e.hil_hs_attempts < 10) {
                    e.hil_hs_attempts++;
                    e.hil_hs_last_send = frame_time;
                    e.serial->write((const uint8_t*)"\0\0\0\0", 4);
                    e.serial->sendCommand("FINGERPRINT?");
                    DEV_LOG("hil", "Sending FINGERPRINT? (attempt %d/10)", e.hil_hs_attempts);
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                             "Requesting fingerprint...%s",
                             e.hil_hs_attempts > 1 ? " (retrying)" : "");
                }

                // Give up after 10 attempts
                if (e.hil_hs_attempts >= 10 && since_last > 1.0) {
                    e.hil_handshake_phase = HandshakePhase::Failed;
                    e.hil_handshake_ok = false;
                    e.hil_handshake_pending = false;
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                             "No response after %d attempts", e.hil_hs_attempts);
                    snprintf(e.hil_last_error, sizeof(e.hil_last_error),
                             "No FINGERPRINT response after %d attempts.\nDevice may be offline, crashed, or wrong firmware.",
                             e.hil_hs_attempts);
                    log(e.id, "hil", "Handshake failed — no FINGERPRINT response after %d attempts",
                        e.hil_hs_attempts);
                }
            }

            // ── Retry for CONFIG? / BITS? / BITS:N confirm phases ───────
            // If the response is lost, resend with flush.
            if (e.serial && e.serial->isOpen() && e.hil_handshake_pending
                && (e.hil_handshake_phase == HandshakePhase::WaitConfig
                 || e.hil_handshake_phase == HandshakePhase::WaitBits
                 || e.hil_handshake_phase == HandshakePhase::WaitBitsConfirm)) {
                bool is_config  = (e.hil_handshake_phase == HandshakePhase::WaitConfig);
                bool is_confirm = (e.hil_handshake_phase == HandshakePhase::WaitBitsConfirm);
                char bits_set_cmd[32];
                snprintf(bits_set_cmd, sizeof(bits_set_cmd), "BITS:%d", e.config.bit_depth);
                const char* cmd = is_config ? "CONFIG?" : (is_confirm ? bits_set_cmd : "BITS?");
                double since_last = frame_time - e.hil_hs_last_send;
                if (since_last > 0.5 && e.hil_hs_attempts < 5) {
                    e.hil_hs_attempts++;
                    e.hil_hs_last_send = frame_time;
                    // Full flush to drain any residual COBS state
                    e.serial->write((const uint8_t*)"\0\0\0\0", 4);
                    e.serial->sendCommand(cmd);
                    DEV_LOG("hil", "Retrying %s (attempt %d/5)", cmd, e.hil_hs_attempts);
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                             "%s (retry %d)...",
                             is_config ? "Querying geometry" : "Querying bit depth",
                             e.hil_hs_attempts);
                }
                if (e.hil_hs_attempts >= 5 && since_last > 2.0) {
                    // Capture label BEFORE changing phase
                    char fail_msg[128];
                    snprintf(fail_msg, sizeof(fail_msg),
                             "No response to %s after 5 attempts", cmd);
                    e.hil_handshake_phase = HandshakePhase::Failed;
                    e.hil_handshake_ok = false;
                    e.hil_handshake_pending = false;
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "%s", fail_msg);
                    snprintf(e.hil_last_error, sizeof(e.hil_last_error), "%s", fail_msg);
                    log(e.id, "hil", "Handshake failed — %s", fail_msg);
                }
            }

            // ── HIL Pipeline: only active when ESP32 is connected ──
            // When offline, no work — no IK, no animation, no TX.
            if (e.serial && e.serial->isOpen()) {
                // Compute physical input for TX and history
                float physical[6];
                for (int i = 0; i < 6; i++) {
                    physical[i] = (scaled_pct[i] / 100.0f) * e.config.axis_scales.scale[i];
                    if (e.config.axis_scales.is_angle[i])
                        physical[i] *= (float)(M_PI / 180.0);
                }
                memcpy(e.state.input_physical, physical, sizeof(physical));

                // Drain command queue — one per frame, COBS framing handles interleaving
                if (!e.hil_cmd_queue.empty()) {
                    e.serial->sendCommand(e.hil_cmd_queue.front().c_str());
                    e.hil_cmd_queue.erase(e.hil_cmd_queue.begin());
                }

                // Prepare raw TX packet (only after handshake)
                if (e.hil_handshake_ok) {
                    float max_raw = txMaxRawForBitDepth(e.config.bit_depth);
                    float home = max_raw * 0.5f;
                    uint32_t raw[6];
                    for (int i = 0; i < 6; i++) {
                        float r = (scaled_pct[i] / 100.0f) * home + home;
                        if (r < 0.0f) r = 0.0f;
                        if (r > max_raw) r = max_raw;
                        raw[i] = (uint32_t)(r + 0.5f);
                    }
                    { uint32_t tmp = raw[0]; raw[0] = raw[1]; raw[1] = tmp; }
                    memcpy(e.hil_tx_raw, raw, sizeof(raw));

                    // RAW-HIL snapshot: pre-cueing input_pct in app axis order
                    // (surge=0, sway=1), NO swap — the ESP cues then swaps.
                    // Baked applies MCA/intensity/gain here; raw ships the input
                    // untouched so the device's single cue engine owns the feel.
                    memcpy(e.hil_tx_raw_f, e.state.input_pct, sizeof(e.hil_tx_raw_f));
                }

                // Cue auto-sync: any cue-param change (UI/API/preset) → device
                if (e.hil_handshake_ok) syncCueSettings(e);

                // Retry TELRATE if telemetry isn't flowing after handshake
                if (e.hil_handshake_ok && !e.hil_tel_active && e.hil_cmd_queue.empty()) {
                    double now_t = std::chrono::duration<double>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    if (now_t - e.hil_tel_curr_time > 2.0) {
                        e.hil_tel_curr_time = now_t;
                        char cmd[32];
                        snprintf(cmd, sizeof(cmd), "TELRATE:%d", e.hil_tel_target_hz);
                        e.hil_cmd_queue.push_back(cmd);
                    }
                }

                // Telemetry: ESP32 owns output angles when active
                ESP32Telemetry tel = e.serial->getLatestTelemetry();
                e.hil_tel_active = (tel.seq != 0 && e.rate_tel_hz > 0.0f);
                if (tel.seq != e.hil_tel_seq) {
                    e.hil_tel_seq = tel.seq;

                    // NOTE: Spike rejection removed — COBS framing (fail=0) structurally
                    // prevents the stray-transport corruption this was guarding against.
                    // The old 15° threshold was rejecting valid high-intensity telemetry.

                    memcpy(e.state.output_angles, tel.angles, sizeof(tel.angles));

                    float max_util = 0.0f;
                    float range = e.config.platform.servo_max_rad - e.config.platform.servo_min_rad;
                    for (int i = 0; i < 6; i++) {
                        e.state.output_angles_deg[i] = tel.angles[i] * (180.0f / (float)M_PI);
                        e.state.output_steps[i] = e.state.output_angles_deg[i] * e.config.platform.steps_per_degree;
                        float util = fabsf(tel.angles[i]) / (range * 0.5f) * 100.0f;
                        if (util > 100.0f) util = 100.0f;
                        e.state.servo_util[i] = util;
                        if (util > max_util) max_util = util;
                    }
                    e.state.max_util = max_util;
                    e.state.ik_seq++;
                }

                e.rate_ik_hz = (float)fps;
                e.rate_tx_hz = (float)e.hil_tx_hz;
                e.rate_tel_hz = e.serial->telemetryRate();
            } else {
                // Offline — zero everything
                e.rate_ik_hz = 0.0f;
                e.rate_tx_hz = 0.0f;
                e.rate_tel_hz = 0.0f;
                e.hil_tel_active = false;
            }
        }

        // Record history
        e.pushHistory(frame_time);

        // Per-entity DFT (for spectrogram, every ~100ms)
        {
            static double last_entity_dft[8] = {};
            int ei = (int)(&e - &entities[0]);
            if (ei < 8 && e.history_count >= 64 && frame_time - last_entity_dft[ei] >= 0.1) {
                last_entity_dft[ei] = frame_time;
                // Estimate sample rate from history timestamps
                int N = e.history_count;
                if (N > HISTORY_LEN) N = HISTORY_LEN;
                int newest = (e.history_head - 1 + HISTORY_LEN) % HISTORY_LEN;
                int oldest = (e.history_head - N + HISTORY_LEN) % HISTORY_LEN;
                double dt = (double)(e.history_time[newest] - e.history_time[oldest]);
                float sr = (dt > 0.01) ? (float)((N - 1) / dt) : 60.0f;
                e.spectrum_freq_max = sr * 0.5f;

                int start = oldest;
                for (int axis = 0; axis < 6; axis++) {
                    for (int k = 0; k < SPECTRUM_BINS; k++) {
                        double freq = (double)k / (double)SPECTRUM_BINS * (double)e.spectrum_freq_max;
                        double re = 0.0, im = 0.0;
                        for (int n = 0; n < N; n++) {
                            int si = (start + n) % HISTORY_LEN;
                            double angle = -2.0 * M_PI * freq * (double)n / (double)sr;
                            re += e.history_input[axis][si] * cos(angle);
                            im += e.history_input[axis][si] * sin(angle);
                        }
                        e.spectrum[axis][k] = (float)(sqrt(re * re + im * im) / (double)N);
                    }
                }
            }
        }
    }

    // Auto-save: compare settings against last-saved snapshot each frame
    {
        static int    s_lograte = -1, s_max = -1, s_rec_rate = -1;
        static int    s_input_source = -1;
        static bool   s_autoscroll = false;
        static float  s_intensity[8] = {};  // up to 8 entities
        static float  s_gain[8][6] = {};
        static bool   s_invert[8][6] = {};
        static int    s_bit_depth[8] = {};
        static int    s_entity_count = 0;
        static char   s_hil_port[8][32] = {};
        static int    s_hil_baud[8] = {};
        static int    s_hil_tx_hz[8] = {};
        static bool   s_hil_auto[8] = {};
        static bool   s_hil_net[8] = {};
        static char   s_hil_host[8][64] = {};
        static int    s_hil_udp[8] = {};
        static int    s_hil_tcp[8] = {};
        static bool   s_hil_raw[8] = {};
        static int    s_hil_backend[8] = {};
        static int    s_mp6_cadence[8] = {};
        static bool   s_mp6_auto[8] = {};
        static bool   s_mp6_onboard[8] = {};
        static int    s_mp6_pair[8] = {};
        static bool   s_mp6_cfg_present[8] = {};
        static uint32_t s_mp6_cfg_hash[8] = {};
        static bool   s_mp6_design_supplied[8] = {};
        static float  s_mp6_design[8][4] = {};
        static float  s_mp6_design_bot_a[8] = {};
        static float  s_mp6_design_bot_b[8] = {};
        static float  s_mp6_design_top_a[8] = {};
        static float  s_mp6_design_top_b[8] = {};
        static float  s_mp6_design_crank[8] = {};
        static float  s_mp6_design_rod[8] = {};
        static float  s_mp6_design_zoff[8] = {};
        static int    s_mp6_prov[8] = {};
        static bool   s_mp6_pz_sup[8] = {};
        static char   s_mp6_port[8][32] = {};
        static bool   s_mp6_cs_sup[8] = {};
        static int    s_dyn_id = -1;
        static bool   s_inited = false;

        auto mp6_cfg_hash = [](const Entity& e) -> uint32_t {
            uint32_t h = 2166136261u;
            const char* s = e.mp6.platform_config_text;
            size_t n = e.mp6.platform_config_present ? strlen(s) : 0u;
            for (size_t i = 0; i < n; ++i) {
                h ^= (unsigned char)s[i];
                h *= 16777619u;
            }
            return h;
        };

        auto snapshot_matches = [&]() -> bool {
            if (s_lograte != console_log_rate) return false;
            if (s_max != console_max) return false;
            if (s_autoscroll != console_auto_scroll) return false;
            if (s_rec_rate != record_rate_hz) return false;
            if (s_input_source != (int)input_source) return false;
            if (s_dyn_id != selected_dynamics_id) return false;
            if (s_entity_count != (int)entities.size()) return false;
            for (int ei = 0; ei < (int)entities.size() && ei < 8; ei++) {
                if (s_intensity[ei] != entities[ei].config.intensity) return false;
                if (s_bit_depth[ei] != entities[ei].config.bit_depth) return false;
                for (int a = 0; a < 6; a++) {
                    if (s_gain[ei][a] != entities[ei].config.axis_gain[a]) return false;
                    if (s_invert[ei][a] != entities[ei].config.axis_invert[a]) return false;
                }
                if (entities[ei].type == EntityType::HIL) {
                    if (strcmp(s_hil_port[ei], entities[ei].hil_port) != 0) return false;
                    if (s_hil_baud[ei] != entities[ei].hil_baud) return false;
                    if (s_hil_tx_hz[ei] != entities[ei].hil_tx_hz) return false;
                    if (s_hil_auto[ei] != entities[ei].hil_auto_connect) return false;
                    if (s_hil_net[ei] != entities[ei].hil_network) return false;
                    if (s_hil_backend[ei] != (int)entities[ei].hil_backend) return false;
                    if (strcmp(s_hil_host[ei], entities[ei].hil_host) != 0) return false;
                    if (s_hil_udp[ei] != entities[ei].hil_udp_port) return false;
                    if (s_hil_tcp[ei] != entities[ei].hil_tcp_port) return false;
                    if (s_hil_raw[ei] != entities[ei].hil_raw_mode) return false;
                    auto& m6 = entities[ei].mp6;
                    if (s_mp6_cadence[ei] != m6.cmd2_hz) return false;
                    if (s_mp6_auto[ei] != m6.auto_connect) return false;
                    if (s_mp6_onboard[ei] != m6.onboard_control) return false;
                    if (s_mp6_pair[ei] != m6.platform_number_expected) return false;
                    if (s_mp6_cfg_present[ei] != m6.platform_config_present) return false;
                    if (s_mp6_cfg_hash[ei] != mp6_cfg_hash(entities[ei])) return false;
                    if (s_mp6_design_supplied[ei] != m6.design.supplied) return false;
                    if (m6.design.bot_a_mm != s_mp6_design_bot_a[ei]) return false;
                    if (m6.design.bot_b_mm != s_mp6_design_bot_b[ei]) return false;
                    if (m6.design.top_a_mm != s_mp6_design_top_a[ei]) return false;
                    if (m6.design.top_b_mm != s_mp6_design_top_b[ei]) return false;
                    if (m6.design.crank_length_mm != s_mp6_design_crank[ei]) return false;
                    if (m6.design.rod_length_mm != s_mp6_design_rod[ei]) return false;
                    if (m6.design.z_off_mm != s_mp6_design_zoff[ei]) return false;
                    if (m6.design.pivot_z_provenance != s_mp6_prov[ei]) return false;
                    if (m6.design.point_zero_supplied != s_mp6_pz_sup[ei]) return false;
                    if (strcmp(m6.link_port, s_mp6_port[ei]) != 0) return false;
                    if (m6.controller_settings.supplied != s_mp6_cs_sup[ei]) return false;
                }
            }
            return true;
        };

        auto take_snapshot = [&]() {
            s_lograte = console_log_rate;
            s_max = console_max;
            s_autoscroll = console_auto_scroll;
            s_rec_rate = record_rate_hz;
            s_input_source = (int)input_source;
            s_dyn_id = selected_dynamics_id;
            s_entity_count = (int)entities.size();
            for (int ei = 0; ei < (int)entities.size() && ei < 8; ei++) {
                s_intensity[ei] = entities[ei].config.intensity;
                s_bit_depth[ei] = entities[ei].config.bit_depth;
                for (int a = 0; a < 6; a++) {
                    s_gain[ei][a] = entities[ei].config.axis_gain[a];
                    s_invert[ei][a] = entities[ei].config.axis_invert[a];
                }
                if (entities[ei].type == EntityType::HIL) {
                    snprintf(s_hil_port[ei], sizeof(s_hil_port[ei]), "%s", entities[ei].hil_port);
                    s_hil_baud[ei] = entities[ei].hil_baud;
                    s_hil_tx_hz[ei] = entities[ei].hil_tx_hz;
                    s_hil_auto[ei] = entities[ei].hil_auto_connect;
                    s_hil_net[ei] = entities[ei].hil_network;
                    s_hil_backend[ei] = (int)entities[ei].hil_backend;
                    snprintf(s_hil_host[ei], sizeof(s_hil_host[ei]), "%s", entities[ei].hil_host);
                    s_hil_udp[ei] = entities[ei].hil_udp_port;
                    s_hil_tcp[ei] = entities[ei].hil_tcp_port;
                    s_hil_raw[ei] = entities[ei].hil_raw_mode;
                    auto& m6 = entities[ei].mp6;
                    s_mp6_cadence[ei] = m6.cmd2_hz;
                    s_mp6_auto[ei] = m6.auto_connect;
                    s_mp6_onboard[ei] = m6.onboard_control;
                    s_mp6_pair[ei] = m6.platform_number_expected;
                    s_mp6_cfg_present[ei] = m6.platform_config_present;
                    s_mp6_cfg_hash[ei] = mp6_cfg_hash(entities[ei]);
                    s_mp6_design_supplied[ei] = m6.design.supplied;
                    s_mp6_design[ei][0] = m6.design.base_radius_mm;
                    s_mp6_design[ei][1] = m6.design.platform_radius_mm;
                    s_mp6_design[ei][2] = m6.design.x_offset_mm;
                    s_mp6_design_bot_a[ei] = m6.design.bot_a_mm;
                    s_mp6_design_bot_b[ei] = m6.design.bot_b_mm;
                    s_mp6_design_top_a[ei] = m6.design.top_a_mm;
                    s_mp6_design_top_b[ei] = m6.design.top_b_mm;
                    s_mp6_design_crank[ei] = m6.design.crank_length_mm;
                    s_mp6_design_rod[ei] = m6.design.rod_length_mm;
                    s_mp6_design_zoff[ei] = m6.design.z_off_mm;
                    s_mp6_prov[ei] = m6.design.pivot_z_provenance;
                    s_mp6_pz_sup[ei] = m6.design.point_zero_supplied;
                    snprintf(s_mp6_port[ei], sizeof(s_mp6_port[ei]), "%s", m6.link_port);
                    s_mp6_cs_sup[ei] = m6.controller_settings.supplied;
                }
            }
        };

        if (!s_inited) { take_snapshot(); s_inited = true; }
        else if (!snapshot_matches()) {
            take_snapshot();
            saveSettings();
        }
    }
}

void Entity::pushHistory(double t) {
    int idx = history_head;
    history_time[idx] = (float)t;
    for (int i = 0; i < 6; i++) {
        if (type == EntityType::HIL && hil_backend == HilBackend::MP6) {
            // MP6 "angles" channel: calibrated feedback steps when a fresh
            // value was observed (NaN = gap, implot skips it — no fabricated
            // zeros). history_input stays the modelled physical pose.
            history_angles[i][idx] =
                (mp6.feedback_valid && mp6.feedback_cal[i] != SHRT_MIN)
                    ? (float)mp6.feedback_cal[i] : std::nanf("");
        } else {
            history_angles[i][idx] = state.output_angles_deg[i];
        }
        history_input[i][idx] = state.input_physical[i];
    }
    history_head = (history_head + 1) % HISTORY_LEN;
    if (history_count < HISTORY_LEN) history_count++;
}
