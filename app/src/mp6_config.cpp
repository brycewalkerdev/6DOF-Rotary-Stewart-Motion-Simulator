// mp6_config.cpp — implementation of the PlatformConfig.pltc import and the
// Load_Coefficients calibration port. See mp6_config.h for evidence pointers.

#include "mp6_config.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace mp6 {

// ---------------------------------------------------------------------------
// .NET rounding parity
// ---------------------------------------------------------------------------

double mp6_net_round(double v) {
    // Math.Round(double): banker's rounding (nearest, ties to even).
    const double f = std::floor(v);
    const double diff = v - f;
    if (diff > 0.5) {
        return f + 1.0;
    }
    if (diff < 0.5) {
        return f;
    }
    // Exactly .5: choose the even neighbor. Handle negatives via parity of f.
    const double even = (std::fmod(f, 2.0) == 0.0) ? f : f + 1.0;
    // -0.0 must stay negative zero observable? Math.Round(-0.5) == -0.0 in
    // .NET; value equality is what callers rely on.
    return even;
}

float mp6_net_roundf(float v) {
    return static_cast<float>(mp6_net_round(static_cast<double>(v)));
}

int mp6_normalize_step_rev(double value) { return static_cast<int>(mp6_net_round(value)); }
int mp6_normalize_pot_lo(double value) {
    return static_cast<int>(mp6_net_round(static_cast<double>(static_cast<float>(value))));
}

// ---------------------------------------------------------------------------
// Parsing helpers
// ---------------------------------------------------------------------------

namespace {

std::vector<std::string> split_tabs(const std::string& line) {
    std::vector<std::string> fields;
    std::string cur;
    for (char c : line) {
        if (c == '\t') {
            fields.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    fields.push_back(cur);
    return fields;
}

// VB Conversion.Val-style value names in the snapshot are plain numerics;
// we parse strictly and report anything unparsable instead of silently
// substituting zeros (the recovered Conversion.Val would throw into the
// form's On Error trap and lose the field).
bool parse_number(const std::string& text, double* out) {
    if (text.empty()) {
        return false;
    }
    const char* begin = text.c_str();
    char* end = nullptr;
    const double v = std::strtod(begin, &end);
    if (end == begin) {
        return false;
    }
    // tolerate trailing whitespace only
    while (*end != '\0') {
        if (!std::isspace(static_cast<unsigned char>(*end))) {
            return false;
        }
        ++end;
    }
    if (!std::isfinite(v)) {
        return false;
    }
    *out = v;
    return true;
}

Scalar read_column(const ConfigRow& row, int column,
                   std::vector<ConfigDiagnostic>* diags) {
    Scalar s;
    if (column < 0 || column >= static_cast<int>(row.value.size())) {
        if (diags) {
            diags->push_back({ConfigDiagnostic::Severity::kWarning, row.header,
                              "column " + std::to_string(column) + " not present in line"});
        }
        return s;
    }
    const std::string& raw = row.value[static_cast<size_t>(column)];
    double v = 0.0;
    if (raw.empty()) {
        if (diags) {
            diags->push_back({ConfigDiagnostic::Severity::kWarning, row.header,
                              "column " + std::to_string(column) + " empty"});
        }
        return s;
    }
    if (!parse_number(raw, &v)) {
        if (diags) {
            diags->push_back({ConfigDiagnostic::Severity::kError,
                              row.header + "[" + std::to_string(column) + "]",
                              "unparsable numeric \"" + raw + "\""});
        }
        return s;
    }
    s.value = v;
    s.present = true;
    s.raw = raw;
    return s;
}

void require(const Scalar& s, const char* row, int column,
             std::vector<ConfigDiagnostic>* diags) {
    if (!s.present && diags) {
        diags->push_back({ConfigDiagnostic::Severity::kError,
                          std::string(row) + (column >= 0
                                                  ? "[" + std::to_string(column) + "]"
                                                  : ""),
                          "required field missing or unparsable"});
    }
}

const ConfigRow* find_row(const Mp6PlatformConfig& cfg, const std::string& header) {
    for (const ConfigRow& r : cfg.rows) {
        if (r.header == header) {
            return &r;
        }
    }
    return nullptr;
}

void read_pid(const Mp6PlatformConfig& cfg, int n, Mp6PidRow* out,
              std::vector<ConfigDiagnostic>* diags) {
    const ConfigRow* row = find_row(cfg, "PID " + std::to_string(n));
    if (row == nullptr) {
        if (diags) {
            diags->push_back({ConfigDiagnostic::Severity::kError, "PID " + std::to_string(n),
                              "row missing"});
        }
        return;
    }
    const Scalar fields[] = {read_column(*row, 0, diags), read_column(*row, 1, diags),
                             read_column(*row, 2, diags), read_column(*row, 3, diags),
                             read_column(*row, 4, diags)};
    if (diags) {
        for (int column = 0; column < 5; ++column) {
            if (!fields[column].present) {
                diags->push_back({ConfigDiagnostic::Severity::kError,
                                  row->header + "[" + std::to_string(column) + "]",
                                  "required PID value missing or invalid"});
            }
        }
    }
    out->p = fields[0].value;
    out->d = fields[1].value;
    out->integral = fields[2].value;
    out->dfilter = fields[3].value;
    out->tconst = fields[4].value;
    out->resolved = std::all_of(std::begin(fields), std::end(fields),
                                [](const Scalar& field) { return field.present; });
}

} // namespace

// ---------------------------------------------------------------------------
// from_pltc
// ---------------------------------------------------------------------------

bool Mp6PlatformConfig::from_pltc(const std::string& text, Mp6PlatformConfig& out) {
    out = Mp6PlatformConfig{};

    size_t pos = 0;
    bool any_header = false;
    while (pos <= text.size()) {
        size_t eol = text.find_first_of("\r\n", pos);
        const bool at_end = (eol == std::string::npos);
        std::string line = at_end ? text.substr(pos) : text.substr(pos, eol - pos);
        pos = at_end ? text.size() + 1 : eol + 1;

        // skip blank-only lines
        bool blank = true;
        for (char c : line) {
            if (c != '\t' && !std::isspace(static_cast<unsigned char>(c))) {
                blank = false;
                break;
            }
        }
        if (blank) {
            continue;
        }

        const std::vector<std::string> f = split_tabs(line);
        ConfigRow row;
        row.header = f[0];
        if (f.size() > 1) {
            row.tooltip = f[1];
        }
        // Alternating value/label pairs from field index 2 (DGV column k).
        int k = 0;
        for (size_t i = 2; i + 1 < f.size() || i < f.size(); i += 2, ++k) {
            if (i >= f.size()) {
                break;
            }
            row.value.push_back(f[i]);
            // The next field is this value's in-file label, if present.
            if (i + 1 < f.size()) {
                row.in_label.push_back(f[i + 1]);
            } else {
                row.in_label.push_back(std::string());
                break;
            }
        }
        // Dangling label after the last value (e.g. "Gear Ratio", "TConst").
        // The loop above already skipped labels interleaved with values; a
        // trailing field with no value slot after it is dropped from `value`
        // and recorded: with pairs value,label... a trailing label arises when
        // the last pair's label is the final field (already captured in
        // in_label). Rows whose last field is a value keep it here. Nothing
        // further to store: recovered DGV logic also discards it.
        while (static_cast<int>(row.value.size()) < kPltcColumns) {
            row.value.push_back(std::string());
            row.in_label.push_back(std::string());
        }
        for (const std::string& unl : row.in_label) {
            (void)unl;
        }
        row.trailing.clear();

        // If the line ended with an odd number of fields after the header
        // pair, the final field is a value without a following label; it is
        // still a column per the DGV trace only if it lands on a value slot.
        // (The recovered writer always emits value,label pairs or a dangling
        // label, both covered above.)
        (void)k;
        out.rows.push_back(row);
        any_header = true;
    }

    if (!any_header) {
        out.diagnostics.push_back(
            {ConfigDiagnostic::Severity::kError, "file", "no parseable rows"});
        return false;
    }

    std::vector<ConfigDiagnostic>* diags = &out.diagnostics;

    // --- scalar rows -------------------------------------------------------
    if (const ConfigRow* r = find_row(out, "X Offset")) out.x_offset_mm = read_column(*r, 0, diags);
    if (const ConfigRow* r = find_row(out, "Z Offset")) out.z_offset_row_mm = read_column(*r, 0, diags);
    if (const ConfigRow* r = find_row(out, "Angle Offset")) out.angle_offset_deg = read_column(*r, 0, diags);
    if (const ConfigRow* r = find_row(out, "Crank Length")) out.crank_length_mm = read_column(*r, 0, diags);
    if (const ConfigRow* r = find_row(out, "Rod Length")) out.rod_length_mm = read_column(*r, 0, diags);
    if (const ConfigRow* r = find_row(out, "Centre Strut")) {
        out.centre_strut_dia_mm = read_column(*r, 0, diags);
        out.helper_struts = read_column(*r, 1, diags);
    }
    if (const ConfigRow* r = find_row(out, "Payload Weight"))
        out.payload_kg = read_column(*r, 0, diags);
    if (const ConfigRow* r = find_row(out, "Step/Rev")) out.step_rev = read_column(*r, 0, diags);
    if (const ConfigRow* r = find_row(out, "Rotate Direction"))
        out.rotate_direction = read_column(*r, 0, diags);
    if (const ConfigRow* r = find_row(out, "Update Rate")) out.update_rate = read_column(*r, 0, diags);
    if (const ConfigRow* r = find_row(out, "Max Motor RPM"))
        out.max_motor_rpm = read_column(*r, 0, diags);
    if (const ConfigRow* r = find_row(out, "Max Travel (rad)")) {
        out.travel_min_rad = read_column(*r, 0, diags);
        out.travel_max_rad = read_column(*r, 1, diags);
    }
    if (const ConfigRow* r = find_row(out, "Accel/Decel/Deadband")) {
        out.accel_scale = read_column(*r, 0, diags);
        out.accel_p1 = read_column(*r, 1, diags);
        out.accel_p0 = read_column(*r, 2, diags);
        out.decel_scale = read_column(*r, 3, diags);
        out.deadband_pid = read_column(*r, 4, diags);
        out.stopcount_limit = read_column(*r, 5, diags);
    }
    if (const ConfigRow* r = find_row(out, "Step Correction")) {
        for (int k = 0; k < 4; ++k) out.step_err_corr[k] = read_column(*r, k, diags);
    }
    if (const ConfigRow* r = find_row(out, "Bump Delay")) {
        out.bump_fr_delay = read_column(*r, 0, diags);
        out.bump_scale = read_column(*r, 1, diags);
    }
    for (int n = 0; n < kPidRows; ++n) {
        read_pid(out, n, &out.pid[n], diags);
    }
    for (int i = 0; i < kAxes; ++i) {
        const ConfigRow* r = find_row(out, "Pot " + std::to_string(i + 1));
        if (r == nullptr) {
            diags->push_back({ConfigDiagnostic::Severity::kError, "Pot " + std::to_string(i + 1),
                              "row missing"});
            continue;
        }
        Mp6AxisPots& a = out.axis[i];
        const Scalar fields[] = {read_column(*r, 0, diags), read_column(*r, 1, diags),
                                 read_column(*r, 2, diags), read_column(*r, 3, diags),
                                 read_column(*r, 4, diags), read_column(*r, 5, diags)};
        if (diags) {
            for (int column : {0, 1, 2, 5}) {
                if (!fields[column].present) {
                    diags->push_back({ConfigDiagnostic::Severity::kError,
                                      r->header + "[" + std::to_string(column) + "]",
                                      "required calibration value missing or invalid"});
                }
            }
        }
        a.pot_lo = fields[0].value;
        a.angle_docked_deg = fields[1].value;
        a.pot_scale = fields[2].value;
        a.mswitch_on = fields[3].value;
        a.mswitch_off = fields[4].value;
        a.gearbox = fields[5].value;
        for (int k = 0; k < 6; ++k) {
            a.present[k] = fields[k].present;
        }
        // Manifest note: the in-file label chain is decorative ("Pot - Docked"
        // precedes the PotLo value); the source reads by column index
        // (Load_Coefficients L18649, L18667..L18675).
        for (int k = 0; k < static_cast<int>(r->in_label.size()); ++k) {
            (void)k;
        }
    }

    // --- declared-but-unconsumed notes -------------------------------------
    if (out.z_offset_row_mm.present) {
        diags->push_back(
            {ConfigDiagnostic::Severity::kInfo, "Z Offset",
             "row present in snapshot but no consumer found in recovered source "
             "(Module2.ZOff comes from the CalcPlatformGeom_1 design table; "
             "Z_Offset = ZOff - sin(AngOffset)*CrankLength - 20 + lblZ)"});
    }
    if (out.update_rate.present) {
        diags->push_back(
            {ConfigDiagnostic::Severity::kInfo, "Update Rate",
             "row declared but Load_Coefficients hardcodes Control_Update_Target = 1/60; "
             "treat 60 Hz as the only evidenced cadence"});
    }
    if (out.bump_scale.present) {
        diags->push_back({ConfigDiagnostic::Severity::kInfo, "Bump Delay",
                          "in-file \"Wheelbase\" label has no value slot in snapshot"});
    }
    diags->push_back(
        {ConfigDiagnostic::Severity::kInfo, "Pot n",
         "col5 value (e.g. 20) is consumed as the per-axis Gearbox; the trailing "
         "in-file label \"Gear Ratio\" has no value slot and the preceding label "
         "\"MSwitch Off\" is unused by the recovered consumers"});
    diags->push_back(
        {ConfigDiagnostic::Severity::kInfo, "MotConv",
         "\"X/Y/Z Point Zero\" sliders (lblX/lblY/lblZ) are not pinned to a "
         "MotConv txt field by the handoff; the config keeps them unresolved "
         "and they must be confirmed (recovered defaults 0) before live send"});

    return true;
}

// ---------------------------------------------------------------------------
// Manifest
// ---------------------------------------------------------------------------

std::vector<FieldRef> mp6_config_manifest() {
    std::vector<FieldRef> m;
    auto add = [&m](const char* row, int col, const char* key, FieldStatus st,
                    const char* ev) { m.push_back({row, col, key, st, ev}); };

    add("X Offset", 0, "x_offset_mm", FieldStatus::kConsumed,
        "Load_Component -> Module2.X_Offset (frm_Main.cs L16609)");
    add("Z Offset", 0, "z_offset_row_mm", FieldStatus::kDeclaredUnconsumed,
        "no DGVData reader in recovered source; design ZOff via CalcPlatformGeom_1");
    add("Angle Offset", 0, "angle_offset_deg", FieldStatus::kConsumed,
        "Module2.AngOffset (L18654), used in Z_Offset formula (L16610)");
    add("Crank Length", 0, "crank_length_mm", FieldStatus::kConsumed,
        "Module2.CrankLength (L18655), MotorPos tables/limit math");
    add("Rod Length", 0, "rod_length_mm", FieldStatus::kConsumed,
        "Module2.RodLength[1..6] (L18656-18663)");
    add("Centre Strut", 0, "centre_strut_dia_mm", FieldStatus::kConsumed,
        "Module2.StrutDiaInch = col0/25.4 (L18635)");
    add("Centre Strut", 1, "helper_struts", FieldStatus::kConsumed,
        "Module2.HelperStruts (L18636)");
    add("Payload Weight", 0, "payload_kg", FieldStatus::kConsumed,
        "Module2.PayloadWeight (L18637)");
    add("Step/Rev", 0, "step_rev", FieldStatus::kConsumed,
        "Module2.StepRev (L18640)");
    add("Rotate Direction", 0, "rotate_direction", FieldStatus::kConsumed,
        "cmd-5 param 36 (L31884); TDir comparisons (L22882, L27106)");
    add("Update Rate", 0, "update_rate", FieldStatus::kDeclaredUnconsumed,
        "source hardcodes Control_Update_Target = 1/60 (L18638)");
    add("Max Motor RPM", 0, "max_motor_rpm", FieldStatus::kConsumed,
        "Module2.MaxRPM (L18643), MaxPPS = round(RPM*StepRev/60) (L18644)");
    add("Max Travel (rad)", 0, "travel_min_rad", FieldStatus::kConsumed,
        "Module2.MinRadians (L18641), MotorPos limit math (L18393)");
    add("Max Travel (rad)", 1, "travel_max_rad", FieldStatus::kConsumed,
        "Module2.MaxRadians (L18642), MotorPos limit math (L18389)");
    add("Accel/Decel/Deadband", 0, "accel_scale", FieldStatus::kConsumed,
        "AccelScale (L18619); cmd-5 param 40 chain (L31722)");
    add("Accel/Decel/Deadband", 1, "accel_p1", FieldStatus::kConsumed,
        "txtAccelP1 (L18621); in-file label \"Accel Factor\" precedes value");
    add("Accel/Decel/Deadband", 2, "accel_p0", FieldStatus::kConsumed,
        "txtAccelP0 (L18622); in-file label \"Accel P1\" precedes value");
    add("Accel/Decel/Deadband", 3, "decel_scale", FieldStatus::kConsumed,
        "Module2.DecelScale (L18623); cmd-5 param 42 = round(x*100) (L31726)");
    add("Accel/Decel/Deadband", 4, "deadband_pid", FieldStatus::kConsumed,
        "Module2.DeadbandPID (L18624); cmd-5 param 28 (L31863)");
    add("Accel/Decel/Deadband", 5, "stopcount_limit", FieldStatus::kConsumed,
        "Module2.StoppedCountLimit (L18625); cmd-5 param 29 (L31865)");
    add("Step Correction", 0, "step_err_corr[0]", FieldStatus::kConsumed,
        "StepErrCorrArray[0] (L18629); cmd-5 param 39 = round(x*1e4) (L31844)");
    add("Step Correction", 1, "step_err_corr[1]", FieldStatus::kConsumed,
        "StepErrCorrArray[1]");
    add("Step Correction", 2, "step_err_corr[2]", FieldStatus::kConsumed,
        "StepErrCorrArray[2]");
    add("Step Correction", 3, "step_err_corr[3]", FieldStatus::kConsumed,
        "StepErrCorrArray[3]");
    add("Bump Delay", 0, "bump_fr_delay", FieldStatus::kConsumed,
        "BumpDelayScale (L18633)");
    add("Bump Delay", 1, "bump_scale", FieldStatus::kConsumed,
        "BumpDelayWheelbase consumer name; in-file label \"Scale\" dangling");
    for (int n = 0; n < kPidRows; ++n) {
        std::string rowname = "PID " + std::to_string(n);
        add(rowname.c_str(), 0, "pid[n].p", FieldStatus::kConsumed,
            "PID_SetValues Proportional_Aim (L31573)");
        add(rowname.c_str(), 1, "pid[n].d", FieldStatus::kConsumed,
            "PID_SetValues Derivative_Aim (L31581)");
        add(rowname.c_str(), 2, "pid[n].integral", FieldStatus::kConsumed,
            "PID_SetValues Integral_Term (L31587)");
        add(rowname.c_str(), 3, "pid[n].dfilter", FieldStatus::kConsumed,
            "Module2.DFilter (L31588); cmd-5 param 25 (L31852)");
        add(rowname.c_str(), 4, "pid[n].tconst", FieldStatus::kConsumed,
            "Module2.TConst (L31589); cmd-5 param 26 (L31854)");
    }
    for (int i = 1; i <= kAxes; ++i) {
        const std::string rowname = "Pot " + std::to_string(i);
        add(rowname.c_str(), 0, "axis[i-1].pot_lo", FieldStatus::kConsumed,
            "Module2.PotLo = DGVData(\"Pot n\", 0) (L18667..18671); cmd-5 "
            "params 101+ (L31758); in-file label \"Pot - Docked\" is decorative");
        add(rowname.c_str(), 1, "axis[i-1].angle_docked_deg", FieldStatus::kConsumed,
            "Module2.AngLo = (n%2)*2-1 * DGVData(\"Pot n\", 1) (L18668, "
            "L18670..18672); \"Angle - Docked\" degrees (snapshot: 55); "
            "cmd-5 params 111+ (L31776)");
        add(rowname.c_str(), 2, "axis[i-1].pot_scale", FieldStatus::kConsumed,
            "pScale = |StepRev*Gearbox/(num15*360)|, num15 = DGVData(\"Pot n\", "
            "2) (L18669, L18675); \"Pot Scale\" (snapshot: +/-12); "
            "cmd-5 params 121+ (L31794)");
        add(rowname.c_str(), 3, "axis[i-1].mswitch_on", FieldStatus::kDeclaredUnconsumed,
            "in-file label \"MSwitch On\"; no DGVData consumer found in this pass");
        add(rowname.c_str(), 4, "axis[i-1].mswitch_off", FieldStatus::kDeclaredUnconsumed,
            "in-file label \"MSwitch Off\"; no DGVData consumer found in this pass");
        add(rowname.c_str(), 5, "axis[i-1].gearbox", FieldStatus::kConsumed,
            "Module2.Gearbox = DGVData(\"Pot n\", 5) (L18649); value 20 under "
            "the dangling \"Gear Ratio\" label");
    }
    m.push_back({"(runtime)", -1, "onboard_control", FieldStatus::kUnresolved,
                 "recovered app setting, not in pltc; explicit integrator decision"});
    m.push_back({"(MotConv)", -1, "point_zero_x/y/z", FieldStatus::kUnresolved,
                 "frm_MotConv \"X/Y/Z Point Zero\" sliders -> lblX/lblY/lblZ; "
                 "txt field encoding unresolved in handoff"});
    return m;
}

// ---------------------------------------------------------------------------
// Calibration derivation (Load_Coefficients L18646..18680)
// ---------------------------------------------------------------------------

bool mp6_config_derive_calibration(const Mp6PlatformConfig& cfg,
                                   Mp6AxisCalibration out[kAxes],
                                   std::string* err) {
    std::string local_err;
    if (err == nullptr) {
        err = &local_err;
    }
    err->clear();

    if (!cfg.step_rev.present || cfg.step_rev.value <= 0.0) {
        *err = "Step/Rev missing or not positive";
        return false;
    }
    const int step_rev = mp6_normalize_step_rev(cfg.step_rev.value);

    for (int i = 0; i < kAxes; ++i) {
        const Mp6AxisPots& a = cfg.axis[i];
        const std::string name = "Pot " + std::to_string(i + 1);
        const int one_based = i + 1;
        if (!a.present[0] || !a.present[1] || !a.present[2] || !a.present[5]) {
            *err = name + ": required columns (PotLo, Angle-Docked, PotScale, "
                        "Gearbox) missing";
            return false;
        }
        // float semantics of the recovered statics
        const int pot_lo = mp6_normalize_pot_lo(a.pot_lo);
        const float angle_docked =
            static_cast<float>(a.angle_docked_deg);
        const float pot_scale_raw = static_cast<float>(a.pot_scale);
        const float gearbox = static_cast<float>(a.gearbox);

        Mp6AxisCalibration& c = out[i];
        c.axis_sign = static_cast<double>(one_based % 2 * 2 - 1); // (n%2)*2-1

        const float ang_lo = static_cast<float>(c.axis_sign) * angle_docked;
        c.ang_lo_deg = static_cast<double>(ang_lo);
        c.p_scale = std::fabs(static_cast<float>(step_rev) * gearbox /
                              (pot_scale_raw * 360.0f));
        c.p_offset = (0.0f - c.p_scale) * static_cast<float>(pot_lo) +
                     ang_lo * gearbox * static_cast<float>(step_rev) / 360.0f;
        // float product+sum, the way lblR displays it (L18679: PotLo*pScale
        // + pOffset in float); double promotion would drift ~1.3e-3 (N2).
        c.docked_steps = static_cast<double>(
            static_cast<float>(pot_lo) * c.p_scale + c.p_offset);
        c.angle_to_step = 180.0 / M_PI * static_cast<double>(gearbox) *
                          static_cast<double>(step_rev) / 360.0;
        c.rod_angle_zero_rad = 0.0; // frm_Main.L9238: RodAngle_Zero = new double[7]
    }
    return true;
}

int mp6_feedback_calibrated_steps(const Mp6AxisCalibration& cal, double word) {
    // (int)Math.Round((float)num13 * pScale / 8f + pOffset)  (L32233)
    const float scaled =
        static_cast<float>(word) * cal.p_scale / 8.0f + cal.p_offset;
    return static_cast<int>(mp6_net_roundf(scaled));
}

// ---------------------------------------------------------------------------
// Readiness
// ---------------------------------------------------------------------------

namespace {

void require_scalar(const Scalar& s, const char* where, const char* what,
                    std::vector<ConfigDiagnostic>* issues) {
    if (!s.present) {
        if (issues) {
            issues->push_back({ConfigDiagnostic::Severity::kError, where, what});
        }
        return;
    }
    if (!std::isfinite(s.value)) {
        if (issues) {
            issues->push_back({ConfigDiagnostic::Severity::kError, where,
                               std::string(what) + " (value not finite)"});
        }
    }
}

} // namespace

bool mp6_ready_for_targets(const Mp6PlatformConfig& cfg,
                           std::vector<ConfigDiagnostic>* issues,
                           PivotProvenance pivot_z_provenance) {
    std::vector<ConfigDiagnostic> local;
    std::vector<ConfigDiagnostic>* d = issues ? issues : &local;
    d->clear();

    require_scalar(cfg.crank_length_mm, "Crank Length", "required", d);
    require_scalar(cfg.rod_length_mm, "Rod Length", "required", d);
    require_scalar(cfg.step_rev, "Step/Rev", "required", d);
    require_scalar(cfg.travel_min_rad, "Max Travel (rad)", "min travel required", d);
    require_scalar(cfg.travel_max_rad, "Max Travel (rad)", "max travel required", d);
    require_scalar(cfg.max_motor_rpm, "Max Motor RPM", "required (MaxPPS derivation)", d);
    require_scalar(cfg.angle_offset_deg, "Angle Offset", "required (Z_Offset derivation)", d);
    require_scalar(cfg.x_offset_mm, "X Offset", "required (X_Offset derivation)", d);

    if (cfg.travel_min_rad.present && cfg.travel_max_rad.present &&
        !(cfg.travel_min_rad.value < 0.0 && cfg.travel_max_rad.value > 0.0 &&
          cfg.travel_min_rad.value < cfg.travel_max_rad.value)) {
        d->push_back({ConfigDiagnostic::Severity::kError, "Max Travel (rad)",
                      "limits must straddle zero with min < max"});
    }
    if (cfg.crank_length_mm.present && cfg.rod_length_mm.present &&
        !(cfg.rod_length_mm.value > cfg.crank_length_mm.value &&
          cfg.crank_length_mm.value > 0.0)) {
        d->push_back({ConfigDiagnostic::Severity::kError, "Crank Length",
                      "must have 0 < crank < rod"});
    }
    if (!cfg.point_zero_resolved) {
        d->push_back({ConfigDiagnostic::Severity::kError, "MotConv",
                      "\"X/Y/Z Point Zero\" offsets unresolved; confirm 0 or set "
                      "resolved values before computing X_Offset/Z_Offset"});
    }

    // Base/motor pivot Z and platform attachment Z come from
    // Other\<Platform>.txt in the original app, which is not in the handoff.
    // The geometry API surfaces them as explicit inputs, so readiness here
    // cannot verify the values; the caller states provenance explicitly.
    // A finite fixture/placeholder value must never authorize targets.
    switch (pivot_z_provenance) {
    case PivotProvenance::kReviewed:
        d->push_back(
            {ConfigDiagnostic::Severity::kInfo, "Geometry",
             "pivot/attachment z provenance marked reviewed: real "
             "Other\\<Platform>.txt values supplied and operator-reviewed"});
        break;
    case PivotProvenance::kFixtureOnly:
        d->push_back(
            {ConfigDiagnostic::Severity::kWarning, "Geometry",
             "pivot/attachment z are OFFLINE TEST FIXTURE placeholders "
             "(-30/+10), never valid for a real rig"});
        break;
    case PivotProvenance::kUnreviewed:
    default:
        d->push_back(
            {ConfigDiagnostic::Severity::kError, "Geometry",
             "pivot/attachment z provenance unreviewed: real "
             "Other\\<Platform>.txt values were never supplied/reviewed "
             "(pass PivotProvenance::kReviewed only after a UI-reviewed "
             "import); offline tests may use kFixtureOnly"});
        break;
    }

    for (int i = 0; i < kAxes; ++i) {
        const Mp6AxisPots& a = cfg.axis[i];
        const std::string n = "Pot " + std::to_string(i + 1);
        if (!a.present[0] || !a.present[1] || !a.present[2] || !a.present[5]) {
            d->push_back({ConfigDiagnostic::Severity::kError, n,
                          "requires columns 0 (PotLo), 1 (Angle-Docked), "
                          "2 (PotScale), 5 (Gearbox)"});
            continue;
        }
        if (!(a.gearbox > 0.0)) {
            d->push_back({ConfigDiagnostic::Severity::kError, n, "gearbox must be > 0"});
        }
        if (!(a.pot_scale != 0.0)) {
            d->push_back({ConfigDiagnostic::Severity::kError, n, "pot scale must be != 0"});
        }
        if (!(std::fabs(a.angle_docked_deg) < 90.0)) {
            d->push_back({ConfigDiagnostic::Severity::kError, n,
                          "docked angle implausible (|deg| >= 90)"});
        }
        if (!a.present[3] || !a.present[4]) {
            d->push_back({ConfigDiagnostic::Severity::kWarning, n,
                          "MSwitch On/Off columns empty (display-only in the "
                          "recovered source, no consumer traced)"});
        }
    }

    for (const ConfigDiagnostic& diag : *d) {
        if (diag.severity == ConfigDiagnostic::Severity::kError) {
            return false;
        }
    }
    return true;
}

bool mp6_ready_for_onboard_config(const Mp6PlatformConfig& cfg,
                                  std::vector<ConfigDiagnostic>* issues,
                                  PivotProvenance pivot_z_provenance) {
    std::vector<ConfigDiagnostic> local;
    std::vector<ConfigDiagnostic>* d = issues ? issues : &local;
    d->clear();

    if (!mp6_ready_for_targets(cfg, d, pivot_z_provenance)) {
        // keep d populated; fall through to append onboard-specific errors
        bool had_error = false;
        for (const ConfigDiagnostic& diag : *d) {
            if (diag.severity == ConfigDiagnostic::Severity::kError) {
                had_error = true;
                break;
            }
        }
        if (had_error) {
            return false;
        }
    }

    if (!cfg.onboard_control_set) {
        d->push_back({ConfigDiagnostic::Severity::kError, "(runtime)",
                      "onboard-control mode decision not set; OnBoardControl "
                      "(cmd-5 param 20) must be explicit"});
        return false;
    }

    require_scalar(cfg.accel_scale, "Accel/Decel/Deadband", "col0 (AccelScale)", d);
    require_scalar(cfg.accel_p1, "Accel/Decel/Deadband", "col1 (AccelP1)", d);
    require_scalar(cfg.accel_p0, "Accel/Decel/Deadband", "col2 (AccelP0)", d);
    require_scalar(cfg.decel_scale, "Accel/Decel/Deadband", "col3 (DecelScale)", d);
    require_scalar(cfg.deadband_pid, "Accel/Decel/Deadband", "col4 (DeadbandPID)", d);
    require_scalar(cfg.stopcount_limit, "Accel/Decel/Deadband", "col5 (StopCountLimit)", d);
    require_scalar(cfg.rotate_direction, "Rotate Direction", "required (cmd-5 param 36)", d);
    for (int k = 0; k < 4; ++k) {
        require_scalar(cfg.step_err_corr[k], "Step Correction",
                       ("col " + std::to_string(k)).c_str(), d);
    }
    for (int n = 0; n < kPidRows; ++n) {
        if (!cfg.pid[n].resolved) {
            d->push_back({ConfigDiagnostic::Severity::kError, "PID " + std::to_string(n),
                          "row missing/unresolved"});
        }
    }

    // MaxPPS is a cmd-5 parameter and must be materialized by the caller
    // (review-2 N3): "derivable but never computed" must not pass readiness.
    // The caller computes cfg.max_pps = round(MaxRPM * normalized StepRev / 60) —
    // snapshot: 230*3200/60 = 12266.7 -> 12267 — or sources it explicitly.
    if (cfg.max_pps <= 0) {
        d->push_back({ConfigDiagnostic::Severity::kError, "MaxPPS",
                      "not materialized: the caller must set cfg.max_pps "
                      "explicitly (snapshot: 230*3200/60 -> 12267)"});
    } else if (cfg.step_rev.present && cfg.max_motor_rpm.present &&
               std::isfinite(cfg.step_rev.value) && cfg.step_rev.value > 0.0 &&
               mp6_net_round(cfg.step_rev.value) >= 1.0 &&
               mp6_net_round(cfg.step_rev.value) <= INT_MAX &&
               cfg.max_motor_rpm.value > 0.0) {
        // Normalize Step/Rev before matching the recovered MaxPPS formula.
        const int step_rev = mp6_normalize_step_rev(cfg.step_rev.value);
        const double exact = cfg.max_motor_rpm.value * step_rev / 60.0;
        if (std::isfinite(exact) && exact >= 1.0 &&
            exact <= static_cast<double>(INT32_MAX)) {
            const double expected = mp6_net_round(exact);
            if (static_cast<double>(cfg.max_pps) != expected) {
                d->push_back({ConfigDiagnostic::Severity::kError, "MaxPPS",
                              "materialized value inconsistent with "
                              "Max Motor RPM * Step/Rev / 60"});
            }
        }
    }

    for (const ConfigDiagnostic& diag : *d) {
        if (diag.severity == ConfigDiagnostic::Severity::kError) {
            return false;
        }
    }
    return true;
}

} // namespace mp6
