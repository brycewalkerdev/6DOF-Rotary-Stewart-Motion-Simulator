#pragma once
/*
 * MP6 platform configuration: typed import/validation of the recovered
 * PlatformConfig.pltc snapshot plus the per-axis calibration derivation that
 * the recovered MotSim app performs.
 *
 * Pure C++17 core (namespace mp6). No Win32, no UI, no filesystem access, no
 * hardware. The caller supplies the file text.
 *
 * Evidence:
 *   motum_resources/motum-replacement-handoff/configuration-snapshot/PlatformConfig.pltc
 *     (TSV layout: header, tooltip, then alternating value/description pairs;
 *      confirmed against frm_Main.Load_Coefficients DGV_Setup fill at
 *      recovered-source/MotSim/frm_Main.cs L18555..L18614: column k value =
 *      field 3+2k (1-based), its in-file label = field 4+2k.)
 *   frm_Main.Load_Coefficients (L18505..L18768): per-axis calibration lists
 *   frm_Main.PID_SetValues (L31559): PID row consumption
 *   frm_Main.SendConfigToBoard (L31639..): onboard parameter set (context)
 *
 * Layout rules recovered from Load_Coefficients (values are authoritative,
 * in-file labels are decorative and sometimes misaligned):
 *   "Pot n"   col0 PotLo (in-file label "Pot - Docked" is decorative),
 *             col1 Angle-Docked (snapshot 55; AngLo = (n%2)*2-1 * col1),
 *             col2 PotScale (snapshot -12/+12; pScale denominator),
 *             col3 MSwitch-On, col4 MSwitch Off (both display-only,
 *             consumers untraced), col5 Gear Ratio (the value 20 is
 *             consumed as Module2.Gearbox).
 *   "PID n"   col0 P, col1 D, col2 I, col3 DFilter, col4 TConst
 *   "Max Travel (rad)" col0 = MinRadians, col1 = MaxRadians
 *
 * The snapshot is a REFERENCE. Everything this module marks unresolved or
 * unconsumed must be resolved by an integrator/reviewer before any live
 * send; readiness queries refuse to assert values the recovered source
 * does not supply.
 */

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace mp6 {

inline constexpr int kAxes = 6;        // axes 0..5 here; recovered source is 1-based 1..6
inline constexpr int kPidRows = 4;     // "PID 0" .. "PID 3"
inline constexpr int kPltcColumns = 6; // DGV_Setup ColumnCount = 6 (Load_Coefficients)

// .NET Math.Round(double) semantics: nearest, ties to even.
double mp6_net_round(double v);
// Same policy for a float operand (promoted exactly, still ties-to-even).
float mp6_net_roundf(float v);

// ---------------------------------------------------------------------------
// Parsed-file model
// ---------------------------------------------------------------------------

enum class FieldStatus : std::uint8_t {
    kConsumed,           // consumer verified in recovered source
    kDeclaredUnconsumed, // row present in snapshot, no consumer found in source
    kUnresolved,         // needed for parity, value absent/unpinnable in snapshot
    kLabelOnly           // in-file label without its own value slot
};

struct FieldRef {         // one manifest entry of the consumed-fields manifest
    const char* row;      // pltc row header, e.g. "Pot 1"
    int column;           // DGV column 0..5, or -1 for the whole row
    const char* key;      // canonical name used by the config API
    FieldStatus status;
    const char* evidence; // recovered-source location or explanatory note
};

struct ConfigDiagnostic {
    enum class Severity : std::uint8_t { kInfo, kWarning, kError };
    Severity severity;
    std::string where;    // pointer, e.g. "Pot 3[3]" (row header [column])
    std::string message;
};

// Provenance of the Other\<Platform>.txt pivot/attachment z coordinates (see
// mp6_ready_for_targets). Default kUnreviewed keeps unrecovered values from
// authorizing target readiness; kReviewed exists so a UI-reviewed import can
// clear the block without baking real data into code.
enum class PivotProvenance : std::uint8_t {
    kUnreviewed = 0,
    kFixtureOnly,
    kReviewed,
};

struct ConfigRow {        // one pltc line parsed the way DGV_Setup is filled
    std::string header;                // field[0]
    std::string tooltip;               // field[1] (row description slot)
    std::vector<std::string> value;    // columns 0..5 (field 2+2k)
    std::vector<std::string> in_label; // label token following each value, if any
    std::vector<std::string> trailing; // dangling labels after the last value
};

struct Scalar {           // one DGV cell value, exactly as written
    double value = 0.0;
    bool present = false;
    std::string raw;
};

struct Mp6PidRow {        // "PID n": P=col0, D=col1, I=col2, DFilter=col3, TConst=col4
    double p = 0.0, d = 0.0, integral = 0.0, dfilter = 0.0, tconst = 0.0;
    bool resolved = false;
};

struct Mp6AxisPots {      // "Pot n" row columns (snapshot line 20 layout)
    // Column semantics verified against Load_Coefficients reads and the
    // snapshot labels (both note columns 3/4 are display-only MSwitch slots):
    //   Pot 1   3543  Pot - Docked  55  Angle - Docked  -12  Pot Scale
    //           2258  MSwitch On    2039  MSwitch Off    20  Gear Ratio
    // The in-file labels are decorative; the source reads by column index.
    double pot_lo = 0.0;           // col 0 ("Pot - Docked" label is decorative)
    double angle_docked_deg = 0.0; // col 1 ("Angle - Docked", +/- deg)
    double pot_scale = 0.0;        // col 2 ("Pot Scale", pScale denominator)
    double mswitch_on = 0.0;       // col 3 ("MSwitch On"; consumers untraced)
    double mswitch_off = 0.0;      // col 4 ("MSwitch Off"; consumers untraced)
    double gearbox = 0.0;          // col 5 ("Gear Ratio" label; Gearbox value)
    bool present[6] = {};
};

// Runtime mode flag. NOT present in the pltc snapshot: OnBoardControl is a
// recovered app/board setting (cmd-5 parameter 20, recovered-source
// frm_Main.SendConfigToBoard). It must be set explicitly by the integrator;
// the parser never flips it.
struct Mp6PlatformConfig {
    std::vector<ConfigRow> rows;          // file order, empty lines skipped
    std::vector<ConfigDiagnostic> diagnostics;

    Scalar x_offset_mm;           // "X Offset" col0
    Scalar z_offset_row_mm;       // "Z Offset" col0  (declared, unconsumed)
    Scalar angle_offset_deg;      // "Angle Offset" col0
    Scalar crank_length_mm;       // "Crank Length" col0
    Scalar rod_length_mm;         // "Rod Length" col0 (copied to all six axes)
    Scalar centre_strut_dia_mm;   // "Centre Strut" col0
    Scalar helper_struts;         // "Centre Strut" col1
    Scalar payload_kg;            // "Payload Weight" col0
    Scalar step_rev;              // "Step/Rev" col0
    Scalar rotate_direction;      // "Rotate Direction" col0
    Scalar update_rate;           // "Update Rate" col0 (declared; source hardcodes 1/60)
    Scalar max_motor_rpm;         // "Max Motor RPM" col0
    Scalar travel_min_rad;        // "Max Travel (rad)" col0
    Scalar travel_max_rad;        // "Max Travel (rad)" col1
    Scalar accel_scale;           // "Accel/Decel/Deadband" col0
    Scalar accel_p1;              // col1 (in-file label "Accel Factor")
    Scalar accel_p0;              // col2 (in-file label "Accel P1")
    Scalar decel_scale;           // col3 (in-file label "Accel P0")
    Scalar deadband_pid;          // col4 (in-file label "Decel Factor")
    Scalar stopcount_limit;       // col5 (in-file label "Deadband")
    Scalar bump_fr_delay;         // "Bump Delay" col0
    Scalar bump_scale;            // "Bump Delay" col1 (label "Wheelbase" dangles)
    Scalar step_err_corr[4];      // "Step Correction" col0..3
    Mp6PidRow pid[kPidRows];
    Mp6AxisPots axis[kAxes];

    bool onboard_control = false;  // integrator-set mode selection
    bool onboard_control_set = false;

    // MotConv "X/Y/Z Point Zero" rotation-center offsets (frm_MotConv sliders,
    // applied through Module2.lblX/lblY/lblZ, Load_Component L16609-16610).
    // The MotConv txt field that stores these sliders is not pinned down in
    // the handoff; unresolved unless the integrator confirms values.
    double point_zero_x_mm = 0.0, point_zero_y_mm = 0.0, point_zero_z_mm = 0.0;
    bool point_zero_resolved = false;

    // Derived onboard quantities (Load_Coefficients):
    int max_pps = 0;       // Math.Round(MaxRPM * StepRev / 60)          L18644
                           // Caller-materialized (only the caller can compute
                           // it in float/double context, L18644); onboard
                           // readiness REQUIRES a positive consistent value.
    int init_freq_hz = 0;  // Math.Round(2.2 * StepRev * Gearbox[1] / 60) L18653

    // Parse a PlatformConfig.pltc text. Returns false only for structural
    // failures (no parseable header rows at all); missing/invalid fields are
    // reported as diagnostics and left unpresent.
    static bool from_pltc(const std::string& text, Mp6PlatformConfig& out);
};

// Consumed-fields manifest (row, DGV column, API key, status, evidence).
std::vector<FieldRef> mp6_config_manifest();

// ---------------------------------------------------------------------------
// Load_Coefficients calibration port (per axis, 0-based i = source axis-1)
// ---------------------------------------------------------------------------

struct Mp6AxisCalibration {
    // Source statics: Module2.AngLo (float), pScale (float), pOffset (float),
    // PotLo (int), Gearbox (float); frm_Main.AngleToStep (double),
    // RodAngle_Zero (double[7], only ever zero-initialized in recovered source).
    double axis_sign = 0.0;     // (i%2)*2-1 with i = source axis number
    double ang_lo_deg = 0.0;    // axis_sign * Angle-Docked          (float math)
    float p_scale = 0.0f;       // |StepRev*Gearbox/(PotScale*360)|  (float math)
    float p_offset = 0.0f;      // -pScale*PotLo + AngLo*Gearbox*StepRev/360
    double docked_steps = 0.0;  // PotLo*pScale + pOffset (init docked position)
    double angle_to_step = 0.0; // Gearbox*StepRev/(2*pi) steps per rod radian
    double rod_angle_zero_rad = 0.0; // recovered table is always zeros (L9238)
};

// Derives all six axes. Returns false if required per-axis fields are missing
// or non-finite geometry-calibration inputs (gearbox/pot_scale/step_rev) are
// unusable; details go into *err. Float arithmetic mirrors the recovered
// source exactly where parity matters (p_scale/p_offset/ang_lo).
bool mp6_config_derive_calibration(const Mp6PlatformConfig& cfg,
                                   Mp6AxisCalibration out[kAxes],
                                   std::string* err);

// Telemetry feedback decode port (frm_Main L32233):
//   calibrated steps = Math.Round(word * pScale / 8 + pOffset)  (.NET, half-even)
int mp6_feedback_calibrated_steps(const Mp6AxisCalibration& cal, double word);

// ---------------------------------------------------------------------------
// Readiness
// ---------------------------------------------------------------------------

// Geometry + calibration sufficient for host-side command-2 step targets.
// Unknown mode/calibration fields do not invent values: unresolved items
// block readiness with diagnostics.
//
// Pivot provenance: base-pivot and top-attachment Z coordinates come from the
// unrecovered Other\<Platform>.txt component file. This function cannot see
// that data (it lives outside the pltc snapshot), so `pivot_z_provenance`
// states the caller's position:
//   kUnreviewed (default) -> readiness fails with an Error until the caller
//        confirms review.
//   kFixtureOnly          -> offline test fixture placeholders (-30/+10); draws
//                            a Warning: never valid for a real rig.
//   kReviewed             -> integrator/UI supplied real Other\<Platform>.txt
//                            values and an operator has reviewed them; draws
//                            an Info note. Callers may source this flag from
//                            a reviewed UI input; it is never set implicitly.
bool mp6_ready_for_targets(const Mp6PlatformConfig& cfg,
                           std::vector<ConfigDiagnostic>* issues,
                           PivotProvenance pivot_z_provenance =
                               PivotProvenance::kUnreviewed);

// Additionally requires an explicit onboard-control mode decision and the
// whole cmd-5 parameter set resolvable (PID rows, deadband, accel/decel,
// step correction, MaxPPS, rotate direction, PotLo/AngLo/pScale/pOffset/
// Gearbox doubles, DFilter/TConst, CommsTimeoutMS surface decision). Same
// pivot provenance semantics as mp6_ready_for_targets (default blocks).
bool mp6_ready_for_onboard_config(const Mp6PlatformConfig& cfg,
                                  std::vector<ConfigDiagnostic>* issues,
                                  PivotProvenance pivot_z_provenance =
                                      PivotProvenance::kUnreviewed);

} // namespace mp6
