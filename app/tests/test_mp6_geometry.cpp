/*
 * Offline unit tests for the MP6 geometry/config core.
 *
 * Style follows app/tests/test_mp6_protocol.cpp: plain expect helpers, a main
 * returning an exit code, no test framework. Expected values are hardcoded
 * here from the recovered sources (independent transcriptions) rather than
 * derived from the production code paths:
 *
 *   PlatformConfig.pltc snapshot values  -> handoff configuration-snapshot
 *   Load_Coefficients calibration        -> frm_Main.cs L18640..L18680
 *   CalcPlatformGeom_1 design points     -> frm_Design.cs L591..L881
 *   CalcPlatform_3 transform / StepAim   -> frm_Main.cs L16635..L16954
 *   MotorPos_6DOF_C solver               -> frm_Main.cs L17141..L18472
 *
 * Unrecovered geometry: Other\<Platform>.txt (pivot/top z coordinates) is not
 * in the handoff. Tests use a documented fixture (kFixture*) and never claim
 * it is the rig's true value.
 */

#include "mp6_config.h"
#include "mp6_geometry.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
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

// Exact-type equality (template: no conversion happens, so MSVC /W4
// narrowing warnings cannot fire for double/int/uint32 mixes). Values used
// with it are exactly representable in their own types, so == is meaningful.
template <typename A, typename B>
void expectEq(A actual, B expected, const char* actualExpr,
              const char* expectedExpr, int line) {
    ++g_checks;
    if (!(actual == expected)) {
        ++g_failures;
        std::printf("FAIL line %d: %s == %s (actual %s, expected %s)\n", line,
                    actualExpr, expectedExpr,
                    std::to_string(actual).c_str(),
                    std::to_string(expected).c_str());
    }
}

void expectNear(double actual, double expected, double epsilon, int line) {
    ++g_checks;
    if (!(std::fabs(actual - expected) <= epsilon)) {
        ++g_failures;
        std::printf("FAIL line %d: %f vs %f (|diff| > %g)\n", line, actual,
                    expected, epsilon);
    }
}

#define CHECK(cond) expectTrue(static_cast<bool>(cond), #cond, __LINE__)
#define CHECK_EQ(a, b) expectEq((a), (b), #a, #b, __LINE__)
#define CHECK_NEAR(a, b, eps) expectNear((a), (b), (eps), __LINE__)

constexpr double kPi = 3.141592653589793;

// ---------------------------------------------------------------------------
// PlatformConfig.pltc snapshot text, reconstructed field-for-field from the
// handoff file (generic rows 18 fields, Pot rows 14, exactly as verified in
// the snapshot tab dump).
// ---------------------------------------------------------------------------

std::string joinFields(const std::vector<std::string>& fields) {
    std::string out;
    for (size_t i = 0; i < fields.size(); ++i) {
        if (i != 0) {
            out += '\t';
        }
        out += fields[i];
    }
    return out;
}

std::vector<std::string> genericRow(const char* header, const char* tooltip,
                                    const std::vector<std::string>& pairs) {
    std::vector<std::string> fields = {header, tooltip};
    for (const std::string& s : pairs) {
        fields.push_back(s);
    }
    while (fields.size() < 18) {
        fields.push_back("");
    }
    return fields;
}

std::vector<std::string> potRow(const char* header, const char* potLo,
                                const char* potScale, const char* mswitchOn,
                                const char* mswitchOff) {
    // 14 fields exactly like the real snapshot (PlatformConfig.pltc lines
    // 20..25): value at fields 2+2k, decorative label at 3+2k. The row's
    // "55" is the Angle - Docked value; +/-12 is Pot Scale.
    return {header,        "",            potLo,      "Pot - Docked",
            "55",          "Angle - Docked", potScale,    "Pot Scale",
            mswitchOn,     "MSwitch On",  mswitchOff, "MSwitch Off",
            "20",          "Gear Ratio"};
}

std::string snapshotText() {
    std::vector<std::string> lines;
    lines.push_back(joinFields(genericRow("X Offset", "", {"0"})));
    lines.push_back(joinFields(genericRow("Z Offset", "", {"506"})));
    lines.push_back(joinFields(genericRow("Angle Offset", "", {"0"})));
    lines.push_back(joinFields(genericRow("Crank Length", "", {"230"})));
    lines.push_back(joinFields(genericRow("Rod Length", "", {"645"})));
    lines.push_back(joinFields(
        genericRow("Centre Strut", "", {"72", "Diameter (mm)", "0"})));
    lines.push_back(joinFields(genericRow("Payload Weight", "(kg)", {"88"})));
    lines.push_back(joinFields(genericRow("Step/Rev", "", {"3200"})));
    lines.push_back(joinFields(genericRow(
        "Rotate Direction", "-1: Counter Rotate, 1: Normal Rotate", {"-1"})));
    lines.push_back(joinFields(genericRow("Update Rate", "", {"60"})));
    lines.push_back(joinFields(genericRow("Max Motor RPM", "", {"230"})));
    // The real Max Travel row carries no tooltip; "Min"/"Max" sit one slot
    // early and the numbers land on the DGV value columns (verified layout).
    lines.push_back(joinFields(std::vector<std::string>{
        "Max Travel (rad)", "Min", "-0.9", "Max", "0.9",
        "", "", "", "", "", "", "", "", "", ""}));
    lines.push_back(joinFields(genericRow(
        "Accel/Decel/Deadband", "",
        {"8.5", "Accel Factor", "-0.26", "Accel P1", "60000", "Accel P0",
         "1.25", "Decel Factor", "30", "Deadband", "15", "StopCount"})));
    lines.push_back(joinFields(genericRow(
        "Step Correction", "",
        {"0.0075", "Potti Corr PID 0", "0.002", "Potti Corr PID 1", "0.002",
         "Potti Corr PID 2", "0.002", "Potti Corr PID 3"})));
    lines.push_back(joinFields(genericRow(
        "PID 0", "", {"11", "Proportional", "30", "Derivative", "0", "Integral",
                      "20", "DFilter", "20", "TConst"})));
    lines.push_back(joinFields(genericRow(
        "PID 1", "", {"9", "Proportional", "0", "Derivative", "0", "Integral",
                      "20", "DFilter", "50", "TConst"})));
    lines.push_back(joinFields(genericRow(
        "PID 2", "", {"9", "Proportional", "0", "Derivative", "0", "Integral",
                      "20", "DFilter", "20", "TConst"})));
    lines.push_back(joinFields(genericRow(
        "PID 3", "", {"15", "Proportional", "0", "Derivative", "0", "Integral",
                      "20", "DFilter", "20", "TConst"})));
    lines.push_back(joinFields(genericRow(
        "Bump Delay", "F/R Input Delay (Speed & Wheelbase)",
        {"0.005", "Scale", "3.1", "Wheelbase"})));
    lines.push_back(joinFields(potRow("Pot 1", "3543", "-12", "2258", "2039")));
    lines.push_back(joinFields(potRow("Pot 2", "650", "12", "1669", "1874")));
    lines.push_back(joinFields(potRow("Pot 3", "3553", "-12", "3011", "2784")));
    lines.push_back(joinFields(potRow("Pot 4", "609", "12", "1879", "2095")));
    lines.push_back(joinFields(potRow("Pot 5", "3297", "-12", "2188", "1965")));
    lines.push_back(joinFields(potRow("Pot 6", "631", "12", "1970", "2188")));
    std::string text;
    for (size_t i = 0; i < lines.size(); ++i) {
        text += lines[i];
        text += "\r\n";
    }
    return text;
}

std::string replaceOnce(std::string text, const std::string& from,
                        const std::string& to) {
    const size_t at = text.find(from);
    if (at == std::string::npos) return {};
    text.replace(at, from.size(), to);
    return text;
}

void testConsumedParseFailures() {
    struct Case { const char* label; const char* from; const char* to; bool pid; };
    const Case cases[] = {
        {"PID malformed P", "\t11\tProportional", "\tbogus\tProportional", true},
        {"PID missing P", "\t11\tProportional", "\t\tProportional", true},
        {"PID NaN P", "\t11\tProportional", "\tNaN\tProportional", true},
        {"PID Inf P", "\t11\tProportional", "\tInf\tProportional", true},
        {"PID overflow P", "\t11\tProportional", "\t1e9999\tProportional", true},
        {"Pot malformed Lo", "\t3543\tPot - Docked", "\tbogus\tPot - Docked", false},
        {"Pot missing Lo", "\t3543\tPot - Docked", "\t\tPot - Docked", false},
        {"Pot NaN Lo", "\t3543\tPot - Docked", "\tNaN\tPot - Docked", false},
        {"Pot Inf Lo", "\t3543\tPot - Docked", "\tInf\tPot - Docked", false},
        {"Pot overflow Lo", "\t3543\tPot - Docked", "\t1e9999\tPot - Docked", false},
    };
    for (const Case& c : cases) {
        const std::string damaged = replaceOnce(snapshotText(), c.from, c.to);
        CHECK(!damaged.empty());
        mp6::Mp6PlatformConfig cfg;
        CHECK(mp6::Mp6PlatformConfig::from_pltc(damaged, cfg));
        bool parse_error = false;
        for (const auto& d : cfg.diagnostics) {
            if (d.severity == mp6::ConfigDiagnostic::Severity::kError) parse_error = true;
        }
        CHECK(parse_error);
        std::vector<mp6::ConfigDiagnostic> issues;
        if (c.pid) {
            CHECK(!cfg.pid[0].resolved);
            CHECK_EQ(cfg.pid[0].p, 0.0); // value alone cannot certify fabricated P=0
            cfg.onboard_control_set = true;
            cfg.max_pps = 12267;
            CHECK(!mp6::mp6_ready_for_onboard_config(cfg, &issues,
                                                    mp6::PivotProvenance::kReviewed));
        } else {
            CHECK(!cfg.axis[0].present[0]);
            mp6::Mp6AxisCalibration cal[mp6::kAxes];
            std::string error;
            CHECK(!mp6::mp6_config_derive_calibration(cfg, cal, &error));
            cfg.point_zero_resolved = true;
            CHECK(!mp6::mp6_ready_for_targets(cfg, &issues,
                                              mp6::PivotProvenance::kReviewed));
        }
    }
}

// ---------------------------------------------------------------------------
// Fixture geometry. The pivot z values are NOT the rig's: they stand in for
// the unrecovered Other\MOT155-D2.txt z coordinates.
//   base pivot z = -30 mm per axis, top attachment z = +10 mm per axis
//   (top z nonzero so the exact-AtanKD transform is well-defined)
// With X_Offset/Z_Offset = (0, 530) every pivot-to-target distance lands near
// 646 mm, reachable inside [RodLen - CrankLen, RodLen + CrankLen] = [415,875].
// ---------------------------------------------------------------------------

constexpr double kFixtureBaseZ = -30.0;
constexpr double kFixtureTopZ = 10.0;
constexpr double kFixtureXOffset = 0.0;
constexpr double kFixtureZOffset = 530.0;
constexpr double kAngleToStep = 180.0 / kPi * 20.0 * 3200.0 / 360.0;

double cos30() {
    return std::cos(30.0 * kPi / 180.0);
}
double tan30() {
    return std::tan(30.0 * kPi / 180.0);
}
double sin_deg(double deg) {
    return std::sin(deg * kPi / 180.0);
}
double cos_deg(double deg) {
    return std::cos(deg * kPi / 180.0);
}

mp6::Mp6Pose poseWith(double lat, double lon, double vert, double roll,
                      double pitch, double yaw) {
    mp6::Mp6Pose p;
    p.lat_mm = lat;
    p.lon_mm = lon;
    p.vert_mm = vert;
    p.roll = roll;
    p.pitch = pitch;
    p.yaw = yaw;
    return p;
}

// Independent transcription of PlatTop/PlatBase for MOT155-D2
// (frm_Design.CalcPlatformGeom_1 Typ == 2), with the Load_Component y-flip.
struct PlanarRef {
    double base_x[6], base_y[6], top_x[6], top_y[6];
};

PlanarRef mot155d2Planar() {
    const double topA = 520.0, topB = 14.0, botA = 840.0, botB = 230.0;
    const double t1x = (-topA) / 2.0 / cos30(), t1y = 0.0;
    const double t2x = topA / 2.0 * tan30(), t2y = (-topA) / 2.0;
    const double t3x = topA / 2.0 * tan30(), t3y = topA / 2.0;
    const double b1x = botA / 2.0 / cos30(), b1y = 0.0;
    const double b2x = (-botA) / 2.0 * tan30(), b2y = (-botA) / 2.0;
    const double b3x = (-botA) / 2.0 * tan30(), b3y = botA / 2.0;
    PlanarRef p;
    // PlatTop rows 1..6 (pre-flip y), then the y-flip.
    const double pre_top_x[6] = {t3x + sin_deg(120) * topB,
                                 t2x - sin_deg(240) * topB,
                                 t2x + sin_deg(240) * topB,
                                 t1x - sin_deg(0) * topB,
                                 t1x + sin_deg(0) * topB,
                                 t3x - sin_deg(120) * topB};
    const double pre_top_y[6] = {t3y + cos_deg(120) * topB,
                                 t2y - cos_deg(240) * topB,
                                 t2y + cos_deg(240) * topB,
                                 t1y - cos_deg(0) * topB,
                                 t1y + cos_deg(0) * topB,
                                 t3y - cos_deg(120) * topB};
    const double pre_base_x[6] = {b1x + sin_deg(0) * botB,
                                  b1x - sin_deg(0) * botB,
                                  b2x + sin_deg(120) * botB,
                                  b2x - sin_deg(120) * botB,
                                  b3x + sin_deg(240) * botB,
                                  b3x - sin_deg(240) * botB};
    const double pre_base_y[6] = {b1y + cos_deg(0) * botB,
                                  b1y - cos_deg(0) * botB,
                                  b2y + cos_deg(120) * botB,
                                  b2y - cos_deg(120) * botB,
                                  b3y + cos_deg(240) * botB,
                                  b3y - cos_deg(240) * botB};
    for (int i = 0; i < 6; ++i) {
        p.top_x[i] = pre_top_x[i];
        p.top_y[i] = -pre_top_y[i];
        p.base_x[i] = pre_base_x[i];
        p.base_y[i] = -pre_base_y[i];
    }
    return p;
}

// Shared fixture: config + calibration + design + geometry setup.
struct Fixture {
    mp6::Mp6PlatformConfig cfg;
    mp6::Mp6AxisCalibration cal[mp6::kAxes];
    mp6::Mp6DesignParams design;
    mp6::Mp6PlanarAttachments planar;
    mp6::Mp6GeometrySetup setup;
};

bool buildFixture(Fixture* f, std::string* err) {
    *err = "";
    if (!mp6::Mp6PlatformConfig::from_pltc(snapshotText(), f->cfg)) {
        *err = "snapshot parse failed";
        return false;
    }
    if (!mp6::mp6_config_derive_calibration(f->cfg, f->cal, err)) {
        return false;
    }
    if (!mp6::mp6_design(mp6::mp6_design_from_name("MOT155-D2"), &f->design)) {
        *err = "design table lookup failed";
        return false;
    }
    f->planar = mp6::mp6_planar_attachments(f->design);
    std::array<double, mp6::kAxes> bz, tz;
    bz.fill(kFixtureBaseZ);
    tz.fill(kFixtureTopZ);
    f->setup = mp6::mp6_geometry_setup(f->planar, bz, tz);
    return true;
}

bool defaultSolver(mp6::Mp6CrankSolver& solver, const Fixture& f,
                   std::string* err) {
    std::array<double, mp6::kAxes> rod_len;
    rod_len.fill(f.cfg.rod_length_mm.value);
    std::array<double, mp6::kAxes> zero;
    zero.fill(0.0);
    return solver.init(f.setup, rod_len, f.cfg.crank_length_mm.value,
                       f.cfg.travel_min_rad.value,
                       f.cfg.travel_max_rad.value, zero, err);
}

bool angleToSteps(const Fixture& f, std::array<double, mp6::kAxes>* ats) {
    for (int i = 0; i < mp6::kAxes; ++i) {
        (*ats)[i] = f.cal[i].angle_to_step;
    }
    return true;
}

// Transforms one pose through the production path into solver targets.
std::array<mp6::Mp6Point3, mp6::kAxes> targetsFor(const Fixture& f,
                                                 const mp6::Mp6Pose& pose) {
    std::array<mp6::Mp6Point3, mp6::kAxes> targets = f.setup.top_attachment_mm;
    mp6::mp6_transform_points(targets, pose, kFixtureXOffset,
                              kFixtureZOffset);
    return targets;
}

// Closed-form circle intersection: crank end E(t) = pivot + CrankLen *
// (cos t * u + sin t * zhat) with |E(t) - target| = RodLen requires
// cos t * (v.u) + sin t * v.z = K, K = (CL^2 + |v|^2 - RodLen^2) / (2*CL).
// Returns the in-range aim nearest zero.
bool nearestZeroResidualAim(const double pivot[3], const double target[3],
                            double rod_len, double crank_len,
                            double crank_x_offset, double crank_init,
                            double* aim_out) {
    const double v[3] = {target[0] - pivot[0], target[1] - pivot[1],
                         target[2] - pivot[2]};
    const double u[2] = {std::cos(crank_x_offset), std::sin(crank_x_offset)};
    const double A = v[0] * u[0] + v[1] * u[1];
    const double B = v[2];
    const double v2 =
        v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    const double K =
        (crank_len * crank_len + v2 - rod_len * rod_len) / (2.0 * crank_len);
    const double n2 = A * A + B * B;
    if (n2 < K * K) {
        return false;
    }
    const double g = std::sqrt(n2 - K * K);
    double best = 1e30;
    for (int k = 0; k < 2; ++k) {
        const double sign = (k == 0) ? 1.0 : -1.0;
        const double c = (K * A - sign * B * g) / n2;
        const double s = (K * B + sign * A * g) / n2;
        double aim = std::atan2(s, c) - crank_init;
        while (aim > 2.475) {
            aim -= 2.0 * kPi;
        }
        while (aim < -2.475) {
            aim += 2.0 * kPi;
        }
        if (std::fabs(aim) < best) {
            best = std::fabs(aim);
            *aim_out = aim;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

void testSnapshotImport() {
    mp6::Mp6PlatformConfig cfg;
    CHECK(mp6::Mp6PlatformConfig::from_pltc(snapshotText(), cfg));

    CHECK(cfg.x_offset_mm.present);
    CHECK_NEAR(cfg.x_offset_mm.value, 0.0, 1e-12);
    CHECK(cfg.z_offset_row_mm.present);
    CHECK_NEAR(cfg.z_offset_row_mm.value, 506.0, 1e-12);
    CHECK_NEAR(cfg.angle_offset_deg.value, 0.0, 1e-12);
    CHECK_NEAR(cfg.crank_length_mm.value, 230.0, 1e-12);
    CHECK_NEAR(cfg.rod_length_mm.value, 645.0, 1e-12);
    CHECK_NEAR(cfg.centre_strut_dia_mm.value, 72.0, 1e-12);
    CHECK_NEAR(cfg.helper_struts.value, 0.0, 1e-12);
    CHECK_NEAR(cfg.payload_kg.value, 88.0, 1e-12);
    CHECK_NEAR(cfg.step_rev.value, 3200.0, 1e-12);
    CHECK_NEAR(cfg.rotate_direction.value, -1.0, 1e-12);
    CHECK_NEAR(cfg.update_rate.value, 60.0, 1e-12);
    CHECK_NEAR(cfg.max_motor_rpm.value, 230.0, 1e-12);
    CHECK_NEAR(cfg.travel_min_rad.value, -0.9, 1e-12);
    CHECK_NEAR(cfg.travel_max_rad.value, 0.9, 1e-12);

    CHECK_NEAR(cfg.accel_scale.value, 8.5, 1e-12);
    CHECK_NEAR(cfg.accel_p1.value, -0.26, 1e-12);
    CHECK_NEAR(cfg.accel_p0.value, 60000.0, 1e-12);
    CHECK_NEAR(cfg.decel_scale.value, 1.25, 1e-12);
    CHECK_NEAR(cfg.deadband_pid.value, 30.0, 1e-12);
    CHECK_NEAR(cfg.stopcount_limit.value, 15.0, 1e-12);

    CHECK_NEAR(cfg.bump_fr_delay.value, 0.005, 1e-15);
    CHECK_NEAR(cfg.bump_scale.value, 3.1, 1e-12);

    const double step_corr[4] = {0.0075, 0.002, 0.002, 0.002};
    for (int k = 0; k < 4; ++k) {
        CHECK(cfg.step_err_corr[k].present);
        CHECK_NEAR(cfg.step_err_corr[k].value, step_corr[k], 1e-15);
    }

    const double pid[4][5] = {
        {11, 30, 0, 20, 20}, {9, 0, 0, 20, 50},
        {9, 0, 0, 20, 20},   {15, 0, 0, 20, 20},
    };
    for (int n = 0; n < mp6::kPidRows; ++n) {
        CHECK(cfg.pid[n].resolved);
        CHECK_NEAR(cfg.pid[n].p, pid[n][0], 1e-12);
        CHECK_NEAR(cfg.pid[n].d, pid[n][1], 1e-12);
        CHECK_NEAR(cfg.pid[n].integral, pid[n][2], 1e-12);
        CHECK_NEAR(cfg.pid[n].dfilter, pid[n][3], 1e-12);
        CHECK_NEAR(cfg.pid[n].tconst, pid[n][4], 1e-12);
    }

    // Column semantics verified independently against the snapshot labels and
    // the Load_Coefficients reads (F1): col0 PotLo, col1 Angle-Docked (55),
    // col2 PotScale (+/-12), col3/4 MSwitch On/Off, col5 Gearbox (20).
    const double pot[6][6] = {
        {3543, 55, -12, 2258, 2039, 20}, {650, 55, 12, 1669, 1874, 20},
        {3553, 55, -12, 3011, 2784, 20}, {609, 55, 12, 1879, 2095, 20},
        {3297, 55, -12, 2188, 1965, 20}, {631, 55, 12, 1970, 2188, 20},
    };
    for (int i = 0; i < mp6::kAxes; ++i) {
        CHECK_NEAR(cfg.axis[i].pot_lo, pot[i][0], 1e-12);           // col 0
        CHECK_NEAR(cfg.axis[i].angle_docked_deg, pot[i][1], 1e-12); // col 1
        CHECK_NEAR(cfg.axis[i].pot_scale, pot[i][2], 1e-12);        // col 2
        CHECK_NEAR(cfg.axis[i].mswitch_on, pot[i][3], 1e-12);       // col 3
        CHECK_NEAR(cfg.axis[i].mswitch_off, pot[i][4], 1e-12);      // col 4
        CHECK_NEAR(cfg.axis[i].gearbox, pot[i][5], 1e-12);          // col 5
        for (int k = 0; k < 6; ++k) {
            CHECK(cfg.axis[i].present[k]);
        }
    }

    // Declared-unconsumed rows and manifest notes surface as diagnostics.
    int info_z_offset = 0, info_update_rate = 0, info_pot_note = 0,
        info_motconv = 0, errors = 0;
    for (const mp6::ConfigDiagnostic& d : cfg.diagnostics) {
        if (d.severity == mp6::ConfigDiagnostic::Severity::kError) {
            ++errors;
        }
        if (d.severity == mp6::ConfigDiagnostic::Severity::kInfo) {
            if (d.where == "Z Offset") ++info_z_offset;
            if (d.where == "Update Rate") ++info_update_rate;
            if (d.where == "Pot n") ++info_pot_note;
            if (d.where == "MotConv") ++info_motconv;
        }
    }
    CHECK_EQ(errors, 0);
    CHECK_EQ(info_z_offset, 1);
    CHECK_EQ(info_update_rate, 1);
    CHECK_EQ(info_pot_note, 1);
    CHECK_EQ(info_motconv, 1);

    const auto manifest = mp6::mp6_config_manifest();
    CHECK(manifest.size() > 60);
    int unresolved = 0;
    for (const mp6::FieldRef& fr : manifest) {
        if (fr.status == mp6::FieldStatus::kUnresolved) {
            ++unresolved;
        }
    }
    CHECK(unresolved == 2); // runtime mode + MotConv point-zero (MSwitch
                            // On/Off are now kDeclaredUnconsumed, not blockers)
}

void testReadiness() {
    mp6::Mp6PlatformConfig cfg;
    CHECK(mp6::Mp6PlatformConfig::from_pltc(snapshotText(), cfg));

    std::vector<mp6::ConfigDiagnostic> issues;
    // Unresolved "Point Zero" sliders block by default (no invention).
    CHECK(!mp6::mp6_ready_for_targets(cfg, &issues));
    cfg.point_zero_resolved = true;

    // F4: unrecovered pivot z must block target readiness until the caller
    // states provenance. The default (kUnreviewed) fails even though the
    // fixture's finite placeholders parse fine; kFixtureOnly draws a Warning
    // but is offline-test-only; kReviewed clears the block.
    issues.clear();
    CHECK(!mp6::mp6_ready_for_targets(cfg, &issues));
    bool provenance_error = false;
    for (const mp6::ConfigDiagnostic& d : issues) {
        if (d.severity == mp6::ConfigDiagnostic::Severity::kError &&
            d.where == "Geometry" &&
            d.message.find("provenance") != std::string::npos) {
            provenance_error = true;
        }
    }
    CHECK(provenance_error);
    issues.clear();
    CHECK(mp6::mp6_ready_for_targets(cfg, &issues,
                                     mp6::PivotProvenance::kFixtureOnly));
    bool fixture_warning = false;
    for (const mp6::ConfigDiagnostic& d : issues) {
        if (d.severity == mp6::ConfigDiagnostic::Severity::kWarning &&
            d.where == "Geometry" &&
            d.message.find("FIXTURE") != std::string::npos) {
            fixture_warning = true;
        }
    }
    CHECK(fixture_warning);
    issues.clear();
    CHECK(mp6::mp6_ready_for_targets(cfg, &issues,
                                     mp6::PivotProvenance::kReviewed));

    issues.clear();
    CHECK(!mp6::mp6_ready_for_onboard_config(cfg, &issues));
    cfg.onboard_control = true;
    cfg.onboard_control_set = true;
    // N3: MaxPPS must be materialized by the caller; a not-yet-computed value
    // no longer passes.
    issues.clear();
    CHECK(!mp6::mp6_ready_for_onboard_config(cfg, &issues,
                                             mp6::PivotProvenance::kReviewed));
    bool maxpps_error = false;
    for (const mp6::ConfigDiagnostic& d : issues) {
        if (d.severity == mp6::ConfigDiagnostic::Severity::kError &&
            d.where == "MaxPPS" &&
            d.message.find("materialized") != std::string::npos) {
            maxpps_error = true;
        }
    }
    CHECK(maxpps_error);
    // Snapshot value: 230 (MaxRPM) * 3200 (Step/Rev) / 60 = 12266.7 -> 12267.
    cfg.max_pps = 12267;
    issues.clear();
    CHECK(mp6::mp6_ready_for_onboard_config(cfg, &issues,
                                            mp6::PivotProvenance::kReviewed));
    cfg.step_rev.value = 3200.6;
    cfg.max_pps = 12270;
    issues.clear();
    CHECK(mp6::mp6_ready_for_onboard_config(cfg, &issues,
                                            mp6::PivotProvenance::kReviewed));
    cfg.max_pps = 12269;
    issues.clear();
    CHECK(!mp6::mp6_ready_for_onboard_config(cfg, &issues,
                                             mp6::PivotProvenance::kReviewed));
    cfg.step_rev.value = 3200;
    cfg.max_pps = 12267;
    issues.clear();
    // Inconsistent materialization must also block.
    cfg.max_pps = 12266;
    issues.clear();
    CHECK(!mp6::mp6_ready_for_onboard_config(cfg, &issues,
                                             mp6::PivotProvenance::kReviewed));
    maxpps_error = false;
    for (const mp6::ConfigDiagnostic& d : issues) {
        if (d.severity == mp6::ConfigDiagnostic::Severity::kError &&
            d.where == "MaxPPS" &&
            d.message.find("inconsistent") != std::string::npos) {
            maxpps_error = true;
        }
    }
    CHECK(maxpps_error);
    cfg.max_pps = 12267; // restore for downstream expectations

    mp6::Mp6PlatformConfig broken;
    CHECK(mp6::Mp6PlatformConfig::from_pltc(snapshotText(), broken));
    broken.point_zero_resolved = true;
    broken.rod_length_mm.present = false;
    issues.clear();
    CHECK(!mp6::mp6_ready_for_targets(
        broken, &issues, mp6::PivotProvenance::kReviewed));
}

void testCalibration() {
    mp6::Mp6PlatformConfig cfg;
    CHECK(mp6::Mp6PlatformConfig::from_pltc(snapshotText(), cfg));
    mp6::Mp6AxisCalibration cal[mp6::kAxes];
    std::string err;
    CHECK(mp6::mp6_config_derive_calibration(cfg, cal, &err));
    CHECK_EQ(mp6::mp6_normalize_pot_lo(3543.75), 3544);
    CHECK_EQ(mp6::mp6_normalize_pot_lo(3543.5), 3544);
    CHECK_EQ(mp6::mp6_normalize_pot_lo(3544.5), 3544);
    CHECK_EQ(mp6::mp6_normalize_step_rev(3200.6), 3201);
    auto fractional = cfg;
    fractional.axis[0].pot_lo = 3543.75;
    CHECK(mp6::mp6_config_derive_calibration(fractional, cal, &err));
    const float source_scale = std::fabs(3200.0f * 20.0f / (-12.0f * 360.0f));
    const float source_offset = -source_scale * 3544.0f + 55.0f * 20.0f * 3200.0f / 360.0f;
    CHECK_NEAR(cal[0].p_offset, source_offset, 0.001);
    fractional.step_rev.value = 3200.6;
    CHECK(mp6::mp6_config_derive_calibration(fractional, cal, &err));
    const float rounded_step_scale = std::fabs(3201.0f * 20.0f / (-12.0f * 360.0f));
    CHECK_NEAR(cal[0].p_scale, rounded_step_scale, 1e-6);
    CHECK(mp6::mp6_config_derive_calibration(cfg, cal, &err));

    // Independent float-math re-derivation from the SOURCE semantics
    // (Load_Coefficients L18667..L18680: PotLo=col0, AngLo=(n%2)*2-1*col1,
    // pScale=|StepRev*Gearbox/(col2*360)|). Column assignment re-verified
    // against the snapshot (col1 = 55 "Angle - Docked", col2 = +/-12
    // "Pot Scale"); the old ±12 Angle-Docked / pScale≈0.0786 values were the
    // F1 shifted-column bug.
    const int step_rev = 3200;
    // per axis n = 1..6: {PotLo, Angle-Docked, PotScale}
    const double pot[6][3] = {
        {3543, 55, -12}, {650, 55, 12},  {3553, 55, -12},
        {609, 55, 12},   {3297, 55, -12}, {631, 55, 12},
    };
    CHECK_NEAR(kAngleToStep, 10185.916357915688, 1e-6);
    for (int n = 1; n <= 6; ++n) {
        const int i = n - 1;
        const double sign = static_cast<double>(n % 2 * 2 - 1);
        const float ang_lo =
            static_cast<float>(sign) * static_cast<float>(pot[i][1]);
        const float p_scale = std::fabs(static_cast<float>(step_rev) * 20.0f /
                                        (static_cast<float>(pot[i][2]) * 360.0f));
        const float p_offset =
            (0.0f - p_scale) * static_cast<float>(pot[i][0]) +
            ang_lo * 20.0f * static_cast<float>(step_rev) / 360.0f;
        CHECK_NEAR(cal[i].axis_sign, sign, 1e-12);
        CHECK_NEAR(cal[i].ang_lo_deg, static_cast<double>(ang_lo), 1e-9);
        // Source regression: AngLo = +55 (odd) / -55 (even), from col1 = 55.
        CHECK_NEAR(cal[i].ang_lo_deg, sign * 55.0, 1e-9);
        CHECK_NEAR(cal[i].p_scale, static_cast<double>(p_scale), 1e-7);
        // pScale = |3200*20/(12*360)| = 64000/4320 ~ 14.8148148 (all axes,
        // since |PotScale| = 12 everywhere in the snapshot).
        CHECK_NEAR(cal[i].p_scale, 64000.0 / 4320.0, 1e-6);
        CHECK_NEAR(cal[i].p_offset, static_cast<double>(p_offset), 1e-4);
        // Source-float regression (review-2 N2): the port evaluates
        // PotLo*pScale + pOffset as a float product + float sum, exactly the
        // way the recovered lblR expression (L18679) evaluates it; the
        // expectation is re-derived with the same float sequence, not the
        // production call.
        const float docked = static_cast<float>(pot[i][0]) * p_scale + p_offset;
        CHECK_NEAR(cal[i].docked_steps, static_cast<double>(docked), 1e-9);
        CHECK_NEAR(cal[i].angle_to_step, kAngleToStep, 1e-9);
        // D invariant: PotLo*pScale + pOffset == AngLo*Gearbox*StepRev/360,
        // i.e. docked_steps = +/- 55*20*3200/360 = +/- 9777.7773 (float:
        // +9777.77734375 odd / -9777.7763671875 even) for every axis
        // regardless of PotLo (PotLo term cancels exactly).
        CHECK_NEAR(pot[i][0] * p_scale + p_offset,
                   static_cast<double>(ang_lo) * 20.0 * step_rev / 360.0,
                   0.01); // float accumulation shifts ~2e-3 from the exact
        CHECK_NEAR(cal[i].docked_steps, static_cast<double>(ang_lo) * 20.0 *
                                                 static_cast<double>(step_rev) /
                                                 360.0,
                   0.01); // double quotient
        CHECK_EQ(cal[i].rod_angle_zero_rad == 0.0 ? 1 : 0, 1);
    }
    // p_offset literals (float math): axis0 = -p_scale*3543 + 55*20*3200/360,
    // axis1 = -p_scale*650 - 55*20*3200/360.
    CHECK_NEAR(cal[0].p_offset, -42711.11, 0.01);
    CHECK_NEAR(cal[1].p_offset, -19407.41, 0.01);

    // Feedback decode from L32233: round(word*pScale/8 + pOffset).
    // word 0 -> round(p_offset): -42711.11 -> -42711, -19407.41 -> -19407.
    CHECK_EQ(mp6::mp6_feedback_calibrated_steps(cal[0], 0.0), -42711);
    CHECK_EQ(mp6::mp6_feedback_calibrated_steps(cal[1], 0.0), -19407);
    // Feedback spot literals: word = PotLo (raw pot count at the docked
    // position) decodes to num17 - (7/8)*PotLo*pScale via float math:
    // axis0 = 9777.777 - 45927.777 = -36150, axis1 = -9777.777 - 8425.925.
    CHECK_EQ(mp6::mp6_feedback_calibrated_steps(cal[0], 3543.0), -36150);
    CHECK_EQ(mp6::mp6_feedback_calibrated_steps(cal[1], 650.0), -18204);
}

void testNetRound() {
    // .NET Math.Round ties-to-even.
    CHECK_EQ(mp6::mp6_net_round(0.5), 0);
    CHECK_EQ(mp6::mp6_net_round(1.5), 2);
    CHECK_EQ(mp6::mp6_net_round(2.5), 2);
    CHECK_EQ(mp6::mp6_net_round(-0.5), 0);
    CHECK_EQ(mp6::mp6_net_round(-1.5), -2);
    CHECK_EQ(mp6::mp6_net_round(-2.5), -2);
    CHECK_EQ(mp6::mp6_net_round(3.5), 4);
    CHECK_EQ(mp6::mp6_net_round(-3.5), -4);
    CHECK_EQ(mp6::mp6_net_round(2.4), 2);
    CHECK_EQ(mp6::mp6_net_round(2.6), 3);
    CHECK_EQ(mp6::mp6_net_round(-2.6), -3);
    CHECK_EQ(mp6::mp6_net_roundf(0.5f), 0.0f);
    CHECK_EQ(mp6::mp6_net_roundf(1.5f), 2.0f);
}

void testDesignTables() {
    mp6::Mp6DesignParams d;
    CHECK(mp6::mp6_design(2, &d));
    CHECK_NEAR(d.bot_a_mm, 840.0, 1e-12);
    CHECK_NEAR(d.bot_b_mm, 230.0, 1e-12);
    CHECK_NEAR(d.top_a_mm, 520.0, 1e-12);
    CHECK_NEAR(d.top_b_mm, 14.0, 1e-12);
    CHECK_NEAR(d.crank_length_mm, 230.0, 1e-12);
    CHECK_NEAR(d.off_x_mm, -25.0, 1e-12);
    CHECK_NEAR(d.pcdx_mm, -270.0, 1e-12);
    CHECK_NEAR(d.pcdy_mm, 90.0, 1e-12);
    CHECK_NEAR(d.rod_length_mm, 645.0, 1e-12);
    CHECK_NEAR(d.z_off_mm, 550.0, 1e-12);

    CHECK_EQ(mp6::mp6_design_from_name("MOT155-D"), 0);
    CHECK_EQ(mp6::mp6_design_from_name("MOT155-D1"), 1);
    CHECK_EQ(mp6::mp6_design_from_name("MOT155-D2"), 2);
    CHECK_EQ(mp6::mp6_design_from_name("2010+"), 3);
    CHECK_EQ(mp6::mp6_design_from_name("2010"), 4);
    CHECK_EQ(mp6::mp6_design_from_name("2009"), 5);
    CHECK_EQ(mp6::mp6_design_from_name("Manual"), 6);
    CHECK_EQ(mp6::mp6_design_from_name("MOT155"), -1);
    CHECK_EQ(mp6::mp6_design_from_name(""), -1);
    mp6::Mp6DesignParams manual;
    CHECK(!mp6::mp6_design(6, &manual)); // Manual: DGV-driven, no fixed table
    CHECK(!mp6::mp6_design(7, &manual));
    CHECK(!mp6::mp6_design(-1, &manual));

    // Design 0 spot values straight from the source switch.
    mp6::Mp6DesignParams d0;
    CHECK(mp6::mp6_design(0, &d0));
    CHECK_NEAR(d0.top_b_mm, 40.0, 1e-12);
    CHECK_NEAR(d0.rod_length_mm, 600.0, 1e-12);
    CHECK_NEAR(d0.z_off_mm, 521.0, 1e-12);

    // Planar attachments: independent transcription + analytic spot values
    // (420/cos30 pivots, 260*tan30 top lanes; y-flip carried in both).
    const PlanarRef ref = mot155d2Planar();
    const mp6::Mp6PlanarAttachments got = mp6::mp6_planar_attachments(d);

    CHECK_NEAR(got.base_x_mm[0], 420.0 / cos30(), 1e-9);
    CHECK_NEAR(got.base_y_mm[0], -230.0, 1e-9);
    CHECK_NEAR(got.top_x_mm[0], 260.0 * tan30() + 14.0 * sin_deg(120), 1e-9);
    CHECK_NEAR(got.top_y_mm[0], -(260.0 + 14.0 * cos_deg(120)), 1e-9);
    CHECK_NEAR(got.base_x_mm[2], -420.0 * tan30() + 230.0 * sin_deg(120), 1e-9);
    CHECK_NEAR(got.base_y_mm[2], -(-420.0 + 230.0 * cos_deg(120)), 1e-9);
    CHECK_NEAR(got.top_x_mm[3], -260.0 / cos30(), 1e-9);
    CHECK_NEAR(got.top_y_mm[3], 14.0, 1e-9);
    CHECK_NEAR(got.top_x_mm[4], -260.0 / cos30(), 1e-9);
    CHECK_NEAR(got.top_y_mm[4], -14.0, 1e-9);

    for (int i = 0; i < mp6::kAxes; ++i) {
        CHECK_NEAR(got.base_x_mm[i], ref.base_x[i], 1e-9);
        CHECK_NEAR(got.base_y_mm[i], ref.base_y[i], 1e-9);
        CHECK_NEAR(got.top_x_mm[i], ref.top_x[i], 1e-9);
        CHECK_NEAR(got.top_y_mm[i], ref.top_y[i], 1e-9);
    }
}

void testOffsets() {
    mp6::Mp6OffsetsInputs oi;
    oi.x_offset_cfg_mm = 0.0;
    oi.angle_offset_deg = 0.0;
    oi.crank_length_mm = 230.0;
    oi.design_z_off_mm = 550.0;
    oi.point_zero_x_mm = 0.0;
    oi.point_zero_z_mm = 0.0;
    const mp6::Mp6Offsets o = mp6::mp6_derive_offsets(oi);
    CHECK_NEAR(o.z_offset_mm, 530.0, 1e-12); // 550 - 0 - 20 + 0
    CHECK_NEAR(o.x_offset_mm, 0.0, 1e-12);

    mp6::Mp6OffsetsInputs oi2 = oi;
    oi2.point_zero_z_mm = 5.0;
    oi2.point_zero_x_mm = -7.0;
    const mp6::Mp6Offsets o2 = mp6::mp6_derive_offsets(oi2);
    CHECK_NEAR(o2.z_offset_mm, 535.0, 1e-12);
    CHECK_NEAR(o2.x_offset_mm, -7.0, 1e-12);

    mp6::Mp6OffsetsInputs oi3 = oi;
    oi3.angle_offset_deg = 10.0;
    const mp6::Mp6Offsets o3 = mp6::mp6_derive_offsets(oi3);
    CHECK_NEAR(o3.z_offset_mm,
               550.0 - std::sin(10.0 * kPi / 180.0) * 230.0 - 20.0, 1e-12);
}

void testPoseMapping() {
    const double pct[mp6::kAxes] = {10.0, -20.0, 5.0, 0.02, -0.03, 0.01};
    mp6::Mp6Pose pose;
    mp6::mp6_pose_from_pipeline_pct(pct, mp6::kPipelineToPoseChannels, &pose);
    // Surge -> lon(+X), Sway -> lat(+Y), Heave -> vert(+Z) by declaration.
    CHECK_NEAR(pose.lon_mm, 10.0, 1e-12);
    CHECK_NEAR(pose.lat_mm, -20.0, 1e-12);
    CHECK_NEAR(pose.vert_mm, 5.0, 1e-12);
    CHECK_NEAR(pose.roll, 0.02, 1e-15);
    CHECK_NEAR(pose.pitch, -0.03, 1e-15);
    CHECK_NEAR(pose.yaw, 0.01, 1e-15);

    CHECK_EQ(static_cast<int>(mp6::kPipelineToPoseChannels[0].component),
             static_cast<int>(mp6::kPoseLon));
    CHECK_EQ(static_cast<int>(mp6::kPipelineToPoseChannels[1].component),
             static_cast<int>(mp6::kPoseLat));
    CHECK_EQ(static_cast<int>(mp6::kPipelineToPoseChannels[2].component),
             static_cast<int>(mp6::kPoseVert));
    CHECK_EQ(static_cast<int>(mp6::kPoseLat), 0);
    CHECK_EQ(static_cast<int>(mp6::kPoseLon), 1);
    CHECK_EQ(static_cast<int>(mp6::kPoseVert), 2);
    CHECK_EQ(static_cast<int>(mp6::kPoseRoll), 3);
    CHECK_EQ(static_cast<int>(mp6::kPosePitch), 4);
    CHECK_EQ(static_cast<int>(mp6::kPoseYaw), 5);

    // Explicit scale/sign; no silent inheritance.
    mp6::Mp6PipelineChannelMapping map[mp6::kAxes] = {};
    map[0] = {mp6::kPoseLon, -2.0};                       // Surge inverted
    map[1] = {mp6::kPoseLat, 0.5};                        // Sway scaled
    map[2] = {mp6::kPoseVert, 1.0};
    map[3] = {mp6::kPoseRoll, kPi / 180.0};               // deg -> rad
    mp6::mp6_pose_from_pipeline_pct(pct, map, &pose);
    CHECK_NEAR(pose.lon_mm, -20.0, 1e-12);
    CHECK_NEAR(pose.lat_mm, -10.0, 1e-12);
    CHECK_NEAR(pose.vert_mm, 5.0, 1e-12);
    CHECK_NEAR(pose.roll, 0.02 * kPi / 180.0, 1e-15);
    CHECK_NEAR(pose.pitch, 0.0, 1e-15);                   // unmapped -> 0
    CHECK_NEAR(pose.yaw, 0.0, 1e-15);
}

void testTransform() {
    // Neutral pose with z != 0 round-trips the planar points.
    mp6::Mp6Pose neutral;
    std::array<mp6::Mp6Point3, mp6::kAxes> pts = {};
    pts[0] = {162.2354753445304, -253.0, 10.0};
    pts[3] = {-300.2222166246418, 14.0, 10.0};
    pts[4] = {-300.2222166246418, -14.0, 10.0};
    mp6::mp6_transform_points(pts, neutral, 0.0, 530.0);
    CHECK_NEAR(pts[0][0], 162.2354753445304, 1e-9);
    CHECK_NEAR(pts[0][1], -253.0, 1e-9);
    CHECK_NEAR(pts[0][2], 540.0, 1e-9);
    CHECK_NEAR(pts[3][2], 540.0, 1e-9);
    CHECK_NEAR(pts[4][2], 540.0, 1e-9);

    // AtanKD exact-port quirk: an exactly-zero first component yields angle
    // 0, not the quadrant-correct value (Module1 L314..L330). Point (-1,0,0)
    // with pitch = pi/2: the pitch stage calls atan_kd(z=0, x=-1) -> 0, so
    // x = sin(pi/2)*1 = 1 and z = cos(pi/2)*1 ~ 0. The stage order in
    // CalcPlatform_3 (L16792..L16812) is roll, then pitch, then yaw, and the
    // yaw stage then sees y == 0 and quirks again: atan_kd(y=0, x=1) -> 0
    // maps the (x, y) pair (1, 0) to (sin 0, cos 0) = (0, 1). The recovered
    // unit produces (0, 1, ~0) as well; it is honoured, not "fixed" to atan2.
    std::array<mp6::Mp6Point3, mp6::kAxes> quirky = {};
    quirky[0] = {-1.0, 0.0, 0.0};
    mp6::mp6_transform_points(quirky, poseWith(0, 0, 0, 0, kPi / 2.0, 0), 0.0,
                              0.0);
    CHECK_NEAR(quirky[0][0], 0.0, 1e-9);  // yaw stage collapsed (1, 0) -> (0, 1)
    CHECK_NEAR(quirky[0][1], 1.0, 1e-9);
    CHECK_NEAR(quirky[0][2], 0.0, 1e-9);  // cos(pi/2) * 1

    // At zero pose the transform is the identity as long as every stage's
    // first component is nonzero: roll/pitch use z, yaw uses y (this holds
    // for z == 0 quirk only when the stages still round-trip - all four sign
    // corners stay exact here).
    std::array<mp6::Mp6Point3, mp6::kAxes> id = {};
    id[0] = {1.0, 2.0, -3.0};
    id[1] = {-2.0, -3.0, 4.0};
    id[2] = {5.0, -1.0, 2.0};
    id[3] = {-4.0, 3.0, -6.0};
    mp6::mp6_transform_points(id, poseWith(0, 0, 0, 0, 0, 0), 0.0, 0.0);
    CHECK_NEAR(id[0][0], 1.0, 1e-9);
    CHECK_NEAR(id[0][1], 2.0, 1e-9);
    CHECK_NEAR(id[0][2], -3.0, 1e-9);
    CHECK_NEAR(id[2][0], 5.0, 1e-9);
    CHECK_NEAR(id[2][1], -1.0, 1e-9);
    CHECK_NEAR(id[2][2], 2.0, 1e-9);
    CHECK_NEAR(id[3][0], -4.0, 1e-9);
    CHECK_NEAR(id[3][1], 3.0, 1e-9);
    CHECK_NEAR(id[3][2], -6.0, 1e-9);

    // Translation order: y += lat, x += lon + X_Offset, z += vert + Z_Offset.
    std::array<mp6::Mp6Point3, mp6::kAxes> t = {};
    t[0] = {1.0, 2.0, 3.0};
    mp6::mp6_transform_points(t, poseWith(4.0, 5.0, 6.0, 0, 0, 0), 100.0,
                              200.0);
    CHECK_NEAR(t[0][0], 1.0 + 5.0 + 100.0, 1e-9);
    CHECK_NEAR(t[0][1], 2.0 + 4.0, 1e-9);
    CHECK_NEAR(t[0][2], 3.0 + 6.0 + 200.0, 1e-9);

    // Roll of (y,z)-pair (10, 0) by +pi/4 rotates via the sin/cos pair.
    std::array<mp6::Mp6Point3, mp6::kAxes> r = {};
    r[0] = {0.0, 10.0, 0.0};
    mp6::mp6_transform_points(r, poseWith(0, 0, 0, kPi / 4.0, 0, 0), 0.0, 0.0);
    CHECK_NEAR(r[0][1], 10.0 * sin_deg(45), 1e-9);
    CHECK_NEAR(r[0][2], 10.0 * cos_deg(45), 1e-9);
}

void testSolverZeroPose() {
    Fixture f;
    std::string err;
    CHECK(buildFixture(&f, &err));
    mp6::Mp6CrankSolver solver;
    CHECK(defaultSolver(solver, f, &err));
    std::array<double, mp6::kAxes> ats;
    angleToSteps(f, &ats);
    CHECK_EQ(solver.last_crank_pos(0), 1000);

    mp6::Mp6SolveResult res;
    solver.solve(targetsFor(f, mp6::Mp6Pose{}), ats, false, false, &res);
    for (int i = 0; i < mp6::kAxes; ++i) {
        CHECK(res.axis[i].ok);
        CHECK_EQ(res.axis[i].flags &
                     (mp6::kFlagLimitHigh | mp6::kFlagLimitLow |
                      mp6::kFlagResidual),
                 0u);
        CHECK(std::fabs(res.axis[i].residual_mm) < 1.5);
        CHECK(std::fabs(res.axis[i].angle_aim_rad) < 0.25);
        CHECK_EQ(res.calc_error & (1u << i), 0u);
        // RodCrankAngle diagnostic in range and nonzero (a ~ 646 mm lies in
        // the stored 415..875 span).
        CHECK(!res.axis[i].rod_crank_out_of_range);
        CHECK(res.axis[i].rod_crank_ang_deg > 0.0);
        CHECK(res.axis[i].rod_crank_ang_deg <= 180.0);
    }
    CHECK_EQ(res.calc_error, 0u);

    // Closed-form cross-check per axis: the achieved crank angle matches the
    // exact circle-line intersection; residual at the solution is tiny.
    const double crank_x_off[mp6::kAxes] = {5.235987755982989,
                                            4.1887902047863905,
                                            1.0471975511965976, 0.0,
                                            3.141592653589793,
                                            2.0943951023931953};
    const double crank_init[mp6::kAxes] = {3.141592653589793,
                                           0.0,
                                           3.141592653589793, 0.0,
                                           3.141592653589793, 0.0};
    for (int i = 0; i < mp6::kAxes; ++i) {
        double pivot[3] = {f.planar.base_x_mm[i], f.planar.base_y_mm[i],
                           kFixtureBaseZ};
        double target[3] = {f.planar.top_x_mm[i], f.planar.top_y_mm[i],
                            kFixtureTopZ + kFixtureZOffset};
        double aim = 0.0;
        CHECK(nearestZeroResidualAim(pivot, target,
                                     f.cfg.rod_length_mm.value,
                                     f.cfg.crank_length_mm.value,
                                     crank_x_off[i], crank_init[i], &aim));
        CHECK(std::fabs(aim - res.axis[i].angle_aim_rad) < 5e-3);
        CHECK(std::fabs(res.axis[i].residual_mm) < 0.02);
        CHECK(std::fabs(static_cast<double>(res.axis[i].step_target)) < 4096);
        CHECK(!mp6::mp6_step_word_reserved(res.axis[i].step_target));
    }

    const auto joint = mp6::Mp6CrankSolver::crank_joint_from_angle_aim(
        f.setup, 0, f.cfg.crank_length_mm.value, 0.125);
    const double angle = 0.125 + crank_init[0];
    CHECK_NEAR(joint[0], f.setup.base_pivot_mm[0][0] + f.cfg.crank_length_mm.value * std::cos(angle) * std::cos(crank_x_off[0]), 1e-9);
    CHECK_NEAR(joint[1], f.setup.base_pivot_mm[0][1] + f.cfg.crank_length_mm.value * std::cos(angle) * std::sin(crank_x_off[0]), 1e-9);
    CHECK_NEAR(joint[2], f.setup.base_pivot_mm[0][2] + f.cfg.crank_length_mm.value * std::sin(angle), 1e-9);
}

void testSolverMoves() {
    Fixture f;
    std::string err;
    CHECK(buildFixture(&f, &err));
    mp6::Mp6CrankSolver solver;
    CHECK(defaultSolver(solver, f, &err));
    std::array<double, mp6::kAxes> ats;
    angleToSteps(f, &ats);

    // Baseline neutral solve.
    mp6::Mp6SolveResult base;
    solver.solve(targetsFor(f, mp6::Mp6Pose{}), ats, false, false, &base);
    for (int i = 0; i < mp6::kAxes; ++i) {
        CHECK(base.axis[i].ok);
    }
    const std::int32_t base_step[mp6::kAxes] = {
        base.axis[0].step_target, base.axis[1].step_target,
        base.axis[2].step_target, base.axis[3].step_target,
        base.axis[4].step_target, base.axis[5].step_target};

    // Isolated DOF moves (all must stay solvable, residual within the
    // 0.0025 rad table quantisation).
    const mp6::Mp6Pose cases[] = {
        poseWith(20, 0, 0, 0, 0, 0),        // lat
        poseWith(0, 20, 0, 0, 0, 0),        // lon
        poseWith(0, 0, 20, 0, 0, 0),        // vert up
        poseWith(0, 0, -20, 0, 0, 0),       // vert down
        poseWith(0, 0, 0, 0.03, 0, 0),      // roll
        poseWith(0, 0, 0, 0, 0.03, 0),      // pitch
        poseWith(0, 0, 0, 0, 0, 0.05),      // yaw
        poseWith(10, 10, -15, 0.02, 0.02, 0.02), // combined
    };
    for (const mp6::Mp6Pose& c : cases) {
        mp6::Mp6SolveResult res;
        solver.solve(targetsFor(f, c), ats, false, false, &res);
        for (int i = 0; i < mp6::kAxes; ++i) {
            CHECK(res.axis[i].ok);
            CHECK(std::fabs(res.axis[i].residual_mm) < 2.5);
            CHECK_EQ(res.axis[i].flags & mp6::kFlagResidual, 0u);
        }
        CHECK_EQ(res.calc_error, 0u);
    }

    // Idempotence: the same pose twice -> identical targets.
    mp6::Mp6SolveResult idem1, idem2;
    solver.solve(targetsFor(f, cases[7]), ats, false, false, &idem1);
    solver.solve(targetsFor(f, cases[7]), ats, false, false, &idem2);
    for (int i = 0; i < mp6::kAxes; ++i) {
        CHECK_EQ(idem2.axis[i].step_target, idem1.axis[i].step_target);
    }

    // History transition: neutral -> lat-20 -> neutral must return the
    // aims to ~the baseline (quantisation tolerance ~ one scan step).
    solver.solve(targetsFor(f, cases[0]), ats, false, false, &idem1);
    mp6::Mp6SolveResult back;
    solver.solve(targetsFor(f, mp6::Mp6Pose{}), ats, false, false, &back);
    for (int i = 0; i < mp6::kAxes; ++i) {
        CHECK(back.axis[i].ok);
        CHECK_NEAR(static_cast<double>(back.axis[i].step_target),
                   static_cast<double>(base_step[i]), 30);
    }

    // A lateral move must move every target away from the neutral solution.
    solver.solve(targetsFor(f, mp6::Mp6Pose{}), ats, false, false, &back);
    mp6::Mp6SolveResult moved;
    solver.solve(targetsFor(f, poseWith(20, 0, 0, 0, 0, 0)), ats, false, false,
                 &moved);
    int changed = 0;
    for (int i = 0; i < mp6::kAxes; ++i) {
        if (moved.axis[i].step_target != base_step[i]) {
            ++changed;
        }
    }
    CHECK_EQ(changed, 6);
}

void testSolverLimitsAndRetention() {
    Fixture f;
    std::string err;
    CHECK(buildFixture(&f, &err));
    mp6::Mp6CrankSolver solver;
    CHECK(defaultSolver(solver, f, &err));
    std::array<double, mp6::kAxes> ats;
    angleToSteps(f, &ats);

    mp6::Mp6SolveResult base;
    solver.solve(targetsFor(f, mp6::Mp6Pose{}), ats, false, false, &base);
    for (int i = 0; i < mp6::kAxes; ++i) {
        CHECK(base.axis[i].ok);
    }
    const double aim0 = base.axis[0].angle_aim_rad;
    const std::int32_t step0 = base.axis[0].step_target;
    const int pos0 = solver.last_crank_pos(0);

    // Tight live limits (Module2.MinRadians/MaxRadians read per solve).
    solver.set_travel_limits(-0.002, 0.002);
    mp6::Mp6SolveResult tight;
    solver.solve(targetsFor(f, mp6::Mp6Pose{}), ats, false, false, &tight);
    bool any_high = false, any_low = false;
    for (int i = 0; i < mp6::kAxes; ++i) {
        CHECK(!tight.axis[i].ok);
        any_high = any_high || (tight.axis[i].flags & mp6::kFlagLimitHigh) != 0u;
        any_low = any_low || (tight.axis[i].flags & mp6::kFlagLimitLow) != 0u;
        CHECK_EQ(tight.calc_error & (1u << i), 1u << i);
    }
    CHECK(any_high);
    CHECK(any_low);
    CHECK_EQ(tight.calc_error, 0x3Fu);
    // Retention on failure: RodAngleAim/StepAim keep the previous values and
    // LastCrankPos is untouched.
    CHECK_NEAR(tight.axis[0].angle_aim_rad, aim0, 1e-12);
    CHECK_EQ(tight.axis[0].step_target, step0);
    CHECK_EQ(solver.last_crank_pos(0), pos0);

    // Docking bypasses limits entirely: same pose succeeds again.
    mp6::Mp6SolveResult dock;
    solver.solve(targetsFor(f, mp6::Mp6Pose{}), ats, true, false, &dock);
    for (int i = 0; i < mp6::kAxes; ++i) {
        CHECK(dock.axis[i].ok);
        CHECK((dock.axis[i].flags &
               (mp6::kFlagLimitHigh | mp6::kFlagLimitLow)) == 0u);
    }
    CHECK_EQ(dock.calc_error, 0u);

    // Test mode widens limits by 1.1x: a pose that fails normally passes
    // with the widened ceiling. Limits sit just under a positive aim.
    const double aim_mag = std::fabs(aim0);
    solver.set_travel_limits(-aim_mag / 1.05, aim_mag / 1.05);
    mp6::Mp6SolveResult normal_cut;
    solver.solve(targetsFor(f, mp6::Mp6Pose{}), ats, false, false,
                 &normal_cut);
    CHECK(!normal_cut.axis[0].ok);
    mp6::Mp6SolveResult test_cut;
    solver.solve(targetsFor(f, mp6::Mp6Pose{}), ats, false, true, &test_cut);
    CHECK(test_cut.axis[0].ok);

    // Unreachable pose (vert -280 mm pulls the targets too close): every
    // axis settles with residual >= 4 mm and raises its CalcError bit.
    solver.set_travel_limits(-0.9, 0.9);
    mp6::Mp6SolveResult far;
    solver.solve(targetsFor(f, poseWith(0, 0, -280, 0, 0, 0)), ats, false,
                 false, &far);
    for (int i = 0; i < mp6::kAxes; ++i) {
        CHECK(!far.axis[i].ok);
        CHECK((far.axis[i].flags & mp6::kFlagResidual) != 0u);
        CHECK_EQ(far.calc_error & (1u << i), 1u << i);
    }
    CHECK_EQ(far.calc_error, 0x3Fu);
    CHECK(std::fabs(far.axis[0].residual_mm) >= 4.0);
}

void testStepWordReservation() {
    CHECK(mp6::mp6_step_word_reserved(-32766));
    CHECK(!mp6::mp6_step_word_reserved(-32765));
    CHECK(!mp6::mp6_step_word_reserved(0));
    CHECK(!mp6::mp6_step_word_reserved(32766));
    // word = StepAim + 32767; word 1 is reserved (frm_Main L34168..L34170).
    CHECK_EQ(-32766 + 32767, 1);
}

void testSolverInitGuards() {
    Fixture f;
    std::string err;
    CHECK(buildFixture(&f, &err));

    mp6::Mp6CrankSolver solver;
    std::array<double, mp6::kAxes> rod_len;
    rod_len.fill(f.cfg.rod_length_mm.value);
    std::array<double, mp6::kAxes> zero;
    zero.fill(0.0);

    err = "";
    CHECK(!solver.init(f.setup, rod_len, f.cfg.crank_length_mm.value, 0.9,
                       -0.9, zero, &err)); // max <= min
    CHECK(!err.empty());

    err = "";
    auto bad_rod = rod_len;
    bad_rod[2] = -1.0;
    CHECK(!solver.init(f.setup, bad_rod, f.cfg.crank_length_mm.value, -0.9,
                       0.9, zero, &err));
    CHECK(!err.empty());

    err = "";
    auto bad_geom = f.setup;
    bad_geom.base_pivot_mm[4][2] = std::nan("");
    CHECK(!solver.init(bad_geom, rod_len, f.cfg.crank_length_mm.value, -0.9,
                       0.9, zero, &err));
    CHECK(!err.empty());

    // reset() forces a full rebuild and prior-solution reset.
    CHECK(defaultSolver(solver, f, &err));
    solver.reset();
    CHECK_EQ(solver.last_crank_pos(0), 1000);
}

} // namespace

int main() {
    testSnapshotImport();
    testConsumedParseFailures();
    testReadiness();
    testCalibration();
    testNetRound();
    testDesignTables();
    testOffsets();
    testPoseMapping();
    testTransform();
    testSolverZeroPose();
    testSolverMoves();
    testSolverLimitsAndRetention();
    testStepWordReservation();
    testSolverInitGuards();

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
