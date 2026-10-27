/*
 * MP6 platform geometry and rotary-crank motion core - implementation.
 * Source citations omit the directory prefix; every reference is to
 * motum_resources/motum-replacement-handoff/recovered-source/MotSim/.
 */

#include "mp6_geometry.h"

#include <cmath>

namespace mp6 {

namespace {

constexpr double kPi = 3.141592653589793;      // .NET Math.PI
constexpr double kDegToRad = kPi / 180.0;      // num2 = 1/(180/pi), frm_Design L602

// Module1.AtanKD (Module1.cs L314..L330), ported verbatim. This is NOT
// atan2: when the first component is exactly zero the recovered source
// returns 0 rather than the quadrant-correct angle.
double atan_kd(double x, double y) {
    if (x != 0.0) {
        double a = std::atan(y / x);
        if (x < 0.0) {
            a = kPi + a;
        }
        return a;
    }
    return 0.0;
}

// Shape points of the hexagon construction (CalcPlatformGeom_1 L741..L798).
struct HexRow {
    double x, y;
};

HexRow top_shape(const Mp6DesignParams& d, int k) {
    // array() for the top platform, frame k = 1..3 (L742..L756)
    switch (k) {
    case 1: return {(-d.top_a_mm) / 2.0 / std::cos(30.0 * kDegToRad), 0.0};
    case 2: return {d.top_a_mm / 2.0 * std::tan(30.0 * kDegToRad), (-d.top_a_mm) / 2.0};
    default: return {d.top_a_mm / 2.0 * std::tan(30.0 * kDegToRad), d.top_a_mm / 2.0};
    }
}

HexRow base_shape(const Mp6DesignParams& d, int k) {
    // array2() for the base platform, frame k = 1..3 (L771..L785)
    switch (k) {
    case 1: return {d.bot_a_mm / 2.0 / std::cos(30.0 * kDegToRad), 0.0};
    case 2: return {(-d.bot_a_mm) / 2.0 * std::tan(30.0 * kDegToRad), (-d.bot_a_mm) / 2.0};
    default: return {(-d.bot_a_mm) / 2.0 * std::tan(30.0 * kDegToRad), d.bot_a_mm / 2.0};
    }
}

double sin_deg180(int deg, double rad_per_deg = kDegToRad) {
    return std::sin(deg * rad_per_deg);
}

double cos_deg180(int deg, double rad_per_deg = kDegToRad) {
    return std::cos(deg * rad_per_deg);
}

} // namespace

// ---------------------------------------------------------------------------
// Design table
// ---------------------------------------------------------------------------

// Values transcribed from frm_Design.CalcPlatformGeom_1, switch (Typ) at
// L650..L740. Exact literals from the recovered source.
bool mp6_design(int typ, Mp6DesignParams* out) {
    if (out == nullptr) {
        return false;
    }
    Mp6DesignParams d;
    switch (typ) {
    case 0: // "MOT155-D"
        d = {840.0, 230.0, 520.0, 40.0, 230.0, -25.0, -270.0, 90.0, 600.0, 521.0};
        break;
    case 1: // "MOT155-D1"
        d = {840.0, 230.0, 520.0, 12.0, 230.0, -25.0, -270.0, 90.0, 620.0, 521.0};
        break;
    case 2: // "MOT155-D2"
        d = {840.0, 230.0, 520.0, 14.0, 230.0, -25.0, -270.0, 90.0, 645.0, 550.0};
        break;
    case 3: // "2010+"
        d = {746.0, 226.0, 480.0, 20.0, 200.0, 19.0, 232.0, 90.0, 570.0, 503.0};
        break;
    case 4: // "2010"
        d = {746.0, 226.0, 320.5, 20.0, 200.0, 19.0, 232.0, 90.0, 570.0, 506.0};
        break;
    case 5: // "2009"
        d = {580.0, 260.0, 320.5, 20.0, 200.0, 19.0, 232.0, 90.0, 570.0, 506.0};
        break;
    default:
        // Typ 6 ("Manual") has no fixed numbers: it reads the frm_Design DGV
        // (L724..L739). Callers must supply their own parameters.
        return false;
    }
    *out = d;
    return true;
}

int mp6_design_from_name(const std::string& name) {
    static const char* kNames[kAxes + 1] = {
        "MOT155-D", "MOT155-D1", "MOT155-D2", "2010+", "2010", "2009", "Manual",
    };
    if (name.empty()) {
        return -1;
    }
    for (int i = 0; i <= kAxes; ++i) {
        if (name == kNames[i]) {
            return i;
        }
    }
    return -1;
}

// PlatTop / PlatBase, CalcPlatformGeom_1 L757..L798, already carrying the
// Load_Component y-flip (frm_Main L16590..L16594).
Mp6PlanarAttachments mp6_planar_attachments(const Mp6DesignParams& design) {
    Mp6PlanarAttachments p{};
    const double top_b = design.top_b_mm;
    const double bot_b = design.bot_b_mm;

    // PlatTop (L757..L769): rows 1..6 use shape frames 3,2,2,1,1,3 with the
    // (sin, cos) angle pairs (120,120), (240,240), (240,240), (0,0), (0,0),
    // (120,120) and sign pairs +,+  -,-  +,+  -,-  +,+  -,-.
    const HexRow t3 = top_shape(design, 3); // array[3]
    const HexRow t2 = top_shape(design, 2); // array[2]
    const HexRow t1 = top_shape(design, 1); // array[1]

    p.top_x_mm[0] = t3.x + sin_deg180(120) * top_b; // PlatTop[1] (L758)
    p.top_y_mm[0] = t3.y + cos_deg180(120) * top_b;
    p.top_x_mm[1] = t2.x - sin_deg180(240) * top_b; // PlatTop[2] (L760)
    p.top_y_mm[1] = t2.y - cos_deg180(240) * top_b;
    p.top_x_mm[2] = t2.x + sin_deg180(240) * top_b; // PlatTop[3] (L762)
    p.top_y_mm[2] = t2.y + cos_deg180(240) * top_b;
    p.top_x_mm[3] = t1.x - sin_deg180(0) * top_b;   // PlatTop[4] (L764)
    p.top_y_mm[3] = t1.y - cos_deg180(0) * top_b;
    p.top_x_mm[4] = t1.x + sin_deg180(0) * top_b;   // PlatTop[5] (L766)
    p.top_y_mm[4] = t1.y + cos_deg180(0) * top_b;
    p.top_x_mm[5] = t3.x - sin_deg180(120) * top_b; // PlatTop[6] (L768)
    p.top_y_mm[5] = t3.y - cos_deg180(120) * top_b;

    const HexRow b1 = base_shape(design, 1); // array2[1]
    const HexRow b2 = base_shape(design, 2); // array2[2]
    const HexRow b3 = base_shape(design, 3); // array2[3]

    p.base_x_mm[0] = b1.x + sin_deg180(0) * bot_b;   // PlatBase[1] (L787)
    p.base_y_mm[0] = b1.y + cos_deg180(0) * bot_b;
    p.base_x_mm[1] = b1.x - sin_deg180(0) * bot_b;   // PlatBase[2] (L789)
    p.base_y_mm[1] = b1.y - cos_deg180(0) * bot_b;
    p.base_x_mm[2] = b2.x + sin_deg180(120) * bot_b; // PlatBase[3] (L791)
    p.base_y_mm[2] = b2.y + cos_deg180(120) * bot_b;
    p.base_x_mm[3] = b2.x - sin_deg180(120) * bot_b; // PlatBase[4] (L793)
    p.base_y_mm[3] = b2.y - cos_deg180(120) * bot_b;
    p.base_x_mm[4] = b3.x + sin_deg180(240) * bot_b; // PlatBase[5] (L795)
    p.base_y_mm[4] = b3.y + cos_deg180(240) * bot_b;
    p.base_x_mm[5] = b3.x - sin_deg180(240) * bot_b; // PlatBase[6] (L797)
    p.base_y_mm[5] = b3.y - cos_deg180(240) * bot_b;

    // Load_Component y-flip: num20 = j == 1 ? 1 : -1 (L16590).
    for (int i = 0; i < kAxes; ++i) {
        p.top_y_mm[i] = -p.top_y_mm[i];
        p.base_y_mm[i] = -p.base_y_mm[i];
    }
    return p;
}

// ---------------------------------------------------------------------------
// Offsets (Load_Component L16609..L16610)
// ---------------------------------------------------------------------------

Mp6Offsets mp6_derive_offsets(const Mp6OffsetsInputs& in) {
    Mp6Offsets o;
    o.x_offset_mm = in.x_offset_cfg_mm + in.point_zero_x_mm;
    o.z_offset_mm = in.design_z_off_mm -
                    std::sin(in.angle_offset_deg / (180.0 / kPi)) * in.crank_length_mm -
                    20.0 + in.point_zero_z_mm;
    return o;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

Mp6GeometrySetup mp6_geometry_setup(const Mp6PlanarAttachments& planar,
                                    const std::array<double, kAxes>& base_z_mm,
                                    const std::array<double, kAxes>& top_z_mm) {
    Mp6GeometrySetup g{};
    for (int i = 0; i < kAxes; ++i) {
        g.base_pivot_mm[i] = {planar.base_x_mm[i], planar.base_y_mm[i],
                              base_z_mm[i]};
        g.top_attachment_mm[i] = {planar.top_x_mm[i], planar.top_y_mm[i],
                                  top_z_mm[i]};
    }
    return g;
}

// ---------------------------------------------------------------------------
// Pose transform (CalcPlatform_3 L16792..L16815)
// ---------------------------------------------------------------------------

void mp6_transform_points(std::array<Mp6Point3, kAxes>& points,
                          const Mp6Pose& pose, double x_offset_mm,
                          double z_offset_mm) {
    for (int n = 0; n < kAxes; ++n) {
        double x = points[n][0];
        double y = points[n][1];
        double z = points[n][2];

        // Roll: rotate the (y, z) pair about the origin (L16795..L16800).
        double r = std::sqrt(y * y + z * z);
        double a = atan_kd(z, y);
        y = std::sin(a + pose.roll) * r;
        z = std::cos(a + pose.roll) * r;

        // Pitch: rotate the (x, z) pair (L16801..L16806).
        r = std::sqrt(x * x + z * z);
        a = atan_kd(z, x);
        x = std::sin(a + pose.pitch) * r;
        z = std::cos(a + pose.pitch) * r;

        // Yaw: rotate the (x, y) pair (L16807..L16812).
        r = std::sqrt(x * x + y * y);
        a = atan_kd(y, x);
        x = std::sin(a + pose.yaw) * r;
        y = std::cos(a + pose.yaw) * r;

        // Translation (L16813..L16815). lat -> +Y, lon -> +X, vert -> +Z.
        points[n] = {x + pose.lon_mm + x_offset_mm, y + pose.lat_mm,
                     z + pose.vert_mm + z_offset_mm};
    }
}

void mp6_pose_from_pipeline_pct(const double pct[kAxes],
                                const Mp6PipelineChannelMapping map[kAxes],
                                Mp6Pose* pose_out) {
    if (pose_out == nullptr) {
        return;
    }
    double fields[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int ch = 0; ch < kAxes; ++ch) {
        const int comp = static_cast<int>(map[ch].component);
        if (comp >= 0 && comp < 6) {
            fields[comp] += pct[ch] * map[ch].scale;
        }
    }
    pose_out->lat_mm = fields[kPoseLat];
    pose_out->lon_mm = fields[kPoseLon];
    pose_out->vert_mm = fields[kPoseVert];
    pose_out->roll = fields[kPoseRoll];
    pose_out->pitch = fields[kPosePitch];
    pose_out->yaw = fields[kPoseYaw];
}

// ---------------------------------------------------------------------------
// Rotary crank solver (MotorPos_6DOF_C)
// ---------------------------------------------------------------------------

namespace {

// Solver constants shared by the module; num10 is hard-coded in the source
// (L17576), the table spans +-2.475 rad around CrankInit (L18138), and the
// scan span RStep is computed once at Motion_Init (L17900).
constexpr double kRadPerCount = 0.0025;     // num10
constexpr int kCountLo = 10;                // index clamp floor (L18225)
constexpr int kCountHi = 1990;              // index clamp ceiling (L18213)
constexpr int kResidualAcceptMm = 4;        // |value| < 4 mm acceptance
constexpr int kIndexJumpLimit = 20;         // num12 counter threshold (L17609)

double residual_at_axis(const std::array<double, 2001>& xc,
                        const std::array<double, 2001>& yc,
                        const std::array<double, 2001>& zc, int idx,
                        const double t_x, const double t_y, const double t_z,
                        double rod_length) {
    const double dx = xc[idx] - t_x;
    const double dy = yc[idx] - t_y;
    const double dz = zc[idx] - t_z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) - rod_length;
}

} // namespace

void Mp6CrankSolver::build_tables() {
    // Motion_Init block (L18015..L18187): rebuild every per-axis table and
    // reset the prior solution. Runs on first solve and after reset().
    xxc_ = {};
    yyc_ = {};
    zzc_ = {};
    rod_crank_angle_ = {};

    for (int i = 0; i < kAxes; ++i) {
        // num4 = 1..6 mass loop (L18106..L18131). Pivot positions were taken
        // from Coord[1, num4+1, 1..3] (the base pivot rows) at init().
        caosin_[i] = std::sin(kCrankXOffset[i]);   // L18122
        caocos_[i] = std::cos(kCrankXOffset[i]);   // L18126
        last_crank_pos_[i] = 1000;                 // L18130
        for (int idx = 0; idx <= 2000; ++idx) {
            const double ang =
                (static_cast<double>(idx) - 1000.0) * kRadPerCount +
                kCrankInit[i];                     // num17 (L18138)
            xxc_[i][idx] = pivot_[i][0] + crank_length_ * std::cos(ang) * caocos_[i];
            yyc_[i][idx] = pivot_[i][1] + crank_length_ * std::cos(ang) * caosin_[i];
            zzc_[i][idx] = pivot_[i][2] + crank_length_ * std::sin(ang);
        }
        // RodCrankAngle diagnostic table (L18162..L18187): index = pivot-to-
        // target distance in mm, value = acos(...) * (180/pi) * 10 (deg*10).
        const int lo = static_cast<int>(mp6_net_round(rod_length_[i] - crank_length_));
        const int hi = static_cast<int>(mp6_net_round(rod_length_[i] + crank_length_));
        for (int d = lo; d <= hi; ++d) {
            if (d < 0 || d > 2000) {
                continue; // source would throw only outside 0..2000
            }
            const double c =
                (rod_length_[i] * rod_length_[i] + crank_length_ * crank_length_ -
                 static_cast<double>(d) * static_cast<double>(d)) /
                (2.0 * rod_length_[i] * crank_length_);
            rod_crank_angle_[i][d] = static_cast<int>(mp6_net_round(
                std::acos(c) * (180.0 / kPi) * 10.0));
        }
    }
}

Mp6Point3 Mp6CrankSolver::crank_joint_from_angle_aim(
    const Mp6GeometrySetup& geometry, int axis, double crank_length_mm,
    double angle_aim_plus_zero_rad) {
    const double angle = angle_aim_plus_zero_rad + kCrankInit[axis];
    const double caos = std::sin(kCrankXOffset[axis]);
    const double caoc = std::cos(kCrankXOffset[axis]);
    return {geometry.base_pivot_mm[axis][0] + crank_length_mm * std::cos(angle) * caoc,
            geometry.base_pivot_mm[axis][1] + crank_length_mm * std::cos(angle) * caos,
            geometry.base_pivot_mm[axis][2] + crank_length_mm * std::sin(angle)};
}

bool Mp6CrankSolver::init(const Mp6GeometrySetup& geometry,
                          const std::array<double, kAxes>& rod_length_mm,
                          double crank_length_mm, double min_rad, double max_rad,
                          const std::array<double, kAxes>& angle_zero_rad,
                          std::string* err) {
    if (err != nullptr) {
        *err = {};
    }
    auto finite_all = [](const std::array<Mp6Point3, kAxes>& pts) {
        for (int i = 0; i < kAxes; ++i) {
            for (double v : pts[i]) {
                if (!std::isfinite(v)) {
                    return false;
                }
            }
        }
        return true;
    };
    bool bad = false;
    if (!finite_all(geometry.base_pivot_mm) || !finite_all(geometry.top_attachment_mm)) {
        bad = true;
    }
    for (int i = 0; i < kAxes && !bad; ++i) {
        if (!std::isfinite(rod_length_mm[i]) || !std::isfinite(angle_zero_rad[i]) ||
            rod_length_mm[i] <= 0.0) {
            bad = true;
        }
    }
    if (!std::isfinite(crank_length_mm) || crank_length_mm < 0.0 ||
        !std::isfinite(min_rad) || !std::isfinite(max_rad) || max_rad <= min_rad) {
        bad = true;
    }
    if (bad) {
        if (err != nullptr) {
            *err = "mp6 geometry setup has unusable values: caller must resolve "
                   "the unrecovered Other\\<Platform>.txt pivot z coordinates and "
                   "the snapshot rod/crank lengths before solving";
        }
        return false;
    }

    pivot_ = geometry.base_pivot_mm;
    rod_length_ = rod_length_mm;
    rod_angle_zero_ = angle_zero_rad;
    crank_length_ = crank_length_mm;
    min_rad_ = min_rad;
    max_rad_ = max_rad;
    // RStep = round((MaxRadians - MinRadians) / 2 / num10) (L17900).
    r_step_ = static_cast<int>(
        mp6_net_round((max_rad_ - min_rad_) / 2.0 / kRadPerCount));
    // LastCrankPos = 1000 (L18130). build_tables() re-seeds it when the
    // Motion_Init gate runs on the first solve; seeding here too just makes
    // the accessor observe 1000 before that first solve (the recovered
    // statics would still be 0 there - observable difference documented in
    // the parity report).
    for (int i = 0; i < kAxes; ++i) {
        last_crank_pos_[i] = 1000;
    }
    motion_init_ = false; // tables rebuilt on next solve (L18015)
    return true;
}

void Mp6CrankSolver::reset() {
    motion_init_ = false; // L18004..L18012
    for (int i = 0; i < kAxes; ++i) {
        last_crank_pos_[i] = 1000; // build_tables L18130 re-seeds on next solve
    }
}

void Mp6CrankSolver::solve(const std::array<Mp6Point3, kAxes>& target_mm,
                           const std::array<double, kAxes>& angle_to_step,
                           bool docking, bool test_mode, Mp6SolveResult* out) {
    if (out == nullptr) {
        return;
    }
    *out = Mp6SolveResult{};

    if (!motion_init_) {
        motion_init_ = true;      // L18022
        build_tables();           // L18026..L18187
    }

    for (int i = 0; i < kAxes; ++i) {
        const int n = i + 1; // source 1-based axis number (num4)

        // L18198: RodCalcOK[num4] = true
        bool ok = true;
        std::uint32_t flags = 0;

        const double t_x = target_mm[i][0]; // num6 (L18202)
        const double t_y = target_mm[i][1]; // num7 (L18206)
        const double t_z = target_mm[i][2]; // num8 (L18210)

        // Prior index clamp IN PLACE (L18214..L18232).
        int last = last_crank_pos_[i];
        if (last > kCountHi) {
            last = kCountHi;
        }
        if (last < kCountLo) {
            last = kCountLo;
        }
        last_crank_pos_[i] = last;

        // Residuals at the prior index and index+1 (L18236..L18240).
        double num20 = residual_at_axis(xxc_[i], yyc_[i], zzc_[i], last, t_x, t_y,
                                        t_z, rod_length_[i]);
        double num21 = residual_at_axis(xxc_[i], yyc_[i], zzc_[i], last + 1, t_x,
                                        t_y, t_z, rod_length_[i]);

        // Direction pick by the smaller of the two (L18244..L18287).
        double num22;  // best residual so far
        int num23;     // first scan index
        int num24;     // last scan index
        int num25;     // direction
        int num11;     // best index
        if (std::fabs(num21) < std::fabs(num20)) {
            num22 = num21;                       // L18251
            num23 = last + 2;                    // L18255
            num24 = num23 + r_step_;             // L18259
            if (num24 > kCountHi) {
                num24 = kCountHi;                // L18270
            }
            num25 = 1;                           // L18274
            num11 = num23 - 1;                   // L18278
        } else {
            num22 = num20;                       // L18282
            num23 = last - 1;                    // L18286
            num24 = num23 - r_step_;             // L18290
            if (num24 < kCountLo) {
                num24 = kCountLo;                // L18301
            }
            num25 = -1;                          // L18305
            num11 = num23 + 1;                   // L18309
        }
        double num16 = std::fabs(num22);         // L18313

        // Scan loop (L18323..L18349). The VB signed-compare trick
        // ((num25 >> 31) ^ num13) <= ((num25 >> 31) ^ num24) is spelled out
        // here as an ascending / descending while.
        double num15 = 0.0; // last evaluated residual (C# default 0)
        for (int num13 = num23;
             num25 > 0 ? num13 <= num24 : num13 >= num24; num13 += num25) {
            num15 = residual_at_axis(xxc_[i], yyc_[i], zzc_[i], num13, t_x, t_y,
                                     t_z, rod_length_[i]);
            if (std::fabs(num15) < std::fabs(num22)) {   // L18334
                num22 = num15;                           // L18341
                num11 = num13;                           // L18345
            }
            if (std::fabs(num15) > num16) {              // L18349 early exit
                break;
            }
            num16 = std::fabs(num15);                    // L18350..L18352
        }

        // Fractional index (L18355..L18367). num15 == num22 (including the
        // degenerate unevaluated case) takes the integer form.
        double num28;
        if (num15 != num22) {
            num28 = static_cast<double>(num11) - num22 / (num15 - num22) *
                                                  static_cast<double>(num25);
        } else {
            num28 = static_cast<double>(num11);
        }

        // Crank angle at the found index (L18371).
        double num5 = (num28 - 1000.0) * kRadPerCount + kCrankInit[i];

        // Limits, skipped when Docking (L18375..L18431).
        if (!docking) {
            const double lim_hi = test_mode ? max_rad_ * 1.1 : max_rad_;
            const double lim_lo = test_mode ? min_rad_ * 1.1 : min_rad_;
            const double aim_raw = (num28 - 1000.0) * kRadPerCount;
            if (aim_raw > lim_hi) {
                num28 = static_cast<double>((last > 1000) - (last < 1000)) *
                            lim_hi / kRadPerCount + 1000.0;
                ok = false;
                flags |= kFlagLimitHigh;
            } else if (aim_raw < lim_lo) {
                num28 = static_cast<double>((last > 1000) - (last < 1000)) *
                            (0.0 - lim_lo) / kRadPerCount + 1000.0;
                ok = false;
                flags |= kFlagLimitLow;
            }
        }

        // L18433..L18451: when the axis is already failing (limit clamp),
        // num5 is recomputed from the CLAMPED num28 and
        // CrankCurrent/LastCrankAngle update without LastCrankPos.
        if (!ok) {
            num5 = (num28 - 1000.0) * kRadPerCount + kCrankInit[i];
        }

        // Crank joint + residual gate (L18435..L18439, L17215..L17620).
        const Mp6Point3 crank_joint = crank_joint_from_angle_aim(
            Mp6GeometrySetup{pivot_, {}}, i, crank_length_, num5 - kCrankInit[i]);
        double xc = crank_joint[0];
        double yc = crank_joint[1];
        double zc = crank_joint[2];
        const double dx = xc - t_x;
        const double dy = yc - t_y;
        const double dz = zc - t_z;
        double value = std::sqrt(dx * dx + dy * dy + dz * dz) - rod_length_[i];
        if (!(std::fabs(value) < static_cast<double>(kResidualAcceptMm))) {
            ok = false;
            flags |= kFlagResidual;
        }

        // num12 jump counter only survives as a diagnostic here (L17609).
        if (std::fabs(static_cast<double>(num11) - static_cast<double>(last)) >
            static_cast<double>(kIndexJumpLimit)) {
            flags |= kFlagIndexJump;
        }

        // IL_1081..IL_1092: CrankCurrent/LastCrankAngle update on both paths.
        last_crank_angle_[i] = num5;

        // IL_10b7..IL_1165: joint output + RodAngleAim assigned ONLY on
        // success; failures add the CalcError bit instead (L17838..L17845).
        Mp6AxisSolveResult& ar = out->axis[i];
        if (ok) {
            last_crank_pos_[i] = num11; // L17807
            ar.crank_joint_mm[0] = xc;
            ar.crank_joint_mm[1] = yc;
            ar.crank_joint_mm[2] = zc;
            angle_aim_[i] = num5 - kCrankInit[i] - rod_angle_zero_[i]; // L17803/17844
        } else {
            out->calc_error |= (1u << i); // CalcError += 2^(num4-1)
        }
        ar.ok = ok;
        ar.flags = flags;
        ar.crank_angle_rad = num5;
        ar.angle_aim_rad = angle_aim_[i];
        ar.residual_mm = value;
        // CalcPlatform_3 L16823: StepAim = round(RodAngleAim * AngleToStep)
        // is assigned for EVERY axis, including stale aims after failures.
        ar.step_target = static_cast<std::int32_t>(
            mp6_net_round(ar.angle_aim_rad * angle_to_step[i]));

        // IL_11d6..IL_1251: RodCrankAngCurr diagnostic. Index = rounded
        // pivot-to-target distance; in-bounds-but-unstored entries are 0
        // exactly as in the source; outside 0..2000 the VB error trap fired
        // (CalcError 0x40) - this port flags instead.
        const double a = std::sqrt((pivot_[i][0] - t_x) * (pivot_[i][0] - t_x) +
                                   (pivot_[i][1] - t_y) * (pivot_[i][1] - t_y) +
                                   (pivot_[i][2] - t_z) * (pivot_[i][2] - t_z));
        const int idx_a = static_cast<int>(mp6_net_round(a));
        if (idx_a >= 0 && idx_a <= 2000) {
            ar.rod_crank_ang_deg =
                static_cast<double>(rod_crank_angle_[i][idx_a]) * 0.1;
            ar.rod_crank_out_of_range = false;
        } else {
            ar.rod_crank_out_of_range = true;
        }
        (void)n;
    }
}

} // namespace mp6
