#pragma once
/*
 * MP6 platform geometry and rotary-crank motion core.
 *
 * Pure C++17 core (namespace mp6). No Win32, no UI, no filesystem access, no
 * hardware. Geometry and solver semantics are ported verbatim from the
 * recovered MotSim C# sources:
 *
 *   frm_Main.Load_Component (L16424..L16633):
 *     planar attachment load + y-flip + X_Offset/Z_Offset derivation
 *   frm_Design.CalcPlatformGeom_1 (frm_Design.cs L591..L881):
 *     design table (MOT155-D .. Manual) and PlatBase/PlatTop planar points
 *   frm_Main.CalcPlatform_3 (L16635..L16954):
 *     pose transform (roll -> pitch -> yaw, AtanKD-exact) and StepAim rounding
 *   frm_Main.MotorPos_6DOF_C (L17141..L18472):
 *     rotary crank actuator solver (table scan + interpolation + limits)
 *   Module1.AtanKD (Module1.cs L314..L330):
 *     quadrant behaviour ported EXACTLY (differs from atan2 at x == 0)
 *   Module2 static types (Module2.cs): RodLength/CrankLength double,
 *     RodAngle_Zero double[7] (only ever zero-initialised at frm_Main L9238)
 *
 * NOT invented here: pivot z coordinates come from the unrecovered
 * Other\<Platform>.txt points file (Load_Component L16666: Coord[num7, num8, 3]
 * is loaded, rows stay untouched by CalcPlatformGeom_1); the caller supplies
 * them explicitly. RodLength/CrankLength ARE overridden from the pltc header
 * by Load_Coefficients (frm_Main L18655..L18663) after Load_Component copies
 * the design values; the caller decides which source to use.
 */

#include <array>
#include <cstdint>
#include <string>

#include "mp6_config.h" // kAxes, mp6_net_round (shared .NET rounding policy)

namespace mp6 {

// ---------------------------------------------------------------------------
// Pose
// ---------------------------------------------------------------------------

// One motion pose in MP6/recovered-source physical units. Convention fixed by
// CalcPlatform_3 L16748..L16815:
//   lat_mm  -> +Y translation        lon_mm -> +X translation (plus X_Offset)
//   vert_mm -> +Z translation (plus Z_Offset)
//   roll/pitch/yaw in radians, applied roll -> pitch -> yaw about the origin
//   (Point_Current[1] which is all zeros, L16689/L16771).
struct Mp6Pose {
    double lat_mm = 0.0;
    double lon_mm = 0.0;
    double vert_mm = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

// ---------------------------------------------------------------------------
// Design table (frm_Design.CalcPlatformGeom_1)
// ---------------------------------------------------------------------------

// Design types in optDesign order (frm_Design L615..L633):
//   0 MOT155-D, 1 MOT155-D1, 2 MOT155-D2, 3 "2010+", 4 "2010", 5 "2009",
//   6 Manual.
struct Mp6DesignParams {
    double bot_a_mm = 0.0;         // BotA
    double bot_b_mm = 0.0;         // BotB
    double top_a_mm = 0.0;         // TopA
    double top_b_mm = 0.0;         // TopB
    double crank_length_mm = 0.0;  // Module2.CrankL (design value)
    double off_x_mm = 0.0;         // OffX (motor location offset, visualisation)
    double pcdx_mm = 0.0;          // PCDX
    double pcdy_mm = 0.0;          // PCDY
    double rod_length_mm = 0.0;    // Module2.RodL (design value)
    double z_off_mm = 0.0;         // Module2.ZOff (feeds Z_Offset derivation)
};

// Returns false for typ outside 0..6 (the Manual design 6 reads the DGV and
// has no fixed numbers here; callers must supply their own Mp6DesignParams).
bool mp6_design(int typ, Mp6DesignParams* out);

// Name lookup: "MOT155-D" .. "Manual"; returns 0..6 or -1 when not found.
// (CompareString exact-match semantics of frm_Design L614..L639.)
int mp6_design_from_name(const std::string& name);

// Planar attachment coordinates AFTER the Load_Component y-flip
// (frm_Main L16584..L16600: Coord[1, i+1, j] = PlatBase[i, j-1] * (j == 1 ?
// 1 : -1), same for PlatTop). Axis index 0..5 = source axes 1..6.
struct Mp6PlanarAttachments {
    std::array<double, kAxes> base_x_mm;  // Coord[1, i+1, 1]
    std::array<double, kAxes> base_y_mm;  // Coord[1, i+1, 2] = -PlatBase[i,1]
    std::array<double, kAxes> top_x_mm;   // Coord[2, i+1, 1]
    std::array<double, kAxes> top_y_mm;   // Coord[2, i+1, 2] = -PlatTop[i,1]
};

// Transcribed verbatim from CalcPlatformGeom_1 L741..L798 (30-degree hexagon
// layout). Only the planar (x, y) values; z stays caller-supplied.
Mp6PlanarAttachments mp6_planar_attachments(const Mp6DesignParams& design);

// ---------------------------------------------------------------------------
// Offsets (Load_Component L16609..L16610)
// ---------------------------------------------------------------------------

struct Mp6OffsetsInputs {
    double x_offset_cfg_mm = 0.0;   // pltc "X Offset"    -> DGVData("X Offset", 0)
    double angle_offset_deg = 0.0;  // pltc "Angle Offset"-> Module2.AngOffset (L18654)
    double crank_length_mm = 0.0;   // pltc "Crank Length" override (L18655) or
                                    // design CrankL (L16608); caller picks source
    double design_z_off_mm = 0.0;   // design table Module2.ZOff
    double point_zero_x_mm = 0.0;   // Module2.lblX (MotConv "X Point Zero")
    double point_zero_z_mm = 0.0;   // Module2.lblZ (MotConv "Z Point Zero")
};

struct Mp6Offsets {
    double x_offset_mm = 0.0;  // X_Offset = Val(X Offset) + lblX
    double z_offset_mm = 0.0;  // Z_Offset = ZOff - sin(AngOffset in rad) *
                               //            CrankLength - 20 + lblZ
};

Mp6Offsets mp6_derive_offsets(const Mp6OffsetsInputs& in);

// ---------------------------------------------------------------------------
// Full geometry setup
// ---------------------------------------------------------------------------

// Base pivot (crank rotation axis position) and top attachment points in
// base frame, millimetres. z coordinates come from the unrecovered
// Other\<Platform>.txt file - the caller must supply them; this module never
// invents them.
using Mp6Point3 = std::array<double, 3>;  // x, y, z

struct Mp6GeometrySetup {
    std::array<Mp6Point3, kAxes> base_pivot_mm;      // Coord[1, i+1, 1..3]
    std::array<Mp6Point3, kAxes> top_attachment_mm;  // Point_Initial[2, i+1, 1..3]
};

Mp6GeometrySetup mp6_geometry_setup(const Mp6PlanarAttachments& planar,
                                    const std::array<double, kAxes>& base_z_mm,
                                    const std::array<double, kAxes>& top_z_mm);

// ---------------------------------------------------------------------------
// Pose transform ( frm_Main.CalcPlatform_3 L16792..L16815 )
// ---------------------------------------------------------------------------

// Applies roll (rotate (y,z) pair), then pitch (rotate (x,z) pair), then yaw
// (rotate (x,y) pair) about the origin using the EXACT AtanKD quadrant
// semantics (atan(y/x) with +pi when x < 0, 0 when x == 0 - NOT atan2), then
// translates: y += lat, x += lon + X_Offset, z += vert + Z_Offset.
// In source the same loop covers Point_Current rows 2..9; here the caller
// passes whatever point set it needs (for the solver: the 6 top attachments).
void mp6_transform_points(std::array<Mp6Point3, kAxes>& points,
                          const Mp6Pose& pose, double x_offset_mm,
                          double z_offset_mm);

// ---------------------------------------------------------------------------
// Pipeline channel mapping (declaration table only - verify before hardware)
//
// Upstream motion pipeline channels arrive in the order
//   [Surge, Sway, Heave, Roll, Pitch, Yaw]
// and map onto the MP6 pose as
//   Surge -> lon (+X), Sway -> lat (+Y), Heave -> vert (+Z),
//   Roll/Pitch/Yaw unchanged.
// Scale and sign are EXPLICIT per channel; nothing is silently inherited from
// the ESP32 firmware's surge/sway handling. integrators must verify the scales
// against the sending pipeline before any live send.
// ---------------------------------------------------------------------------

enum Mp6PoseComponent : int {
    kPoseLat = 0,   // +Y mm
    kPoseLon = 1,   // +X mm
    kPoseVert = 2,  // +Z mm
    kPoseRoll = 3,  // rad
    kPosePitch = 4, // rad
    kPoseYaw = 5    // rad
};

struct Mp6PipelineChannelMapping {
    Mp6PoseComponent component;
    double scale;  // pose value = pipeline value * scale (sign folded in)
};

// Default declaration: order-preserving map to MP6 components, unit scale.
inline constexpr Mp6PipelineChannelMapping kPipelineToPoseChannels[kAxes] = {
    {kPoseLon, 1.0},   // Surge
    {kPoseLat, 1.0},   // Sway
    {kPoseVert, 1.0},  // Heave
    {kPoseRoll, 1.0},  // Roll
    {kPosePitch, 1.0}, // Pitch
    {kPoseYaw, 1.0},   // Yaw
};

// pose_out[i-th field] = sum over channels mapped to that component of
// pct[channel] * map[channel].scale. Fields with no mapped channel stay 0.
void mp6_pose_from_pipeline_pct(const double pct[kAxes],
                                const Mp6PipelineChannelMapping map[kAxes],
                                Mp6Pose* pose_out);

// ---------------------------------------------------------------------------
// Rotary crank solver (frm_Main.MotorPos_6DOF_C L17141..L18472)
// ---------------------------------------------------------------------------

// Step target bookkeeping for the command-2 path. The board reserves
// encoded word 1; word = StepAim + 32767, so word == 1 exactly when
// StepAim == -32766 (recovered packing frm_Main L34168..L34170).
inline constexpr std::int32_t kStepTargetReserved = -32766;
inline constexpr bool mp6_step_word_reserved(std::int32_t step_target) {
    return step_target == kStepTargetReserved;
}

enum Mp6AxisFlag : std::uint32_t {
    kFlagNone = 0,
    kFlagLimitHigh = 1u << 0,  // rod angle above MaxRadians (x1.1 in test mode)
    kFlagLimitLow = 1u << 1,   // below MinRadians
    kFlagResidual = 1u << 2,   // |dist(crank joint, target) - RodLength| >= 4 mm
    kFlagOutOfRange = 1u << 3, // pivot distance outside the RodCrankAngle
                               // 0..2000 array (VB would have thrown bit 0x40;
                               // this port flags and continues instead)
    kFlagIndexJump = 1u << 4,  // |best index - prior index| > 20 (source
                               // importance: num12 counter L17609, unused)
};

struct Mp6AxisSolveResult {
    bool ok = false;              // Module2.RodCalcOK[n] after the call
    std::uint32_t flags = 0;      // Mp6AxisFlag bits
    double crank_angle_rad = 0.0; // num5 (LastCrankAngle[n]; updated even when
                                  // the axis fails - source parity)
    double angle_aim_rad = 0.0;   // RodAngleAim[n]. RETAINED from the previous
                                  // solve when this axis failed (source
                                  // assigns it only on success, L1143 block).
    std::int32_t step_target = 0; // StepAim = round(angle_aim * AngleToStep)
                                  //(.NET half-even, CalcPlatform_3 L16823);
                                  // always assigned, including stale aims.
    double residual_mm = 0.0;     // signed value = |crank joint - target| -
                                  // RodLength for the finally used num5
    double crank_joint_mm[3] = {0.0, 0.0, 0.0}; // Coord[1, num4+7, 1..3]
                                                // (valid only when ok)
    double rod_crank_ang_deg = 0.0; // RodCrankAngCurr[n] diagnostic; 0 when
                                    // unstored, flagged when index out of
                                    // range (source traps, we flag)
    bool rod_crank_out_of_range = false;
};

struct Mp6SolveResult {
    Mp6AxisSolveResult axis[kAxes];
    std::uint32_t calc_error = 0;      // bit (axis-1 source numbering) per
                                       // failing axis, CalcError semantics
                                       // L17840 (2^(n-1)); 0x40 legacy trap
                                       // from the VB handler is NOT set.
};

// Per-entity solver. Carries the prior-solution state that MotorPos_6DOF_C
// keeps in module/form statics (LastCrankPos, LastCrankAngle static locals,
// RodAngleAim, calculated tables, Motion_Init).
class Mp6CrankSolver {
public:
    Mp6CrankSolver() = default;

    // Forward crank geometry for the solver's angle-aim + zero convention.
    static Mp6Point3 crank_joint_from_angle_aim(const Mp6GeometrySetup& geometry,
                                                int axis, double crank_length_mm,
                                                double angle_aim_plus_zero_rad);

    // Builds the per-axis crank tables and resets the prior solution.
    //   geometry        : base pivots (crank axes) and top attachments (mm)
    //   rod_length_mm   : per-axis RodLength (Module2.RodLength, double)
    //   crank_length_mm : Module2.CrankLength
    //   min/max_rad     : Module2.MinRadians/MaxRadians (travel limits)
    //   angle_zero_rad  : RodAngle_Zero (always zeros in recovered source,
    //                     frm_Main L9238 - kept as an explicit input)
    // Returns false with *err set for unusable inputs (non-finite size,
    // rod <= 0, crank < 0, max_rad <= min_rad). Never invents replacements.
    bool init(const Mp6GeometrySetup& geometry,
              const std::array<double, kAxes>& rod_length_mm,
              double crank_length_mm, double min_rad, double max_rad,
              const std::array<double, kAxes>& angle_zero_rad, std::string* err);

    // Mirrors MotorPos_6DOF_C(Reset = true): the geometry/limit tables and the
    // prior crank positions are rebuilt on the next solve.
    void reset();

    // Live travel limits. In the recovered source Module2.MinRadians /
    // MaxRadians are globals read on every solve, while the scan span RStep
    // is computed once at Motion_Init from the limits known then. init()
    // seeds both; this setter updates the solve-time limits only. Applies
    // only when max_rad > min_rad (parsed limits must straddle sensibly).
    void set_travel_limits(double min_rad, double max_rad) {
        if (max_rad > min_rad) {
            min_rad_ = min_rad;
            max_rad_ = max_rad;
        }
    }

    // One CalcPlatform_3 substep. targets must already be transformed
    // (mp6_transform_points + offsets). On return:
    //   axis[i]        : per-axis state/diagnostics (struct docs above)
    //   calc_error     : overlapping-bit semantics of the source CalcError
    //
    // state assumptions ported from the source:
    //   * table index step num10 = 0.0025 rad per count (L17576), tables
    //     index 0..2000 centred on 1000 = CrankInit (L18138)
    //   * prior index clamped into 10..1990 IN PLACE (L18214..L18232)
    //   * direction chosen by which of idx/idx+1 residuals is smaller
    //     (L18244), scan span RStep = round((max-min)/2/num10) (L17900,
    //     360 for +-0.9 rad), early exit once |residual| grows (L18349)
    //   * fractional index interpolation num28 = num11 -
    //     num22/(num15-num22)*dir (L18363, integer form when num15 == num22)
    //   * limits applied BEFORE the residual gate (L18405..L18431), clamped
    //     angle anchored on Sign(LastCrankPos - 1000), always invalidating
    //   * docking = true bypasses the limits entirely (L18375)
    //   * residual acceptance |value| < 4 mm (L17616) then stores
    //     CrankCurrent/LastCrankAngle/RodAngleAim only on success
    void solve(const std::array<Mp6Point3, kAxes>& target_mm,
               const std::array<double, kAxes>& angle_to_step, bool docking,
               bool test_mode, Mp6SolveResult* out);

    // Prior-solution table index (clamped 10..1990) currently held for axis0.
    int last_crank_pos(int axis0) const { return last_crank_pos_[axis0]; }

private:
    void build_tables();

    std::array<std::array<double, 2001>, kAxes> xxc_{};  // num4, index
    std::array<std::array<double, 2001>, kAxes> yyc_{};
    std::array<std::array<double, 2001>, kAxes> zzc_{};
    std::array<std::array<int, 2001>, kAxes> rod_crank_angle_{}; // deg*10
    std::array<Mp6Point3, kAxes> pivot_{};      // Xo/Yo/Zo
    std::array<double, kAxes> caosin_{};        // sin(CrankXOffset[i])
    std::array<double, kAxes> caocos_{};        // cos(CrankXOffset[i])
    static constexpr double kCrankInit[kAxes] = {3.141592653589793, 0.0,
                                                 3.141592653589793, 0.0,
                                                 3.141592653589793, 0.0};
    static constexpr double kCrankXOffset[kAxes] = {
        5.235987755982989,   // 5*pi/3
        4.1887902047863905,  // 4*pi/3
        1.0471975511965976,  // pi/3
        0.0,
        3.141592653589793,   // pi
        2.0943951023931953,  // 2*pi/3
    };
    std::array<double, kAxes> rod_length_{};
    std::array<double, kAxes> rod_angle_zero_{};
    double crank_length_ = 0.0;
    double min_rad_ = 0.0;
    double max_rad_ = 0.0;
    int r_step_ = 0;                       // num.. RStep
    std::array<int, kAxes> last_crank_pos_{}; // int, init 1000
    std::array<double, kAxes> last_crank_angle_{}; // static local double[7]
    std::array<double, kAxes> angle_aim_{};        // RodAngleAim
    bool motion_init_ = false;             // static local bool
};

} // namespace mp6
