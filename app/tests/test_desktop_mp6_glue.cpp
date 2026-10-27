#include <cstdlib>
namespace {
// App's global constructor caches documentation mode; establish it before inclusion.
const int documentation_environment = _putenv_s("STEWART_DOCUMENTATION_MODE", "1");
}
#include "mp6_framework_f3_oracle.h"
#include "../src/app.cpp" // Exercise internal production persistence/runtime, without test replicas.
#include "control_server.h"
/*
 * Offline desktop integration glue tests for the MP6 (Motum Mega) backend.
 *
 * Exercises the production App adapter, multi-entity isolation, configuration
 * validation, geometry and point-zero review gates, rotary neutral-steps solve,
 * start-motion pre-feeding, active/powered mutation guards, and shutdown/estop.
 *
 * Uses mp6::MemoryLink to inject scripted controller frames without live COM I/O.
 * Enforces STEWART_DOCUMENTATION_MODE=1.
 */

#include "app.h"
#include "automation.h"
#include "mp6_link.h"
#include "mp6_protocol.h"
#include "mp6_session.h"
#include "mp6_config.h"
#include "mp6_geometry.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
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

void expectEq(long long actual, long long expected, const char* actualExpr,
              const char* expectedExpr, int line) {
    ++g_checks;
    if (actual != expected) {
        ++g_failures;
        std::printf("FAIL line %d: %s == %s (actual %lld, expected %lld)\n",
                    line, actualExpr, expectedExpr, actual, expected);
    }
}

#define CHECK(cond) expectTrue(static_cast<bool>(cond), #cond, __LINE__)
#define CHECK_EQ(a, b) expectEq((a), (b), #a, #b, __LINE__)

using Bytes = std::vector<std::uint8_t>;
using Axes = std::array<std::uint16_t, 6>;
using Ard = std::array<std::uint16_t, 10>;

void putBe16(Bytes& bytes, std::size_t offset, std::uint16_t value) {
    bytes[offset] = static_cast<std::uint8_t>(value >> 8);
    bytes[offset + 1] = static_cast<std::uint8_t>(value & 0xFFu);
}

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

Ard zeroArd() {
    Ard ard{};
    ard.fill(0);
    return ard;
}

void feedSupplyAxes(mp6::MemoryLink* link, const Axes& axes) {
    Ard selector6 = zeroArd();
    selector6[0] = 30;   // pneumatic aim
    selector6[1] = 800;  // AVInput10 -> main board 10.88 V
    selector6[2] = 800;  // AVInput11 -> solenoid supply 10.88 V
    link->inject(makeType48(6, 3, axes, selector6));

    Ard selector7 = zeroArd();
    selector7[0] = 60;     // strut pressure
    selector7[2] = 15855;  // three 5-bit rails at 15 counts * 3 V = 45 V
    link->inject(makeType48(7, 3, axes, selector7));
}

std::string joinFields(const std::vector<std::string>& fields) {
    std::string out;
    for (size_t i = 0; i < fields.size(); ++i) {
        if (i != 0) out += '\t';
        out += fields[i];
    }
    return out;
}

std::vector<std::string> genericRow(const char* header, const char* tooltip,
                                    const std::vector<std::string>& pairs) {
    std::vector<std::string> fields = {header, tooltip};
    for (const std::string& s : pairs) fields.push_back(s);
    while (fields.size() < 18) fields.push_back("");
    return fields;
}

std::vector<std::string> potRow(const char* header, const char* potLo,
                                const char* potScale, const char* mswitchOn,
                                const char* mswitchOff) {
    return {header,        "",            potLo,      "Pot - Docked",
            "55",          "Angle - Docked", potScale, "Pot Scale",
            mswitchOn,     "MSwitch On",  mswitchOff, "MSwitch Off",
            "20",          "Gear Ratio"};
}

std::string makeSnapshotPltc() {
    std::vector<std::string> lines;
    lines.push_back(joinFields(genericRow("X Offset", "", {"0"})));
    lines.push_back(joinFields(genericRow("Z Offset", "", {"506"}))); // Unconsumed row 506
    lines.push_back(joinFields(genericRow("Angle Offset", "", {"0"})));
    lines.push_back(joinFields(genericRow("Crank Length", "", {"230"})));
    lines.push_back(joinFields(genericRow("Rod Length", "", {"645"})));
    lines.push_back(joinFields(genericRow("Centre Strut", "", {"72", "Diameter (mm)", "0"})));
    lines.push_back(joinFields(genericRow("Payload Weight", "(kg)", {"88"})));
    lines.push_back(joinFields(genericRow("Step/Rev", "", {"3200"})));
    lines.push_back(joinFields(genericRow("Rotate Direction", "-1: Counter Rotate, 1: Normal Rotate", {"-1"})));
    lines.push_back(joinFields(genericRow("Update Rate", "", {"60"})));
    lines.push_back(joinFields(genericRow("Max Motor RPM", "", {"230"})));
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
                      "20.5000001", "DFilter", "20", "TConst"})));
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

void setupTestGeometry(Entity& e) {
    // Sourced MOT155-D2 parameters from frm_Design.cs:686
    snprintf(e.mp6.design.design_name, sizeof(e.mp6.design.design_name), "MOT155-D2");
    e.mp6.design.bot_a_mm = 840.0f;
    e.mp6.design.bot_b_mm = 230.0f;
    e.mp6.design.top_a_mm = 520.0f;
    e.mp6.design.top_b_mm = 14.0f;
    e.mp6.design.crank_length_mm = 230.0f;
    e.mp6.design.rod_length_mm = 645.0f;
    e.mp6.design.off_x_mm = -25.0f;
    e.mp6.design.pcdx_mm = -270.0f;
    e.mp6.design.pcdy_mm = 90.0f;
    e.mp6.design.angle_offset_deg = 0.0f;
    e.mp6.design.z_off_mm = 550.0f; // Sourced MOT155-D2 design ZOff: 550 mm -> yields 530 mm Z_Offset
    e.mp6.design.point_zero_x_mm = 0.0f;
    e.mp6.design.point_zero_z_mm = 0.0f;
    for (int i = 0; i < 6; ++i) {
        e.mp6.design.base_z_mm[i] = -30.0f;
        e.mp6.design.top_z_mm[i] = 10.0f;
    }
}

} // namespace

int runScenario(const std::string& mode) {
    std::printf("Scenario %s\n", mode.c_str());
    g_app.mp6_runtimes.clear();
    g_app.next_entity_id = 0;
    g_app.start_ramp_active = false;
    memset(g_app.shared_input, 0, sizeof(g_app.shared_input));
    std::printf("=== MP6 Desktop Glue Integration Tests ===\n");

    // Force documentation mode
    _putenv("STEWART_DOCUMENTATION_MODE=1");

    // Clear and re-initialize application
    g_app.entities.clear();
    g_app.motion_started = false;

    // 1. Multi-entity allocation
    g_app.addEntity("Rig0", EntityType::HIL, HilBackend::MP6);
    g_app.addEntity("Rig1", EntityType::HIL, HilBackend::ESP32);
    Entity& e0 = *g_app.findEntity(0);
    Entity& e1 = *g_app.findEntity(1);
    CHECK_EQ(e0.id, 0);
    CHECK_EQ(e1.id, 1);
    CHECK(e0.hil_backend == HilBackend::MP6);
    CHECK(e1.hil_backend == HilBackend::ESP32);

    // 2. Controller Settings validation and isolation
    Mp6MachineSettings bad_settings{};
    bad_settings.pid_index = 5; // invalid (> 3)
    std::string err;
    CHECK(!g_app.mp6SetControllerSettings(e0, bad_settings, &err));
    CHECK(err.find("pid_index") != std::string::npos);

    bad_settings.pid_index = 0;
    bad_settings.comms_timeout_ms = 50; // too small (< 100)
    CHECK(!g_app.mp6SetControllerSettings(e0, bad_settings, &err));
    CHECK(err.find("comms_timeout_ms") != std::string::npos);

    Mp6MachineSettings good_settings{};
    good_settings.explicit_fields = Mp6MachineSettings::all_fields;
    good_settings.pid_index = 0;
    good_settings.comms_timeout_ms = 1000;
    good_settings.pressure_enabled = true;
    good_settings.pressure_aim_supplied = true;
    good_settings.pressure_aim_psi = 30;
    good_settings.mot_val_final = 63;
    good_settings.sensor_filter = 0.0075f;
    good_settings.av_filter = 0.01f;
    good_settings.history_len = 3;
    good_settings.sensor_filter_start = 0.0075f;
    good_settings.history_len_start = 3;
    good_settings.bench_test = 0;
    good_settings.onboard_shutdown = 1;
    good_settings.deadband_enable = 0;
    good_settings.limits_enable = 1;
    good_settings.info_motor = 1;

    CHECK(g_app.mp6SetControllerSettings(e0, good_settings, &err));
    CHECK(e0.mp6.controller_settings.supplied);
    // Entity 1 settings must remain unsupplied (isolation)
    CHECK(!e1.mp6.controller_settings.supplied);

    // 3. Platform configuration import
    std::string pltc = makeSnapshotPltc();
    int expected_correction = 80;
    if (mode.rfind("quant", 0) == 0) {
        const auto& oracle = frameworkF3Cases[std::atoi(mode.c_str() + 5)];
        float value; memcpy(&value, &oracle.bits, sizeof(value));
        char decimal[40]; snprintf(decimal, sizeof(decimal), "%.9g", double(value));
        auto at = pltc.find("0.0075");
        pltc.replace(at, 6, decimal);
        expected_correction = oracle.wire;
    }
    if (mode == "wirefloat") {
        auto replace_once = [&](const std::string& from, const std::string& to) {
            auto at = pltc.find(from);
            CHECK(at != std::string::npos);
            if (at != std::string::npos) pltc.replace(at, from.size(), to);
        };
        replace_once("\t1.25\tDecel Factor", "\t1.245\tDecel Factor");
        replace_once("\t55\tAngle - Docked", "\t55.05\tAngle - Docked");
        replace_once("\t-12\tPot Scale", "\t16.928045\tPot Scale");
        replace_once("\t20\tGear Ratio", "\t19.045\tGear Ratio");
    }
    if (mode == "potround") {
        auto at = pltc.find("\t3543\tPot - Docked");
        CHECK(at != std::string::npos);
        if (at != std::string::npos) pltc.replace(at + 1, 4, "3543.75");
    }
    if (mode == "stepround") {
        auto at = pltc.find("\t3200\t");
        CHECK(at != std::string::npos);
        if (at != std::string::npos) pltc.replace(at + 1, 4, "3200.6");
    }
    if (mode == "savedzero") {
        auto row_end = pltc.find('\n');
        CHECK(row_end != std::string::npos);
        if (row_end != std::string::npos) pltc.erase(0, row_end + 1); // omit optional X Offset row
    }

    mp6::Mp6PlatformConfig cfg{};
    CHECK(mp6::Mp6PlatformConfig::from_pltc(pltc, cfg));
    mp6::Mp6AxisCalibration parsed_cal[mp6::kAxes];
    CHECK(mp6::mp6_config_derive_calibration(cfg, parsed_cal, &err));

    CHECK(g_app.mp6ImportPlatformConfigText(e0, pltc.c_str(), &err));
    CHECK(e0.mp6.platform_config_present);
    CHECK(e0.mp6.config_ready);
    // Entity 1 must remain without platform config
    CHECK(!e1.mp6.platform_config_present);

    // 4. Geometry and review gates (F2 & F8 verification)
    setupTestGeometry(e0);
    e0.mp6.design.pivot_z_provenance = 2; // Reviewed
    e0.mp6.design.point_zero_supplied = true;
    CHECK(g_app.mp6RefreshDesignGeometry(e0, &err));
    if (mode == "savedzero") {
        e0.mp6.design.bot_b_mm = 230.0f;
        e0.mp6.design.top_b_mm = 14.0f;
        e0.mp6.design.off_x_mm = 0.0f;
        e0.mp6.design.base_radius_mm = 330.0f;
        e0.mp6.design.platform_radius_mm = 50.0f;
        e0.mp6.design.x_offset_mm = 15.0f; // deliberately conflicting legacy alias
        e0.mp6.design.point_zero_x_mm = 5.0f;
        cJSON* document = cJSON_CreateObject();
        saveEntityToJSON(document, e0);
        loadEntityFromJSON(e0, document);
        cJSON_Delete(document);
        CHECK_EQ(e0.mp6.design.off_x_mm, 0);
        CHECK_EQ(e0.mp6.design.x_offset_mm, 15);
        CHECK_EQ(e0.mp6.design.point_zero_x_mm, 5);
        CHECK(e0.mp6.design.supplied);
        CHECK_EQ(e0.mp6.design.pivot_z_provenance, 2);
        const std::string status = DispatchControlCommand(
            R"({"cmd":"mp6_status","args":{"entity":0}})");
        CHECK(status.find("\"base_radius_mm\":230") != std::string::npos);
        CHECK(status.find("\"platform_radius_mm\":14") != std::string::npos);
        CHECK(status.find("\"x_offset_mm\":0") != std::string::npos);
    }

    bool recovered = false;
reconnect:
    {
    // 5. Connect with MemoryLink
    auto link_owner = std::make_unique<mp6::MemoryLink>();
    mp6::MemoryLink* link = link_owner.get();
    link->open("test_mem");
    e0.mp6.platform_number_expected = 1;
    CHECK(g_app.mp6ConnectWithLink(e0, std::move(link_owner), &err));
    CHECK(e0.mp6.link_open);

    // Update entity: processes geometry and inputs now that session is attached
    g_app.mp6UpdateEntity(e0);
    CHECK(e0.mp6.viz.solver_ready);
    if (mode == "savedzero") CHECK_EQ(g_app.mp6_runtimes[e0.id]->x_offset_mm, 5);
    if (mode == "savedzero") {
        const std::string status = DispatchControlCommand(
            R"({"cmd":"mp6_status","args":{"entity":0}})");
        CHECK(status.find("\"base_radius_mm\":230") != std::string::npos);
        CHECK(status.find("\"platform_radius_mm\":14") != std::string::npos);
        CHECK(status.find("\"x_offset_mm\":0") != std::string::npos);
        CHECK_EQ(e0.mp6.design.point_zero_x_mm, 5);
        CHECK_EQ(g_app.mp6_runtimes[e0.id]->x_offset_mm, 5);
    }

    g_app.frame_time += 0.016;
    g_app.update();
    CHECK(e0.mp6.viz.rod_lines_valid);
    if (mode != "savedzero") {
        CHECK(std::abs(e0.mp6.viz.base_pivot[0][0] - 484.974213) < 0.001);
        CHECK(std::abs(e0.mp6.viz.base_pivot[0][1] + 230) < 0.001);
        CHECK(std::abs(e0.mp6.viz.base_pivot[0][2] + 30) < 0.001);
        CHECK(std::abs(e0.mp6.viz.rod_top[0][0] - 162.235428) < 0.001);
        CHECK(std::abs(e0.mp6.viz.rod_top[0][1] + 253) < 0.001);
        CHECK(std::abs(e0.mp6.viz.rod_top[0][1] + 253) < 0.001);
        CHECK(std::abs(e0.mp6.viz.rod_top[0][2] - 540) < 0.001);
    }
    CHECK(std::abs(g_app.mp6_runtimes[e0.id]->z_offset_mm - 530) < 0.001);
    if (mode != "wirefloat" && mode != "savedzero") CHECK_EQ(e0.mp6.step_target[0], -54);

    // 6. Identity lifecycle: stage Identify, inject type-42 echo + selector-1 telemetry
    CHECK(g_app.mp6Command(e0, Mp6Intent::Identify, &err));
    g_app.frame_time += 0.016;
    g_app.mp6UpdateEntity(e0);
    link->inject(makeType42(3, 253));

    // Initialize simulated axis feedback to docked steps
    Axes ax{};
    for (int j = 0; j < 6; ++j) {
        float potFloat = ((float)parsed_cal[j].docked_steps - parsed_cal[j].p_offset) * 8.0f / parsed_cal[j].p_scale;
        int word = mp6::roundHalfEvenToInt(static_cast<double>(potFloat));
        ax[j] = static_cast<uint16_t>(std::clamp(word, 0, 65535));
    }

    Ard pairing = zeroArd();
    pairing[0] = 0;
    pairing[1] = 1; // platform number = 1
    pairing[2] = 3;
    link->inject(makeType48(1, 3, ax, pairing)); // selector 1, platform 1
    feedSupplyAxes(link, ax);
    g_app.frame_time += 0.016;
    g_app.mp6UpdateEntity(e0);
    CHECK(e0.mp6.identity_ok);
    CHECK(e0.mp6.paired_config);

    // Helper for simulated plant pump across 22-frame command-5 profile transactions and 60 Hz telemetry
    int cmd5_observed = 0;
    bool acknowledge_profiles = true;
    int normal_step_correction = -999;
    int normal_dfilter = -999;
    int first_pressure_word = -1;
    int wire_decel = -999, wire_gearbox = -999, wire_ang_lo = -999, wire_p_scale = -999;
    int wire_pot_lo = -999, wire_p_offset = -999, wire_step_rev = -999, wire_max_pps = -999;
    int first_running_target = INT_MIN;
    auto record_config = [&](const Bytes& w) {
        for (int pair = 0; pair < 3; ++pair) {
            int o = 3 + pair * 4;
            int key = (w[o] | (w[o+1] << 8)) - 32767;
            int val = (w[o+2] | (w[o+3] << 8)) - 32767;
            if (key == 39 && cmd5_observed > 44) normal_step_correction = val;
            if (key == 25 && cmd5_observed > 44) normal_dfilter = val;
            if (key == 42) wire_decel = val;
            if (key == 141) wire_gearbox = val;
            if (key == 111) wire_ang_lo = val;
            if (key == 121) wire_p_scale = val;
            if (key == 21) wire_step_rev = val;
            if (key == 34) wire_max_pps = val;
            if (key == 101) wire_pot_lo = val;
            if (key == 131) wire_p_offset = val;
        }
    };
    auto pump_ticks = [&](int count) {
        for (int i = 0; i < count; ++i) {
            auto writes = link->takeWrites();
            for (const auto& w : writes) {
                if (w.size() >= 4 && w[2] == 4 && first_pressure_word == -1) first_pressure_word = w[3];
                if (w.size() >= 3 && w[0] == 0xFE && w[1] == 0xFC && w[2] == 5) {
                    ++cmd5_observed;
                    record_config(w);
                    if (acknowledge_profiles && cmd5_observed % 22 == 0) {
                        link->inject(makeType43(57)); // End ACK
                    }
                }
                else if (w.size() >= 18 && w[0] == 0xFE && w[1] == 0xFC && w[2] == 2) {
                    if (e0.mp6.lifecycle == Mp6Lifecycle::Running && first_running_target == INT_MIN)
                        first_running_target = (w[3] | (w[4] << 8)) - 32767;
                    // Update simulated feedback to follow commanded step targets
                    for (int j = 0; j < 6; ++j) {
                        const size_t base = 3 + static_cast<size_t>(j) * 2;
                        unsigned raw = static_cast<unsigned>(w[base]) | (static_cast<unsigned>(w[base + 1]) << 8);
                        int target = static_cast<int>(raw) - 32767;
                        float potFloat = ((float)target - parsed_cal[j].p_offset) * 8.0f / parsed_cal[j].p_scale;
                        int word = mp6::roundHalfEvenToInt(static_cast<double>(potFloat));
                        ax[j] = static_cast<uint16_t>(std::clamp(word, 0, 65535));
                    }
                }
            }
            feedSupplyAxes(link, ax);
            link->inject(makeType48(1, 0x03, ax, pairing)); // status 3: configReceived + avOk
            g_app.frame_time += 0.016;
            g_app.mp6UpdateEntity(e0);
        }
    };

    // Pending terminal safety actions cannot be overwritten by global Stop.
    if (mode == "priority") {
        CHECK(g_app.mp6Command(e0, Mp6Intent::Estop, &err));
        g_app.stopMotion();
        CHECK(e0.mp6.pending_intent == Mp6Intent::Estop);
        g_app.mp6UpdateEntity(e0);
        CHECK(e0.mp6.lifecycle == Mp6Lifecycle::SafeOff);
        CHECK(g_app.mp6Command(e0, Mp6Intent::Disconnect, &err));
        g_app.stopMotion();
        CHECK(e0.mp6.pending_intent == Mp6Intent::Disconnect);
        g_app.mp6UpdateEntity(e0);
        CHECK(!e0.mp6.link_open);
        return 0;
    }

    // 7. Configure transaction (PID 1 Profile)
    CHECK(g_app.mp6Command(e0, Mp6Intent::Configure, &err));
    g_app.frame_time += 0.016;
    g_app.mp6UpdateEntity(e0);
    CHECK(e0.mp6.sending_config);
    if (mode == "configurestop") {
        g_app.stopMotion();
        CHECK(e0.mp6.pending_intent == Mp6Intent::Stop);
        pump_ticks(40);
        link->inject(makeType43(57));
        pump_ticks(2);
        CHECK(!e0.mp6.config_send_ok);
        CHECK(!e0.mp6.sending_config);
        CHECK(e0.mp6.lifecycle != Mp6Lifecycle::Running);
        return 0;
    }


    // Pump until Configure ACK is accepted
    for (int i = 0; i < 60 && !e0.mp6.config_send_ok; ++i) {
        pump_ticks(1);
        if (i % 200 == 0) {
            std::printf("tick %d: lifecycle=%d note=%s last_error=%s\n", i, (int)e0.mp6.lifecycle, e0.mp6.lifecycle_note, e0.mp6.last_error);
        }
    }
    CHECK(e0.mp6.config_send_ok);
    CHECK(e0.mp6.av_ok);
    if (mode == "wirefloat") {
        CHECK_EQ(wire_decel, 124); // 1.245f * 100f -> 124.5, half-even
        CHECK_EQ(wire_gearbox, 1904); // 19.045f * 100f -> 1904.5, half-even
        CHECK_EQ(wire_ang_lo, 550); // 55.05f * 10f -> 550.5, half-even
        CHECK_EQ(wire_p_scale, 10000); // derived 10.0005f * 1000f -> 10000.5
        return 0;
    }

    if (mode == "stale" || mode == "failedrefresh") {
        e0.mp6.design.pivot_z_provenance = 0;
        if (mode == "failedrefresh") e0.mp6.design.design_name[0] = 0;
        bool accepted = g_app.mp6RefreshDesignGeometry(e0, &err);
        CHECK(accepted == (mode == "stale"));
        pump_ticks(2);
        CHECK(!e0.mp6.viz.solver_ready);
        CHECK(!g_app.mp6_runtimes[e0.id]->neutral_valid);
        CHECK(!e0.mp6.step_target_valid);
        CHECK(!e0.mp6.config_send_ok);
        CHECK(!g_app.mp6Command(e0, Mp6Intent::Initialize, &err));
        return 0;
    }

    if (mode == "badreplacement") {
        for (const char* bad : {"bogus", "", "NaN", "Inf", "1e999", "999999999"}) {
            for (const char* cell : {"PID 0\t\t11", "Pot 1\t\t3543"}) {
                std::string text = makeSnapshotPltc();
                size_t at = text.find(cell);
                size_t num = text.find("\t\t", at) + 2;
                text.replace(num, text.find('\t', num) - num, bad);
                link->takeWrites();
                CHECK(g_app.mp6ImportPlatformConfigText(e0, text.c_str(), &err));
                CHECK(!e0.mp6.config_ready);
                CHECK(!e0.mp6.config_send_ok);
                CHECK(!g_app.mp6_runtimes[e0.id]->pending_pid_valid);
                CHECK(!g_app.mp6Command(e0, Mp6Intent::Configure, &err));
                CHECK(!g_app.mp6Command(e0, Mp6Intent::Initialize, &err));
                pump_ticks(2);
                CHECK(!e0.mp6.viz.solver_ready);
                CHECK(!g_app.mp6_runtimes[e0.id]->neutral_valid);
                // An already authenticated session may retain its idle identity poll.
                for (const auto& write : link->takeWrites()) CHECK_EQ(write[2], 3);
            }
        }
        for (int profile = 0; profile < 3; ++profile) {
            std::string text = makeSnapshotPltc();
            std::string key = "PID " + std::to_string(profile) + "\t\t";
            auto at = text.find(key);
            size_t filter_start = text.rfind('\t', text.find("DFilter", at) - 2) + 1;
            text.replace(filter_start, text.find('\t', filter_start) - filter_start, "40000"); // numeric, but rejected by setPidProfile wire validation
            link->takeWrites();
            CHECK(g_app.mp6ImportPlatformConfigText(e0, text.c_str(), &err));
            CHECK(!e0.mp6.config_ready);
            CHECK(!g_app.mp6_runtimes[e0.id]->pending_pid_valid);
            CHECK(std::string(e0.mp6.config_note).find("PID profile") != std::string::npos);
            CHECK(!g_app.mp6Command(e0, Mp6Intent::Configure, &err));
            pump_ticks(1);
            for (const auto& write : link->takeWrites()) CHECK_EQ(write[2], 3);
        }
        return 0;
    }

    if (mode == "pressureoff") {
        auto disabled_pressure = good_settings;
        disabled_pressure.pressure_enabled = false;
        disabled_pressure.pressure_aim_supplied = false;
        disabled_pressure.pressure_aim_psi = 0;
        CHECK(g_app.mp6SetControllerSettings(e0, disabled_pressure, &err));
        pump_ticks(1);
        CHECK(g_app.mp6Command(e0, Mp6Intent::Configure, &err));
        pump_ticks(60);
        CHECK(e0.mp6.config_send_ok);
    }

    // 8. Initialize & Parked hold (PID 2 Profile)
    CHECK(g_app.mp6Command(e0, Mp6Intent::Initialize, &err));
    bool reached_parked = false;
    for (int i = 0; i < 2000; ++i) {
        pump_ticks(1);
        if (i % 200 == 0) {
            std::printf("init tick %d: lifecycle=%d note=%s last_error=%s\n", i, (int)e0.mp6.lifecycle, e0.mp6.lifecycle_note, e0.mp6.last_error);
        }
        if (e0.mp6.lifecycle == Mp6Lifecycle::Parked) {
            reached_parked = true;
            break;
        }
    }
    if (!reached_parked) {
        std::printf("DID NOT REACH PARKED: lifecycle_note: %s, last_error: %s\n", e0.mp6.lifecycle_note, e0.mp6.last_error);
    }
    CHECK(reached_parked);
    CHECK_EQ(static_cast<int>(e0.mp6.lifecycle), static_cast<int>(Mp6Lifecycle::Parked));
    // Verify applied PID profile is Lifecycle (2)
    CHECK_EQ(e0.mp6.applied_pid_mode, 2);
    if (mode == "savedzero") {
        CHECK_EQ(g_app.mp6_runtimes[e0.id]->x_offset_mm, 5);
        CHECK(e0.mp6.config_send_ok);
        return 0;
    }
    if (mode == "pressureoff") {
        CHECK_EQ(first_pressure_word, 5); // disabled aim 0, protocol +5
        return 0;
    }


    if (mode == "parkedstop" || mode == "stagedstartstop" || mode == "dockedstop") {
        if (mode == "stagedstartstop") CHECK(g_app.mp6Command(e0, Mp6Intent::Start, &err));
        if (mode == "parkedstop") {
            g_app.motion_started = true;
            g_app.frame_time += .016;
            g_app.update();
            CHECK(e0.mp6.step_target_valid);
            CHECK(g_app.mp6Command(e0, Mp6Intent::Start, &err));
            pump_ticks(60);
            CHECK(e0.mp6.lifecycle == Mp6Lifecycle::Running);
        }
        if (mode == "dockedstop") {
            CHECK(g_app.mp6Command(e0, Mp6Intent::Park, &err));
            for (int i = 0; i < 2000 && e0.mp6.lifecycle != Mp6Lifecycle::Docked; ++i) pump_ticks(1);
            CHECK(e0.mp6.lifecycle == Mp6Lifecycle::Docked);
            CHECK(g_app.mp6Command(e0, Mp6Intent::Shutdown, &err));
            pump_ticks(2);
        }
        g_app.stopMotion();
        CHECK(e0.mp6.pending_intent == Mp6Intent::Stop);
        pump_ticks(80);
        link->inject(makeType43(57));
        pump_ticks(5);
        CHECK(!g_app.motion_started);
        CHECK(!e0.mp6.shutdown_pending);
        CHECK(e0.mp6.lifecycle != Mp6Lifecycle::Running);
        CHECK(e0.mp6.lifecycle != Mp6Lifecycle::SafeOff);
        if (mode == "parkedstop") {
            CHECK(e0.mp6.lifecycle == Mp6Lifecycle::Identified); // App view of observed Stopped
            auto restart = DispatchControlCommand(R"({"cmd":"motion","args":{"on":true}})");
            CHECK(restart.find("\"motion_started\":true") != std::string::npos);
            CHECK(e0.mp6.pending_intent == Mp6Intent::None);
            pump_ticks(3);
            CHECK(e0.mp6.lifecycle != Mp6Lifecycle::Running);
            for (const auto& write : link->takeWrites()) CHECK(write[2] != 2);
        }
        return 0;
    }

    // Update pipeline: verify targets are pre-fed while Parked
    g_app.motion_started = mode != "perstart";
    if (mode == "jump") {
        e0.config.input_filter.enabled = 0;
        e0.config.mca.enabled = 0;
        e0.state.input_pct[0] = 10;
    }
    g_app.frame_time += 0.016;
    g_app.update();
    CHECK(e0.mp6.step_target_valid);

    // 9. Explicit Start: initiates Normal PID 0 transaction before Running
    link->takeWrites(); // flush any existing writes
    if (mode == "lostack") acknowledge_profiles = false;
    if (mode == "perstart") {
        auto reply = DispatchControlCommand(R"({"cmd":"motion","args":{"on":true}})");
        CHECK(reply.find("\"motion_started\":true") != std::string::npos);
        CHECK(e0.mp6.pending_intent == Mp6Intent::Start);
    } else {
        CHECK(g_app.mp6Command(e0, Mp6Intent::Start, &err));
    }
    g_app.frame_time += 0.016;
    g_app.update();

    // Verify: Start request is asynchronous — intent is accepted, session is in PID 0 profile transaction (Configuring) or Parked hold, but NOT Running yet!
    CHECK(e0.mp6.lifecycle == Mp6Lifecycle::Configuring || e0.mp6.lifecycle == Mp6Lifecycle::Parked);
    if (mode == "lostack") {
        pump_ticks(1000);
        CHECK(e0.mp6.lifecycle != Mp6Lifecycle::Running);
        CHECK(!e0.mp6.config_send_ok);
        CHECK(!g_app.mp6Command(e0, Mp6Intent::Initialize, &err));
        acknowledge_profiles = true;
        CHECK(g_app.mp6Command(e0, Mp6Intent::Configure, &err));
        pump_ticks(60);
        CHECK(e0.mp6.config_send_ok);
        CHECK(g_app.mp6Command(e0, Mp6Intent::Initialize, &err));
        for (int i = 0; i < 2000 && e0.mp6.lifecycle != Mp6Lifecycle::Parked; ++i) pump_ticks(1);
        CHECK(e0.mp6.lifecycle == Mp6Lifecycle::Parked);
        CHECK(g_app.mp6Command(e0, Mp6Intent::Shutdown, &err));
        for (int i = 0; i < 2000 && e0.mp6.lifecycle != Mp6Lifecycle::SafeOff; ++i) pump_ticks(1);
        CHECK(e0.mp6.lifecycle == Mp6Lifecycle::SafeOff);
        return 0;
    }
    if (mode == "startstop") {
        g_app.stopMotion();
        CHECK(e0.mp6.pending_intent == Mp6Intent::Stop);
        pump_ticks(40);
        link->inject(makeType43(57));
        pump_ticks(2);
        CHECK(e0.mp6.lifecycle != Mp6Lifecycle::Running);
        CHECK(!g_app.motion_started);
        return 0;
    }


    // Assert: NO command-2 target stream frames sent before Normal ACK!
    auto mid_writes = link->takeWrites();
    bool found_premature_cmd2 = false;
    for (const auto& w : mid_writes) {
        if (w.size() >= 3 && w[0] == 0xFE && w[1] == 0xFC && w[2] == 2) {
            found_premature_cmd2 = true;
            break;
        }
        if (w.size() >= 3 && w[0] == 0xFE && w[1] == 0xFC && w[2] == 5) {
            ++cmd5_observed;
            record_config(w);
            if (acknowledge_profiles && cmd5_observed % 22 == 0) {
                link->inject(makeType43(57)); // Normal PID ACK
            }
        }
    }
    CHECK(!found_premature_cmd2);

    // Pump ticks to complete Normal profile transaction and receive ACK + fresh observation
    bool reached_running = false;
    for (int i = 0; i < 60; ++i) {
        pump_ticks(1);
        if (e0.mp6.lifecycle == Mp6Lifecycle::Running) {
            reached_running = true;
            break;
        }
    }
    CHECK(reached_running);
    CHECK_EQ(static_cast<int>(e0.mp6.lifecycle), static_cast<int>(Mp6Lifecycle::Running));
    CHECK_EQ(e0.mp6.applied_pid_mode, 0); // Normal PID 0 applied!
    CHECK(g_app.motion_started);
    CHECK_EQ(normal_dfilter, 20); // source Single rounds 20.5000001 to represented20.5, ties even
    if (mode == "potround") {
        CHECK_EQ(wire_pot_lo, 3544);
        CHECK_EQ(wire_p_offset, -4273);
    }
    if (mode == "stepround") CHECK_EQ(wire_step_rev, 3201);
    if (mode == "stepround") CHECK_EQ(wire_max_pps, 12270);
    CHECK_EQ(normal_step_correction, expected_correction); // Framework Single F3 oracle, not snprintf
    CHECK(g_app.mp6_runtimes[e0.id]->start_blend_active);


    // Verify command-2 frames are actively streaming in Running state
    auto running_writes = link->takeWrites();
    bool found_cmd2 = false;
    for (const auto& w : running_writes) {
        if (w.size() >= 6 && w[0] == 0xFE && w[1] == 0xFC && w[2] == 2) {
            found_cmd2 = true;
            if (first_running_target == INT_MIN) first_running_target = (w[3] | (w[4] << 8)) - 32767;
            break;
        }
    }
    CHECK(found_cmd2);
    CHECK_EQ(first_running_target, -54);
    if (mode == "jump") {
        CHECK(e0.mp6.step_target[0] > 100);
        for (int i = 0; i < 5; ++i) {
            e0.state.input_pct[0] = i % 2 ? 8 : 10;
            g_app.frame_time += .016;
            g_app.update(); // a changing input source cannot replace the wire blend
            CHECK(g_app.mp6_runtimes[e0.id]->start_blend_active);
            auto output = link->takeWrites();
            for (const auto& write : output) {
                if (write[2] == 2) {
                    int target = (write[3] | (write[4] << 8)) - 32767;
                    CHECK(target > -60 && target < -20);
                }
            }
            feedSupplyAxes(link, ax);
        }
        return 0;
    }
    if (mode == "perstart") return 0;
    if (mode.rfind("quant", 0) == 0) return 0;
    if (mode == "recovery" && !recovered) {
        e0.enabled = false;
        g_app.requestEstop();
        g_app.update();
        CHECK(e0.mp6.lifecycle == Mp6Lifecycle::SafeOff);
        CHECK(e0.mp6.estop_latched);
        g_app.mp6Disconnect(e0);
        auto replacement = std::make_unique<mp6::MemoryLink>();
        auto* raw = replacement.get(); raw->open("plain-reconnect");
        CHECK(g_app.mp6ConnectWithLink(e0, std::move(replacement), &err));
        g_app.mp6UpdateEntity(e0);
        CHECK(e0.mp6.estop_latched); // auto/plain reconnect cannot clear it
        CHECK(!e0.mp6.identity_ok);
        CHECK(!e0.mp6.config_send_ok);
        CHECK(raw->takeWrites().empty());
        CHECK(g_app.mp6RecoverKnownState(e0, &err));
        CHECK(!e0.mp6.estop_latched);
        CHECK(!e0.mp6.link_open);
        CHECK(!e0.mp6.identity_ok);
        CHECK(!e0.mp6.config_send_ok);
        CHECK(!g_app.mp6Command(e0, Mp6Intent::Start, &err));
        e0.enabled = true;
        recovered = true;
        goto reconnect;
    }
    if (mode == "recovery") {
        CHECK(recovered);
        CHECK(!e0.mp6.estop_latched);
        CHECK(e0.mp6.lifecycle == Mp6Lifecycle::Running);
        return 0;
    }

    if (mode == "disabled") {
        e0.enabled = false;
        g_app.requestEstop();
        g_app.update();
        CHECK(e0.mp6.lifecycle == Mp6Lifecycle::SafeOff);
        CHECK(e0.mp6.pending_intent == Mp6Intent::None);
        CHECK(e0.mp6.estop_latched);
        return 0;
    }

    auto edited = e0.mp6.design;
    edited.bot_a_mm += 100;
    CHECK(!g_app.mp6AcceptDesign(e0, edited, &err));
    CHECK(std::abs(e0.mp6.design.bot_a_mm - 840) < .001);
    CHECK(e0.mp6.viz.solver_ready);
    CHECK(!g_app.mp6InvalidateAuthority(e0, true, true));
    CHECK(e0.mp6.config_send_ok);
    CHECK(e0.mp6.viz.solver_ready);

    // 10. Active-motion mutation guards
    CHECK(!g_app.mp6RefreshDesignGeometry(e0, &err));
    CHECK(err.find("motors are enabled") != std::string::npos || err.find("actively moving") != std::string::npos);

    CHECK(!g_app.mp6ImportPlatformConfigText(e0, pltc.c_str(), &err));
    CHECK(err.find("motors are enabled") != std::string::npos || err.find("actively moving") != std::string::npos);

    CHECK(!g_app.mp6SetControllerSettings(e0, good_settings, &err));
    CHECK(err.find("motors are enabled") != std::string::npos || err.find("actively moving") != std::string::npos);

    // 11. Stop Motion (motors remain enabled in Stopped state)
    g_app.stopMotion();
    g_app.frame_time += 0.016;
    g_app.update();
    pump_ticks(5);

    // In Stopped state, motors remain powered!
    CHECK(g_app.mp6IsMotorPowerRequested(e0));

    // Powered-state mutation guards in Stopped state:
    CHECK(!g_app.mp6RefreshDesignGeometry(e0, &err));
    CHECK(err.find("motors are enabled; shut down first") != std::string::npos);

    CHECK(!g_app.mp6ImportPlatformConfigText(e0, pltc.c_str(), &err));
    CHECK(err.find("motors are enabled; shut down first") != std::string::npos);

    CHECK(!g_app.mp6SetControllerSettings(e0, good_settings, &err));
    CHECK(err.find("motors are enabled; shut down first") != std::string::npos);

    CHECK(!g_app.mp6Command(e0, Mp6Intent::Identify, &err));
    CHECK(err.find("motors are enabled; shut down first") != std::string::npos);

    // 12. Orderly Shutdown from Stopped state:
    // PID 2 ACK -> source 500 ms pause -> descent to dock -> fresh-proof vent -> SafeOff
    CHECK(g_app.mp6Command(e0, Mp6Intent::Shutdown, &err));
    if (mode == "shutdownstop") {
        pump_ticks(2);
        g_app.stopMotion();
        CHECK(e0.mp6.pending_intent == Mp6Intent::Stop);
        pump_ticks(1900);
        link->inject(makeType43(57));
        pump_ticks(2);
        CHECK(!e0.mp6.shutdown_pending);
        CHECK(e0.mp6.lifecycle != Mp6Lifecycle::SafeOff);
        return 0;
    }

    bool reached_safeoff = false;
    for (int i = 0; i < 2000; ++i) {
        pump_ticks(1);
        if (e0.mp6.lifecycle == Mp6Lifecycle::SafeOff) {
            reached_safeoff = true;
            break;
        }
    }
    CHECK(reached_safeoff);
    CHECK_EQ(static_cast<int>(e0.mp6.lifecycle), static_cast<int>(Mp6Lifecycle::SafeOff));
    // Motors are now unpowered
    CHECK(!g_app.mp6IsMotorPowerRequested(e0));

    // 13. Unpowered static edits are now accepted (F3 atomic invalidation)
    CHECK(g_app.mp6ImportPlatformConfigText(e0, "", &err)); // Clear configuration
    CHECK(!e0.mp6.platform_config_present);
    CHECK(!e0.mp6.config_ready);
    CHECK(!e0.mp6.config_send_ok);
    CHECK(!e0.mp6.av_ok);

    // Shared production UI actions, including explicitly confirmed zero point offsets.
    Mp6DesignState draft;
    CHECK(!mp6ReviewDesign(draft, Mp6ReviewAction::ConfirmPivots));
    CHECK(mp6ReviewDesign(draft, Mp6ReviewAction::MarkPivots));
    CHECK(mp6CanConfirmPivots(draft));
    CHECK_EQ(draft.pivot_z_provenance, 0);
    CHECK(mp6ReviewDesign(draft, Mp6ReviewAction::ConfirmPivots));
    CHECK_EQ(draft.pivot_z_provenance, 2);
    CHECK(!mp6CanConfirmPivots(draft));
    CHECK(mp6ReviewDesign(draft, Mp6ReviewAction::ConfirmPointZeros));
    CHECK(draft.point_zero_supplied);
    CHECK_EQ(draft.point_zero_x_mm, 0);
    CHECK_EQ(draft.point_zero_z_mm, 0);

    // 15. E-Stop latching
    g_app.requestEstop();
    CHECK(e0.mp6.estop_latched);
    CHECK(!g_app.motion_started);

    // Entity 1 was never connected or running (isolation verified)
    CHECK(!e1.mp6.link_open);

    std::printf("Checks: %d, Failures: %d\n", g_checks, g_failures);
    if (g_failures == 0) {
        std::printf("ALL DESKTOP GLUE CHECKS PASSED\n");
        return 0;
    }
    return 1;
    }
}

// Only window/UI operations are stubbed. Dispatcher, App and Session are production.
cJSON* UIAutomationState() { return cJSON_CreateObject(); }
cJSON* UIAutomationConfigure(const cJSON*, std::string&) { return cJSON_CreateObject(); }
bool CaptureWindowPNG(const char*, int*, int*) { return false; }
bool SetAppWindowSize(int, int) { return false; }
void GetAppWindowSize(int*, int*) {}

void testBoundaries() {
    g_app.entities.reserve(8);
    for (const auto& oracle : frameworkF3Cases) {
        float value; memcpy(&value, &oracle.bits, sizeof(value));
        CHECK_EQ(mp6QuantizeStepCorrection(value), oracle.wire);
    }

    Entity& e = *g_app.findEntity(0);
    std::string why;
    auto blocked_reset = DispatchControlCommand(R"({"cmd":"mp6_recover_known_state","args":{"entity":0}})");
    CHECK(blocked_reset.find("\"ok\":false") != std::string::npos);
    CHECK(DispatchControlCommand(R"({"cmd":"capabilities"})").find("mp6_recover_known_state") != std::string::npos);
    CHECK(g_app.mp6RecoverKnownState(e, &why));
    CHECK(!g_app.mp6SetControllerSettings(e, Mp6MachineSettings{}, &why));
    CHECK(why.find("missing explicit") != std::string::npos);
    auto settings = e.mp6.controller_settings;
    settings.sensor_filter_start = .0234f;
    settings.history_len_start = 9;
    CHECK(g_app.mp6SetControllerSettings(e, settings, &why));
    strcpy(e.mp6.link_port, "COM89");
    cJSON* json = cJSON_CreateObject();
    saveEntityToJSON(json, e);
    Entity restored;
    restored.id = 90;
    loadEntityFromJSON(restored, json);
    CHECK(restored.mp6.controller_settings.supplied);
    CHECK_EQ(restored.mp6.controller_settings.history_len_start, 9);
    CHECK(std::abs(restored.mp6.controller_settings.sensor_filter_start - .0234f) < 1e-7f);
    CHECK(strcmp(restored.mp6.link_port, "COM89") == 0);
    cJSON* cs = cJSON_GetObjectItem(cJSON_GetObjectItem(json, "mp6"), "controller_settings");
    cJSON_DeleteItemFromObject(cs, "sensor_filter_start");
    Entity legacy; legacy.id = 91;
    loadEntityFromJSON(legacy, json);
    CHECK(!legacy.mp6.controller_settings.supplied);
    cJSON_Delete(json);

    // Exercise real save/load for malformed persisted design records.
    Entity saved_design;
    saved_design.id = 92;
    saved_design.type = EntityType::HIL;
    saved_design.hil_backend = HilBackend::MP6;
    setupTestGeometry(saved_design);
    saved_design.mp6.design.supplied = true;
    saved_design.mp6.design.pivot_z_provenance = 2;
    saved_design.mp6.design.point_zero_supplied = true;
    saved_design.mp6.design.base_radius_mm = saved_design.mp6.design.bot_b_mm;
    saved_design.mp6.design.platform_radius_mm = saved_design.mp6.design.top_b_mm;
    saved_design.mp6.design.x_offset_mm = saved_design.mp6.design.off_x_mm;
    saved_design.mp6.platform_config_present = true;
    snprintf(saved_design.mp6.platform_config_text, sizeof(saved_design.mp6.platform_config_text),
             "%s", makeSnapshotPltc().c_str());
    saved_design.mp6.controller_settings = e.mp6.controller_settings;
    for (int i = 0; i < 6; ++i) {
        saved_design.mp6.design.base_z_mm[i] = -30.0f;
        saved_design.mp6.design.top_z_mm[i] = 10.0f;
    }
    saved_design.mp6.design.base_z_mm[0] = 0.0f;
    saved_design.mp6.design.top_z_mm[0] = 0.0f;
    {
        cJSON* valid_document = cJSON_CreateObject();
        saveEntityToJSON(valid_document, saved_design);
        Entity valid_loaded;
        valid_loaded.id = saved_design.id;
        valid_loaded.type = EntityType::HIL;
        loadEntityFromJSON(valid_loaded, valid_document);
        CHECK(valid_loaded.mp6.design.supplied);
        CHECK_EQ(valid_loaded.mp6.design.pivot_z_provenance, 2);
        CHECK(valid_loaded.mp6.design.point_zero_supplied);
        CHECK_EQ(valid_loaded.mp6.design.point_zero_x_mm, 0);
        CHECK_EQ(valid_loaded.mp6.design.base_z_mm[0], 0);
        CHECK_EQ(valid_loaded.mp6.design.top_z_mm[0], 0);
        cJSON_Delete(valid_document);
    }
    {
        struct InvalidDimension { const char* key; float zero; };
        const InvalidDimension invalid[] = {
            {"bot_b_mm", 0}, {"top_b_mm", 0}, {"bot_a_mm", 0},
            {"top_a_mm", 0}, {"crank_length_mm", 0}, {"rod_length_mm", 0}
        };
        int id = 93;
        for (const auto& field : invalid) {
            cJSON* document = cJSON_CreateObject();
            saveEntityToJSON(document, saved_design);
            cJSON* design = cJSON_GetObjectItem(cJSON_GetObjectItem(document, "mp6"), "design");
            cJSON_ReplaceItemInObject(design, field.key, cJSON_CreateNumber(field.zero));
            cJSON_ReplaceItemInObject(design, "off_x_mm", cJSON_CreateNumber(0));
            cJSON_ReplaceItemInObject(design, "base_radius_mm", cJSON_CreateNumber(230));
            cJSON_ReplaceItemInObject(design, "platform_radius_mm", cJSON_CreateNumber(14));
            cJSON_ReplaceItemInObject(design, "x_offset_mm", cJSON_CreateNumber(15));
            cJSON_ReplaceItemInObject(design, "point_zero_x_mm", cJSON_CreateNumber(5));
            Entity loaded;
            loaded.id = id++;
            loaded.type = EntityType::HIL;
            loaded.mp6.platform_number_expected = 1;
            loadEntityFromJSON(loaded, document);
            CHECK_EQ(loaded.mp6.design.off_x_mm, 0);
            CHECK_EQ(loaded.mp6.design.x_offset_mm, 15);
            CHECK_EQ(loaded.mp6.design.point_zero_x_mm, 5);
            CHECK(!loaded.mp6.design.supplied);
            CHECK(!loaded.mp6.config_ready);
            CHECK(!loaded.mp6.viz.solver_ready);
            auto owner = std::make_unique<mp6::MemoryLink>();
            mp6::MemoryLink* link = owner.get();
            link->open("invalid-loaded-design");
            CHECK(g_app.mp6ConnectWithLink(loaded, std::move(owner), &why));
            CHECK(!loaded.mp6.design.supplied);
            CHECK(!loaded.mp6.config_ready);
            CHECK(!g_app.mp6_runtimes[loaded.id]->solver_ready);
            CHECK(!g_app.mp6_runtimes[loaded.id]->neutral_valid);
            CHECK(g_app.mp6Command(loaded, Mp6Intent::Identify, &why));
            g_app.mp6UpdateEntity(loaded);
            link->inject(makeType42(3, 253));
            g_app.frame_time += 0.016;
            g_app.mp6UpdateEntity(loaded);
            CHECK(loaded.mp6.identity_ok);
            CHECK(!loaded.mp6.config_ready);
            CHECK(!g_app.mp6Command(loaded, Mp6Intent::Configure, &why));
            CHECK(why.find("design geometry rejected") != std::string::npos);
            CHECK(!g_app.mp6Command(loaded, Mp6Intent::Initialize, &why));
            auto writes = link->takeWrites();
            for (const auto& write : writes)
                CHECK(!(write.size() >= 3 && write[0] == 0xFE && write[1] == 0xFC
                        && (write[2] == 2 || write[2] == 5)));
            CHECK(!g_app.mp6_runtimes[loaded.id]->solver_ready);
            CHECK(!g_app.mp6_runtimes[loaded.id]->neutral_valid);
            g_app.mp6Disconnect(loaded);
            cJSON_Delete(document);
        }
    }
    using CorruptDesign = void (*)(cJSON*);
    const CorruptDesign corruptions[] = {
        [](cJSON* d) { cJSON_ReplaceItemInObject(d, "point_zero_x_mm", cJSON_CreateString("0")); },
        [](cJSON* d) { cJSON_DeleteItemFromObject(d, "point_zero_x_mm"); },
        [](cJSON* d) { cJSON_ReplaceItemInArray(cJSON_GetObjectItem(d, "base_z_mm"), 0, cJSON_CreateNull()); },
        [](cJSON* d) { cJSON_DeleteItemFromArray(cJSON_GetObjectItem(d, "top_z_mm"), 5); },
        [](cJSON* d) { cJSON_ReplaceItemInObject(d, "point_zero_x_mm", cJSON_CreateNumber(NAN)); }
    };
    for (CorruptDesign corrupt : corruptions) {
        cJSON* document = cJSON_CreateObject();
        saveEntityToJSON(document, saved_design);
        cJSON* design = cJSON_GetObjectItem(cJSON_GetObjectItem(document, "mp6"), "design");
        corrupt(design);
        Entity loaded;
        loaded.id = saved_design.id;
        loaded.type = EntityType::HIL;
        loadEntityFromJSON(loaded, document);
        CHECK(!loaded.mp6.design.supplied);
        CHECK_EQ(loaded.mp6.design.pivot_z_provenance, 0);
        CHECK(!loaded.mp6.design.point_zero_supplied);
        CHECK(!loaded.mp6.viz.solver_ready);
        loaded.mp6.link_open = true; // exercise production command guards after real load
        loaded.mp6.identity_ok = true;
        loaded.mp6.config_ready = true;
        CHECK(!g_app.mp6Command(loaded, Mp6Intent::Configure, &why));
        CHECK(why.find("design attachment") != std::string::npos);
        loaded.mp6.paired_config = true;
        loaded.mp6.config_send_ok = true;
        CHECK(!g_app.mp6Command(loaded, Mp6Intent::Initialize, &why));
        CHECK(why.find("neutral zero-pose") != std::string::npos);
        g_app.mp6Disconnect(loaded);
        cJSON_Delete(document);
    }

    for (const char* command : {"mp6_disconnect", "mp6_connect", "mp6_command", "mp6_controller_settings"}) {
        for (const char* id : {"999", "1", "\"999\"", "0.9", "null", "true", "-1", "2147483648", "1e100"}) {
            std::string request = std::string("{\"cmd\":\"") + command + "\",\"args\":{\"entity\":" + id + "}}";
            cJSON* reply = cJSON_Parse(DispatchControlCommand(request).c_str());
            CHECK(cJSON_IsFalse(cJSON_GetObjectItem(reply, "ok")));
            const char* message = cJSON_GetObjectItem(reply, "error")->valuestring;
            CHECK(std::string(message).find("documentation mode") == std::string::npos);
            cJSON_Delete(reply);
        }
    }
    CHECK(g_app.mp6RecoverKnownState(e, &why));
    e.mp6.controller_settings = Mp6MachineSettings{};
    auto response = DispatchControlCommand(R"({"cmd":"mp6_controller_settings","args":{"entity":0}})");
    CHECK(response.find("missing explicit") != std::string::npos);
    CHECK(!e.mp6.controller_settings.supplied);
    Entity& second = g_app.addEntity("Second MP6", EntityType::HIL, HilBackend::MP6);
    auto second_owner = std::make_unique<mp6::MemoryLink>();
    second_owner->open("second");
    CHECK(g_app.mp6ConnectWithLink(second, std::move(second_owner), &why));
    response = DispatchControlCommand(R"({"cmd":"mp6_disconnect","args":{}})");
    CHECK(response.find("\"ok\":true") != std::string::npos);
    CHECK(second.mp6.link_open); // omitted target selects first MP6 only
    response = DispatchControlCommand(R"({"cmd":"mp6_disconnect","args":{"entity":2}})");
    CHECK(response.find("\"ok\":true") != std::string::npos);
    CHECK(!second.mp6.link_open);
    response = DispatchControlCommand(R"({"cmd":"mp6_connect","args":{"entity":0}})");
    CHECK(response.find("disabled in documentation mode") != std::string::npos);

    cJSON* full = cJSON_CreateObject();
    e.mp6.controller_settings = settings;
    saveEntityToJSON(full, e);
    cJSON* full_settings = cJSON_GetObjectItem(cJSON_GetObjectItem(full, "mp6"), "controller_settings");
    cJSON_AddNumberToObject(full_settings, "entity", 0);
    char* body = cJSON_PrintUnformatted(full_settings);
    e.mp6.controller_settings = Mp6MachineSettings{};
    response = DispatchControlCommand(std::string("{\"cmd\":\"mp6_controller_settings\",\"args\":") + body + "}");
    cJSON_free(body);
    cJSON_Delete(full);
    CHECK(response.find("\"ok\":true") != std::string::npos);
    CHECK(e.mp6.controller_settings.supplied);
    response = DispatchControlCommand(R"({"cmd":"mp6_controller_settings","args":{"entity":0,"sensor_filter_start":0.0234,"history_len_start":9}})");
    CHECK(response.find("\"ok\":true") != std::string::npos);
    CHECK_EQ(e.mp6.controller_settings.history_len_start, 9);
    for (const char* bad_value : {"null", "\"0.1\"", "true", "1e999"}) {
        response = DispatchControlCommand(std::string(R"({"cmd":"mp6_controller_settings","args":{"entity":0,"sensor_filter_start":)") + bad_value + "}}");
        CHECK(response.find("\"ok\":false") != std::string::npos);
    }

    settings.explicit_fields = Mp6MachineSettings::all_fields;
    CHECK(g_app.mp6SetControllerSettings(e, settings, &why));
    setupTestGeometry(e);
    e.mp6.design.supplied = true;
    e.mp6.design.pivot_z_provenance = 2;
    e.mp6.design.point_zero_supplied = true;
    for (const char* bad : {"bogus", "", "NaN", "Inf", "1e999", "999999999"}) {
        for (const char* cell : {"PID 0\t\t11", "Pot 1\t\t3543"}) {
            std::string text = makeSnapshotPltc();
            size_t at = text.find(cell);
            size_t num = text.find("\t\t", at) + 2;
            text.replace(num, text.find('\t', num) - num, bad);
            CHECK(g_app.mp6ImportPlatformConfigText(e, text.c_str(), &why));
            CHECK(!e.mp6.config_ready);
            auto owner = std::make_unique<mp6::MemoryLink>();
            auto* link = owner.get(); link->open("invalid");
            CHECK(g_app.mp6ConnectWithLink(e, std::move(owner), &why));
            g_app.mp6UpdateEntity(e);
            CHECK(!g_app.mp6Command(e, Mp6Intent::Configure, &why));
            CHECK(link->takeWrites().empty());
            CHECK(!g_app.mp6_runtimes[e.id]->pending_pid_valid);
            g_app.mp6Disconnect(e);
        }
    }
}

int main() {
    for (const char* mode : {"normal", "lostack", "pressureoff", "wirefloat", "potround", "stepround", "savedzero", "priority", "badreplacement", "configurestop", "stale", "failedrefresh", "startstop", "parkedstop", "stagedstartstop", "dockedstop", "shutdownstop", "perstart", "jump", "disabled", "recovery", "quant0", "quant1", "quant2", "quant3", "quant4", "quant5"})
        runScenario(mode);
    testBoundaries();
    printf("Total desktop checks: %d, failures: %d\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
