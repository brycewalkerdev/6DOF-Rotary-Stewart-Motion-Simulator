cmake_minimum_required(VERSION 3.20)
project(stewart-platform LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Some fetched deps (e.g. cJSON v1.7.18) still declare cmake_minimum_required
# < 3.5, which CMake >= 4.0 (the VS 2026 CI runner) rejects outright. Provide a
# compatible policy floor so those subprojects still configure. This variable is
# read by each subproject's cmake_minimum_required; it is silently ignored on
# CMake < 3.31, so it is safe for older local toolchains too.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# ── Fetch dependencies ──────────────────────────────────────────────
include(FetchContent)

# GLFW (windowing + OpenGL context)
FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
    GIT_SHALLOW    TRUE
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# Dear ImGui (immediate-mode GUI)
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.8-docking
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

# ImPlot (plotting for ImGui)
FetchContent_Declare(implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        v0.16
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(implot)

# cJSON (lightweight JSON parser)
FetchContent_Declare(cjson
    GIT_REPOSITORY https://github.com/DaveGamble/cJSON.git
    GIT_TAG        v1.7.18
    GIT_SHALLOW    TRUE
)
set(ENABLE_CJSON_TEST OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_AND_STATIC_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(cjson)

# ── ImGui library (compiled from fetched sources) ───────────────────
add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC glfw)
# ImGui uses its own built-in GL loader (imgui_impl_opengl3_loader.h) by default

# ── ImPlot library ──────────────────────────────────────────────────
add_library(implot_lib STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)
target_include_directories(implot_lib PUBLIC ${implot_SOURCE_DIR})
target_link_libraries(implot_lib PUBLIC imgui_lib)

# ── Shared math from the stewart-core submodule (single source of truth = atan2f) ──
set(CONTROLLER_DIR ${CMAKE_SOURCE_DIR}/../controller)     # still needed below for cobs.h
set(STEWART_CORE ${CONTROLLER_DIR}/stewart-core)

add_library(stewart_math STATIC
    ${STEWART_CORE}/src/InverseKinematics.cpp
    ${STEWART_CORE}/src/AxisScaling.cpp
    ${STEWART_CORE}/src/MotionCueing.cpp
)
target_include_directories(stewart_math PUBLIC ${STEWART_CORE}/include)
# Desktop build — disable ESP-IDF specific code
target_compile_definitions(stewart_math PRIVATE
    SIL_BUILD=1
    _USE_MATH_DEFINES
)

# ── Main application ────────────────────────────────────────────────
add_executable(stewart-platform WIN32
    src/main.cpp
    src/automation.cpp
    src/app.cpp
    src/ui_panels.cpp
    src/platform_viz.cpp
    src/serial_port.cpp
    src/udp_transport.cpp
    src/plugin_manager.cpp
    src/test_harness_panel.cpp
    src/control_server.cpp
    src/mp6_protocol.cpp
    src/mp6_link.cpp
    src/mp6_session.cpp
    src/mp6_config.cpp
    src/mp6_geometry.cpp
)
target_include_directories(stewart-platform PRIVATE
    src
    ${CONTROLLER_DIR}/include
    ${cjson_SOURCE_DIR}
)
target_link_libraries(stewart-platform PRIVATE
    imgui_lib
    implot_lib
    stewart_math
    cjson
)
target_compile_definitions(stewart-platform PRIVATE
    SIL_BUILD=1
    _USE_MATH_DEFINES
)

# Windows: link winsock2 for UDP, setupapi for serial enumeration, suppress console
if(WIN32)
    target_link_libraries(stewart-platform PRIVATE ws2_32 setupapi opengl32)
    set_target_properties(stewart-platform PROPERTIES LINK_FLAGS "/ENTRY:mainCRTStartup")
endif()

# Read-only safety regression: hardware transports are disabled before any I/O.
include(CTest)
if(BUILD_TESTING)
    add_executable(stewart-recording-tests tests/test_recording.cpp)
    add_test(NAME recording_precision COMMAND stewart-recording-tests)
endif()
if(BUILD_TESTING AND WIN32)
    add_executable(stewart-docs-safety-tests
        tests/test_documentation_mode.cpp
        src/automation.cpp
        src/serial_port.cpp
        src/udp_transport.cpp
    )
    target_include_directories(stewart-docs-safety-tests PRIVATE src ${cjson_SOURCE_DIR})
    target_link_libraries(stewart-docs-safety-tests PRIVATE cjson ws2_32 setupapi)
    add_test(NAME documentation_transport_guards COMMAND stewart-docs-safety-tests)
    set_tests_properties(documentation_transport_guards PROPERTIES
        ENVIRONMENT "STEWART_DOCUMENTATION_MODE=1")
endif()

# MP6 binary codec — pure C++17, no Win32/UI dependencies. Deterministic
# offline conformance coverage for the Unmodified Motum Mega controller stream
# (TX excluding-sequence checksum, reserved word 1, sequence cycle, RX parser
# resync, selector decodes). See docs/MP6.md.
if(BUILD_TESTING)
    add_executable(stewart-mp6-protocol-tests
        tests/test_mp6_protocol.cpp
        src/mp6_protocol.cpp
    )
    target_include_directories(stewart-mp6-protocol-tests PRIVATE src)
    add_test(NAME mp6_protocol_codec COMMAND stewart-mp6-protocol-tests)
endif()

# MP6 session/link lifecycle — scripted in-memory controller (no device),
# byte-exact TX assertions per .mp6-work/session-implementation.md. Doc mode is
# forced so the serial guard itself is exercised (325 checks, serial-refusal
# included).
if(BUILD_TESTING AND WIN32)
    add_executable(stewart-mp6-session-tests
        tests/test_mp6_session.cpp
        src/mp6_session.cpp
        src/mp6_link.cpp
        src/mp6_protocol.cpp
        src/automation.cpp
    )
    target_include_directories(stewart-mp6-session-tests PRIVATE src)
    add_test(NAME mp6_session_lifecycle COMMAND stewart-mp6-session-tests)
    set_tests_properties(mp6_session_lifecycle PROPERTIES
        ENVIRONMENT "STEWART_DOCUMENTATION_MODE=1")

    # MP6 config/geometry core — recovered-source transcription tests (design
    # tables, calibration, crank solver, transform parity). Host-only, no
    # serial/session dependencies.
    add_executable(stewart-mp6-geometry-tests
        tests/test_mp6_geometry.cpp
        src/mp6_config.cpp
        src/mp6_geometry.cpp
    )
    target_include_directories(stewart-mp6-geometry-tests PRIVATE src)
    target_compile_definitions(stewart-mp6-geometry-tests PRIVATE _USE_MATH_DEFINES)
    add_test(NAME mp6_config_geometry COMMAND stewart-mp6-geometry-tests)

    # MP6 desktop glue test — production App adapter offline integration
    add_executable(stewart-mp6-desktop-tests
        tests/test_desktop_mp6_glue.cpp
        src/control_server.cpp
        src/automation.cpp
        src/serial_port.cpp
        src/udp_transport.cpp
        src/plugin_manager.cpp
        src/mp6_protocol.cpp
        src/mp6_link.cpp
        src/mp6_session.cpp
        src/mp6_config.cpp
        src/mp6_geometry.cpp
    )
    target_include_directories(stewart-mp6-desktop-tests PRIVATE
        src
        ${CONTROLLER_DIR}/include
        ${cjson_SOURCE_DIR}
    )
    target_compile_definitions(stewart-mp6-desktop-tests PRIVATE
        SIL_BUILD=1
        _USE_MATH_DEFINES
    )
    target_link_libraries(stewart-mp6-desktop-tests PRIVATE
        stewart_math
        cjson
        ws2_32
        setupapi
    )
    add_test(NAME mp6_desktop_glue COMMAND stewart-mp6-desktop-tests)
    set_tests_properties(mp6_desktop_glue PROPERTIES
        ENVIRONMENT "STEWART_DOCUMENTATION_MODE=1")
endif()
