# MP6 (Motum Mega rotary Stewart) backend

Part of the desktop app's HIL support. The MP6 backend talks to the Motum
Arduino Mega university kit controller (the "MP6" recovered-app protocol) using
a second, independent serial link. It is a separate code path from the ESP32
COBS HIL backend — settings stay compatible, transports and wires do not mix.

Status: **Implemented with offline production-path regression coverage; hardware is not commissioned.** The onboard control mode
(command 2 stage-target streaming at 60 Hz) is the primary supported control mode;
host PID was reviewed and rejected. This page describes the implementation,
safety architecture, and provenance boundaries.

## Scope at a glance

| Area | State |
| --- | --- |
| Binary protocol codec | Done, byte-exact offline tests (`mp6_protocol_codec`: 741 checks) |
| Session engine (single TX owner) | Done, scripted in-memory tests (`mp6_session_lifecycle`: 1027 checks) |
| Config import (`.pltc`) | Done: typed parse + calibration derivation |
| Rotary geometry (crank solver) | Done: exact AtanKD crank solve, neutral zero pose, travel limits (`mp6_config_geometry`: 744 checks (original 684 plus 60 parser regressions)) |
| Desktop app integration | Done: UI, control API, persistence, telemetry mirroring, glue tests (`mp6_desktop_glue`: 1871 checks across 25 deterministic scenarios and boundary checks) |
| Python automation (MCP / JSON-RPC) | Done: multi-entity support, shutdown verb, controller settings (26/26 tests passing; wrapper framing tests) |

## Safety Posture & Architectural Boundaries

- **Documentation mode is default.** The app never opens a real COM port for
  the MP6 backend (or any backend) unless documentation mode is off, and every
  COM open is additionally guarded. See [App automation](APP_AUTOMATION.md).
- **E-STOP is a single funnel.** The toolbar button and the control API
  `estop` both go through `App::requestEstop()`, which stages the MP6 stop
  intent (bypassing fault-latch refusals) and stops SIL motion. E-STOP latches
  until the explicit known-physical-state recovery action. A plain or automatic reconnect retains the desktop latch.
- **Fault codes latch.** Codes 210/220/230/240/250 latch in the session and
  suppress further TX until the session is replaced after physical recovery; only
  Disconnect and E-stop are accepted while latched.
- **Active-motion and powered-state mutation guards.** Platform configuration
  import, geometry refreshes, controller settings changes, and Identify commands
  are strictly refused while the platform is actively moving or while motors
  are enabled (`Running`, `Initializing`, `Parking`, `Parked`, `Docked`, or
  `Stopped`). To edit configuration or geometry, the operator must execute an
  orderly shutdown first.
- **Powered identity loss boundary.** If communications or identity is lost
  while motors are powered (`IdentityLost`), the host cannot transmit
  authenticated stop frames, shutdown sequences, or re-identify commands. The
  operator must use independent physical machine controls (hardware E-stop
  pushbutton, motor drive breaker / main disconnect) to remove motor power and
  re-establish a known physical machine state before reconnecting. Reconnect
  does **NOT** automatically re-arm or re-identify a powered fault.
- **500 ms stale-only vent wait.** Sourced dock settle requires valid telemetry
  before pneumatic venting. If stored observations still prove dock position but
  telemetry is temporarily delayed across a profile ACK, the session grants a
  bounded 500 ms wait (an explicit engineering choice, not primary source). Any
  explicit AV loss, axis out of tolerance, or off-dock observation aborts
  immediately with zero vents.
- **Calibration feedback continuation.** During PID profile transitions (e.g.
  across 22-frame command 5 transactions), the controller retains its calibration.
  The session maintains calibration usability across the transaction, preventing
  spurious feedback invalidation.
- **Normal PID profile applied on explicit Start.** Sourced from
  `frm_Main.cs:28031, 34642`: initialization applies Configure (PID 1) and
  Lifecycle (PID 2) profiles, holding neutral in Parked under PID 2. An explicit
  Start command initiates the full 22-frame transaction for Normal PID (mode 0),
  requiring controller ACK and fresh readiness observation before entering
  Running. No command-2 target stream frames are emitted prior to the Normal PID
  ACK.
- **Cadence normalisation and sustained shortfall.** The target stream cadence
  is fixed to 60 Hz (the sole protocol-supported rate). Sustained rate shortfall
  and stalls are accurately tracked in `missed_cadence_slots` and
  `achieved_target_hz`.
- **Unused parameter 43 live packet conflict.** Parameter 43 in configuration
  transactions is 40, whereas a separate 2-frame live slider packet uses 4. This
  live slider path is unused in the lifecycle state machine and left unimplemented.
- **Provenance-gated real data.** The pivot z points file
  (`Other\<Platform>.txt`) is unrecovered, so per-axis pivot z values are
  explicit user entries gated by a three-state provenance flag
  (Unreviewed → **blocks the crank solver**, Reviewed after an explicit
  two-step UI review action). Fixture placeholders never reach the live path,
  and editing values after a review drops the flag back to Unreviewed.
- **MotConv point zeros confirmation.** MotConv rotation-center offsets
  (Module2.lblX/lblZ) require explicit user confirmation via "Confirm Point Zeros"
  before the crank solver is authorized.
- **Neutral/docked semantics follow the session verbatim**: neutral `Parked`
  (ready-neutral hold after initialization) and observed `Docked` (calibration
  dock settle, may vent) are distinct reported states — neither is relabelled
  as the other, and neither is presented as a shutdown success.
- **Target pre-feeding at Parked**: The desktop pipeline pre-feeds valid step
  targets while in `Parked` (as well as `Running`), satisfying the session's
  fresh target readiness guard and cleanly resolving the start-motion deadlock.

## What the App Shows

- Entity type **HIL · MP6**: entity menu → *Add HIL (MP6 Motum Mega)*, or the
  control API with `"backend": "mp6"`.
- Settings tab (MP6 branch): backend selector (locked while the link is open),
  connect/disconnect, fixed 60 Hz cadence display with achieved streaming rate
  and missed slots, expected platform number/pairing, the `.pltc` config
  import, explicit design fields, controller parameters form, and an observation-driven
  readiness checklist.
- Platform tab (design form): design-table numbers, per-axis pivot z entry
  with the two-step review gate, MotConv point zeros confirmation, and Apply —
  the crank solver rebuilds on the main thread and its readiness note names
  exactly what is still missing.
- 3D view: base/platform attachments drawn **only** from the explicitly
  entered design geometry; the viewport overlay explicitly notes: "target pose
  only — measured feedback in I/O table". The two are never conflated, and the
  model is never labelled as feedback. When a fresh crank solve ran, the rotary
  chain is drawn from the solver's own recorded joints.
- Time-series "Servo Angles / Feedback": MP6 entities plot calibrated feedback
  steps; uncalibrated axes (`SHRT_MIN`) display as "no calibration".

## Control API (TCP Control Server) & MCP Tools

Verbs for MP6 (real connects and lifecycle commands are refused in documentation mode; validated host configuration staging remains available):

- `mp6_status` — mirrored observation including `achieved_target_hz`,
  `missed_cadence_slots`, `cadence_normalised`, `shutdown_pending`, and
  `applied_pid_mode` (always allowed).
- `mp6_connect` / `mp6_disconnect` — raw serial link lifecycle (never COBS).
- `mp6_command` — stage one intent:
  `identify|configure|initialize|start|stop|park|shutdown|disconnect`.
  Strict multi-entity targeting: a supplied ID must be a finite integral JSON number in `[0, INT_MAX]`, identify an existing entity, and select the MP6 backend. Only an omitted ID selects a default.
- `mp6_config_import` — parse/derive/stage raw `.pltc` text.
- `mp6_recover_known_state` with `physical_state_confirmed: true` — disconnect and clear the desktop latch after independent physical controls establish a known state. It does not connect or identify.
- `mp6_controller_settings` — set explicit machine-level controller parameters
  and pressure plan.

Global motion control:
- `motion` (`{"on": true|false}`) — calls `App::startMotion()` and
  `App::stopMotion()`, properly coordinating SIL motion, input sources, and
  staging MP6 intents.

## Configuration & Machine Settings Semantics

### Source Snapshot Defaults vs. Actual Rig Values vs. User-Configured Settings

`PlatformConfig.pltc` supplies machine numbers (StepRev, pot tables, gear
ratios, accel/decel) and the app derives per-axis pot↔step calibration. The
file does **not** supply the design geometry (BotA/BotB/TopA/TopB) or pivot z
coordinates — those are explicit entity fields and gate readiness.

**MaxPPS is materialized by the desktop app at import**: the cmd-5 MaxPPS
parameter equals `round(Max Motor RPM × Step/Rev / 60)` (snapshot:
230×3200/60 → 12267). The `.pltc` "Max Motor RPM" row is machine data and
wins when present; when it is absent, an explicit MaxRPM field in the MP6
settings supplies the value.

**Design ZOff vs. Pltc Row 506**: In accordance with `frm_Design.cs:686` and
`frm_Main.cs:16610`, `Module2.ZOff` for `MOT155-D2` is 550 mm, yielding the
source-backed 530 mm `Z_Offset`. The `.pltc` "Z Offset" row 506 is unconsumed
in the primary source and remains strictly unconsumed here.

**Quantization:** The recovered .NET Framework `Single.ToString("F3")` first
uses a seven-significant-digit decimal representation and then rounds fixed
three-decimal output away from zero. That text is parsed and multiplied by
10000 before exact half-even wire rounding: `.0075f` becomes `"0.008"`, then
80; the negative becomes -80. Promoting the binary float into C `snprintf`
F3 is different. A checked-in oracle contains 30 results generated with installed
Windows PowerShell 5.1/.NET Framework, including adjacent floats and signs;
six variants also run through the complete App Normal-PID wire transaction.
P/D/filter parameters retain their source float intermediates; the production
fixture's DFilter `20.5000001` becomes float `20.5` and wire 20.

### Explicit settings, drafts and recovery

Initial controller settings require all 16 fields to be explicitly supplied:
`bench_test`, `comms_timeout_ms`, `onboard_shutdown`, `mot_val_final`,
`sensor_filter`, `history_len`, `sensor_filter_start`, `history_len_start`,
`deadband_enable`, `av_filter`, `limits_enable`, `info_motor`, `pid_index`,
`pressure_enabled`, `pressure_aim_supplied`, `pressure_aim_psi`.
The UI exposes every value and an individual confirmation for each field.
An empty initial API update and a default-constructed C++ record are rejected.
Partial API edits inherit only a previously complete accepted record. JSON
booleans must be booleans; integer settings reject fractional numbers. Filters
must be finite and 0..1, histories 1..10, timeout 100..10000 ms, motor selector
1..6, pressure 0..150 psi, and MotValFinal 63 or 100. Binary switches are 0/1.
`pid_index` is retained metadata (0..2); the lifecycle selects modes 1/2/0/2.
Run filter/history apply to Normal; Start filter/history apply to Configure
and Lifecycle. Both pairs and explicit supply provenance persist; legacy
records missing fields or supply provenance remain unresolved.

The geometry form edits a draft. Review and point-zero actions affect that
draft until Apply accepts it. Apply guards the accepted entity before mutation.
An accepted change, cleared input or failed validation retires solver, neutral,
target and configuration ACK authority; Initialize requires a rebuilt, reviewed
current neutral and a fresh configuration ACK. Powered edits are refused so the
known orderly-shutdown path remains available.

Both Start entry points enable the global input gate. Each MP6 rig separately
holds its neutral target during Normal profile application and begins a 0.6 s
smoothstep blend when ACK and fresh readiness actually allow Running. Its first
command-2 target is neutral even if another rig or input source is already
running. Global STOP works when the global flag is off, cancels every attached
session's pending lifecycle work, and preserves stronger E-STOP/disconnect intents.

After STOP, a rig in `Stopped` cannot resume through Start alone. Use Park or
Shutdown as appropriate, then Initialize to observed neutral readiness before
Start. The global Start button does not bypass this session requirement.

MP6 pose amplitudes use the entity's axis scales, intensity and per-axis gains.
Review these against the actual rig workspace during commissioning; a reachable
single-axis pose does not establish that combined motion is reachable.

For an E-STOP, fault, SafeOff or powered identity loss, use independent physical
controls to establish a known machine state first. Use **Known physical state:
reset & reconnect** in the UI, or `mp6_recover_known_state` with explicit
confirmation followed by `mp6_connect`. Fresh Identify, pairing, Configure,
Initialize and observed readiness are required. Auto-connect never clears the
latch or re-arms the rig. The offline test completes this whole recovery cycle.

A lost powered PID-profile ACK leaves the controller's applied tuning unverified.
With identity/pairing and healthy telemetry still present, issue explicit
Configure and wait for its ACK, then Initialize and wait for observed Parked,
then Shutdown and observe the Lifecycle ACK, dock settle and SafeOff. The App
regression deliberately drops Normal-profile ACKs and completes this sequence.
If identity is lost or physical state is uncertain, use the physical-controls
recovery path instead. No automatic retry may be interpreted as permission to
resume physical motion.

No real COM connection, wire capture, hardware response/polarity verification,
visual GUI journey, recovered executable or live ESP32 test was performed.
Synthetic feedback follows emitted targets and does not establish real machine
safety. Actual pivots, point zeros, workspace, signs and response must be supplied,
reviewed and commissioned on the user's rig. Original MotSim filter reproduction
remains deferred; the existing desktop cueing pipeline remains in use.

## Build & Offline Test Matrix

```powershell
& "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build app/build --config Release
& "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\ctest.exe" --test-dir app/build -C Release --output-on-failure
python -m unittest discover -s app/tools -v
```

All 6 CTest test suites pass (100%):
1. `recording_precision` — Frame recording rounding and serialization.
2. `documentation_transport_guards` — COM port safety gates under `STEWART_DOCUMENTATION_MODE=1`.
3. `mp6_protocol_codec` (741 checks) — Byte-exact frame encoder/decoder and checksum verification.
4. `mp6_session_lifecycle` (1027 checks) — Single-writer session state machine, tie rounding, dock AV refusal, and guards.
5. `mp6_config_geometry` (744 checks: original 684 plus 60 parser regressions) — .pltc configuration import and rotary crank kinematics.
6. `mp6_desktop_glue` (1871 checks) — Production App, internal persistence and real control-server dispatcher with MemoryLink and scripted telemetry. Repeated baseline assertions run across 25 scenarios: Stop during configuration/profile work and observed Parked/Docked; late ACKs; stale/failed geometry; malformed and rejected profile replacement; per-rig Start blending; disabled E-STOP and complete explicit recovery; pressure disable; lost-ACK recovery; source geometry and wire quantization. Shared production UI review actions are tested; no GUI click or visual layout test is claimed.

Python MCP / Automation test suite: 26/26 tests passing. These establish wrapper framing; C++ tests establish dispatcher behavior.
