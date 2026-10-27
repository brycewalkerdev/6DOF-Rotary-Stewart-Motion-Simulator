# MP6 first live test checklist

First connection of the desktop app's MP6 backend to the real, unmodified
Motum Arduino Mega controller. The software has offline acceptance only (see
[MP6](MP6.md)): no COM traffic, powered motion or commissioning has been
verified. This checklist raises risk one step at a time. Do not skip ahead; at
each gate, stop if anything differs from what is expected.

Record the results in the [log table](#results-log) at the end.

## Abort criteria (any phase)

Press the **physical** E-stop / open the motor-drive breaker immediately if:

- any axis moves when no motion was commanded, or moves the wrong way;
- motion is larger or faster than expected, or an axis hits a stop;
- a strut, crank or rod binds, slips or makes unusual noise;
- the app shows `IdentityLost` while motors are powered (the host can no longer
  send authenticated stop frames; see [MP6 § safety](MP6.md));
- a fault code 210/220/230/240/250 appears.

After any abort, establish a known physical state with the physical controls
*first*, then use **Known physical state: reset & reconnect**. Never treat a
UI click as proof of the machine's state.

## Phase 0 — People, machine, fallback

- [ ] Two people: one operating the app, one at the physical E-stop and motor
      breaker with a clear view of the platform. Nobody on or under the
      platform; the motion envelope is clear.
- [ ] Physical E-stop and motor drive breaker / main disconnect located and
      tested (they cut motor power independently of the PC).
- [ ] Pneumatic supply: regulator and dump/vent valve within reach; know the
      pressure the original MotSim used for this payload.
- [ ] The original MotSim installation is intact and still drives the rig
      normally (it is the fallback and the reference). Record its version.
- [ ] Platform is docked (fully down) and the struts are vented before starting.
- [ ] Build is current: `mp6` at or after `2a3ef38`; Release build and CTest 6/6
      pass on this PC (commands in [MP6 § build](MP6.md)).

## Phase 1 — Collect the real machine data

The app refuses readiness until all of these are explicitly supplied. Take
them from this rig's original MotSim install and measurements, **not** from the
recovered reference snapshot in `motum_resources/`.

- [ ] This rig's `PlatformConfig.pltc` (StepRev, pot tables, gear ratios,
      accel/decel, Max Motor RPM).
- [ ] Platform number expected from the controller's identity (pairing).
- [ ] Design geometry: BotA, BotB, TopA, TopB, crank and rod lengths, ZOff/X
      offset for this platform design.
- [ ] Per-axis pivot z values. The original `Other\<Platform>.txt` was not
      recovered: copy it from the original install, or measure.
- [ ] MotConv point zeros (lblX / lblZ) from the original install.
- [ ] All 16 controller settings (see [MP6 § controller settings](MP6.md)),
      including `bench_test`, `comms_timeout_ms`, `onboard_shutdown`,
      `mot_val_final` and the pressure plan. Copy the values the original app
      uses on this rig; do not invent them.
- [ ] Screenshot or note the original MotSim's neutral pose and per-axis
      docked/neutral step values, for later comparison.

## Phase 2 — Offline dry run (documentation mode, no hardware)

Launch with `STEWART_DOCUMENTATION_MODE=1` so no COM port can open.

- [ ] Add an **HIL · MP6** entity; import the real `.pltc`; enter the design
      fields, pivot z (complete the two-step review), point zeros (Confirm),
      and all 16 controller settings (each confirmed). Apply the Platform form.
- [ ] Readiness checklist shows every non-link item satisfied; the Platform
      readiness note names nothing missing.
- [ ] MaxPPS shown equals `round(MaxRPM × StepRev / 60)` with your numbers.
- [ ] The 3D view's neutral pose and crank angles look physically plausible.
      Compare neutral steps against the original app's values from Phase 1.
- [ ] Save settings, restart the app, reload: all values and confirmations
      survive.

## Phase 3 — Wire check without the controller (recommended)

This catches protocol or parameter mistakes at zero mechanical risk.

- [ ] Install a virtual null-modem pair (e.g. com0com) and a serial
      logger on the far end.
- [ ] Launch **without** `STEWART_DOCUMENTATION_MODE` (normal mode). Connect
      the MP6 entity to the virtual port (115200 baud).
- [ ] Confirm the app sends identity probes (command 3) and nothing else
      before identity, and that it times out cleanly with no reply.
- [ ] If possible, capture the original MotSim's traffic to the real
      controller with a passive serial sniffer and compare its 22 configuration
      frames (command 5) parameter-by-parameter with what this app would send
      for the same data (the desktop test log and `mp6_status` expose the
      values). Investigate any difference before Phase 5.

## Phase 4 — Real controller, motors unpowered, air off

Motor drive breaker **open**, pneumatic supply off/vented, controller powered
over USB.

- [ ] Launch in normal mode. Connect to the controller's COM port. The link
      holds DTR low, so connecting should **not** reset the Mega. Note whether
      it resets anyway (watch its LED / boot behaviour).
- [ ] **Identify:** identity arrives (type-42 echo within 200 ms, up to ten
      probes) and the platform number matches the expected value (pairing
      matched).
- [ ] Telemetry: `mp6_status` / I/O table updates continuously. Move each
      crank gently by hand, if it is safe to, and confirm the matching pot
      feedback channel changes, in the expected direction (checks axis order
      and pot sign).
- [ ] Docked feedback: each active axis reads within about 40 calibrated steps
      of its calibrated dock position. If not, the calibration (PotLo,
      PotScale, Angle-Docked, Gearbox) is wrong. Stop here and fix it.
- [ ] **Configure:** all 22 command-5 frames ACKed; "config transaction
      verified" shows in readiness. Do **not** Initialize.
- [ ] Toolbar **E-STOP**: latches; further motion commands are refused. Run the
      known-state recovery (reset & reconnect, fresh Identify and Configure).
- [ ] Pull the USB cable: the app reports the link/identity loss and does not
      auto-re-arm on reconnect.
- [ ] Disconnect cleanly.

## Phase 5 — Motors powered: Initialize and Shutdown only

Air at the original app's starting pressure. Person at the physical E-stop,
hand on it.

- [ ] Connect, Identify, Configure (fresh ACK).
- [ ] **Initialize:** the controller powers the motors, checks the docked
      position, sets pressure and raises from dock to neutral over ≥ 1 s.
      Watch every axis: all rise together, smoothly, by similar amounts, and
      stop at neutral. State reaches observed **Parked**.
- [ ] Compare the physical neutral against the original MotSim's neutral
      (level, centred, same height). A tilted or offset neutral means
      geometry, pivot z, point zeros or an axis sign is wrong. Shut down and
      fix it before going further.
- [ ] **Shutdown:** platform descends using the shutdown PID profile, settles
      at the dock (observed **Docked**), then vents. No vent before the dock
      is confirmed.
- [ ] Repeat Initialize → Shutdown once more for consistency.
- [ ] From Parked, press the **physical** E-stop: motion stops, and the app
      latches/reports the fault code. Recover per the abort procedure.

## Phase 6 — First commanded motion

Start from Parked. Use the **Manual Sliders** plugin with the intensity
slider turned well down, so every motion is small and deliberate.

- [ ] **Start:** Normal PID profile ACKed, the first targets equal neutral,
      then a 0.6 s blend into Running. The platform should not move
      noticeably at Start.
- [ ] `achieved_target_hz` ≈ 60 and `missed_cadence_slots` stays near zero.
- [ ] Optional axis test (commissioning): with intensity still low and the
      physical E-stop operator ready, enable **Axis test (commissioning)**.
      Move each axis by +2° one at a time, return it to neutral, and confirm
      the correct physical crank moves in the expected direction and faint
      measured crank ghosts track the solid commanded cranks in the 3D view.
      The platform is shown at neutral, not as a solved pose; rods that cannot
      reach their neutral attachments are marked unsolved. Stop immediately on
      any mismatch.
      Disable Axis test, then confirm it is disabled and the return-to-neutral
      motion and pose blend have completed before continuing.
- [ ] One degree of freedom at a time, small amplitude, slow: heave up/down,
      then surge, sway, roll, pitch, yaw. For each, confirm the **direction**
      matches the app's 3D view and the sign convention you expect, and that
      the returning motion is centred.
- [ ] **Stop** mid-motion: all axes stop; pending lifecycle work cancels.
- [ ] **Park**, then **Shutdown** to Docked and vent.
- [ ] Only after all six DOFs pass: try a short Capture Playback or the target
      game plugin at low intensity, then raise intensity gradually across
      sessions.

## Results log

| Phase | Step | Date / operator | Pass / fail | Observed values / notes |
| --- | --- | --- | --- | --- |
| 0 | Safety and fallback ready | | | |
| 1 | Machine data collected | | | |
| 2 | Offline readiness complete | | | MaxPPS = |
| 3 | Virtual-port wire check | | | |
| 4 | Identify / pairing | | | Platform # = |
| 4 | Docked feedback vs calibration | | | Max error (steps) = |
| 4 | Configure ACK (22 frames) | | | |
| 4 | E-STOP latch and recovery | | | |
| 5 | Initialize → Parked | | | |
| 5 | Neutral matches original app | | | |
| 5 | Shutdown → Docked → vent | | | |
| 5 | Physical E-stop from Parked | | | Fault code = |
| 6 | Start blend, 60 Hz cadence | | | Hz = , missed = |
| 6 | Optional +2° axis test / return | | | Axis order, direction, feedback = |
| 6 | Six DOF directions | | | |
| 6 | Stop / Park / Shutdown | | | |

Report any mismatch with the original MotSim's behaviour along with the
`mp6_status` output and the app log. Those are the first real data points for
commissioning.
