# Bringing BogPilot AP1 features into this fork

Source material: [BogPilot/openpilot](https://github.com/BogPilot/openpilot), branch
`bogpilot-tesla` (head `04d560f3e`, 2026-11-06), FrogPilot base pinned at
`1e23dec6352cef5a36a87be0af7d7a082b7c48a4`. Companion repos:
[BogPilot/bogpilot-fingerprint](https://github.com/BogPilot/bogpilot-fingerprint),
[BogPilot/Driver-Mover](https://github.com/BogPilot/Driver-Mover),
[BogPilot/opendbc](https://github.com/BogPilot/opendbc) (fork of sunnypilot/opendbc, master only).

This document maps what BogPilot has onto **this** tree (sunnypilot `2026.003.000` + Tesla HW1 port,
car code in `opendbc/car/tesla` via `rampage1999/tesla-hw1-opendbc`). It is a port plan, not a merge:
the two forks do not share a car-code layout, params system, UI, or longitudinal stack, so nothing
here is a cherry-pick.

## Structural delta (why this is a rewrite, not a merge)

| Concern | BogPilot (FrogPilot base) | This fork (sunnypilot base) |
|---|---|---|
| Car code path | `selfdrive/car/tesla/*` (32 files) | `opendbc/car/tesla/*` (submodule `tesla-hw1-opendbc`) |
| Car enum | `CAR.TESLA_AP1_MODELS` (saved-car-model fallback, no FW table) | `CAR.TESLA_MODEL_S_HW1 / S_HW2 / S_HW3 / X_HW1 / X_HW2`, FW fingerprint table |
| Safety | `tesla.h` + `FLAG_TESLA_AP1` (bit 3) in `panda/board/safety/` | `opendbc/safety/modes/tesla_legacy.h` (`SafetyModel` @39), `panda/board/` has no safety dir |
| Params | FrogPilot params + **file-backed** `/data/params_bogpilot/*` (prebuilt `params_pyx.so` cannot store new keys) | sunnypilot Params (Python-generated), new keys are normal Params entries |
| Longitudinal | FrogPilot planner + acados MPC (`long_mpc.COMFORT_BRAKE`) | sunnypilot `sunnypilot/selfdrive/controls/lib/longitudinal_planner.py` |
| Speed-limit control | FrogPilot SLC + `frogpilotCarState.dashboardSpeedLimit` | sunnypilot SLC (`sunnypilot/selfdrive/controls/lib/speed_limit`) |
| UI | FrogPilot UI (prebuilt binary on device) | sunnypilot UI (`selfdrive/ui/sunnypilot/...`), built from source |
| OS | pinned AGNOS 10.1 | current AGNOS (python 3.12.3) |

Consequence: each BogPilot module must be re-hosted with `openpilot.selfdrive.car.tesla.*` imports
rewritten to `opendbc.car.tesla.*`, `CAR.TESLA_AP1_MODELS` checks mapped to our HW1 platforms, and
file-backed toggles replaced with real Params keys.

## Feature matrix

| # | Feature | BogPilot source | What it does | Target here | Effort | Risk |
|---|---|---|---|---|---|---|
| 1 | **AP1 blind spot** | `blindspot.py` (3 KB) | Reads `DAS_blindSpotRearLeft/Right` from stock `0x399`, validates the DAS checksum, holds each side 1.0 s, treats 1/2 as detected (3 = SNA is not) | `opendbc/car/tesla/carstate.py` — we already read both bits but as raw `!= 0` (SNA counts as a warning) with no hold and no checksum gate | S | Low |
| 2 | **Instrument-cluster integration** | `cluster.py` (18.6 KB) + panda forward-hook drop in `safety_tesla.h` | Rebuilds `0x399 AutopilotStatus`, `0x389 DAS_status2`, `0x239 DAS_lanes` from DBC signals, adds openpilot state/lane geometry (C2 from `modelV2.position` through-origin fit over 50 m), sends on bus 0 with stock counter + 1; panda drops the stock copy only while openpilot TX is recent (1.5× stock period) | New `opendbc/car/tesla/cluster.py` + `carcontroller.py` hook + **panda safety change** in `tesla_legacy.h` (recent-TX drop) + DBC check | L | Medium — panda safety edit, counter/checksum correctness on a live cluster |
| 3 | **Regen-first / earlier stopping** | `regen_aware.py` (8 KB, prototype), `regen_brake.py` (3.5 KB), `long_smooth.py` (16.8 KB) | AP1 brake pedal is friction-only, regen tops out ~1.0–1.5 m/s²; drops the planner's comfort-brake target from 2.5 → 1.0 m/s² so lead approaches stay in regen; brake-onset ramp at 2 m/s³, drive-release S-curve, throttle-cap filter (24 → ~3 flips/min), standstill hold floor | `longitudinal_planner.py` (comfort brake) + a CarController-side slew module | M | Medium — tuning changes the feel of every drive; must keep FCW/AEB/`ACCEL_MIN` untouched |
| 4 | **Human steering override + soft resume** | `hso.py` (4.6 KB), `engage_ramp.py` (2.9 KB), `steer_counter.py` (3.8 KB), `aol_gate.py` (1.8 KB) | Tinkla HSO: `EPAS_handsOnLevel >= 2` pauses path lateral (type NONE) without dropping cruise; soft engage ramp (300 ms hold of measured wheel, then blend); 0x488 counter continuity with the stock DAS interceptor; AOL low-speed gate to avoid `EAC_ERROR_HANDS_ON` → EPB revoking LKAS | `opendbc/car/tesla/carcontroller.py` + `carstate.py` | M | Medium — lateral actuation edge cases, EPAS fault handling |
| 5 | **Speed-limit control raise** | `slc_raise.py` (19.9 KB), `speed_limit.py` (4.3 KB) | Posted limit from Mobileye `DAS_fusedSpeedLimit` first, then GTW `UI_mapSpeedLimit`/`UI_mppSpeedLimit`; when SLC confirms a higher limit, lift openpilot's cruise target to limit+offset (no fake stalk, no panda loosen) | sunnypilot SLC (`speed_limit/`) + `CarState.dashboardSpeedLimit` equivalent | M | Medium — sunnypilot already has SLC; this adds the AP1 limit sources and the raise behaviour |
| 6 | **Cruise-stalk personalities + follow distance** | `stalk_follow.py` (10 KB), `stalk_fwd_hold.py` (3 KB), `stalk_pull_hold.py` (3.2 KB), `ap1_defaults.py` (17.7 KB) | `DTR_Dist_Rq` scroll maps to follow profiles; stalk tip/full-tip ±1 mph / next multiple of 5; pull = limit+offset; hold-to-toggle Experimental Mode; stalk twist sets personalities 1–7 (relaxed 7 … traffic 1) | sunnypilot `cruise.py` / personality handling + `carstate` stalk parsing | M | Low–Medium — mostly pure mapping; personality count differs (sunnypilot has 3) |
| 7 | **Curve speed** | `curve_speed.py` (8.3 KB) | Reaches the curve-speed target *by* the curve instead of braking gently toward it in-curve | sunnypilot `smart_cruise_control` / CSC | M | Medium |
| 8 | **Radar lead glitch filter** | `lead_filter.py` (5.1 KB) | Rejects a one-frame radar lead jump (46 m lead → 42 m roadside stationary track) that stepped the plan to −2.81 m/s² | `radar_interface.py` / radard side | S | Low |
| 9 | **Steer-fault gate** | `steer_fault.py` (1.3 KB) | `steerWarning` unless EPAS error is `EAC_ERROR_IDLE`; `EAC_FAULT` = permanent | `carstate.py` | S | Low |
| 10 | **Actuator plan** | `actuator_plan.py` (15 KB) | Decides exactly which CAN messages CarController packs; no steering frame while disengaged so stock Mobileye `0x488` keeps flowing | `carcontroller.py` restructure | M | Medium — touches the actuation path |
| 11 | **Platform / safety flags / toggles** | `platform.py`, `safety_flags.py`, `toggles.py` | Platform classification, `FLAG_TESLA_AP1 | FLAG_TESLA_LONG_CONTROL`, dashcam-only rules, Tesla Early toggle set | `values.py` / `interface.py` / Params | S | Low |
| 12 | **Fingerprint tool** | `BogPilot/bogpilot-fingerprint` | Privacy-sanitised fingerprint export from rlogs (no upload), coverage report, AP1/pre-AP/AP2 profiles | Standalone tool — usable as-is against our rlogs | — | None |
| 13 | **Driver-Mover** | `BogPilot/Driver-Mover` | Renders dm annotations + steering angle as a mannequin overlay video from rlogs | Standalone tool — usable as-is | — | None |

Already covered here, so not a port item: HW1 lateral (`0x488` angle) + longitudinal (`0x2b9`)
through the xnor harness, `teslaLegacy` panda safety, FW fingerprint table, IC-cluster *transmit*
groundwork (documented in `todo_HW1.md` but not implemented), Chestnut big-model build.

## Suggested order

**Wave 1 — read-only, no actuation (start here)**
1. Blind spot: checksum gate + 1.0 s hold + SNA handling in `carstate.py` (#1). Unit-testable from
   rlog replay, zero actuation risk.
2. Radar lead glitch filter (#8) and steer-fault gate (#9).
3. Fingerprint + Driver-Mover tooling (#12/#13) — no code change, immediate value for collecting
   other legacy-Tesla rlogs.

**Wave 2 — longitudinal comfort**
4. Regen-sized comfort brake (#3, `regen_brake.py` only first — one constant, big effect), then the
   onset ramp / drive-release / throttle-cap filters from `long_smooth.py`.
5. Curve speed (#7).

**Wave 3 — cluster**
6. `cluster.py` port (#2): DBC signal packing, counter continuity, checksum; panda `tesla_legacy.h`
   recent-TX drop; then IC toggles.

**Wave 4 — lateral + stalk UX**
7. HSO / soft resume / 0x488 counter (#4), actuator plan (#10).
8. Stalk personalities, follow-distance scroll, SLC raise (#5/#6).

## Verification protocol (per wave)

- `opendbc` car tests: `uv run python -m pytest opendbc/car/tesla/tests/test_tesla.py -q` plus new
  unit tests for each pure module (checksum, hold timing, mapping tables).
- Replay against real AP1 rlogs before touching the device; anything that changes actuation gets a
  diff of planned accel/steer on the same route.
- Device: install to `/data/tesla-hw1-port`, `op build`, then an onroad route with logging; check for
  `vehicleMonitor` faults, EPAS faults, and cluster frame counters.
- Never mix a panda safety change with a planner change in the same test drive.

## Risks / open questions

- **Panda safety**: the cluster work needs a forward-hook drop rule in `tesla_legacy.h` that BogPilot
  implements in `safety_tesla.h`. Our safety model numbering and flag layout differ; the drop rule
  must be re-expressed in our declarations and covered by safety unit tests.
- **DBC disagreement**: BogPilot notes `tesla_can.dbc` differs from Tinkla's for `0x399` bits 27–36 and
  `0x389` bits 31–33 (CSA state). Verify against our DBC before trusting either layout.
- **Personalities**: sunnypilot exposes 3 personalities; BogPilot's stalk maps 1–7. Either extend the
  sunnypilot personality set or map the stalk onto the existing three.
- **Params**: do not copy BogPilot's file-backed `/data/params_bogpilot/*` pattern — use real Params
  keys here, otherwise toggles are invisible to the UI.
- **AGNOS**: BogPilot's main branch is pinned to AGNOS 10.1 and will reflash down. Do not flash their
  image onto our device; port code only.

## Credits

BogPilot (BogGyver's Tinkla lineage), Tinkla/Unity by BogGyver, xnor-tech for the HW1 base, and
sunnypilot/comma for the upstream stack. BogPilot is MIT-licensed; keep attribution when porting.

## Progress (2026-11-13)

Ported and unit-tested; each lives on its own branch so it can be reviewed and merged separately.

| Feature | Branch | Commit | Tests |
|---|---|---|---|
| Blind-spot latch (checksum left out, see below) | `tesla-hw1-opendbc` `bsm-legacy-hold` | `ab34e41d` | 7 |
| Steer-fault gate (latched HANDS_ON / HIGH_ANGLE_REQ benign) | `tesla-hw1-opendbc` `bogpilot-wave1` | `40bb1dd4` | included in 11 |
| Posted speed limit (Mobileye fused -> GTW map limit) | `bogpilot-wave1` | `40bb1dd4` | 11 |
| Cruise-stalk buttons (cancel / resume / set) | `bogpilot-wave1` | `40bb1dd4` | 11 |
| Hands-on pause (HSO) + engage steering ramp | `bogpilot-wave2` | `89766ea2` | 13 |
| Comfort-band accel slew (regen-first feel) | `bogpilot-wave3` | `afede288` | 14 |
| Radar lead glitch filter | `tesla-hw1-port` `bogpilot-lead-filter` | `104aa827` | 8 |
| Speed-limit follow (raise/lower cruise on sign change) | `tesla-hw1-port` `bogpilot-slc-follow` | `8369199a` | 9 |

`opendbc/car/tesla/tests/` is 56 tests green as of `bogpilot-wave3`; the radard filter suite is 8 green.

### Notes on the ports

- **Blind spot**: our legacy CarState never set the flags at all. BogPilot's DAS checksum gate is
  *not* ported — `CANPacker` auto-fills counter/checksum bytes, so a pack-based check would be
  self-referential. Doing it properly needs a hand-rolled bit packer (their `cluster.py` has one).
- **Regen-first braking**: the comfort-brake constant (`COMFORT_BRAKE = 2.5` in `long_mpc.py`) is
  **dead code in this tree** — `get_lead0/get_lead1` have no callers, the acados model owns the lead
  cost. So the port is the CarController-side accel slew (`long_smooth.py`), which is where AP1's
  friction-only pedal behaviour actually shows up. The `DAS_jerkMin/Max` narrowing is ported as a
  class (`Ap1JerkLimit`) but **not wired**: the legacy packer already ramps jerk fields itself.
- **Throttle cap**: this tree has no `allowThrottle` planner flag (`gasPressProbs` is only used by
  `accel_boost`), so there is nothing to port onto. That needs a planner feature first.

### Speed-limit follow — how it works and what it depends on

`speed_limit_follow` sits in the planner, not in SLA, because the planner selects
`min(cruise, sccVision, sccMap, speedLimitAssist)` — SLA can only ever lower the target, and its
set-speed confirmation runs through the PCM/cluster path an AP1 car does not have.

- Raises **and** lowers the planner cruise target to `resolver.speed_limit_final_last`
  (limit + your SLC offset) when `SpeedLimitMode = assist`.
- New limit must persist **2 s** and differ by at least **1 mph**; limits below 5 mph are ignored.
- Any manual set-speed change resets the base, so the stalk always wins.
- **Depends on** the opendbc `bogpilot-wave1` speed-limit source (`CarStateSP.speedLimit` from
  Mobileye / GTW map). Without that merge the follow path has no limit to follow.

### Merge status

The train is merged: `tesla-hw1-opendbc` `tesla-hw1-port` = `afede288`, and the metarepo
`tesla-hw1-port` = `f809fb46` pins that commit (`opendbc_repo` gitlink = `afede288`). The review
branches (`bsm-legacy-hold`, `bogpilot-wave1/2/3`, `bogpilot-lead-filter`, `bogpilot-slc-follow`) are
kept for history but the branch tip is now the buildable one.

Tests at the merge tip: **56 passed** in `opendbc/car/tesla/tests/` (run from the pinned submodule) and
**17 passed** across the radard lead-filter and speed-limit-follow suites.

### Still to do

| Feature | Blocker |
|---|---|
| Instrument cluster (`0x399` / `0x389` / `0x239`) | Needs the frame packer plus a **panda `tesla_legacy.h` forward-hook drop rule** (drop stock copy while openpilot TX is recent). Safety-code change: must be device-tested against a live cluster, never bundled with a planner change. |
| SLC raise (limit+offset from Mobileye/Tesla maps) | sunnypilot already has SLC + resolver + assist; needs the AP1 limit sources wired into `speed_limit_assist` and a raise policy that does not fight it. |
| Curve speed (reach target by the curve) | sunnypilot's `SmartCruiseControl` already slows for curves; BogPilot pre-brakes to arrive at the target. Needs CSC internals study before touching. |
| Stalk personalities / follow distance | sunnypilot exposes 3 personalities, BogPilot maps 1-7 on the stalk. Design decision first: extend profiles or map onto the existing three. |
| Actuator plan (no steering frame while disengaged) | Our controller already packs conditionally; the interceptor behaviour depends on panda semantics that need verification on the device. |

