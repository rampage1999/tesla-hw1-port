# Sunnypilot + Tesla HW1 (Model S/X 2012–2016)

Custom comma-device fork adding **Tesla HW1-era legacy support** to sunnypilot: Model S / Model X
(2012–2016, HW1/HW2/HW3 generations) with a dedicated `teslaLegacy` panda safety model.

Built from the xnor-tech tesla-unity 0.9.6 / P6g9YHK6 integration work, rebased onto sunnypilot
master as a clean single-commit port, then hardened and validated:

- **384 on-device tests green** on a comma 4: safety suite (incl. dedicated `test_tesla_legacy.py`),
  car-interface fuzz for all 9 Tesla platforms, platform configs, docs, FW fingerprint gates
- **Real-car validated**: Model S (HW1) — FW fingerprint, `teslaLegacy` + `elm327` safeties active,
  clean drives, no vehicleMonitor faults
- **Chestnut companion-compute working**: big-model tinygrad artifacts compiled on-device
  (`big_driving_tinygrad.pkl` ≈ 763 MiB + warps + dmonitoring), Chestnut detected over USB 3 at 5 Gbps

**Deployed 2026-11-14:** device rebooted onto `9df57722` / opendbc `afede288`. `scons` completed with no
errors, both ported module sets import cleanly in the running tree, `ModelRunnerTypeCache=2` and the big
model artifacts are intact. Free space raised from 8.9 GB to 23 GB by removing three stale source trees
(`/data/openpilot`, `SunnyPilot-TeslaHW1`, `sunnypilot-hw1-chestnut-source/new`) and pruning realdata older
than 7 days. Nothing has been validated on-road yet — the car was off, so no `carState`/`carParams` came up.

## Platforms

| Platform | Years/Gens | Safety |
|---|---|---|
| TESLA_MODEL_S_HW1 / S_HW2 / S_HW3 | Model S 2012–2017 | `teslaLegacy` |
| TESLA_MODEL_X_HW1 / X_HW2 | Model X 2016–2017 | `teslaLegacy` |

Companion repo: **[rampage1999/tesla-hw1-opendbc](https://github.com/rampage1999/tesla-hw1-opendbc)**
branch `tesla-hw1-port` — safety C code, car interfaces, DBC, fingerprints, tests. The metarepo's
`.gitmodules` points `opendbc_repo` there so standard submodule flows resolve.

## Install on a comma device

One-liner (installs to `/data/tesla-hw1-port`, never touches your active fork):

```bash
curl -fsSL https://raw.githubusercontent.com/rampage1999/tesla-hw1-port/tesla-hw1-port/install-tesla-port.sh | bash
```

Then, **offroad**:

```bash
sudo cp /data/continue.sh /data/continue.sh.bak-$(date +%s)
sudo cp /data/continue-tesla-port.sh /data/continue.sh
sudo reboot
```

First boot compiles the stack from source (~5 min on device). Rollback is one `cp` + reboot (see
installer output).

## Requirements / notes

- comma 3 / 3X / 4 (validated on comma 4 + Chestnut USB companion)
- Panda firmware: `teslaLegacy` safety must be flashed — this fork carries the xnor-signed
  `panda_h7.bin.signed` in `panda/board/obj/`. If your panda runs different firmware, the device
  will prompt/flash on boot (dev-flash flow).
- Model S/X (2012–2016) require the external-panda wiring for longitudinal on HW1 (DAS), matching
  the xnor integration this derives from.
- `MAX_LAT_ACCEL_MEASURED` raised to 3.0 m/s² for the legacy platforms (override.toml).
- Budget ~15 GB free on `/data` before the first on-device compile; the big-model artifacts alone
  are ~0.9 GB and the scons tree is large.

## Current deployment status (verified 2026-11-13)

Reference rig: comma four, dongle `5ba8a90b0996b249`, host `comma-13711a46`, sunnypilot
`2026.003.000`, running from **`/data/tesla-hw1-port/openpilot`** (`/data/continue.sh` launches
`/data/tesla-hw1-port/launch_openpilot.sh`), offroad.

| Item | State |
|---|---|
| Branch / commit | `tesla-hw1-port` @ `9df57722` (`opendbc_repo` pinned at `afede288`), deployed and built on-device 2026-11-14 |
| Base | sunnypilot `master` `02ef6e4` + Tesla HW1 port commits |
| `opendbc_repo` | `ae1901e9` (branch `tesla-hw1-port`) — matches the metarepo gitlink |
| `panda` / `tinygrad_repo` | `8a16e08` / `92b742f` (master) |
| Panda | comma four internal panda (`cuatro`), offroad `safetyModel=noOutput` |
| Chestnut | present over USB 3 (5 Gbps), VID `0x3801`, `chestnut_config/comma-13711a46.bin` provisioned |
| Big model | compiled on-device: `big_driving_tinygrad.pkl` 799,941,965 B, `big_driving_warp_1344x760/1928x1208_tinygrad.pkl`, `driving_tinygrad.pkl` 70 MB, `dmonitoring_model_tinygrad.pkl` 20 MB |
| Model runner | `ModelRunnerTypeCache=2` (chestnut path cached); `openpilot.sunnypilot.modeld_v2.warmup` runs on boot |
| Model manager | qcom + chestnut model lists from `sunnypilot/sunnypilot-models` gh-pages (`driving_models_chestnut_v28.json`) |
| Disk | 75 GB / 89 GB used — **10 % free**, clean up before the next full build |

Notes from that verification:

- The big-model build is reproducible on-device; the earlier "Chestnut PCIe link never trains"
  blocker on the older `chestnut-ap1` line does **not** apply to this tree — the Chestnut enumerates
  as a USB 3 device here (`custom 8516b768-CLEAN`, `linkErrorCount=4`).
- Big-model **runtime engagement during a drive is not yet re-verified** in this snapshot; the
  device was offroad (ignition off) with only the warmup process active. Next step is an onroad
  route with logging to confirm the chestnut runner is actually used.
- The `chestnut-ap1` fork of `openpilot` (older line, `rampage1999/openpilot`) is superseded by
  this repo and is kept only for reference.

## Porting BogPilot features

BogPilot (FrogPilot-based AP1 fork) has features worth bringing over: native blind-spot hold,
instrument-cluster frames, regen-first braking, hands-on steering override, speed-limit raise from
Mobileye/Tesla maps, and cruise-stalk personalities. The file-by-file plan, target mapping and wave
order live in [`docs/tesla/BOGPILOT_INTEGRATION.md`](docs/tesla/BOGPILOT_INTEGRATION.md).

**Port progress (2026-11-13):** blind spot, steer-fault gate, posted speed limit, cruise-stalk buttons, hands-on pause + engage steering ramp, comfort-band accel slew and the radard lead glitch filter are ported and unit-tested on review branches. Cluster, SLC raise, curve speed, stalk personalities and the actuator plan still need design/safety work — see the table in the plan.

## Status

Personal fork, not affiliated with sunnypilot/comma. Safety-critical code — treat as experimental;
keep hands ready. PRs/issues welcome.

## Layout

- branch `tesla-hw1-port` — sunnypilot master + port commits (gitlink → tesla-hw1-opendbc)
- `install-tesla-port.sh` — on-device installer (bundle fast-path, GitHub Releases)
- `out/`-style bundles are published as GitHub Releases; the installer fetches them automatically
