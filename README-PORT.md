# Sunnypilot + Tesla HW1 (Model S/X 2012–2016)

Custom comma-device fork adding **Tesla HW1-era legacy support** to sunnypilot: Model S / Model X
(2012–2016, HW1/HW2/HW3 generations) with a dedicated `teslaLegacy` panda safety model.

Built from the xnor-tech tesla-unity 0.9.6 / P6g9YHK6 integration work, rebased onto sunnypilot
master as a clean single-commit port, then hardened and validated:

- **384 on-device tests green** on a comma 4: safety suite (incl. dedicated `test_tesla_legacy.py`),
  car-interface fuzz for all 9 Tesla platforms, platform configs, docs, FW fingerprint gates
- **Real-car validated**: Model S (HW1) — FW fingerprint, `teslaLegacy` + `elm327` safeties active,
  clean drives, no vehicleMonitor faults
- Chestnut companion-compute compatible (big-model bundle verified: "Big Model Ready")

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

## Status

Personal fork, not affiliated with sunnypilot/comma. Safety-critical code — treat as experimental;
keep hands ready. PRs/issues welcome.

## Layout

- branch `tesla-hw1-port` — sunnypilot master + one port commit (gitlink → tesla-hw1-opendbc)
- `install-tesla-port.sh` — on-device installer (bundle fast-path, GitHub Releases)
- `out/`-style bundles are published as GitHub Releases; the installer fetches them automatically
