# arctos-pi

Offline GUI + CAN controller for the Arctos 6-axis robot. Talks directly to
MKS SERVO42D / SERVO57D closed-loop stepper drivers over CAN — no ROS.

Designed to run on **Raspberry Pi 5** with a 7" HDMI touchscreen, and on
**x86_64 Linux** for development (with a mock CAN bus).

**Status:** phases 1–3 complete and working end-to-end with a mock bus.
Phase 6 install/deploy is done so the stack can be exercised on real
hardware. Phases 4 (teach/record) and 5 (program queue) are scaffolded
and will be completed after on-bench validation.

## Hardware

| Item | Notes |
|---|---|
| Robot | Arctos 6-axis |
| Drivers | MKS SERVO42D and 57D, CAN IDs 1..6 @ 500 kbit/s |
| Adapter | MKS CANable v1.0 Pro (slcan over USB) |
| Host | Raspberry Pi 5, 7" HDMI touchscreen (1024×600) |
| Host (dev) | Any x86_64 Linux with Python 3.11+ |

## Quick start — laptop dev (no hardware)

```bash
git clone git@github.com:3dberg/arctos-pi.git
cd arctos-pi
./install.sh --no-service
source .venv/bin/activate
./run_dev.sh                   # uvicorn --reload on :8000
# open http://localhost:8000
```

Mock CAN backend is the default — jog buttons drive a virtual motor so the
whole UI is clickable without a driver attached. Run the tests:

```bash
pytest                         # 26 passing
```

## Deploy — Raspberry Pi 5 with CAN adapter

```bash
git clone git@github.com:3dberg/arctos-pi.git ~/arctos-pi
cd ~/arctos-pi
./install.sh --kiosk           # installs deps, udev rule, systemd user unit, chromium kiosk
cp config.example.yaml config.yaml
# edit config.yaml: set can.backend: slcan, set gear ratios and soft limits
systemctl --user start arctos
```

Plug in the MKS CANable v1.0 Pro — the udev rule creates a stable
`/dev/arctos-canable` symlink regardless of which USB port it's on.
Set `can.channel: /dev/arctos-canable` in `config.yaml`, or leave it
`null` to let the autodetector find it.

### Before first real-motor test
The MKS command encodings in `backend/mks.py` were derived from the V1.0.6
manual, not captured from a working setup. **Verify before trusting any
flash-persisting write.** Recommended bring-up:

1. Start with `can.backend: dry_run` — no frames are sent.
2. Click around the UI; check `journalctl --user -u arctos -f` for `DRY`
   log lines showing the frame bytes you'd transmit.
3. Switch to `slcan`, disconnect motor power, run jog, observe with
   `candump can0` if you have a second CAN node, or inspect the CANable
   LEDs.
4. With motor power restored and E-stop reachable, jog one axis at low
   speed. Confirm direction and gear ratio, adjust `invert` /
   `gear_ratio` / `pulses_per_rev` in config.
5. Only then apply per-axis microsteps / current from the Driver Config
   tab (these write to driver flash).

## Configuration

See `config.example.yaml`. The axis block for each motor:

```yaml
- can_id: 1
  name: J1
  gear_ratio: 13.5              # output shaft turns per motor turn
  pulses_per_rev: 3200           # matches microstep setting on the driver
  invert: false                  # flip direction in software
  max_speed: 1500                # 0xF6 / 0xFD speed units (0..3000)
  soft_limit_min: -180           # degrees, at the output
  soft_limit_max:  180
  default_current_ma: 1600       # 42D max 3000, 57D max 5200
  default_microsteps: 16
```

## Architecture

```
 browser ── HTTP + WebSocket ──▶ FastAPI (backend/api.py)
                                         │
                                         ▼
                                  Motion (backend/motion.py)
                                         │ pulses, limits, gear
                                         ▼
                                  CanBus (backend/can_bus.py)
                                  ├── slcan    — real CANable Pro
                                  ├── mock     — dev / CI
                                  └── dry_run  — log only, bring-up safe
                                         │
                                         ▼
                                  MKS protocol (backend/mks.py)
```

**Safety:** UI E-stop, atomic soft-limit check per axis, WS heartbeat
watchdog that stops all jogs if the browser drops off. These are
complements to — not replacements for — a physical E-stop wired into
motor power.

## Roadmap

- [x] 1. MKS protocol module + tests
- [x] 2. CAN bus wrapper + motion coordinator
- [x] 3. FastAPI + WS + minimal jog UI
- [x] 4. Teach / record + program JSON format
- [ ] 5. Program queue + legacy `.tap` / `.txt` loader *(scaffolded in `backend/programs.py`)*
- [x] 6. Install script + systemd user unit + chromium kiosk

### Phase 4 — Teach / Record (done)

Implemented in `backend/teach.py` + Teach tab in the UI. "Capture waypoint"
snapshots the current joint degrees (and gripper position when enabled);
list edit supports reorder, delete, and per-waypoint tweaks of `dwell_ms`
and `speed_pct`. Programs save/load as JSON under `~/.arctos/programs/`,
schema version 1:

```json
{ "name": "pick-place", "version": 1,
  "waypoints": [
    { "joints": {"J1": 0.0, "J2": -45.0, ...}, "dwell_ms": 250,
      "speed_pct": 0.4, "gripper": 200 }
  ] }
```

Replay belongs to Phase 5 — capture-only for now keeps the surface
small while hardware bring-up is in progress.

### Phase 5 (next) — Multi-program queue

Adds a Programs tab:
- Library view listing all JSON programs in `~/.arctos/programs/`.
- Drag-or-tap to add entries to a run queue with per-entry repeat count.
- Runner with pause / resume / skip / stop-all.
- Legacy loader: reads arctosgui `.tap` g-code and `.txt` raw-CAN files
  (same formats the existing `arctosgui/convert.py` produces) and
  converts them to native waypoints for replay.
- Live progress over the WebSocket (current program, waypoint index).

## License

TBD.
