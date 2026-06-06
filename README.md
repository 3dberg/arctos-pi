# arctos-pi

Offline GUI + CAN controller for the Arctos 6-axis robot. Talks directly to
MKS SERVO42D / SERVO57D closed-loop stepper drivers over CAN.

An **optional ROS2 + MoveIt2 layer** sits on top of the same control code so
the arm can be planned with collision awareness and, later, driven by AI
agents over a clean ROS2 action/topic surface — see
[ROS2 / MoveIt2](#ros2--moveit2-optional). The core stack still runs
standalone with zero ROS installed.

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

## ROS2 / MoveIt2 (optional)

The `ros2_ws/` colcon workspace adds a ROS2 layer **on top of** the existing
Python control stack — it does not replace it. The tested `backend/mks.py`,
`backend/can_bus.py`, and `backend/motion.py` stay the single source of truth;
ROS2 reuses them.

```
 touchscreen / browser ─ HTTP+WS ─▶ FastAPI (backend/api.py)
                                       │  optional in-process rclpy client
                                       ▼
   /joint_states ◀── arctos_bridge ──▶ Motion ─▶ CanBus ─▶ MKS/CAN
   move_group (MoveIt2) ──FollowJointTrajectory──▶ arctos_bridge
```

Packages (`ros2_ws/src/`):

| Package | Role |
|---|---|
| `arctos_description` | URDF/xacro (primitive geometry); joint limits generated from `config.yaml` |
| `arctos_bridge` | rclpy node: serves `/joint_states` + `FollowJointTrajectory` + estop/enable, reusing `Motion` |
| `arctos_moveit_config` | SRDF, kinematics, OMPL, move_group + RViz launches |
| `arctos_bringup` | controller config + sim / real / demo launches |
| `arctos_robots` | robot-type registry (maps `robot_type` → its bundle) |

**Single source of truth.** Joint names and limits live in `config.yaml`.
Regenerate the ROS2 artifacts whenever it changes:

```bash
python -m backend.ros_export --config config.yaml \
    --out ros2_ws/src/arctos_description/config/
```

**Why a bridge node, not a `ros2_control` hardware interface?** ros2_control
hardware plugins are loaded via pluginlib (C++ shared libraries), so the tested
Python MKS/CAN stack can't be a hardware plugin directly. The `arctos_bridge`
node wraps it and serves the same `FollowJointTrajectory` action MoveIt drives,
so the MoveIt config is identical whether execution goes through the bridge
(real Python stack) or a `joint_trajectory_controller` backed by
`mock_components` (pure sim). A native C++ `SystemInterface` remains a future
option.

**Single CAN owner.** On a ROS2 deployment the `arctos_bridge` node owns the
CAN bus. The FastAPI server then talks to the robot as a ROS2 *client* (start
it with `ARCTOS_ROS=1` in a sourced ROS2 env); its `/api/ros/*` endpoints back
the **Motion / MoveIt** tab in the touchscreen UI. Do not run two processes
that both open the bus on real hardware.

### Dev quick start (x86, no hardware)

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-moveit ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash
pip install -e .                      # expose `backend` to ROS nodes

cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch arctos_description view_robot.launch.py     # see the arm in RViz
ros2 launch arctos_bringup demo.launch.py               # bridge + MoveIt + RViz (MockBus)
#   in RViz MotionPlanning: set a goal -> Plan -> Execute
```

ROS2 distro: **Jazzy** (Ubuntu 24.04 LTS) is the target. The Pi runs the same
launches with `can.backend: socketcan` / `channel: can0`.

## Roadmap

- [x] 1. MKS protocol module + tests
- [x] 2. CAN bus wrapper + motion coordinator
- [x] 3. FastAPI + WS + minimal jog UI
- [x] 4. Teach / record + program JSON format
- [ ] 5. Program queue + legacy `.tap` / `.txt` loader *(scaffolded in `backend/programs.py`)*
- [x] 6. Install script + systemd user unit + chromium kiosk
- [~] 7. ROS2 + MoveIt2 layer *(ros2_ws/: description, bridge, MoveIt config,
      robot registry; FastAPI ROS client + Motion/MoveIt UI tab). Authored;
      build/run validation on a ROS2 (Jazzy) box and Pi deployment pending.*

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
