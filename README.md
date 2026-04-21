# arctos-pi

Offline GUI + CAN controller for the Arctos robot, designed to run on Raspberry Pi 5
with a 7" HDMI touchscreen and also on x86_64 Linux for development.

**Status:** pre-alpha. Phase 1 complete (MKS protocol encoder + tests).

## Hardware

- **Robot:** Arctos (6-axis)
- **Drivers:** MKS SERVO42D / SERVO57D (closed-loop steppers), CAN 500 kbit/s, IDs 1..6
- **Adapter:** MKS CANable v1.0 Pro (slcan over USB)
- **Target host:** Raspberry Pi 5 with 7" touchscreen; x86_64 Linux also supported

## Quick start (development, no hardware)

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[dev]'
pytest            # unit tests
# backend/mock CAN bus mode: coming in phase 2
```

## Phases

- [x] 1. MKS protocol module + tests
- [ ] 2. CAN bus wrapper + motion coordinator
- [ ] 3. FastAPI + WS + minimal jog UI
- [ ] 4. Teach / record + program JSON format
- [ ] 5. Program queue + legacy `.tap` / `.txt` loader
- [ ] 6. Install script + systemd + kiosk mode

## Safety

This software can command real motion on a real robot. Do not run it near the
machine without a physical e-stop wired into the motor power supply. UI E-stop
is a convenience, not a substitute.
