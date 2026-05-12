#!/usr/bin/env bash
# Flash the arctos gripper firmware onto an Arduino Nano via arduino-cli.
#
# Usage:
#   ./flash.sh                       # flash gripper firmware, auto-detect port
#   ./flash.sh --sketch canping      # flash the SPI smoke-test sketch instead
#   ./flash.sh --port /dev/ttyUSB0   # explicit port
#   ./flash.sh --old-bootloader      # for Nanos with the original ATmega328P bootloader
#                                    # (try this if upload fails with "stk500_recv() not in sync")
#   ./flash.sh --crystal 8           # MCP2515 module has an 8 MHz crystal (default 16)
#   ./flash.sh --debug               # define DEBUG_SERIAL for chatty bring-up output
#   ./flash.sh --monitor             # open serial monitor at 115200 after upload

set -euo pipefail
cd "$(dirname "$0")"

SKETCH="gripper"
PORT=""
FQBN="arduino:avr:nano"
CRYSTAL=16
DEBUG=0
MONITOR=0
while [ $# -gt 0 ]; do
    case "$1" in
        --sketch)          SKETCH="$2"; shift 2 ;;
        --port)            PORT="$2"; shift 2 ;;
        --old-bootloader)  FQBN="arduino:avr:nano:cpu=atmega328old"; shift ;;
        --crystal)         CRYSTAL="$2"; shift 2 ;;
        --debug)           DEBUG=1; shift ;;
        --monitor)         MONITOR=1; shift ;;
        -h|--help)         sed -n '2,13p' "$0"; exit 0 ;;
        *) echo "unknown flag: $1" >&2; exit 2 ;;
    esac
done

SKETCH_DIR="$(pwd)/$SKETCH"
if [ ! -f "$SKETCH_DIR/$SKETCH.ino" ]; then
    echo "no sketch at $SKETCH_DIR/$SKETCH.ino" >&2
    echo "available sketches:" >&2
    for d in */; do
        name="${d%/}"
        [ -f "$name/$name.ino" ] && echo "  $name" >&2
    done
    exit 1
fi

# ---- arduino-cli ----
if ! command -v arduino-cli >/dev/null; then
    echo "arduino-cli not installed."
    echo "install with:"
    echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=\$HOME/.local/bin sh"
    echo "  (then make sure ~/.local/bin is on PATH)"
    exit 1
fi

# ---- AVR core ----
if ! arduino-cli core list 2>/dev/null | awk '{print $1}' | grep -qx 'arduino:avr'; then
    echo "==> installing arduino:avr core (one-time)"
    arduino-cli core update-index
    arduino-cli core install arduino:avr
fi

# ---- libraries ----
# Servo is technically bundled with arduino:avr but arduino-cli's library
# resolver doesn't always discover the bundled copy, so install it via the
# lib manager too (idempotent).
INSTALLED=$(arduino-cli lib list 2>/dev/null | awk 'NR>1 {print tolower($1)}')
NEED_INDEX=0
for lib in mcp_can servo; do
    if ! printf '%s\n' "$INSTALLED" | grep -qx "$lib"; then
        if [ "$NEED_INDEX" = "0" ]; then
            arduino-cli lib update-index
            NEED_INDEX=1
        fi
        echo "==> installing library: $lib"
        arduino-cli lib install "$lib"
    fi
done

# ---- port detection ----
if [ -z "$PORT" ]; then
    candidates=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true)
    n=$(printf '%s\n' "$candidates" | grep -c . || true)
    if [ "$n" -eq 0 ]; then
        echo "no /dev/ttyUSB* or /dev/ttyACM* found. plug in the Nano and re-run, or pass --port" >&2
        exit 1
    fi
    if [ "$n" -gt 1 ]; then
        echo "multiple serial ports found:" >&2
        printf '  %s\n' $candidates >&2
        echo "pick one with --port /dev/ttyXXX" >&2
        exit 1
    fi
    PORT="$candidates"
fi
echo "==> sketch:  $SKETCH"
echo "==> port:    $PORT"
echo "==> fqbn:    $FQBN"
echo "==> crystal: ${CRYSTAL} MHz"
echo "==> debug:   $([ "$DEBUG" = 1 ] && echo on || echo off)"

# ---- compile flags ----
EXTRA_FLAGS=""
if [ "$CRYSTAL" = "8" ]; then
    EXTRA_FLAGS="$EXTRA_FLAGS -DMCP_CRYSTAL_8MHZ"
fi
if [ "$DEBUG" = "1" ]; then
    EXTRA_FLAGS="$EXTRA_FLAGS -DDEBUG_SERIAL"
fi

# ---- compile ----
echo "==> compiling"
COMPILE_ARGS=(--fqbn "$FQBN" "$SKETCH_DIR")
if [ -n "$EXTRA_FLAGS" ]; then
    COMPILE_ARGS+=(--build-property "compiler.cpp.extra_flags=$EXTRA_FLAGS")
fi
arduino-cli compile "${COMPILE_ARGS[@]}"

# ---- upload ----
echo "==> uploading"
arduino-cli upload --fqbn "$FQBN" --port "$PORT" "$SKETCH_DIR"

echo "==> done."
echo "   monitor:    arduino-cli monitor -p $PORT -c baudrate=115200"
echo "   re-flash:   ./flash.sh ${PORT:+--port $PORT}"

if [ "$MONITOR" = "1" ]; then
    echo "==> opening serial monitor (Ctrl+C to exit)"
    arduino-cli monitor -p "$PORT" -c baudrate=115200
fi
