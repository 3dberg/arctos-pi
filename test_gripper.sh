#!/usr/bin/env bash
# Gripper smoke test. Drives the gripper through open / close / sweep via the
# running arctos-pi service. Verifies the full path: HTTP API -> backend ->
# CAN -> Arduino MCP2515 -> servo.
#
# Usage:
#   ./test_gripper.sh                # run against http://localhost:8000
#   ./test_gripper.sh --host 1.2.3.4 # run against a remote host
#   ./test_gripper.sh --candump      # also tee live CAN traffic from can0
#   ./test_gripper.sh --slow         # 500ms between sweep steps (default 50ms)
set -euo pipefail

HOST="localhost:8000"
DO_CANDUMP=0
STEP_DELAY="0.05"
while [ $# -gt 0 ]; do
    case "$1" in
        --host) HOST="$2"; shift 2 ;;
        --candump) DO_CANDUMP=1; shift ;;
        --slow) STEP_DELAY="0.5"; shift ;;
        -h|--help)
            sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "unknown flag: $1" >&2; exit 2 ;;
    esac
done

BASE="http://$HOST"
CANDUMP_PID=""
cleanup() { [ -n "$CANDUMP_PID" ] && kill "$CANDUMP_PID" 2>/dev/null || true; }
trap cleanup EXIT

# --- helpers ---
say() { printf "\n\033[1;36m==>\033[0m %s\n" "$*"; }
ok()  { printf "    \033[1;32mok\033[0m %s\n" "$*"; }
die() { printf "    \033[1;31merr\033[0m %s\n" "$*" >&2; exit 1; }

call() {
    local method="$1" path="$2" body="${3:-}"
    local args=(-sS -o /tmp/gripper_test.out -w "%{http_code}" -X "$method" "$BASE$path")
    if [ -n "$body" ]; then
        args+=(-H "Content-Type: application/json" -d "$body")
    fi
    local code
    code=$(curl "${args[@]}" || echo "000")
    if [ "$code" != "200" ]; then
        die "$method $path -> HTTP $code: $(cat /tmp/gripper_test.out 2>/dev/null || true)"
    fi
    cat /tmp/gripper_test.out
}

# --- preflight ---
say "preflight"
command -v curl >/dev/null || die "curl not installed"
command -v jq >/dev/null || JQ_MISSING=1

CFG_JSON=$(call GET /api/config)
if [ "${JQ_MISSING:-0}" = "1" ]; then
    enabled=$(printf '%s' "$CFG_JSON" | python3 -c 'import sys,json;c=json.load(sys.stdin);print(str(c.get("gripper",{}).get("enabled",False)).lower())')
    can_id=$(printf '%s'  "$CFG_JSON" | python3 -c 'import sys,json;c=json.load(sys.stdin);print(c.get("gripper",{}).get("can_id"))')
    open_p=$(printf '%s'  "$CFG_JSON" | python3 -c 'import sys,json;c=json.load(sys.stdin);print(c.get("gripper",{}).get("open_position"))')
    close_p=$(printf '%s' "$CFG_JSON" | python3 -c 'import sys,json;c=json.load(sys.stdin);print(c.get("gripper",{}).get("close_position"))')
    backend=$(printf '%s' "$CFG_JSON" | python3 -c 'import sys,json;c=json.load(sys.stdin);print(c.get("can",{}).get("backend"))')
else
    enabled=$(printf '%s' "$CFG_JSON" | jq -r '.gripper.enabled')
    can_id=$(printf '%s'  "$CFG_JSON" | jq -r '.gripper.can_id')
    open_p=$(printf '%s'  "$CFG_JSON" | jq -r '.gripper.open_position')
    close_p=$(printf '%s' "$CFG_JSON" | jq -r '.gripper.close_position')
    backend=$(printf '%s' "$CFG_JSON" | jq -r '.can.backend')
fi
[ "$enabled" = "true" ] || die "gripper.enabled=false in config; set 'gripper.enabled: true' in config.yaml and restart the service"
ok "service up at $BASE"
ok "gripper enabled, can_id=$can_id, open=$open_p, close=$close_p, can.backend=$backend"

# --- optional candump ---
if [ "$DO_CANDUMP" = "1" ]; then
    if ! command -v candump >/dev/null; then
        die "--candump requested but candump not installed (apt install can-utils)"
    fi
    if ! ip link show can0 >/dev/null 2>&1; then
        die "--candump requested but can0 interface not found"
    fi
    say "starting candump can0 (filter id=$can_id)"
    # printf %x of can_id (it may come back as decimal 7)
    hex_id=$(printf "%X" "$can_id")
    candump -t a "can0,${hex_id}:7FF" &
    CANDUMP_PID=$!
    sleep 0.2
fi

# --- test sequence ---
say "enable (motors + gripper)"
call POST /api/enable '{"on":true}' >/dev/null && ok "POST /api/enable {on:true}"

say "open"
call POST /api/gripper/open >/dev/null && ok "POST /api/gripper/open"
sleep 1

say "close"
call POST /api/gripper/close >/dev/null && ok "POST /api/gripper/close"
sleep 1

say "set position 128 (mid)"
call POST /api/gripper '{"position":128}' >/dev/null && ok "POST /api/gripper {position:128}"
sleep 1

say "sweep 0 -> 255 (step 32, delay ${STEP_DELAY}s)"
for p in 0 32 64 96 128 160 192 224 255; do
    call POST /api/gripper "{\"position\":$p}" >/dev/null
    printf "    pos=%3d ok\n" "$p"
    sleep "$STEP_DELAY"
done

say "sweep 255 -> 0"
for p in 255 224 192 160 128 96 64 32 0; do
    call POST /api/gripper "{\"position\":$p}" >/dev/null
    printf "    pos=%3d ok\n" "$p"
    sleep "$STEP_DELAY"
done

say "return to open ($open_p)"
call POST /api/gripper "{\"position\":$open_p}" >/dev/null && ok "back to open"

say "disable (leave system idle)"
call POST /api/enable '{"on":false}' >/dev/null && ok "POST /api/enable {on:false}"

say "done"
echo "    if servo did not move:"
echo "      - check Arduino is powered and on the same CAN bus as can0"
echo "      - run with --candump to confirm frames are hitting the bus"
echo "      - swap open_position/close_position in config.yaml if direction is reversed"
