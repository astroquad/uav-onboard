#!/usr/bin/env bash
# Headless SITL full-mission regression for the Astroquad grid arena.
#
# Launches Gazebo (server-only) + ArduCopter SITL + astroquad-onboard, waits
# for the mission to finish, and machine-checks the outcome from the onboard
# log. This is the baseline gate: run it BEFORE and AFTER any vision or FSM
# change and compare.
#
# PASS criteria (from the [grid-mission] log lines):
#   1. st=MISSION_COMPLETE reached
#   2. final regids= contains every expected marker id (default 1,2,3,4)
#   3. no st=EMERGENCY_LAND ticks
#   4. "[grid-mission] done." printed (clean shutdown, disarm observed)
#
# Usage:
#   bash scripts/sitl_mission_regression.sh
# Env overrides:
#   MARKER_COUNT=4 EXPECTED_IDS=1,2,3,4 LINE_MODE=light_on_dark
#   MISSION_TIMEOUT_S=700 REVISIT_ORDER=desc RUN_TAG=mylabel

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/gazebo_rendering.sh
source "$SCRIPT_DIR/lib/gazebo_rendering.sh"
ONBOARD_DIR="${ONBOARD_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUCOPTER_DIR="${ARDUCOPTER_DIR:-$ARDUPILOT_DIR/ArduCopter}"
SIM_VEHICLE="${SIM_VEHICLE:-$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py}"
ARDUPILOT_VENV="${ARDUPILOT_VENV:-}"
ARDUPILOT_PYTHON="${ARDUPILOT_PYTHON:-}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
ASTROQUAD_WORLD="${ASTROQUAD_WORLD:-$ONBOARD_DIR/sim/gazebo/worlds/grid_arena_test_world.sdf}"
INDOOR_PARAM_FILE="${INDOOR_PARAM_FILE:-$ONBOARD_DIR/sim/gazebo/params/indoor_flow_tfmini_sitl.parm}"

MARKER_COUNT="${MARKER_COUNT:-4}"
EXPECTED_IDS="${EXPECTED_IDS:-1,2,3,4}"
LINE_MODE="${LINE_MODE:-light_on_dark}"
REVISIT_ORDER="${REVISIT_ORDER:-desc}"
MISSION_TIMEOUT_S="${MISSION_TIMEOUT_S:-700}"
SITL_READY_TIMEOUT_S="${SITL_READY_TIMEOUT_S:-180}"
RUN_TAG="${RUN_TAG:-$(date +%Y%m%d_%H%M%S)}"

RUN_DIR="$ONBOARD_DIR/logs/sitl_regression_$RUN_TAG"
GZ_LOG="$RUN_DIR/gazebo.log"
SITL_LOG="$RUN_DIR/sim_vehicle.log"
ONBOARD_LOG="$RUN_DIR/onboard.log"
mkdir -p "$RUN_DIR"

GZ_PID=""
SITL_PID=""
CLEANED_UP=0

cleanup() {
    [[ "$CLEANED_UP" == "1" ]] && return
    CLEANED_UP=1
    echo "[cleanup] SITL/Gazebo 종료 중..."
    if [[ -n "$SITL_PID" ]]; then
        kill -- "-$SITL_PID" 2>/dev/null || kill "$SITL_PID" 2>/dev/null || true
    fi
    pkill -TERM -f "sim_vehicle.py.*ArduCopter" 2>/dev/null || true
    pkill -TERM -f "arducopter" 2>/dev/null || true
    pkill -TERM -f "mavproxy.py" 2>/dev/null || true
    if [[ -n "$GZ_PID" ]]; then
        kill -- "-$GZ_PID" 2>/dev/null || kill "$GZ_PID" 2>/dev/null || true
    fi
    pkill -TERM -f "gz sim.*grid_arena_test_world" 2>/dev/null || true
    sleep 2
    if [[ -n "$SITL_PID" ]]; then kill -KILL -- "-$SITL_PID" 2>/dev/null || true; fi
    if [[ -n "$GZ_PID" ]]; then kill -KILL -- "-$GZ_PID" 2>/dev/null || true; fi
    pkill -KILL -f "arducopter" 2>/dev/null || true
    pkill -KILL -f "mavproxy.py" 2>/dev/null || true
    pkill -KILL -f "gz sim.*grid_arena_test_world" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

fail() {
    echo "RESULT: FAIL — $*" | tee -a "$RUN_DIR/result.txt" >&2
    exit 1
}

# --- preconditions -----------------------------------------------------------
[[ -d "$ARDUCOPTER_DIR" ]] || fail "ArduCopter 디렉터리 없음: $ARDUCOPTER_DIR"
[[ -x "$SIM_VEHICLE" ]] || fail "sim_vehicle.py 없음: $SIM_VEHICLE"
[[ -f "$ASTROQUAD_WORLD" ]] || fail "world 파일 없음: $ASTROQUAD_WORLD"
[[ -f "$INDOOR_PARAM_FILE" ]] || fail "SITL 파라미터 파일 없음: $INDOOR_PARAM_FILE"
[[ -x "$ONBOARD_DIR/build/astroquad-onboard" ]] || fail "astroquad-onboard 빌드 필요"
command -v gz >/dev/null || fail "gz 명령 없음"
command -v setsid >/dev/null || fail "setsid 없음"
command -v timeout >/dev/null || fail "timeout 없음"

if [[ -z "$ARDUPILOT_VENV" ]]; then
    for candidate in "$ARDUPILOT_DIR/venv-ardupilot" "$ARDUPILOT_DIR/venv" "$ARDUPILOT_DIR/.venv" "$HOME/venv-ardupilot"; do
        [[ -x "$candidate/bin/python3" ]] && { ARDUPILOT_VENV="$candidate"; break; }
    done
fi
if [[ -z "$ARDUPILOT_PYTHON" ]]; then
    if [[ -n "$ARDUPILOT_VENV" ]]; then
        ARDUPILOT_PYTHON="$ARDUPILOT_VENV/bin/python3"
    else
        ARDUPILOT_PYTHON="$(command -v python3 || true)"
    fi
fi
[[ -x "$ARDUPILOT_PYTHON" ]] || fail "ArduPilot python 없음"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds:$ONBOARD_DIR/sim/gazebo/models:$ONBOARD_DIR/sim/gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export PATH="$ARDUPILOT_DIR/Tools/autotest:$(dirname "$ARDUPILOT_PYTHON"):$HOME/.local/bin:$PATH"

echo "=== SITL mission regression ($RUN_TAG) ==="
echo "logs: $RUN_DIR"
configure_gazebo_rendering

# --- clean stale processes ---------------------------------------------------
pkill -f "gz sim.*grid_arena_test_world" 2>/dev/null || true
pkill -f "sim_vehicle.py.*ArduCopter" 2>/dev/null || true
pkill -f "arducopter" 2>/dev/null || true
pkill -f "mavproxy.py" 2>/dev/null || true
sleep 1

# --- launch Gazebo (server-only, camera sensor still renders offscreen) ------
echo "[1/4] Gazebo server 실행..."
(
    cd "$ONBOARD_DIR"
    exec setsid gz sim -s -r -v2 "$ASTROQUAD_WORLD"
) >"$GZ_LOG" 2>&1 &
GZ_PID=$!
sleep 4
kill -0 "$GZ_PID" 2>/dev/null || { tail -40 "$GZ_LOG" >&2 || true; fail "Gazebo 실행 실패 ($GZ_LOG)"; }

# --- launch ArduCopter SITL (headless mavproxy) ------------------------------
echo "[2/4] ArduCopter SITL 실행 (headless)..."
(
    cd "$ARDUCOPTER_DIR"
    exec setsid "$SIM_VEHICLE" -v ArduCopter -f gazebo-iris --model JSON \
        --no-extra-ports --out 127.0.0.1:14550 \
        --add-param-file "$INDOOR_PARAM_FILE" --wipe-eeprom \
        -m --daemon
) <"/dev/null" >"$SITL_LOG" 2>&1 &
SITL_PID=$!

echo "[3/4] SITL 준비 대기 (EKF/IMU, 최대 ${SITL_READY_TIMEOUT_S}s)..."
ready=0
for ((i = 0; i < SITL_READY_TIMEOUT_S; i++)); do
    kill -0 "$SITL_PID" 2>/dev/null || { tail -40 "$SITL_LOG" >&2 || true; fail "sim_vehicle 조기 종료 ($SITL_LOG)"; }
    if grep -qE "EKF3 IMU[01] is using (optical flow|GPS)|using optical flow|Ready to (FLY|fly)|IMU0 is using" "$SITL_LOG" 2>/dev/null; then
        ready=1
        break
    fi
    sleep 1
done
if [[ "$ready" != "1" ]]; then
    tail -40 "$SITL_LOG" >&2 || true
    fail "SITL 준비 신호를 ${SITL_READY_TIMEOUT_S}s 내에 확인하지 못함"
fi
# EKF settle margin after the first ready line.
sleep 10

# --- run the mission ---------------------------------------------------------
echo "[4/4] astroquad-onboard 실행 (timeout ${MISSION_TIMEOUT_S}s)..."
set +e
(
    cd "$ONBOARD_DIR"
    timeout --signal=INT --kill-after=30 "$MISSION_TIMEOUT_S" \
        ./build/astroquad-onboard --config config --target sitl --vision gazebo \
        --world grid --line-mode "$LINE_MODE" --marker-count "$MARKER_COUNT" \
        --revisit-order "$REVISIT_ORDER" --no-video
) >"$ONBOARD_LOG" 2>&1
onboard_rc=$?
set -e

# --- analyze -----------------------------------------------------------------
echo "--- 분석 ($ONBOARD_LOG) ---"
pass=1
notes=()

if grep -q " st=MISSION_COMPLETE" "$ONBOARD_LOG"; then
    notes+=("OK  MISSION_COMPLETE 도달")
else
    pass=0; notes+=("NG  MISSION_COMPLETE 미도달")
fi

if grep -q " st=EMERGENCY_LAND" "$ONBOARD_LOG"; then
    pass=0; notes+=("NG  EMERGENCY_LAND 발생: $(grep -m1 -o 'safety=[^ ]*' "$ONBOARD_LOG" || echo '?')")
else
    notes+=("OK  EMERGENCY_LAND 없음")
fi

final_regids="$(grep -o 'regids=[0-9,‐-]*' "$ONBOARD_LOG" | tail -1 | cut -d= -f2)"
ids_ok=1
IFS=',' read -ra want <<<"$EXPECTED_IDS"
for id in "${want[@]}"; do
    [[ ",$final_regids," == *",$id,"* ]] || ids_ok=0
done
if [[ "$ids_ok" == "1" ]]; then
    notes+=("OK  마커 registry 최종값: regids=$final_regids")
else
    pass=0; notes+=("NG  마커 누락: regids=$final_regids (기대: $EXPECTED_IDS)")
fi

if grep -q "\[grid-mission\] done\." "$ONBOARD_LOG"; then
    notes+=("OK  정상 종료(disarm) 확인")
else
    pass=0; notes+=("NG  정상 종료 로그 없음 (onboard rc=$onboard_rc)")
fi

{
    echo "run: $RUN_TAG"
    echo "world: $ASTROQUAD_WORLD"
    echo "line_mode: $LINE_MODE  markers: $MARKER_COUNT  revisit: $REVISIT_ORDER"
    printf '%s\n' "${notes[@]}"
} | tee -a "$RUN_DIR/result.txt"

if [[ "$pass" == "1" ]]; then
    echo "RESULT: PASS" | tee -a "$RUN_DIR/result.txt"
    exit 0
fi
echo "RESULT: FAIL" | tee -a "$RUN_DIR/result.txt"
exit 1
