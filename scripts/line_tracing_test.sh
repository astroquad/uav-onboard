#!/usr/bin/env bash
# Gazebo + ArduCopter SITL launcher for the Astroquad line-tracing test world.
# Run from WSL: bash ~/astroquad/uav-onboard/scripts/line_tracing_test.sh

set -euo pipefail

ONBOARD_DIR="${ONBOARD_DIR:-$HOME/astroquad/uav-onboard}"
ARDUCOPTER_DIR="${ARDUCOPTER_DIR:-$HOME/ardupilot/ArduCopter}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
ASTROQUAD_WORLD="${ASTROQUAD_WORLD:-$ONBOARD_DIR/sim/gazebo/worlds/line_tracing_test_world.sdf}"
GZ_LOG="${GZ_LOG:-/tmp/gz_line_tracing_test_world.log}"
INDOOR_PARAM_FILE="${INDOOR_PARAM_FILE:-$HOME/mavlink_control_lab/params/indoor_flow_tfmini_sitl.parm}"
WSL_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
WINDOWS_GCS_IP="$(ip route 2>/dev/null | awk '/default/ {print $3; exit}')"
CLEANED_UP=0
MAVPROXY_ARGS=("--console")
if [[ "${MAVPROXY_MAP:-0}" == "1" ]]; then
    MAVPROXY_ARGS+=("--map")
fi
if [[ -n "${MAVPROXY_EXTRA_ARGS:-}" ]]; then
    # shellcheck disable=SC2206
    EXTRA_ARGS=($MAVPROXY_EXTRA_ARGS)
    MAVPROXY_ARGS+=("${EXTRA_ARGS[@]}")
fi

cleanup() {
    if [[ "$CLEANED_UP" == "1" ]]; then
        return
    fi
    CLEANED_UP=1
    if [[ -n "${GZ_PID:-}" ]]; then
        echo
        echo "[cleanup] Gazebo 종료 중..."
        kill -- "-$GZ_PID" 2>/dev/null || kill "$GZ_PID" 2>/dev/null || true
        pkill -TERM -f "^gz sim server" 2>/dev/null || true
        pkill -TERM -f "^gz sim gui" 2>/dev/null || true
        sleep 1
        kill -KILL -- "-$GZ_PID" 2>/dev/null || kill -KILL "$GZ_PID" 2>/dev/null || true
        pkill -KILL -f "^gz sim server" 2>/dev/null || true
        pkill -KILL -f "^gz sim gui" 2>/dev/null || true
        wait "$GZ_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "======================================"
echo " Gazebo + ArduCopter SITL (line_tracing_test_world)"
echo "======================================"

if [[ ! -d "$ARDUCOPTER_DIR" ]]; then
    echo "오류: ArduCopter 디렉터리를 찾을 수 없습니다: $ARDUCOPTER_DIR" >&2
    exit 1
fi

if [[ ! -d "$ARDUPILOT_GAZEBO_DIR" ]]; then
    echo "오류: ardupilot_gazebo 디렉터리를 찾을 수 없습니다: $ARDUPILOT_GAZEBO_DIR" >&2
    exit 1
fi

if [[ ! -f "$ASTROQUAD_WORLD" ]]; then
    echo "오류: Astroquad Gazebo world 파일을 찾을 수 없습니다: $ASTROQUAD_WORLD" >&2
    exit 1
fi

if [[ ! -f "$INDOOR_PARAM_FILE" ]]; then
    echo "오류: indoor flow/rangefinder 파라미터 파일을 찾을 수 없습니다: $INDOOR_PARAM_FILE" >&2
    exit 1
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds:$ONBOARD_DIR/sim/gazebo/models:$ONBOARD_DIR/sim/gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"

echo "[1] 기존 Gazebo / ArduPilot SITL 프로세스 정리..."
pkill -f "gz sim.*line_tracing_test_world" 2>/dev/null || true
pkill -f "^gz sim server" 2>/dev/null || true
pkill -f "^gz sim gui" 2>/dev/null || true
pkill -f "sim_vehicle.py.*ArduCopter" 2>/dev/null || true
pkill -f "arducopter" 2>/dev/null || true
pkill -f "mavproxy.py" 2>/dev/null || true
sleep 1

echo "[2] Gazebo 실행: gz sim -v4 -r $ASTROQUAD_WORLD"
(
    cd "$ONBOARD_DIR"
    exec setsid gz sim -v4 -r "$ASTROQUAD_WORLD"
) >"$GZ_LOG" 2>&1 &
GZ_PID=$!
echo "    Gazebo PID: $GZ_PID"
echo "    Gazebo 로그: $GZ_LOG"
sleep 3

if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "오류: Gazebo 실행에 실패했습니다. 로그를 확인하세요: $GZ_LOG" >&2
    tail -60 "$GZ_LOG" >&2 || true
    exit 1
fi

echo "[3] ArduCopter SITL + MAVProxy 콘솔 실행"
echo "    cd $ARDUCOPTER_DIR"
echo "    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON ${MAVPROXY_ARGS[*]} --out 127.0.0.1:14550 --add-param-file \"$INDOOR_PARAM_FILE\" --wipe-eeprom"
echo
echo "포트 구분:"
echo "    MAVLink SITL -> onboard line_follow_node: UDP 127.0.0.1:14550"
echo "    Astroquad onboard -> Windows GCS telemetry: UDP <Windows GCS IP>:14550"
echo "    Astroquad onboard -> Windows GCS MJPEG video: UDP <Windows GCS IP>:5600"
echo
echo "Windows GCS 실행:"
echo "    PowerShell: .\\build\\uav_gcs_vision_debug.exe --config config"
echo
echo "WSL에서 Windows GCS IP 후보:"
if [[ -n "$WINDOWS_GCS_IP" ]]; then
    echo "    $WINDOWS_GCS_IP"
else
    echo "    ip route | awk '/default/ {print \$3; exit}'"
fi
echo
echo "Astroquad vision-only smoke:"
echo "    cd $ONBOARD_DIR"
echo "    ./build/vision_debug_node --config config --target sitl --vision gazebo --line-mode light_on_dark --video --gcs-ip <windows-gcs-ip>"
echo
echo "Astroquad line-follow smoke:"
echo "    cd $ONBOARD_DIR"
echo "    ./build/line_follow_node --config config --target sitl --vision gazebo --video --gcs-ip <windows-gcs-ip>"
echo
echo "Mission Planner 미러 권장 설정:"
if [[ -n "$WSL_IP" ]]; then
    echo "    Type: UDP Client"
    echo "    Target: ${WSL_IP}:14600"
    echo "    Write Access: ON"
else
    echo "    WSL IP를 자동으로 찾지 못했습니다. 수동으로 WSL IP:14600을 입력하세요."
fi
echo "======================================"

cd "$ARDUCOPTER_DIR"
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON "${MAVPROXY_ARGS[@]}" --out 127.0.0.1:14550 --add-param-file "$INDOOR_PARAM_FILE" --wipe-eeprom
