# PROJECT_SPEC.md — uav-onboard

> 제24회 한국로봇항공기경연대회 중급부문 멀티콥터형 드론 실내 조난자 탐색 온보드 소프트웨어 기준 문서  
> **이 문서는 팀원과 코딩 에이전트가 공통으로 참조하는 Single Source of Truth입니다.**

최종 수정: 2026-05-26

## 1. 프로젝트 목적

`uav-onboard`는 Raspberry Pi 5에서 하향 카메라 비전, 격자 미션 상태머신,
MAVLink 제어, safety, GCS telemetry/debug video 송신을 담당한다.

최종 목표는 GPS 없는 격자 환경에서 이륙, grid snake 탐색, ArUco 조난자
마커 기록, 번호 역순 재방문, 출발점 복귀, 자동 착륙까지 수행하는 것이며,
현재 이 한 사이클 전체가 한 상태머신 안에서 동작한다.

현재 구현 기준은 다음과 같이 나뉜다.

- `astroquad-onboard`: 현재 grid arena 진입 → snake 탐색 → 마커 재방문 →
  vertiport 복귀 → 자동 착륙까지 풀-미션 SITL 및 guarded ArduPilot serial
  mission runtime.
- `uav-onboard-telem`: basic telemetry bring-up sender / development probe.
- `vision_debug_node`: 현재 안정적인 camera/vision/GCS debug runtime.
- `line_follow_node`: SITL 및 guarded ArduPilot serial line-follow staging.

## 2. 책임 범위

| 구분 | 내용 |
|---|---|
| 담당 | Pi camera frame 획득, ArUco/line/intersection detection, local grid-node 판단, mission state machine (진입/탐색/재방문/복귀/착륙), grid/snake policy, marker revisit planning, control intent mapping, MAVLink adapter, safety, telemetry/debug video |
| 미담당 | 자세 안정화와 모터 제어는 ArduPilot flight controller 담당, GCS UI/overlay/log window는 `uav-gcs` 담당 |

역할 분리:

- Onboard가 mission-critical 판단을 한다.
- GCS는 onboard metadata를 시각화하고 기록한다.
- Onboard는 raw camera frame에 overlay를 그리지 않는다.
- Debug video는 best-effort 관제 채널이며 mission logic의 입력이 아니다.

## 3. 현재 구현 상태

| 영역 | 상태 | 구현 위치 |
|---|---|---|
| Raspberry Pi camera capture | 구현됨 | `src/camera/RpicamMjpegSource.*`, `src/vision/RpicamFrameSource.*` |
| Runtime frame sources | 구현됨 | `FakeFrameSource`, `GazeboCameraSource`, `RpicamFrameSource` |
| Vision processor | 구현됨 | `src/vision/VisionProcessor.*` |
| ArUco detector/stabilizer | 구현됨 | `ArucoDetector.*`, `MarkerStabilizer.*` |
| Line mask/detector/stabilizer | 구현됨 | `LineMaskBuilder.*`, `LineDetector.*`, `LineStabilizer.*` |
| Intersection classifier/stabilizer | 구현됨 | `IntersectionDetector.*`, `IntersectionStabilizer.*` |
| Intersection decision | 구현됨 | `src/mission/IntersectionDecision.*` |
| Local grid coordinate tracker | 구현됨 | `src/mission/GridCoordinateTracker.*` |
| Grid mission state machine (entry/snake/revisit/return) | 구현됨 | `src/mission/GridMission.*`, `src/app/AstroquadOnboardApp.*` |
| Snake boundary planner | 구현됨 | `src/mission/SnakePlanner.*` |
| Marker registry/window gate | 구현됨 | `src/mission/MarkerRegistry.*`, `GridMission::MarkerWindow` |
| Marker revisit planner (asc/desc 정렬 + leg 경로 생성) | 구현됨 | `src/mission/MarkerRevisitPlanner.*` |
| Return-home / return-vertiport sequence | 구현됨 | `GridMission` ReturnHome*/ReturnVertiport* states |
| Guided velocity line/marker controller | 구현됨 | `src/control/GuidedVelocityController.*` |
| Grid intent -> setpoint mapper | 구현됨 | `src/control/GridControlMapper.*` |
| Altitude policy | 구현됨 | `src/mission/AltitudePolicy.*` |
| MAVLink UDP transport | 구현됨 | `src/autopilot/UdpMavlinkTransport.*` |
| MAVLink serial transport | 구현됨 | `src/autopilot/SerialMavlinkTransport.*` |
| Autopilot adapter | 구현됨 | `src/autopilot/AutopilotMavlinkAdapter.*` |
| Safety monitor | 부분 구현 | `src/safety/SafetyMonitor.*`, mission-local failsafe |
| GCS telemetry/video publisher (mission/revisit/return phase 포함) | 구현됨 | `src/app/GcsTelemetryPublisher.*` |
| Gazebo line/grid worlds | 구현됨 | `sim/gazebo/`, `scripts/line_tracing_test.sh`, `scripts/grid_arena_test.sh` |
| ArduPilot serial bench tools | 구현됨 | `tools/mavlink_probe.cpp`, `tools/mavlink_motor_test.cpp` |
| GCS command receiver | 미구현 | planned |
| Official coordinate conversion | 미구현 | planned |
| File logging subsystem | 미구현 | `logs/`, future `src/logging` |

## 4. 하드웨어/소프트웨어 기준

2026-07 하드웨어 업그레이드 기준 (이륙중량 약 2.1 kg). FC 파라미터 타깃은
`config/pixhawk6c_indoor_flow.params`, 벤치 기대값은
`config/pixhawk6c_indoor_flow.expected.toml` 참조.

| 모델/장치 | 연결 위치 | 역할 |
|---|---|---|
| Holybro Pixhawk 6C Mini (ArduCopter) | 기체 중앙 | 비행 제어기. 자세 안정화, 모터 출력, EKF3 센서 융합, 모드 관리 |
| MicoAir MTF-01 Optical Flow & Range Sensor | FC TELEM1 (MAVLink) | GPS 없이 수평 속도 추정 + 바닥 거리 측정 |
| Raspberry Pi 5 (4 GB, Active Cooler) | FC USB 또는 UART | OpenCV 비전, mission, MAVLink command sender |
| Raspberry Pi Global Shutter Camera (IMX296 mono) | Raspberry Pi CSI | 하향 영상 입력 (모노 글로벌셔터, 고정초점 CS-mount 렌즈) |
| Holybro S500 V2 frame kit + companion plate | — | 기체 프레임 |
| Sunnysky X2212 980KV (CW/CCW) + Tarot 9450 folding props | FC MAIN OUT → ESC | 추진계 |
| GT-Drone 35A BLHeli_S OPTO ESC (2S–5S) | PDB | 모터 구동 |
| 4S LiPo (Pollyttronics PT-B8000-FX35, XT90S) | PDB/Power Module | 주 전원 (저전압 기준: safety.toml, FC BATT_* 파라미터) |
| Matek BEC12S-PRO | 배터리 → Pi 5 | 컴패니언 5V 전원 (Pi 5 피크 부하 검증 필요) |
| ELRS RC Receiver (BETAFPV Nano 2400, CRSF) | FC TELEM2 (SERIAL2_PROTOCOL=23) | 수동 takeover, 비상 개입 |

| 항목 | 현재 기준 |
|---|---|
| Language | C++17 |
| Build | CMake 3.16+ |
| Config | `tomlplusplus` |
| JSON | `nlohmann/json` |
| Vision | OpenCV 4.x + aruco |
| Camera runtime | `rpicam-vid` MJPEG stdout |
| MAVLink | ArduPilot headers or fetched MAVLink `c_library_v2` |
| Tests | CTest + assert-style unit tests |

## 5. 모듈 구조

```text
FrameSource
  -> VisionProcessor
  -> IntersectionDecisionEngine
  -> GridCoordinateTracker
  -> GridMission        (uses MarkerRegistry / SnakePlanner /
                          AltitudePolicy / MarkerRevisitPlanner)
  -> GridControlMapper
  -> AutopilotMavlinkAdapter

Support:
  GcsTelemetryPublisher sends telemetry/debug video
  SafetyMonitor and mission-local failsafe observe runtime state
```

Library targets:

- `onboard_core`: config, telemetry, mission, control, safety, UDP telemetry.
- `onboard_video`: camera MJPEG model and UDP MJPEG streamer.
- `onboard_vision`: OpenCV detectors, frame sources, processor.
- `onboard_autopilot`: MAVLink adapter and UDP/serial transports.

경계 규칙:

- `VisionProcessor`는 frame metadata를 받아 `VisionResult`를 만든다. MAVLink,
  flight mode, mission state를 모른다.
- `GridMission`은 vision/decision/autopilot snapshot을 받아 state와 control
  intent를 만든다. MAVLink packet을 직접 만들지 않는다.
- `GridControlMapper`는 mission intent를 body velocity/yaw/altitude setpoint로
  바꾼다.
- `AutopilotMavlinkAdapter`는 mode/arm/takeoff/land/setpoint 송수신만 담당한다.
- Debug video는 mission-critical path에 들어가지 않는다.

## 6. Vision 알고리즘 기준

### Line

현재 line path는 `LineMaskBuilder -> LineDetector -> LineStabilizer`다.

- `mask_strategy = "white_fill"` 기본. 현재 잔디 바닥의 흰색 격자선은
  `light_on_dark`/bright fill path로 잡는다. `dark_on_light`는 검은 선을 밝은
  바닥에서 잡는 호환 경로로 유지한다.
- ROI는 `roi_top_ratio` 아래만 처리하고 `process_width`로 축소한다.
- 후보 contour는 면적순으로 제한하고, anchor/lookahead band projection으로
  tracking X를 측정한다.
- 전체 contour centroid가 아니라 line 폭 run의 중심을 control reference로
  쓰므로 T/+ 교차점과 넓은 바닥 blob에 덜 끌린다.
- stabilizer는 EMA, hold, reacquire, offset velocity gate를 적용한다.
- marker polygon 및 marker-like square occluder가 밝은 라인 mask에서 제거될
  수 있다.

### Intersection

- `IntersectionDetector`는 line mask와 raw line을 기준으로 branch ray score를
  계산한다.
- `IntersectionStabilizer`는 raw `straight`, `L`, `T`, `+` 결과를 temporal
  smoothing한다.
- `IntersectionDecisionEngine`은 최근 branch evidence window를 사용한다.
- `node_record_y_min/max` band 안에서만 node record를 낸다.
- weak fourth branch가 `T`를 `+`로 끌어올리지 못하도록
  `high_confidence_score` gate를 둔다.
- `turn_ready`, `overshoot_risk`, `too_late_to_turn`은 mission/debug 판단이며
  직접 autopilot command가 아니다.

### Marker

- ArUco dictionary는 `DICT_4X4_50`.
- Grid mission에서는 vertiport marker ID 23은 grid marker count에서 제외한다.
- Grid marker는 `MarkerWindow`가 최근 프레임에서 같은 ID를 충분히 확인할 때
  commit된다.
- window 안에 서로 다른 실제 marker ID가 섞이면 window를 비워 ambiguous
  commit을 막는다.

## 7. Grid Mission 알고리즘

Grid arena mission state:

```text
IDLE
  -> ARM_TAKEOFF
  -> MARKER_LOCK_YAW
  -> ENTRY_FORWARD
  -> ENTRY_CENTER_ORIGIN
  -> ENTRY_ORIGIN_MARKER_HOVER
  -> SNAKE_LAUNCH_ALIGN
  -> SNAKE_FORWARD
  -> SNAKE_RECORD_NODE
  -> SNAKE_STOP_AT_CENTER
  -> SNAKE_TURN_90
  -> SNAKE_ADVANCE_ONE_CELL
  -> SNAKE_TURN_90_AGAIN
  -> TURN_NODE_MARKER_HOVER       (snake 도중 마커 발견 시 dwell)
  -> SNAKE_COMPLETE
  -> REVISIT_INIT
  -> REVISIT_FORWARD
  -> REVISIT_STOP_AT_TURN
  -> REVISIT_TURN_90
  -> REVISIT_MARKER_HOVER
  -> REVISIT_COMPLETE
  -> RETURN_HOME_INIT
  -> RETURN_HOME_FORWARD
  -> RETURN_HOME_STOP_AT_TURN
  -> RETURN_HOME_TURN_90
  -> RETURN_HOME_ALIGN_ORIGIN
  -> RETURN_HOME_FACE_SOUTH
  -> RETURN_VERTIPORT_FORWARD
  -> RETURN_VERTIPORT_MARKER_HOVER
  -> LAND
  -> MISSION_COMPLETE
  -> DONE

any active state -> EMERGENCY_LAND
```

핵심 규칙 (진입/탐색 공통):

- Gazebo ground-truth pose를 쓰지 않는다.
- 단거리 distance/hover 기준으로 MAVLink `LOCAL_POSITION_NED`, attitude,
  rangefinder, vision event, elapsed time만 사용한다.
- Vertiport와 grid 사이에는 line이 없으므로 `ENTRY_FORWARD`는 yaw-frozen
  blind forward다.
- 첫 grid intersection은 보는 즉시 origin으로 확정하지 않고
  `ENTRY_CENTER_ORIGIN`에서 X/Y 중심과 속도 gate를 통과해야 `(0,0)`이 된다.
- `GridCoordinateTracker::update()`는 peek-only다. Mission gate가 승인할 때만
  `commitAdvance()`로 tracker 상태를 전진시킨다.
- `astroquad-onboard`는 GCS UDP loss에 대비해
  최신 committed grid node를 매 frame telemetry에 다시 실어 보낸다.
- Snake hop은 last committed node의 LOCAL_NED 위치에서 시작한다.
- Cell 중간 `hop_align_start_m..hop_align_end_m` 구간에서만 line following을
  열고, 나머지는 yaw-locked `ForwardBlind`를 사용한다.
- Boundary에서는 `SnakePlanner`가 첫 valid turn 방향을 latch하고, 이후
  boundary마다 left/right를 strict alternation한다.
- 기대 alternation branch가 없으면 반대 방향으로 backtrack하지 않고 snake를
  complete로 보고 다음 단계(재방문)로 전이한다.

마커 재방문 (Revisit):

- Snake 종료 시점의 격자 좌표/heading에서 출발해 `MarkerRevisitPlanner`가
  `RevisitLeg`(목표 marker_id + `RevisitSegment{heading, cells}` 리스트)들을
  생성한다.
- 정렬 순서는 `revisit_order` 설정 (`asc` 또는 `desc`). CLI `--revisit-order`로
  override. 기본은 `desc` (큰 ID부터).
- 각 leg는 segment 단위로 `REVISIT_FORWARD` → `REVISIT_STOP_AT_TURN` →
  `REVISIT_TURN_90`을 반복하며 진행한다. 목표 마커 위에서는
  `REVISIT_MARKER_HOVER`로 dwell한 뒤 `MarkerRegistry.revisited` 플래그를 갱신한다.
- 모든 마커 방문이 끝나면 `REVISIT_COMPLETE` → `RETURN_HOME_INIT`으로 전이한다.

출발점 복귀 (Return Home / Return Vertiport):

- 마지막 재방문 위치에서 `(0,0)`까지 `RETURN_HOME_*` 상태로 격자 경로를 따라
  돌아온다.
- `(0,0)`에 도착하면 `RETURN_HOME_ALIGN_ORIGIN`에서 중심을 다시 잡고,
  `RETURN_HOME_FACE_SOUTH`에서 vertiport 방향(grid south)으로 yaw를 돌린다.
- 그 다음 `RETURN_VERTIPORT_FORWARD`로 vertiport로 yaw-frozen blind forward.
  vertiport ArUco를 잡으면 `RETURN_VERTIPORT_MARKER_HOVER`에서 중심을 맞춘 뒤
  `LAND` → `MISSION_COMPLETE` → `DONE`으로 종료한다.
- 어떤 단계에서든 heartbeat loss / mission timeout / altitude ceiling /
  hop-distance 초과 등이 발생하면 `EMERGENCY_LAND`로 분기한다.

## 8. Control / MAVLink

Primary:

- ArduPilot `GUIDED`
- MAVLink `SET_POSITION_TARGET_LOCAL_NED`
- Body-frame velocity setpoint

Line-follow experiment-only alternate path:

- `line_follow_node --control-backend alt_hold_rc_override`
- ArduPilot `ALT_HOLD`
- Optional line-follow-only `--alt-hold-auto-takeoff` arms and climbs using
  `RC_CHANNELS_OVERRIDE` throttle, then switches to roll/pitch/yaw overrides
  with neutral throttle for line following.
- This path is intentionally not used by `astroquad-onboard`.

현재 intent:

| Intent | 역할 |
|---|---|
| `Idle` | command 없음 |
| `HoldPosition` | body velocity 0 + altitude hold |
| `ForwardBlind` | yaw locked forward flight |
| `LineFollow` | line lateral correction + yaw lock |
| `StopAndCenter` | boundary center deceleration |
| `IntersectionCenter` | first origin/node center X/Y correction |
| `LaunchAlign` | outgoing line lateral/yaw settle before hop |
| `YawAlign` | hold position하면서 target yaw로 정렬 (return-home face-south 등) |
| `YawTurn` | 90도 yaw-only turn |
| `MarkerHover` | marker center hold + yaw target (snake dwell / revisit / vertiport 착륙 전 모두 사용) |
| `Land` | mapper는 setpoint를 내지 않고 LAND/GUIDED landing path가 처리 |

Real hardware boundary:

- `line_follow_node --target ardupilot_serial` has guarded serial support.
- `mavlink_probe` and `mavlink_motor_test` are the required bench tools.
- `astroquad-onboard --target ardupilot_serial --allow-arm-takeoff` has
  guarded serial support and uses the same RC/local-estimate preflight gates as
  the line-follow real path.

## 9. Protocol

공통 protocol 문서:

- `uav-onboard/docs/PROTOCOL.md`
- `uav-gcs/docs/PROTOCOL.md`

현재 문서 version은 v1.8이고 JSON top-level `protocol_version`은 호환성을
위해 integer `1`이다.

주요 telemetry group:

- `system.*`: board, OS, uptime, CPU temp, throttling, load, memory, Wi-Fi
- `camera.*`: sensor/config/focus/exposure/capture FPS/frame seq
- `vision.line.*`: line tracking metadata
- `vision.intersection.*`: stabilized intersection classifier output
- `vision.intersection_decision.*`: sliding-window node/turn decision
- `vision.grid_node.*`: latest committed local grid node
- `vision.drone_position.*`: fractional progress from last committed node
- `vision.markers[]`: current-frame ArUco observations
- `debug.*`: timing, video counters, line workload counters

`MissionTelemetry`는 현재 `astroquad-onboard`가 매 프레임 채워 보낸다. 포함되는
주요 필드는 다음과 같다.

- `mission.state` / `mission.control_intent` / `mission.target_altitude_m`
- `mission.grid.{x,y,heading,snake_dir,valid}`
- `mission.vertiport.{verified,marker_id}` / `mission.vertiport_acquired`
- `mission.markers_expected` / `mission.markers_found[]`
  (각 항목: `id`, `grid_x/y`, `orientation_deg`, `first_seen_s`, `revisited`, `revisited_s`)
- `mission.snake_complete` / `mission.revisit_active` / `mission.revisit_order`
  / `mission.revisit_target_id` / `mission.revisit_remaining`
- `mission.return_active` / `mission.return_phase` / `mission.vertiport_return_active`
- `mission.grid_map_finalized` / `mission.grid_pose_visible`
- `mission.landing_success` / `mission.mission_complete`
- `mission.last_safety_event`

GCS는 이 필드들로 진행 단계, 발견/재방문 마커, 복귀 phase, 착륙 결과까지
구조화된 형태로 렌더한다. `debug.note = "grid_mission"`는 호환을 위해 유지한다.

## 10. Current Config Defaults

`config/vision.toml`:

```toml
[camera]
sensor_model = "imx296"
width = 1456
height = 1088
fps = 12
jpeg_quality = 45
exposure = "sport"
shutter_us = 0   # 경기장에서 실측값으로 AE 고정
gain = 0.0

[line]
mode = "light_on_dark"
mask_strategy = "white_fill"
process_width = 480
roi_top_ratio = 0.08
lookahead_y_ratio = 0.55
lookahead_band_ratio = 0.06

[debug_video]
enabled = false
send_fps = 5
chunk_pacing_us = 150
```

`vision_debug_node`, `line_follow_node`, and `astroquad-onboard`
expose `--fps <n>` as a CLI override for raw debug-video
send FPS. Mission telemetry still follows processed frames; debug video remains
best-effort.

`config/mission.toml [grid_mission]` 주요값:

```toml
vertiport_marker_id = 23
markers_expected = 4
vertiport_altitude_m = 2.0
cruise_altitude_m = 2.0
marker_lock_yaw_delta_deg = 90.0
entry_forward_speed_mps = 0.30
entry_blind_clear_distance_m = 3.00
entry_intersection_min_distance_m = 3.00
entry_center_target_y_norm = 0.55
cell_size_m = 3.0
altitude_ceiling_m = 3.5
hop_align_start_m = 1.4
hop_align_end_m = 1.7
hop_max_distance_m = 3.5
hop_intersection_min_distance_m = 1.0
marker_window_frames = 8
marker_window_min_count = 4

# Marker revisit (snake 완료 후 발견 마커 재방문)
revisit_order = "desc"                   # asc | desc, CLI --revisit-order로 override
revisit_passthrough_regular_nodes = true # 비목표 노드는 통과
```

## 11. Directory / File Roles

```text
uav-onboard/
├─ CMakeLists.txt
├─ PROJECT_SPEC.md
├─ README.md
├─ config/
│  ├─ autopilot.toml
│  ├─ mission.toml
│  ├─ network.toml
│  ├─ ardupilot_serial_usb.expected.toml
│  ├─ ardupilot_serial_usb.params
│  ├─ runtime.ardupilot_serial.toml
│  ├─ runtime.sitl.grid.toml
│  ├─ runtime.sitl.toml
│  ├─ safety.toml
│  └─ vision.toml
├─ docs/PROTOCOL.md
├─ scripts/
│  ├─ grid_arena_test.sh
│  ├─ line_tracing_test.sh
│  └─ setup_rpi_dependencies.sh
├─ sim/gazebo/
├─ src/
│  ├─ app/                  # AstroquadOnboardApp / VisionDebugPipeline / GcsTelemetryPublisher
│  ├─ autopilot/            # MAVLink adapter + UDP/serial transports
│  ├─ camera/               # rpicam MJPEG capture
│  ├─ common/               # config/time helpers
│  ├─ control/              # setpoint model, guided velocity, grid mapper
│  ├─ mission/              # altitude, grid mission (entry/snake/revisit/return),
│  │                        # tracker, snake planner, marker registry, marker revisit planner
│  ├─ network/              # UDP telemetry sender
│  ├─ protocol/             # telemetry JSON serializer
│  ├─ safety/               # safety monitor
│  ├─ video/                # UDP MJPEG streamer
│  ├─ vision/               # detectors, stabilizers, frame sources, processor
│  ├─ main.cpp              # astroquad-onboard entrypoint
│  └─ telemetry_main.cpp    # uav-onboard-telem entrypoint
├─ tests/
└─ tools/
```

Key entrypoints/tools:

| File | Role |
|---|---|
| `src/main.cpp` | thin astroquad-onboard entrypoint |
| `src/app/AstroquadOnboardApp.*` | grid mission runtime composition root |
| `src/telemetry_main.cpp` | telemetry bring-up sender |
| `tools/vision_debug_node.cpp` | camera/vision/GCS debug runtime |
| `tools/line_follow_node.cpp` | line-follow staging mission |
| `tools/mavlink_probe.cpp` | no-arm MAVLink/autopilot bench probe |
| `tools/mavlink_motor_test.cpp` | props-removed motor command check |
| `tools/grid_image_smoke.cpp` | deterministic image-grid smoke |
| `tools/marker_grid_replay.cpp` | marker/grid replay helper |

Current focused tests cover telemetry JSON, line/marker/intersection
stabilizers, intersection decision, guided velocity control, line-follow
mission, grid mission entry/origin/snake/revisit/return-home transitions,
marker revisit planner, safety monitor, fake frame source, vision processor,
and MAVLink UDP/serial adapter behavior.

## 12. Build / Run / Test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Vision debug:

```bash
./build/vision_debug_node --config config --line-only --line-mode light_on_dark --video
```

Line-follow SITL:

```bash
./build/line_follow_node --config config --target sitl --vision gazebo \
  --line-mode light_on_dark --video --gcs-ip <windows-gcs-ip>
```

Grid mission SITL (진입→snake→재방문→복귀→착륙 풀-미션):

```bash
./build/astroquad-onboard --config config --target sitl --vision gazebo \
  --world grid --line-mode light_on_dark --marker-count 4 \
  --revisit-order desc \
  --video --gcs-ip <windows-gcs-ip>
```

`--revisit-order`는 `asc`/`desc` 중 선택 (기본 `desc`).

ArduPilot serial bench:

```bash
./build/mavlink_probe --config config --target ardupilot_serial \
  --duration-ms 12000 --strict-local-estimate
```

## 13. 개발 우선순위

| 순서 | 작업 | 이유/검증 |
|---:|---|---|
| 1 | `astroquad-onboard` SITL 풀-미션 (snake/revisit/return) 안정화 유지 | 현재 대회 미션의 중심 |
| 2 | Real ArduPilot serial grid 풀-미션 guarded flight 검증 | serial/vision/control path 통합 검증 |
| 3 | Revisit/return 회귀 테스트 확대 | 마커 leg 생성, return-home 경로, vertiport 정렬 |
| 4 | `grid_mission_node` alias 제거 (2026-07 완료) | 최종 실행 파일 정리 |
| 5 | GCS command channel 연동 | START/ABORT/EMERGENCY LAND/backend 선택 |
| 6 | File logging/replay | 비행 후 재현성과 보고서 자료 |
| 7 | Safety monitor 확장 | mission-local failsafe 외 공통 safety event 집계 |

## 14. 아직 하지 않는 것

- GCS가 자율주행 판단을 대신하지 않는다.
- Gazebo ground-truth pose를 mission 입력으로 쓰지 않는다.
- 실기체에서 RC/local-estimate preflight gate 없이 arm/takeoff하지 않는다.
- Debug video frame loss를 mission failure로 보지 않는다.
- Revisit/return 단계에서도 ground-truth 좌표가 아니라 격자 기반 hop과
  LOCAL_NED 거리만 사용한다.
- Snake 도중 alternation branch가 비면 backtrack하지 않고 다음 단계(재방문)로
  넘어간다.
