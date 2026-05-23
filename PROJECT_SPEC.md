# PROJECT_SPEC.md — uav-onboard

> 제24회 한국로봇항공기경연대회 중급부문 멀티콥터형 드론 실내 조난자 탐색 온보드 소프트웨어 기준 문서  
> **이 문서는 팀원과 코딩 에이전트가 공통으로 참조하는 Single Source of Truth입니다.**

최종 수정: 2026-05-21

## 1. 프로젝트 목적

`uav-onboard`는 Raspberry Pi 4에서 하향 카메라 비전, 격자 미션 상태머신,
MAVLink 제어, safety, GCS telemetry/debug video 송신을 담당한다.

최종 목표는 GPS 없는 격자 환경에서 이륙, grid snake 탐색, ArUco 조난자
마커 기록, 번호 역순 재방문, 출발점 복귀, 자동 착륙까지 수행하는 것이다.

현재 구현 기준은 다음과 같이 나뉜다.

- `astroquad-onboard`: 현재 grid arena full-snake SITL 및 guarded ArduPilot
  serial mission runtime.
- `uav-onboard-telem`: basic telemetry bring-up sender / development probe.
- `vision_debug_node`: 현재 안정적인 camera/vision/GCS debug runtime.
- `line_follow_node`: SITL 및 guarded ArduPilot serial line-follow staging.
- `grid_mission_node`: `astroquad-onboard`와 같은 entrypoint를 빌드하는
  compatibility/staging alias.

## 2. 책임 범위

| 구분 | 내용 |
|---|---|
| 담당 | Pi camera frame 획득, ArUco/line/intersection detection, local grid-node 판단, mission state machine, grid/snake policy, control intent mapping, MAVLink adapter, safety, telemetry/debug video |
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
| Grid mission state machine | SITL staging | `src/mission/GridMission.*`, `src/app/AstroquadOnboardApp.*` |
| Snake boundary planner | 구현됨 | `src/mission/SnakePlanner.*` |
| Marker registry/window gate | 구현됨 | `src/mission/MarkerRegistry.*`, `GridMission::MarkerWindow` |
| Guided velocity line/marker controller | 구현됨 | `src/control/GuidedVelocityController.*` |
| Grid intent -> setpoint mapper | 구현됨 | `src/control/GridControlMapper.*` |
| Altitude policy | 구현됨 | `src/mission/AltitudePolicy.*` |
| MAVLink UDP transport | 구현됨 | `src/autopilot/UdpMavlinkTransport.*` |
| MAVLink serial transport | 구현됨 | `src/autopilot/SerialMavlinkTransport.*` |
| Autopilot adapter | 구현됨 | `src/autopilot/AutopilotMavlinkAdapter.*` |
| Safety monitor | 부분 구현 | `src/safety/SafetyMonitor.*`, mission-local failsafe |
| GCS telemetry/video publisher | 구현됨 | `src/app/GcsTelemetryPublisher.*` |
| Gazebo line/grid worlds | 구현됨 | `sim/gazebo/`, `scripts/line_tracing_test.sh`, `scripts/grid_arena_test.sh` |
| ArduPilot serial bench tools | 구현됨 | `tools/mavlink_probe.cpp`, `tools/mavlink_motor_test.cpp` |
| GCS command receiver | 미구현 | planned |
| Marker reverse revisit / return home | 미구현 | planned |
| Official coordinate conversion | 미구현 | planned |
| File logging subsystem | 미구현 | `logs/`, future `src/logging` |

## 4. 하드웨어/소프트웨어 기준

| 모델/장치 | 연결 위치 | 역할 |
|---|---|---|
| ArduPilot-compatible flight controller | 기체 중앙 | 비행 제어기. 자세 안정화, 모터 출력, 센서 융합, 모드 관리 |
| MicoAir MTF-01 Optical Flow & Range Sensor | FC TELEM/SERIAL 포트 | GPS 없이 수평 이동 추정 + 바닥 거리 측정 |
| Raspberry Pi 4 | FC USB 또는 TELEM/SERIAL 포트 | OpenCV 비전, mission, MAVLink command sender |
| IMX519 Camera | Raspberry Pi CSI | 하향 영상 입력 |
| Power Module | FC POWER | 전압/전류 측정 |
| RC Receiver | FC RC IN 또는 운용 중인 RC 경로 | 수동 takeover, 비상 개입 |
| ESC/Motor/PDB | FC MAIN OUT + 전원분배 | 실제 모터 구동 |

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
  -> GridMission
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

- `mask_strategy = "white_fill"` 기본. `dark_on_light`에서는 dark fill path를
  사용해 검은 격자선을 흰 바닥에서 잡는다.
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
  -> SNAKE_LAUNCH_ALIGN
  -> SNAKE_FORWARD
  -> SNAKE_RECORD_NODE
  -> SNAKE_STOP_AT_CENTER
  -> SNAKE_TURN_90
  -> SNAKE_ADVANCE_ONE_CELL
  -> SNAKE_TURN_90_AGAIN
  -> SNAKE_COMPLETE
  -> LAND
  -> DONE

any active state -> EMERGENCY_LAND
```

핵심 규칙:

- Gazebo ground-truth pose를 쓰지 않는다.
- 단거리 distance/hover 기준으로 MAVLink `LOCAL_POSITION_NED`, attitude,
  rangefinder, vision event, elapsed time만 사용한다.
- Vertiport와 grid 사이에는 line이 없으므로 `ENTRY_FORWARD`는 yaw-frozen
  blind forward다.
- 첫 grid intersection은 보는 즉시 origin으로 확정하지 않고
  `ENTRY_CENTER_ORIGIN`에서 X/Y 중심과 속도 gate를 통과해야 `(0,0)`이 된다.
- `GridCoordinateTracker::update()`는 peek-only다. Mission gate가 승인할 때만
  `commitAdvance()`로 tracker 상태를 전진시킨다.
- `astroquad-onboard`와 `grid_mission_node` alias는 GCS UDP loss에 대비해
  최신 committed grid node를 매 frame telemetry에 다시 실어 보낸다.
- Snake hop은 last committed node의 LOCAL_NED 위치에서 시작한다.
- Cell 중간 `hop_align_start_m..hop_align_end_m` 구간에서만 line following을
  열고, 나머지는 yaw-locked `ForwardBlind`를 사용한다.
- Boundary에서는 `SnakePlanner`가 첫 valid turn 방향을 latch하고, 이후
  boundary마다 left/right를 strict alternation한다.
- 기대 alternation branch가 없으면 반대 방향으로 backtrack하지 않고 snake를
  complete로 보고 land한다.
- 현재는 marker count 충족 또는 snake complete 후 land한다. Marker ID 역순
  재방문과 출발점 복귀는 아직 구현하지 않았다.

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
- This path is intentionally not used by `astroquad-onboard` or
  `grid_mission_node`.

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
| `YawTurn` | 90도 yaw-only turn |
| `MarkerHover` | marker center hold + yaw target |
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

`MissionTelemetry` richer fields are present in the serializer, but current
`GcsTelemetryPublisher` users do not populate them yet. `astroquad-onboard`
state is currently authoritative in its console log plus `debug.note =
"grid_mission"`.

## 10. Current Config Defaults

`config/vision.toml`:

```toml
[camera]
sensor_model = "imx519"
width = 960
height = 720
fps = 12
mode = "2328:1748:10:P"
jpeg_quality = 45
focus_absolute = 1984
exposure = "sport"

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

`vision_debug_node`, `line_follow_node`, `astroquad-onboard`, and
`grid_mission_node` expose `--fps <n>` as a CLI override for raw debug-video
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
entry_blind_clear_distance_m = 1.80
entry_intersection_min_distance_m = 1.80
entry_center_target_y_norm = 0.55
cell_size_m = 3.0
hop_align_start_m = 1.4
hop_align_end_m = 1.7
hop_max_distance_m = 3.5
hop_intersection_min_distance_m = 1.0
marker_window_frames = 8
marker_window_min_count = 4
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
│  ├─ mission/              # altitude, grid mission, tracker, snake, marker registry
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
| `src/main.cpp` | thin astroquad-onboard/grid_mission_node entrypoint |
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
mission, grid mission entry/origin, safety monitor, fake frame source, vision
processor, and MAVLink UDP/serial adapter behavior.

## 12. Build / Run / Test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Vision debug:

```bash
./build/vision_debug_node --config config --line-only --line-mode dark_on_light --video
```

Line-follow SITL:

```bash
./build/line_follow_node --config config --target sitl --vision gazebo \
  --line-mode dark_on_light --video --gcs-ip <windows-gcs-ip>
```

Grid mission SITL:

```bash
./build/astroquad-onboard --config config --target sitl --vision gazebo \
  --world grid --line-mode dark_on_light --marker-count 4 \
  --video --gcs-ip <windows-gcs-ip>
```

ArduPilot serial bench:

```bash
./build/mavlink_probe --config config --target ardupilot_serial \
  --duration-ms 12000 --strict-local-estimate
```

## 13. 개발 우선순위

| 순서 | 작업 | 이유/검증 |
|---:|---|---|
| 1 | `astroquad-onboard` SITL snake 안정화 | 현재 full-grid 알고리즘의 중심 |
| 2 | GCS mission/grid telemetry 보강 | console-only state를 GCS에도 구조화 |
| 3 | Grid mission unit/integration tests 확대 | Entry/origin/hop/turn/marker commit 회귀 방지 |
| 4 | Real ArduPilot serial grid mission guarded flight 검증 | serial/vision/control path 통합 검증 |
| 5 | `grid_mission_node` alias 제거 여부 결정 | 최종 실행 파일 정리 |
| 6 | Marker reverse revisit / return home policy | 대회 최종 미션 완성 |
| 7 | GCS command channel 연동 | START/ABORT/EMERGENCY LAND/backend 선택 |
| 8 | File logging/replay | 비행 후 재현성과 보고서 자료 |

## 14. 아직 하지 않는 것

- GCS가 자율주행 판단을 대신하지 않는다.
- Gazebo ground-truth pose를 mission 입력으로 쓰지 않는다.
- 실기체에서 RC/local-estimate preflight gate 없이 arm/takeoff하지 않는다.
- Debug video frame loss를 mission failure로 보지 않는다.
- Marker reverse revisit/return-home을 구현된 기능처럼 문서화하지 않는다.
