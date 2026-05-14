# PROJECT_SPEC.md — uav-onboard

> 제24회 한국로봇항공기경연대회 중급부문 멀티콥터형 드론 실내 조난자 탐색 온보드 소프트웨어 기준 문서  
> **이 문서는 팀원과 코딩 에이전트가 공통으로 참조하는 Single Source of Truth입니다.**

최종 수정: 2026-05-13

---

## 1. 프로젝트 목적

GPS 없는 실내 환경에서, 하향 카메라 기반 라인트레이싱과 ArUco 마커 인식을 통해 격자 구역 내 조난자(마커)를 탐색하고, 탐색 결과를 역순으로 재방문한 뒤 자동 복귀·착륙하는 드론 온보드 소프트웨어를 개발한다.

현재 레포는 최종 미션 소프트웨어 전체 중 **카메라 입력, onboard vision, telemetry, debug video bring-up, intersection decision, local grid-node telemetry** 단계까지 구현되어 있다. 비전 인식은 MVP 기준 거의 완료된 것으로 보고, 다음 핵심 작업은 mission state machine, Pixhawk/MAVLink control, safety/failsafe를 연결하는 것이다.

최종 운용 실행 파일은 `./build/uav_onboard`다. 현재 `uav_onboard`는 basic telemetry bring-up sender에 가깝지만, 최종적으로는 vision, mission, control, safety, telemetry, MAVLink를 조립하는 onboard composition root가 되어야 한다. `vision_debug_node`는 계속 비전 bring-up/debug 실행 파일로 유지한다.

문서 계층:

- 전체 시스템 공통 기준은 `development-log/SYSTEM_SPEC.md`를 따른다.
- 72시간/1주일 MVP 계획은 `development-log/MVP_PLAN.md`를 따른다.
- 이 문서는 `uav-onboard` repo의 책임, 모듈, 실행 파일, 빌드/테스트 기준만 상세히 다룬다.
- `development-log/RESEARCH.md`와 `development-log/PLAN.md`는 매 스텝마다 바뀌는 작업용 scratchpad다.

---

## 2. 미션 시나리오 요약

최종 목표 시나리오는 full grid mission이다.

```text
[이륙] -> [라인 추종, 격자 진입] -> [Snake 탐색, 교차점마다 ArUco 인식]
      -> [모든 마커 발견] -> [마커 번호 역순 재방문] -> [출발점 복귀] -> [자동 착륙]
```

1. **이륙**: 이륙 포인트에서 자동 이륙. ArduPilot GUIDED/AUTO 기반 명령을 사용한다.
2. **격자 진입**: 이륙 포인트에서 격자 구역까지 연결된 라인을 따라 이동한다.
3. **Snake 탐색**: 격자 구역 내부를 지그재그로 순회하며 교차점을 방문한다.
4. **마커 인식**: 각 교차점에서 ArUco 마커 ID를 인식하고 격자 좌표와 묶어 저장한다.
5. **역순 재방문**: 발견된 마커를 번호 역순으로 재방문한다.
6. **복귀 및 착륙**: 출발점으로 복귀 후 자동 착륙한다.

### 2.1 Near-Term MVP Scope

72시간 실기체 MVP는 최종 목표의 부분 집합이다.

```text
[MTF-01 bring-up] -> [자동 이륙] -> [짧은 직선 라인 추종] -> [라인 끝/timeout 안전 착륙]
```

MVP에서 제외하는 것:

- full snake grid exploration
- ArUco marker revisit
- official coordinate conversion
- GCS full command UI
- 교차점 회전/분기 의사결정

이 기능들은 최종 목표에 남기되, 첫 실기체 자동비행 gate의 선행 조건으로 두지 않는다.

---

## 3. 이 레포의 책임 범위

| 구분 | 내용 |
|---|---|
| 담당 | Pi camera frame 획득, ArUco/line/intersection detection, local grid-node 판단, vision telemetry 생성, optional raw debug video 송신, mission state/grid/path/control/MAVLink/safety |
| 미담당 | 자세 안정화와 모터 제어는 ArduPilot/Pixhawk 담당, GCS UI/overlay/log window는 `uav-gcs` 담당 |

중요한 역할 분리:

- Onboard는 ArUco marker 정보와 line tracing 정보를 계산해 telemetry로 보낸다.
- Onboard는 영상에 box, contour, text, tracking point overlay를 그리지 않는다.
- GCS raw video는 debug 관제용 best-effort channel이며 mission-critical 경로가 아니다.
- 최종 시연에서는 GCS 없이 onboard만 실행할 수 있어야 하므로, 미션 판단에 필요한 정보는 onboard에 있어야 한다.

---

## 4. 현재 구현 상태

| 영역 | 상태 | 구현 위치 |
|---|---|---|
| Raspberry Pi camera capture | 구현됨 | `src/camera/RpicamMjpegSource.*` |
| UDP telemetry sender | 구현됨 | `src/network/UdpTelemetrySender.*` |
| Telemetry JSON v1.7 serialization | 구현됨 | `src/protocol/TelemetryMessage.*` |
| UDP MJPEG debug video chunk 송신 | 구현됨 | `src/video/*` |
| GCS discovery 수신 후 video unicast | 구현됨 | `tools/vision_debug_node.cpp`, `tools/video_streamer.cpp` |
| ArUco marker detection | 구현됨 | `src/vision/ArucoDetector.*` |
| Marker stabilization | 구현됨 | `src/vision/MarkerStabilizer.*` |
| Line tracing MVP | 구현됨 | `src/vision/LineMaskBuilder.*`, `src/vision/LineDetector.*` |
| Line stabilizer | 구현됨 | `src/vision/LineStabilizer.*` |
| Intersection classifier/stabilizer | 구현됨 | `src/vision/IntersectionDetector.*`, `src/vision/IntersectionStabilizer.*` |
| Intersection decision/local grid node | 구현됨 | `src/mission/IntersectionDecision.*`, `src/mission/GridCoordinateTracker.*` |
| Pi 4 + IMX519 camera/focus/exposure config | 구현됨 | `config/vision.toml`, `src/common/VisionConfig.*` |
| System/camera/debug telemetry | 구현됨 | `src/app/VisionDebugPipeline.cpp` |
| Full mission state machine | 미구현 | planned in `src/mission/` |
| Full grid map/path planner/revisit policy | 미구현 | planned in `src/mission/` |
| Pixhawk/MAVLink serial | 미구현 | `src/autopilot/.gitkeep`, `config/autopilot.toml` |
| Control module | 미구현 | `src/control/.gitkeep` |
| Safety/failsafe | 미구현 | `src/safety/.gitkeep`, `config/safety.toml` |
| File logging | 미구현 | `src/logging/.gitkeep`, `logs/.gitkeep` |

---

## 5. 하드웨어/소프트웨어 환경

### 5.1 Target Hardware

| 모델/장치 | 연결 위치 | 역할 |
|---|---|---|
| Pixhawk1 | 기체 중앙 | 비행 제어기. 자세 안정화, 모터 출력, 센서 융합, 모드 관리 |
| MicoAir MTF-01 Optical Flow & Range Sensor | Pixhawk TELEM2 또는 SERIAL4/5 | GPS 없이 수평 이동 추정 + 바닥 거리 측정. Optical Flow + ToF range 통합 |
| Raspberry Pi 4 | Pixhawk USB 또는 TELEM1 | OpenCV 비전 처리, 라인/교차점/ArUco 판단, MAVLink 제어 명령 전송 |
| IMX519 Camera | Raspberry Pi CSI | 하향 영상 입력. 라인 중심, 라인 각도, ArUco 마커 인식 |
| Power Module | Pixhawk POWER | Pixhawk 전원 공급, 배터리 전압/전류 측정 |
| RC Receiver | Pixhawk RC IN | 수동 조종 takeover, 비상 개입 |
| ESC/Motor/PDB | Pixhawk MAIN OUT + 전원분배 | 실제 모터 구동 |
| TFmini Plus | 기본 구성에서는 제외 | MTF-01 range가 불안정할 때 예비 rangefinder |
| External Compass | 제외 | 일단 Pixhawk 내부 compass/IMU 기준으로 진행 |

제어 primary는 ArduPilot `GUIDED` mode에서 MAVLink `SET_POSITION_TARGET_LOCAL_NED` body-frame velocity setpoint를 보내는 방식이다. 이 방식은 MTF-01 optical flow/range 기반 local estimate가 안정적이어야 한다. `ALT_HOLD + RC_CHANNELS_OVERRIDE`는 fallback backend로만 둔다.

MTF-01 bring-up은 제어 구현 전 gate다. ArduPilot에서 optical flow와 rangefinder가 안정적으로 들어오지 않으면 GUIDED velocity 기반 line-follow 비행은 진행하지 않는다.

### 5.2 Software Dependencies

| 항목 | 현재 기준 |
|---|---|
| Language | C++17 |
| Build | CMake 3.16+ |
| Config | TOML via `tomlplusplus` |
| JSON | `nlohmann/json` |
| Camera runtime | `rpicam-vid` MJPEG stdout |
| Vision | OpenCV 4.x with `aruco` for vision tools/node |
| Network | UDP socket, Windows compatibility where tools support it |
| Tests | CTest + lightweight assert-based tests |
| MAVLink | 아직 vendored/generated headers 없음. exact dialect/version 확정 후 추가 |

`scripts/setup_rpi_dependencies.sh`는 Raspberry Pi OS Lite 64-bit 기준으로 build tools, OpenCV, rpicam/libcamera 계열 package, `iw`, JSON/TOML/dev package를 설치/확인한다.

---

## 6. 온보드 시스템 아키텍처

### 6.1 Target Architecture

ROS를 직접 쓰지는 않지만, 목표 구조는 ROS node graph와 비슷하게 모듈 간 입력/출력 타입만 공유하고 내부 상태를 침범하지 않는 형태다.

```text
CameraSource
  -> VisionPipeline
  -> MissionStateMachine
  -> GuidanceController
  -> ControlBackend
  -> AutopilotMavlinkAdapter

Support:
  SafetyMonitor observes Vision/Mission/Autopilot/GCS state
  TelemetryPublisher observes state snapshots
  CommandReceiver injects high-level mission commands
  DebugVideoPublisher observes camera frames only
```

`vision_debug_node`와 최종 `uav_onboard`는 같은 `VisionPipeline`을 재사용할 수 있다. 차이는 composition root다. `vision_debug_node`는 vision + telemetry + optional debug video만 묶고, `uav_onboard`는 vision + mission + control + safety + telemetry + MAVLink를 묶는다.

### 6.2 Current Implemented Debug Pipeline

```text
IMX519 camera
  -> rpicam-vid --codec mjpeg -o -
  -> RpicamMjpegSource
  -> JPEG frame extraction
  -> one onboard JPEG decode for detectors
  -> ArucoDetector and/or LineDetector
  -> LineStabilizer
  -> TelemetryMessage JSON
  -> UDP telemetry to GCS
  -> optional raw MJPEG UDP debug video to GCS
```

`vision_debug_node`가 현재 통합 bring-up/debug 실행 파일이다. 기본값은 metadata-only이며, GCS camera/overlay 확인이 필요할 때만 `--video`를 붙인다. 제어 코드는 `VisionDebugPipeline` 내부에 넣지 않는다. vision loop는 typed output을 만들고, mission/control runtime이 그 결과를 구독하거나 callback으로 받아 처리한다.

---

## 7. 주요 모듈 요구사항

### 7.1 Camera

- `rpicam-vid` MJPEG stdout을 안정적으로 읽는다.
- IMX519 focus/exposure/AWB/denoise/orientation option을 config와 CLI override로 전달한다.
- 현재 기본 capture는 `960x720`, `12 FPS`, MJPEG quality `45`다.
- Continuous autofocus는 mission run에서 focus hunting 가능성이 있으므로 기본은 manual focus 후보값을 사용한다.

### 7.2 Vision

- Camera frame을 받아 mission 판단에 필요한 수치 측정값만 생성한다.
- ArUco는 marker id, corners, center, orientation을 계산한다.
- Line tracing은 detected/raw/filtered/held/rejected state, tracking point, offset, angle, confidence, contour를 계산한다.
- GCS 표시용 overlay drawing은 하지 않는다.
- 교차점 `+`, `T`, `L` 분류는 다음 단계에서 line following contour와 분리된 `IntersectionCandidate`로 추가한다.

### 7.3 Mission

- 상태머신 운용, 격자 좌표 갱신, marker 저장, path planning을 담당한다.
- Snake 탐색과 역순 marker revisit을 구현해야 한다.
- 현재 `IntersectionDecision`과 `GridCoordinateTracker`는 구현되어 있으나, 전체 mission state machine과 marker revisit policy는 아직 구현 전이다.
- 72시간 MVP에서는 축소 상태머신 `IDLE`, `TAKEOFF`, `LINE_FOLLOW`, `LAND`, `COMPLETE`, `ABORT`만 사용한다.
- Full snake/revisit 상태는 최종 목표에는 남기되 MVP 이후에 활성화한다.

### 7.4 Control

- Mission 판단을 ArduPilot에 보낼 고수준 명령으로 변환한다.
- Velocity setpoint, yaw rate, hover, takeoff, land 명령을 생성한다.
- 현재는 directory placeholder만 존재한다.

### 7.5 Autopilot / MAVLink

- Pixhawk와의 MAVLink 통신만 담당한다.
- Heartbeat, mode, arm/takeoff/land, velocity setpoint, battery/status 수신을 구현해야 한다.
- `config/autopilot.toml`은 현재 `/dev/serial0`, `115200`, MAVLink system/component id 후보값을 담고 있다.
- SITL/Gazebo 개발에서는 UDP/TCP endpoint를 사용하고, 실기체에서는 serial/USB endpoint를 사용한다. Mission/control 코드는 endpoint 종류를 몰라야 한다.

### 7.6 Telemetry and Command

- Telemetry는 UDP JSON으로 GCS에 송신한다.
- Debug vision 단계에서는 processed camera frame마다 telemetry를 송신해 video frame과 `frame_seq`로 맞출 수 있게 한다.
- Command channel은 TCP JSON으로 계획되어 있으며 아직 구현 전이다.
- 72시간 MVP에서는 GCS command channel을 필수 구현 범위에서 제외한다. 시작/중단은 onboard CLI/config, RC takeover, Pixhawk mode/land 절차를 우선 사용한다.

### 7.7 Debug Video

- Raw camera JPEG만 best-effort로 보낸다.
- Old frame은 drop하고 최신 frame만 유지해 vision loop를 막지 않는다.
- 기본 `debug_video.enabled = false`다.
- `--video`를 켠 경우 기본 target은 `5 FPS`, chunk pacing `150us`다.

### 7.8 Safety and Logging

- Safety는 line lost, GCS lost, Pixhawk heartbeat lost, mission timeout, low battery를 감시해야 한다.
- Logging은 headless 운용을 위해 file logging을 제공해야 한다.
- 현재는 config와 placeholder만 존재한다.

---

## 8. 데이터 흐름

### 8.1 Current Vision Telemetry Flow

```text
CameraFrame
  -> ArUco/Line detector output
  -> protocol::TelemetryMessage
  -> UDP telemetry JSON
  -> GCS TelemetryStore / log / overlay
```

### 8.2 Target Mission Control Flow

```text
VisionOutput
  -> Mission::update()
  -> GridMap / MarkerMap / MissionState
  -> MissionCommand
  -> Control::execute()
  -> MAVLink message
  -> Pixhawk
```

---

## 9. 상태머신 목표

```text
IDLE
  -> TAKEOFF
  -> LINE_FOLLOW_TO_GRID
  -> GRID_EXPLORE
  -> REVISIT_MARKERS
  -> RETURN_HOME
  -> LAND
  -> MISSION_COMPLETE

Any state -> EMERGENCY_LAND
```

| 현재 상태 | 전환 조건 | 다음 상태 |
|---|---|---|
| IDLE | GCS START 명령 | TAKEOFF |
| TAKEOFF | 목표 고도 도달 | LINE_FOLLOW_TO_GRID |
| LINE_FOLLOW_TO_GRID | 격자 진입 교차점 감지 | GRID_EXPLORE |
| GRID_EXPLORE | Snake 탐색 완료 또는 marker 목표 충족 | REVISIT_MARKERS |
| REVISIT_MARKERS | 전체 재방문 완료 | RETURN_HOME |
| RETURN_HOME | 출발점 도달 | LAND |
| LAND | 착륙 완료 | MISSION_COMPLETE |
| any | Safety 이벤트 | EMERGENCY_LAND |

---

## 10. Protocol and Telemetry

현재 공통 protocol 문서:

- `uav-onboard/docs/PROTOCOL.md`
- `uav-gcs/docs/PROTOCOL.md`

현재 문서 version은 v1.7이며 JSON top-level `protocol_version`은 호환성을 위해 integer `1`이다.

채널:

| Channel | Direction | Transport | Port | Status |
|---|---|---|---:|---|
| Telemetry | onboard -> GCS | UDP JSON | 14550 | implemented |
| Command | GCS -> onboard | TCP JSON | 14551 | planned |
| Video | onboard -> GCS | UDP MJPEG chunks | 5600 | implemented |
| GCS discovery | GCS -> LAN broadcast | UDP text beacon | 5601 | implemented |

주요 telemetry group:

- `system.*`: board, OS, uptime, CPU temp, throttling, load, memory, Wi-Fi
- `camera.*`: IMX519 sensor/config/focus/exposure/capture FPS/frame seq
- `vision.markers[]`: ArUco marker metadata
- `vision.line.*`: line tracing metadata
- `grid.*`: 현재 placeholder. 기본 row/col은 `-1`
- `debug.*`: onboard timing, video counters, line workload counters

---

## 11. 현재 Runtime Defaults

`config/vision.toml` 기준:

```toml
[camera]
sensor_model = "imx519"
width = 960
height = 720
fps = 12
jpeg_quality = 45
autofocus_mode = ""
lens_position = -1.0
focus_absolute = 1984
focus_device = "/dev/v4l-subdev1"
exposure = "sport"

[debug_video]
enabled = false
send_fps = 5
jpeg_quality = 40
chunk_pacing_us = 150

[line]
mode = "light_on_dark"
mask_strategy = "local_contrast"
process_width = 480
roi_top_ratio = 0.08
lookahead_y_ratio = 0.55
lookahead_band_ratio = 0.06
morph_open_kernel = 1
morph_close_kernel = 7
filter_enabled = true
```

해상도/화질은 ArUco bench용 고화질보다 onboard 처리 여유를 우선한다. 50cm x 50cm marker를 약 2m 고도에서 보는 대회 조건을 우선 가정하며, 부족할 때만 `--camera-width`, `--camera-height`, `--camera-quality`, `--lens-position`으로 튜닝한다.

---

## 12. 현재 디렉토리 구조와 파일 역할

```text
uav-onboard/
├─ .gitignore
├─ CMakeLists.txt
├─ PROJECT_SPEC.md
├─ README.md
├─ config/
│  ├─ autopilot.toml
│  ├─ mission.toml
│  ├─ network.toml
│  ├─ safety.toml
│  └─ vision.toml
├─ docs/PROTOCOL.md
├─ logs/.gitkeep
├─ scripts/
│  ├─ .gitkeep
│  └─ setup_rpi_dependencies.sh
├─ src/
│  ├─ app/VisionDebugPipeline.cpp
│  ├─ app/VisionDebugPipeline.hpp
│  ├─ autopilot/.gitkeep
│  ├─ camera/CameraFrame.hpp
│  ├─ camera/RpicamMjpegSource.cpp
│  ├─ camera/RpicamMjpegSource.hpp
│  ├─ common/NetworkConfig.cpp
│  ├─ common/NetworkConfig.hpp
│  ├─ common/Time.cpp
│  ├─ common/Time.hpp
│  ├─ common/VisionConfig.cpp
│  ├─ common/VisionConfig.hpp
│  ├─ control/.gitkeep
│  ├─ logging/.gitkeep
│  ├─ main.cpp
│  ├─ mission/.gitkeep
│  ├─ network/UdpTelemetrySender.cpp
│  ├─ network/UdpTelemetrySender.hpp
│  ├─ protocol/TelemetryMessage.cpp
│  ├─ protocol/TelemetryMessage.hpp
│  ├─ safety/.gitkeep
│  ├─ video/UdpMjpegStreamer.cpp
│  ├─ video/UdpMjpegStreamer.hpp
│  ├─ video/VideoPacket.cpp
│  ├─ video/VideoPacket.hpp
│  └─ vision/
│     ├─ ArucoDetector.cpp
│     ├─ ArucoDetector.hpp
│     ├─ LineDetector.cpp
│     ├─ LineDetector.hpp
│     ├─ LineStabilizer.cpp
│     ├─ LineStabilizer.hpp
│     ├─ OpenCvCameraSource.cpp
│     ├─ OpenCvCameraSource.hpp
│     └─ VisionTypes.hpp
├─ test_data/
│  ├─ images/.gitkeep
│  ├─ logs/.gitkeep
│  └─ videos/.gitkeep
├─ tests/
│  ├─ CMakeLists.txt
│  ├─ test_line_stabilizer.cpp
│  └─ test_telemetry_line_json.cpp
└─ tools/
   ├─ aruco_detector_tester.cpp
   ├─ camera_preview.cpp
   ├─ line_detector_tuner.cpp
   ├─ mock_gcs_command.cpp
   ├─ replay_vision.cpp
   ├─ video_streamer.cpp
   └─ vision_debug_node.cpp
```

Root/config/docs:

| 파일 | 역할 |
|---|---|
| `CMakeLists.txt` | build graph, dependencies, optional OpenCV vision tools, tests |
| `README.md` | 현행 build/run/bring-up guide |
| `config/network.toml` | GCS IP/ports, telemetry interval |
| `config/vision.toml` | IMX519 camera, debug video, ArUco, line config |
| `config/autopilot.toml` | future Pixhawk/MAVLink serial config |
| `config/mission.toml` | future grid/marker/takeoff mission config |
| `config/safety.toml` | future failsafe thresholds/timeouts |
| `docs/PROTOCOL.md` | onboard/GCS common protocol spec |

Core source:

| 파일 | 역할 |
|---|---|
| `src/main.cpp` | 현재 basic telemetry bring-up app `uav_onboard`; 최종 mission runtime composition root로 확장 예정 |
| `src/app/VisionDebugPipeline.*` | current integrated camera/vision/telemetry/debug-video pipeline |
| `src/camera/*` | rpicam MJPEG capture and frame model |
| `src/common/*` | config parsing and time utility |
| `src/network/UdpTelemetrySender.*` | UDP telemetry sender |
| `src/protocol/TelemetryMessage.*` | telemetry data model and JSON serialization |
| `src/video/*` | UDP MJPEG packet/chunk streamer |
| `src/vision/*` | ArUco, line detector, line stabilizer, OpenCV camera helper, vision types |

Tests/tools:

| 파일 | 역할 |
|---|---|
| `tests/test_line_stabilizer.cpp` | stabilizer regression |
| `tests/test_telemetry_line_json.cpp` | telemetry JSON regression |
| `tools/vision_debug_node.cpp` | main Pi vision debug executable |
| `tools/video_streamer.cpp` | standalone video streamer |
| `tools/line_detector_tuner.cpp` | offline line tuning |
| `tools/aruco_detector_tester.cpp` | offline ArUco test |
| `tools/camera_preview.cpp` | OpenCV camera smoke tool |
| `tools/mock_gcs_command.cpp` | command channel placeholder |
| `tools/replay_vision.cpp` | replay placeholder |

전체 tracked 파일 인덱스:

```text
.gitignore
CMakeLists.txt
PROJECT_SPEC.md
README.md
config/autopilot.toml
config/mission.toml
config/network.toml
config/safety.toml
config/vision.toml
docs/PROTOCOL.md
logs/.gitkeep
scripts/.gitkeep
scripts/setup_rpi_dependencies.sh
src/app/.gitkeep
src/app/VisionDebugPipeline.cpp
src/app/VisionDebugPipeline.hpp
src/autopilot/.gitkeep
src/camera/CameraFrame.hpp
src/camera/RpicamMjpegSource.cpp
src/camera/RpicamMjpegSource.hpp
src/common/.gitkeep
src/common/NetworkConfig.cpp
src/common/NetworkConfig.hpp
src/common/Time.cpp
src/common/Time.hpp
src/common/VisionConfig.cpp
src/common/VisionConfig.hpp
src/control/.gitkeep
src/logging/.gitkeep
src/main.cpp
src/mission/.gitkeep
src/network/.gitkeep
src/network/UdpTelemetrySender.cpp
src/network/UdpTelemetrySender.hpp
src/protocol/.gitkeep
src/protocol/TelemetryMessage.cpp
src/protocol/TelemetryMessage.hpp
src/safety/.gitkeep
src/video/UdpMjpegStreamer.cpp
src/video/UdpMjpegStreamer.hpp
src/video/VideoPacket.cpp
src/video/VideoPacket.hpp
src/vision/.gitkeep
src/vision/ArucoDetector.cpp
src/vision/ArucoDetector.hpp
src/vision/LineDetector.cpp
src/vision/LineDetector.hpp
src/vision/LineStabilizer.cpp
src/vision/LineStabilizer.hpp
src/vision/OpenCvCameraSource.cpp
src/vision/OpenCvCameraSource.hpp
src/vision/VisionTypes.hpp
test_data/images/.gitkeep
test_data/logs/.gitkeep
test_data/videos/.gitkeep
tests/CMakeLists.txt
tests/test_line_stabilizer.cpp
tests/test_telemetry_line_json.cpp
tools/aruco_detector_tester.cpp
tools/camera_preview.cpp
tools/line_detector_tuner.cpp
tools/mock_gcs_command.cpp
tools/replay_vision.cpp
tools/video_streamer.cpp
tools/vision_debug_node.cpp
```

---

## 13. 빌드, 실행, 테스트

Pi setup:

```bash
bash scripts/setup_rpi_dependencies.sh
```

Build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

최종 onboard runtime 목표:

```bash
./build/uav_onboard --config config
```

Metadata-only vision run:

```bash
./build/vision_debug_node --config config --line-only --line-mode light_on_dark
```

GCS camera/overlay까지 확인:

```bash
./build/vision_debug_node --config config --line-only --line-mode light_on_dark --video
```

Tests:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

---

## 14. 개발 우선순위

| 순서 | 작업 | 이유/검증 |
|---:|---|---|
| 1 | 비전 기준선 freeze와 smoke 재검증 | 이미 완성된 vision path를 안정 기준으로 고정 |
| 2 | MAVLink adapter 구현 | SITL/Pixhawk 연결 준비, heartbeat/mode/arm/takeoff/land/velocity |
| 3 | GUIDED velocity control backend 구현 | A안 primary 제어. line offset/angle -> body velocity/yaw |
| 4 | RC override fallback backend 구현 | B안 fallback. mission logic은 공유하고 출력만 교체 |
| 5 | `uav_onboard` mission composition root 확장 | Vision/Mission/Control/Safety/Telemetry/MAVLink 조립 |
| 6 | Mission state machine 구현 | TAKEOFF/LINE_FOLLOW/GRID_EXPLORE/RETURN/LAND |
| 7 | Safety monitor 구현 | line/GCS/Pixhawk/battery/RC takeover timeout |
| 8 | Command channel 구현 | GCS START/ABORT/EMERGENCY LAND/backend 선택 |
| 9 | File logging 구현 | 비행 후 재현과 보고서 자료 |
| 10 | SITL -> props-off bench -> low-altitude grid subset 테스트 | 단계별 통합 검증 |

---

## 15. 향후 확장 가능성

| 항목 | 방향 |
|---|---|
| 카메라 추가 | camera source interface를 명확히 분리해 다중 카메라 지원 |
| Protocol 최적화 | JSON 부하가 커지면 MessagePack/FlatBuffers 검토 |
| 시뮬레이터 연동 | MAVLink adapter를 SITL endpoint로 교체 |
| ROS2 이식 | Vision/Mission/Control 경계 유지 후 node화 |
| Log replay | `tools/replay_vision.cpp`를 실제 replay/tuning 도구로 확장 |
| OTA/deploy | `scripts/`에 rsync/deploy/run helper 추가 |
