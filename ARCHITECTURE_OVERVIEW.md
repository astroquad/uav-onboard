# astroquad-onboard 아키텍처 개요

이 문서는 `astroquad-onboard` 실행 파일이 어떻게 구성되어 있고, 한 프레임이
입력에서 모터 명령까지 어떤 경로로 흘러가는지를 처음 읽는 사람이 한눈에
파악할 수 있도록 정리한 코드 리딩 가이드다.

세부 알고리즘이나 파라미터 튜닝은 다루지 않는다. 그건 [PROJECT_SPEC.md](PROJECT_SPEC.md)
와 각 모듈의 헤더/구현 파일을 참고하면 된다.

현재 미션 범위는 **이륙 → grid snake 탐색 → 발견 마커 역/오름차순 재방문 →
출발 vertiport 복귀 → 자동 착륙**까지 한 사이클 전체를 포함한다.

---

## 1. 전체 온보드 아키텍처

```text
                              ┌──────────────────────────────────────────┐
                              │              astroquad-onboard            │
                              │     (Raspberry Pi 4에서 도는 프로세스)     │
                              └──────────────────────────────────────────┘

  ┌───────────┐   JPEG 프레임      ┌───────────────────┐    VisionResult
  │  Camera   │ ─────────────────▶│  VisionProcessor   │ ───────────────┐
  │ (rpicam/  │   (FrameSource)    │ (Aruco/Line/교차점)│                │
  │  gazebo)  │                    └───────────────────┘                │
  └───────────┘                                                          ▼
                                                          ┌──────────────────────────┐
                                                          │ IntersectionDecisionEngine│  교차점 판단
                                                          └──────────────────────────┘
                                                                       │
                                                                       ▼
                                                          ┌──────────────────────────┐
                                                          │   GridCoordinateTracker   │  격자 좌표 추정
                                                          └──────────────────────────┘
                                                                       │
                                                                       ▼
       ┌──────────────────────────────────────────────────────────────────────────┐
       │                              GridMission                                  │
       │                                                                           │
       │  미션 상태머신:                                                            │
       │   IDLE → ARM_TAKEOFF → MARKER_LOCK_YAW                                    │
       │        → ENTRY_FORWARD → ENTRY_CENTER_ORIGIN → ENTRY_ORIGIN_MARKER_HOVER  │
       │        → SNAKE_LAUNCH_ALIGN → SNAKE_FORWARD/RECORD/TURN/ADVANCE → ...     │
       │        → SNAKE_COMPLETE                                                   │
       │        → REVISIT_INIT → REVISIT_FORWARD/STOP/TURN/MARKER_HOVER → REVISIT_COMPLETE │
       │        → RETURN_HOME_INIT → RETURN_HOME_FORWARD/STOP/TURN/ALIGN/FACE_SOUTH│
       │        → RETURN_VERTIPORT_FORWARD → RETURN_VERTIPORT_MARKER_HOVER         │
       │        → LAND → MISSION_COMPLETE → DONE                                   │
       │   (any active state → EMERGENCY_LAND)                                     │
       │                                                                           │
       │  내부에서 호출하는 모듈:                                                   │
       │   MarkerRegistry / SnakePlanner / AltitudePolicy / MarkerRevisitPlanner   │
       │                                                                           │
       │  출력: GridControlIntent (Forward/Turn/Hover/Land ...)                    │
       └──────────────────────────────────────────────────────────────────────────┘
                                       │  intent + target altitude/yaw
                                       ▼
                       ┌─────────────────────────────────┐
                       │       GridControlMapper          │  intent → 속도 setpoint
                       │   (+ GuidedVelocityController)   │
                       └─────────────────────────────────┘
                                       │  body-frame vx/vy/vz/yaw_rate
                                       ▼
                       ┌─────────────────────────────────┐         ┌─────────────────┐
                       │   AutopilotMavlinkAdapter        │◀───────│  Flight Ctrl.    │
                       │   (Udp/Serial Transport)         │  HB/   │  (ArduPilot)     │
                       │                                  │  state │                  │
                       └─────────────────────────────────┘────────▶└─────────────────┘
                                       │
                                       ▼
                              MAVLink SET_POSITION_TARGET_LOCAL_NED


  ┌──────────────────────────────────┐
  │      GcsTelemetryPublisher        │  매 프레임 텔레메트리/디버그 영상 송신
  │  (UDP → uav-gcs)                  │   (mission/revisit/return 상태 포함)
  └──────────────────────────────────┘
                ▲
                │ (mission/vision/autopilot 스냅샷)
                │
        AstroquadOnboardApp::run() 메인 루프에서 매 프레임 호출
```

핵심 원칙:

- **Vision은 MAVLink를 모른다.** `VisionProcessor`는 이미지에서 라인/마커/교차점만 뽑아낸다.
- **Mission은 MAVLink 패킷을 만들지 않는다.** `GridMission`은 추상적인 `GridControlIntent`만 출력한다.
- **Mapper가 intent를 속도 setpoint로 번역**하고, `AutopilotMavlinkAdapter`가 실제 MAVLink 송신을 담당한다.
- **탐색-재방문-복귀는 한 상태머신 안에서 단계적으로 이어진다.** Snake가 끝나면
  `MarkerRevisitPlanner`가 마커 방문 순서/경로를 만들고, 그게 끝나면
  return-home/return-vertiport 단계가 같은 격자 좌표 위에서 돌아간다.
- **GCS 텔레메트리는 부가 채널**이다. 패킷이 빠져도 미션 판단에는 영향이 없다.

---

## 2. 메인 프로그램 실행 흐름

엔트리 포인트부터 메인 루프까지, 호출되는 주요 함수와 객체를 흐름순으로 정리한다.

### --- src/main.cpp ---

```cpp
int main(int argc, char** argv) {
    onboard::app::AstroquadOnboardApp app;   // 앱 객체 선언
    return app.run(argc, argv);              // 모든 로직은 run() 안에서
}
```

`main`은 그냥 얇은 진입점이다. 실제 구성은 전부 `AstroquadOnboardApp::run`에 있다.

### --- src/app/AstroquadOnboardApp.cpp ---

#### `run(argc, argv)` — 전체 실행의 조립자(composition root)

크게 보면 **준비 → 이륙 → 메인 루프 → 종료** 4단계다.

##### (1) 준비 단계

```text
parseOptions(argc, argv)        // CLI 옵션 파싱 (--target, --marker-count, --revisit-order 등)
loadConfigs(opt, cfg)           // config/*.toml 파일들 읽어서 통합 설정 생성
                                 //  - autopilot.toml / safety.toml / mission.toml
                                 //  - vision.toml / runtime.sitl.grid.toml 등

createFrameSource(cfg.vision_source)        // 카메라 소스 생성 (fake/gazebo/rpicam)
frame_source->open(fs_opts)                  // 카메라 오픈

VisionProcessor processor(...)               // 라인/마커/교차점 검출기

GcsTelemetryPublisher publisher;             // GCS에 텔레메트리/영상 송신할 객체
publisher.open(pub_opts)

// MAVLink 전송 채널 (UDP 또는 시리얼)
transport = SerialMavlinkTransport(...) 또는 UdpMavlinkTransport(...)
AutopilotMavlinkAdapter autopilot(transport, mavlink_ids)

autopilot.waitHeartbeat(30s)                 // FC와 연결 확인
autopilot.requestDefaultStreams()            // FC가 보낼 메시지 스트림 요청
```

##### (2) 미션 스택 구축 및 이륙 단계

```text
GuidedVelocityController line_controller     // 라인 추종용 속도 제어기
GridCoordinateTracker tracker                // 격자 좌표 추적
MarkerRegistry registry                      // 발견된 마커 기록부 (재방문/복귀에서 재사용)
AltitudePolicy altitude                      // 고도 정책
SnakePlanner snake                           // 스네이크 경계 회전 플래너
IntersectionDecisionEngine decision_engine   // 교차점 판단 엔진
GridMission mission(...)                     // 상태머신 (위 모듈들과 MarkerRevisitPlanner를 내부에서 사용)
GridControlMapper mapper(...)                // intent → setpoint 변환기

// 실기체 이륙 전 안전 게이트
waitRcReady(autopilot, ...)                  // RC 신호 들어오는지 확인
waitLocalHoldEstimateReady(...)              // (실기) Optical Flow + EKF 준비 확인

autopilot.setGuidedMode(10s)                 // FC를 GUIDED 모드로
autopilot.arm(15s)                           // 모터 ARM
autopilot.takeoff(vertiport_altitude_m)      // 이륙 명령

mission.start(now_s())                       // 미션 상태머신 시작
```

##### (3) 메인 루프 (한 프레임당 반복)

```text
while (!종료요청 && mission.state() != Done) {

    autopilot.poll(1)                        // FC에서 들어온 MAVLink 메시지 처리

    frame_source->read(frame)                // 카메라 프레임 한 장 가져오기
    vp_out = processor.process(frame)        // 라인/마커/교차점 검출 → VisionResult

    idec = decision_engine.update(...)       // 교차점 판단 (sliding window)
    if (tracker_enabled)                     // SNAKE/REVISIT/RETURN 진행 중일 때만
        node_event = tracker.update(idec, ...)  // 격자 좌표 peek (실제 진행은 아직 안함)

    // 미션 입력 패키지 구성 (autopilot 상태 + 비전 결과 + 교차점 결정)
    GridMissionInput min = { ... }
    mout = mission.update(min)               // ★ 미션 상태머신 한 틱 실행
                                              //   - snake / revisit / return / land 어느 단계든 여기서 처리
                                              //   - 마커 발견은 MarkerRegistry에 기록
                                              //   - SNAKE_COMPLETE 시 MarkerRevisitPlanner로 경로 생성
                                              //   - REVISIT_COMPLETE 시 return-home 좌표/경로 생성

    if (mout.commit_tracker_advance)
        tracker.commitAdvance(...)           // 미션이 승인할 때만 격자 좌표를 진짜로 한 칸 전진

    // intent를 실제 속도 setpoint로 변환
    sp = mapper.compute(cmin)

    // FC에게 명령 보내기
    if (mout.request_land_mode)
        autopilot.setLandMode(5s)            // 착륙 (LAND 단계 또는 EMERGENCY_LAND)
    else
        autopilot.sendBodyVelocity(cmd)      // body-frame 속도 setpoint 송신

    publisher.publish(pin)                   // GCS에 텔레메트리/영상 송신
                                              //  - mission.state, control_intent
                                              //  - revisit_active / return_active / return_phase
                                              //  - markers_found[] (재방문 여부 포함)
                                              //  - landing_success / mission_complete
    logState(...)                            // 콘솔 로그 한 줄

    sleep_until(loop_start + period)         // setpoint 주기 유지 (기본 20Hz)
}
```

##### (4) 종료 단계

```text
autopilot.setLandMode(3s)                    // 안전 착륙 요청
autopilot.poll() 반복으로 disarm 대기
publisher.close()
frame_source->close()
```

---

### 미션 상태머신 단계 요약

`GridMission`은 한 상태머신 안에서 4개의 큰 단계를 순차적으로 진행한다. 각 단계가
끝나면 다음 단계로 자동 전이된다.

| 단계 | 주요 상태 | 하는 일 |
|---|---|---|
| ① 진입 | `ARM_TAKEOFF → MARKER_LOCK_YAW → ENTRY_FORWARD → ENTRY_CENTER_ORIGIN → ENTRY_ORIGIN_MARKER_HOVER` | 이륙, vertiport 마커로 yaw 잠금, blind forward로 grid 진입, 첫 교차점에서 `(0,0)` 확정 |
| ② 탐색 (Snake) | `SNAKE_LAUNCH_ALIGN → SNAKE_FORWARD → SNAKE_RECORD_NODE → SNAKE_STOP_AT_CENTER → SNAKE_TURN_90 → SNAKE_ADVANCE_ONE_CELL → SNAKE_TURN_90_AGAIN → TURN_NODE_MARKER_HOVER → SNAKE_COMPLETE` | 격자 전체를 snake 패턴으로 훑으며 마커 발견 시 `MarkerRegistry`에 기록 |
| ③ 재방문 (Revisit) | `REVISIT_INIT → REVISIT_FORWARD → REVISIT_STOP_AT_TURN → REVISIT_TURN_90 → REVISIT_MARKER_HOVER → REVISIT_COMPLETE` | `MarkerRevisitPlanner`가 발견 마커들을 asc/desc 순으로 정렬하고, 각 마커까지 격자 경로(segments)를 만들어 순차 방문 + hover |
| ④ 복귀 | `RETURN_HOME_INIT → RETURN_HOME_FORWARD/STOP/TURN → RETURN_HOME_ALIGN_ORIGIN → RETURN_HOME_FACE_SOUTH → RETURN_VERTIPORT_FORWARD → RETURN_VERTIPORT_MARKER_HOVER → LAND → MISSION_COMPLETE → DONE` | `(0,0)`로 돌아와 grid south를 향한 뒤 vertiport 마커로 blind forward, 마커 hover, 자동 착륙 |

---

### 주요 보조 모듈들의 역할 (메인 루프에서 참조하는 순서대로)

#### --- src/vision/ ---

| 객체 | 역할 |
|---|---|
| `FrameSource` (`FakeFrameSource` / `GazeboCameraSource` / `RpicamFrameSource`) | 카메라/시뮬레이션 영상 한 장씩 공급 |
| `VisionProcessor` | 한 프레임을 받아 라인/마커/교차점을 모두 검출해 `VisionResult` 생성 |
| `LineMaskBuilder` → `LineDetector` → `LineStabilizer` | 바닥 라인 검출 + 안정화 |
| `ArucoDetector` → `MarkerStabilizer` | ArUco 마커 검출 + 안정화 |
| `IntersectionDetector` → `IntersectionStabilizer` | 교차점 모양(L/T/+) 분류 + 안정화 |

#### --- src/mission/ ---

| 객체 | 역할 |
|---|---|
| `IntersectionDecisionEngine` | 최근 프레임들의 교차점 증거를 모아 `node_record` / `turn_ready` 같은 결정 출력 |
| `GridCoordinateTracker` | 현재 위치한 격자 좌표/방향을 추적 (peek-only, 미션이 승인할 때만 commit) |
| `MarkerRegistry` | 발견한 ArUco 마커 ID/격자 좌표/orientation/첫 발견 시각/재방문 여부 기록 |
| `AltitudePolicy` | 현재 미션 상태에 맞는 목표 고도 산출 |
| `SnakePlanner` | 격자 경계에서 좌/우 교대 회전을 latch & alternate |
| `MarkerRevisitPlanner` | snake 종료 시점 좌표/heading에서 출발해 발견 마커들을 asc/desc 순서로 방문하는 `RevisitLeg`(heading+cells 세그먼트 리스트)들을 생성 |
| **`GridMission`** | **위 모든 모듈을 받아 진입/탐색/재방문/복귀 전체 상태머신을 실행. 출력은 추상적 `GridControlIntent`** |

#### --- src/control/ ---

| 객체 | 역할 |
|---|---|
| `GuidedVelocityController` | 라인 오차/마커 중심 오차 → 측면 속도/yaw rate 계산 (PID 비슷한 역할) |
| `GridControlMapper` | `GridControlIntent`(Forward / YawAlign / YawTurn / MarkerHover / IntersectionCenter / Land 등)를 body-frame 속도 setpoint로 번역 |

#### --- src/autopilot/ ---

| 객체 | 역할 |
|---|---|
| `MavlinkTransport` (인터페이스) → `UdpMavlinkTransport` / `SerialMavlinkTransport` | MAVLink 송수신 채널 (SITL=UDP / 실기=Serial) |
| `AutopilotMavlinkAdapter` | heartbeat 대기, GUIDED 모드 설정, ARM/takeoff/land, body velocity 송신, FC 상태(`AutopilotState`) 보관 |

#### --- src/app/ ---

| 객체 | 역할 |
|---|---|
| `AstroquadOnboardApp` | 모든 모듈을 조립하고 메인 루프를 실행 (composition root) |
| `GcsTelemetryPublisher` | 매 프레임 텔레메트리 JSON과 (옵션) MJPEG 디버그 영상을 GCS로 UDP 송신. revisit/return phase, 발견 마커 목록, 미션 완료 플래그 등도 함께 보냄 |

---

## 정리: 한 프레임이 흘러가는 길

```text
[Camera]
  → FrameSource.read()
  → VisionProcessor.process()              ─── 이미지에서 라인/마커/교차점
  → IntersectionDecisionEngine.update()    ─── 교차점 판단 누적
  → GridCoordinateTracker.update()         ─── (peek) 격자 좌표 후보
  → GridMission.update()                   ─── 상태머신: 진입/탐색/재방문/복귀 어디든 한 틱
                                                · 마커 발견 → MarkerRegistry
                                                · SNAKE_COMPLETE → MarkerRevisitPlanner로 경로 생성
                                                · REVISIT_COMPLETE → return-home 경로로 전이
  → (tracker.commitAdvance() if approved)
  → GridControlMapper.compute()            ─── intent → 속도 setpoint
  → AutopilotMavlinkAdapter.sendBodyVelocity()
  → [Flight Controller (ArduPilot)]
  → [Motors]

  병행: GcsTelemetryPublisher.publish() → [uav-gcs]
```

이 순서가 머릿속에 들어오면, 각 모듈 헤더 파일을 펼쳐 세부를 읽기 시작해도
어디쯤에서 호출되는 코드인지 잃지 않을 수 있다.
