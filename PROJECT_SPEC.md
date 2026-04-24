# PROJECT_SPEC.md — uav-onboard

> 제24회 한국로봇항공기경연대회 중급부문 멀티콥터형 드론 실내 조난자 탐색 온보드 소프트웨어 기준 문서  
> **이 문서는 팀원과 코딩 에이전트가 공통으로 참조하는 Single Source of Truth입니다.**

---

## 1. 프로젝트 목적

GPS 없는 실내 환경에서, 하향 카메라 기반 라인트레이싱과 ArUco 마커 인식을 통해 격자 구역 내 조난자(마커)를 탐색하고, 탐색 결과를 역순으로 재방문한 뒤 자동 복귀·착륙하는 드론 온보드 소프트웨어를 개발한다.

---

## 2. 미션 시나리오 요약

```
[이륙] → [라인 추종, 격자 진입] → [Snake 탐색, 교차점마다 ArUco 인식]
      → [모든 마커 발견] → [마커 번호 역순 재방문] → [출발점 복귀] → [자동 착륙]
```

1. **이륙**: 이륙 포인트에서 자동 이륙 (ArduPilot AUTO 모드 또는 GUIDED 모드 명령)
2. **격자 진입**: 이륙 포인트에서 격자 구역까지 연결된 라인을 따라 이동
3. **Snake 탐색**: 격자 구역 내부를 Snake(지그재그) 패턴으로 순회하며 교차점 방문
4. **마커 인식**: 각 교차점에서 ArUco 마커 ID 인식 및 격자 좌표 기록
5. **역순 재방문**: 발견된 마커를 번호 역순으로 재방문
6. **복귀 및 착륙**: 출발점으로 복귀 후 자동 착륙

---

## 3. 이 레포의 책임 범위

| 담당 | 내용 |
|------|------|
| ✅ 담당 | Pi Camera 프레임 획득, 라인/교차점/ArUco 검출, 상태머신, 격자 좌표 관리, 경로 계획, MAVLink 명령 생성, GCS 텔레메트리 송신, GCS 명령 수신, Safety/Failsafe |
| ❌ 미담당 | 자세 안정화·모터 제어 (ArduPilot 담당), GCS UI (uav-gcs 레포 담당) |

---

## 4. 하드웨어/소프트웨어 환경

### 하드웨어

| 항목 | 내용 |
|------|------|
| Companion Computer | Raspberry Pi Zero 2 W |
| OS | Raspberry Pi OS Lite 64-bit |
| Camera | Pi Camera (기본 1개, 추후 확장 가능) |
| Flight Controller | Pixhawk 1 / 2.4.8 |
| Autopilot | ArduPilot (ArduCopter) |
| 통신 | Pi ↔ Pixhawk: UART MAVLink / Pi ↔ GCS: UDP 소켓 |

### 소프트웨어 의존성

| 항목 | 버전/비고 |
|------|-----------|
| 언어 | C++17 또는 C++20 |
| 빌드 | CMake 3.16+ |
| 비전 | OpenCV 4.x (ArUco 포함) |
| MAVLink | MAVLink C 헤더 라이브러리 (v2) |
| 직렬 통신 | POSIX serial / libserial |
| 네트워크 | POSIX UDP 소켓 |
| 단위 테스트 | Google Test (gtest) |

---

## 5. 온보드 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                     Onboard Program                         │
│                                                             │
│  ┌──────────┐    측정값     ┌──────────────┐                │
│  │  Vision  │ ──────────► │   Mission    │                │
│  │  Module  │              │  StateMachine│                │
│  └──────────┘              └──────┬───────┘                │
│       ▲                           │ 고수준 명령             │
│  Camera                    ┌──────▼───────┐                │
│  Frame                     │   Control    │                │
│                            │   Module     │                │
│                            └──────┬───────┘                │
│                                   │ MAVLink 명령            │
│                      ┌────────────▼───────────┐            │
│                      │   Autopilot / MAVLink  │            │
│                      │      Module            │            │
│                      └────────────┬───────────┘            │
│                                   │ UART                   │
└───────────────────────────────────┼─────────────────────────┘
                                    │
                            ┌───────▼───────┐
                            │   Pixhawk /   │
                            │   ArduPilot   │
                            └───────────────┘

  ┌──────────────────────────────────────┐
  │  공통 지원 모듈                       │
  │  Safety │ Telemetry │ Logging        │
  └──────────────────────────────────────┘
```

---

## 6. 주요 모듈 설명

### 6.1 Vision
- **책임**: 카메라 프레임을 받아 수치 측정값만 생성. 판단하지 않음.
- **출력**: `VisionOutput` 구조체
  - `line_offset` (픽셀 또는 정규화 값): 라인 중심과 화면 중심의 수평 오프셋
  - `line_angle` (도): 라인의 기울기
  - `intersection_score` (0.0~1.0): 교차점 신뢰도
  - `marker_id` (int, -1이면 없음): 검출된 ArUco 마커 ID
  - `marker_offset` (x, y): 마커 중심의 화면 내 위치

### 6.2 Mission
- **책임**: 상태머신 운용, 격자 좌표 갱신, 마커 저장, 경로 계획
- 격자 탐색 전략: Snake(지그재그) 순회
- 복귀 경로 계획: BFS 또는 방문 기록 역추적
- `MissionState` enum으로 현재 상태 관리

### 6.3 Control
- **책임**: Mission의 판단을 ArduPilot에 보낼 고수준 명령으로 변환
- velocity setpoint, yaw rate, hover 명령 등 생성
- ArduPilot GUIDED 모드 기반 SET_POSITION_TARGET_LOCAL_NED 활용

### 6.4 Autopilot / MAVLink
- **책임**: Pixhawk와의 MAVLink 직렬 통신만 담당
- heartbeat 송수신, 명령 전송, 상태 수신 (attitude, battery, mode 등)
- UART 포트 추상화

### 6.5 Telemetry
- **책임**: GCS(uav-gcs)로 상태 정보를 UDP로 송신
- 주기적 상태 패킷: 미션 상태, 격자 맵, 마커 맵, 현재 좌표, 배터리
- 비동기 이벤트 패킷: 마커 발견, 상태 전환, 오류 발생
- GCS → 온보드 명령 수신: 비상착륙, 미션 중단, 미션 시작 등

### 6.6 Safety
- **책임**: 각종 비상 조건 감지 및 안전 조치 트리거
- 처리 대상: line lost timeout, GCS 통신 두절, 배터리 저전압, GCS emergency land 명령

### 6.7 Logging
- **책임**: 파일 기반 로그 기록 (stdout 없음, headless 운용)
- 로그 레벨: DEBUG / INFO / WARN / ERROR
- 로그 파일 위치: `logs/` 디렉토리, 실행 시각 기반 파일명

---

## 7. 데이터 흐름

```
Camera
  │
  ▼
Vision::process(frame)
  │
  └─► VisionOutput { line_offset, line_angle, intersection_score, marker_id, ... }
              │
              ▼
        Mission::update(vision_output)
              │
              ├─► GridMap 갱신
              ├─► MarkerMap 갱신
              ├─► MissionState 전환 결정
              └─► MissionCommand { type, target_velocity, target_yaw, ... }
                          │
                          ▼
                  Control::execute(command)
                          │
                          └─► MAVLink 메시지 생성
                                      │
                                      ▼
                              Autopilot::send(msg)
                                      │
                                      ▼ UART
                                  Pixhawk
```

---

## 8. 상태머신 개요

```
IDLE
  │ (GCS START 명령)
  ▼
TAKEOFF
  │ (목표 고도 도달)
  ▼
LINE_FOLLOW_TO_GRID      ← 이륙 포인트 → 격자 진입 라인 추종
  │ (격자 진입 감지)
  ▼
GRID_EXPLORE             ← Snake 탐색, 교차점마다 ArUco 인식
  │ (모든 교차점 방문 or 탐색 완료 조건)
  ▼
REVISIT_MARKERS          ← 마커 번호 역순으로 재방문
  │ (모든 마커 재방문 완료)
  ▼
RETURN_HOME              ← BFS 경로로 출발점 복귀
  │ (출발점 도달)
  ▼
LAND
  │ (착륙 완료)
  ▼
MISSION_COMPLETE

─────────────────────────
어느 상태에서나:
  Safety 이벤트 → EMERGENCY_LAND
```

### 상태 전환 조건 요약

| 현재 상태 | 전환 조건 | 다음 상태 |
|-----------|-----------|-----------|
| IDLE | GCS START 명령 | TAKEOFF |
| TAKEOFF | 목표 고도 도달 (고도계 or 타이머) | LINE_FOLLOW_TO_GRID |
| LINE_FOLLOW_TO_GRID | 격자 진입 감지 (교차점 score 임계값 초과) | GRID_EXPLORE |
| GRID_EXPLORE | Snake 탐색 완료 | REVISIT_MARKERS |
| REVISIT_MARKERS | 전체 재방문 완료 | RETURN_HOME |
| RETURN_HOME | 출발점 도달 | LAND |
| LAND | 착륙 완료 감지 | MISSION_COMPLETE |
| any | Safety 이벤트 | EMERGENCY_LAND |

---

## 9. 주요 데이터 구조 예시

```cpp
// Vision 출력 (Vision → Mission)
struct VisionOutput {
    float line_offset;          // 픽셀 단위, 양수=오른쪽
    float line_angle;           // 도 단위
    float intersection_score;   // 0.0 ~ 1.0
    int   marker_id;            // -1: 없음
    float marker_offset_x;
    float marker_offset_y;
    uint64_t timestamp_us;
};

// Mission 명령 (Mission → Control)
enum class CommandType { HOVER, VELOCITY, YAW, LAND, TAKEOFF };
struct MissionCommand {
    CommandType type;
    float vx, vy, vz;           // m/s, NED
    float yaw_rate;             // rad/s
};

// 격자 좌표
struct GridCoord { int row, col; };

// 마커 레코드
struct MarkerRecord {
    int        marker_id;
    GridCoord  grid_pos;
    uint64_t   found_time_us;
};

// 미션 상태
enum class MissionState {
    IDLE, TAKEOFF, LINE_FOLLOW_TO_GRID,
    GRID_EXPLORE, REVISIT_MARKERS,
    RETURN_HOME, LAND, MISSION_COMPLETE,
    EMERGENCY_LAND
};
```

---

## 10. GCS와의 통신 요구사항

### 프로토콜
- **전송 계층**: UDP (Wi-Fi)
- **직렬화**: 경량 바이너리 직렬화 또는 JSON (Pi Zero 성능 고려 시 바이너리 우선)
- **포트**: 설정 파일(`config/network.yaml` 또는 `.toml`)로 관리

### 온보드 → GCS (텔레메트리 송신)

| 패킷 | 주기 | 내용 |
|------|------|------|
| STATUS | 1Hz | 미션 상태, 배터리, 고도 |
| GRID_MAP | 이벤트 | 방문한 교차점 목록 |
| MARKER_MAP | 이벤트 | 발견된 마커 ID·좌표 |
| LOG | 이벤트 | 로그 메시지 |

### GCS → 온보드 (명령 수신)

| 명령 | 처리 |
|------|------|
| CMD_START | 미션 시작 (IDLE → TAKEOFF) |
| CMD_ABORT | 미션 중단 후 RETURN_HOME |
| CMD_EMERGENCY_LAND | 즉시 EMERGENCY_LAND |

---

## 11. Pixhawk/ArduPilot 연동 요구사항

- **모드**: GUIDED 모드에서 고수준 명령 사용
- **주요 MAVLink 메시지**:
  - `SET_MODE`: GUIDED/LAND/RTL 모드 전환
  - `COMMAND_LONG (MAV_CMD_NAV_TAKEOFF)`: 자동 이륙
  - `SET_POSITION_TARGET_LOCAL_NED`: velocity setpoint 전송
  - `COMMAND_LONG (MAV_CMD_NAV_LAND)`: 착륙 명령
  - `HEARTBEAT`: 주기적 수신으로 FC 연결 상태 확인
  - `SYS_STATUS`: 배터리, 오류 플래그 수신
  - `ATTITUDE`: 현재 자세 수신
- **UART 설정**: `/dev/serial0`, baud 115200 (설정 파일로 변경 가능)
- **Heartbeat 타임아웃**: 2초 이상 미수신 시 Safety 이벤트 발생

---

## 12. Safety/Failsafe 요구사항

| 조건 | 대응 |
|------|------|
| 라인 추종 실패 (line lost) | N초 이상 지속 시 HOVER → GCS 경보 |
| GCS 통신 두절 | M초 이상 지속 시 RETURN_HOME |
| Pixhawk Heartbeat 두절 | 즉시 EMERGENCY_LAND |
| GCS CMD_EMERGENCY_LAND | 즉시 EMERGENCY_LAND |
| 배터리 저전압 | 임계값 이하 시 RETURN_HOME 또는 EMERGENCY_LAND |
| 미션 타임아웃 | 전체 미션 시간 초과 시 RETURN_HOME |

> N, M 및 임계값은 `config/safety.toml` 설정 파일로 관리한다.

---

## 13. 권장 디렉토리 구조

아래 구조는 **모듈 독립성**, **테스트 용이성**, **Pi 배포 편의성**, **팀 분업**을 고려하여 설계되었다.

```
uav-onboard/
│
├── CMakeLists.txt              # 루트 빌드 파일
├── PROJECT_SPEC.md             # 이 문서
├── README.md                   # 빌드·실행 빠른 안내
│
├── config/                     # 런타임 설정 파일 (코드 재빌드 없이 변경)
│   ├── mission.toml            # 격자 크기, Snake 방향, 탐색 파라미터
│   ├── vision.toml             # 카메라 파라미터, 임계값
│   ├── safety.toml             # Failsafe 타임아웃·임계값
│   └── network.toml            # GCS IP, 포트 설정
│
├── src/                        # 소스 코드 (모듈별 디렉토리)
│   ├── main.cpp                # 진입점: 초기화, 메인 루프
│   │
│   ├── vision/                 # 카메라 프레임 처리, 측정값 생성
│   │   ├── VisionModule.hpp
│   │   ├── VisionModule.cpp
│   │   ├── LineDetector.hpp    # 라인/교차점 검출
│   │   ├── LineDetector.cpp
│   │   ├── ArucoDetector.hpp   # ArUco 마커 검출
│   │   └── ArucoDetector.cpp
│   │
│   ├── mission/                # 상태머신, 좌표 관리, 경로 계획
│   │   ├── MissionManager.hpp
│   │   ├── MissionManager.cpp
│   │   ├── StateMachine.hpp
│   │   ├── StateMachine.cpp
│   │   ├── GridMap.hpp         # 격자 좌표·방문 이력 관리
│   │   ├── GridMap.cpp
│   │   ├── MarkerMap.hpp       # 마커 ID·좌표 저장
│   │   ├── MarkerMap.cpp
│   │   ├── PathPlanner.hpp     # BFS 복귀 경로 계획
│   │   └── PathPlanner.cpp
│   │
│   ├── control/                # MissionCommand → MAVLink 명령 변환
│   │   ├── ControlModule.hpp
│   │   └── ControlModule.cpp
│   │
│   ├── autopilot/              # MAVLink 통신 추상화
│   │   ├── AutopilotInterface.hpp
│   │   ├── AutopilotInterface.cpp
│   │   ├── MavlinkSerial.hpp   # UART MAVLink 송수신
│   │   └── MavlinkSerial.cpp
│   │
│   ├── telemetry/              # GCS UDP 통신
│   │   ├── TelemetryServer.hpp
│   │   ├── TelemetryServer.cpp
│   │   ├── TelemetrySender.hpp
│   │   ├── TelemetrySender.cpp
│   │   ├── GcsCommandReceiver.hpp
│   │   └── GcsCommandReceiver.cpp
│   │
│   ├── safety/                 # Failsafe 감시 및 처리
│   │   ├── SafetyMonitor.hpp
│   │   └── SafetyMonitor.cpp
│   │
│   ├── logging/                # 파일 기반 로거
│   │   ├── Logger.hpp
│   │   └── Logger.cpp
│   │
│   └── common/                 # 공통 데이터 구조·유틸리티
│       ├── Types.hpp           # VisionOutput, MissionCommand, GridCoord 등
│       ├── Config.hpp          # 설정 파일 파서
│       └── Config.cpp
│
├── tests/                      # 단위 테스트 (Google Test)
│   ├── CMakeLists.txt
│   ├── vision/
│   │   ├── test_line_detector.cpp
│   │   └── test_aruco_detector.cpp
│   ├── mission/
│   │   ├── test_state_machine.cpp
│   │   ├── test_grid_map.cpp
│   │   ├── test_marker_map.cpp
│   │   └── test_path_planner.cpp
│   └── mock/
│       ├── MockVisionOutput.hpp   # 테스트용 Vision 출력 주입
│       └── MockAutopilot.hpp      # 테스트용 FC 통신 Mock
│
├── tools/                      # 개발·디버그 보조 도구 (Pi에 배포 불필요)
│   ├── camera_preview.cpp      # 카메라 영상 확인 (노트북 연결 시)
│   ├── line_detector_tuner.cpp # 라인 검출 파라미터 튜닝 도구
│   └── replay_vision.cpp       # 저장된 프레임으로 Vision 재현
│
├── scripts/                    # 배포·운용 스크립트
│   ├── deploy.sh               # Pi로 빌드 결과물 rsync
│   ├── cross_compile.sh        # 크로스 컴파일 환경 설정
│   └── run_mission.sh          # Pi 위에서 미션 실행 (config 경로 전달)
│
├── docs/                       # 추가 설계 문서
│   ├── aruco_layout.md         # 경기장 ArUco 마커 배치 계획
│   ├── grid_coordinate.md      # 격자 좌표계 정의
│   └── mavlink_commands.md     # 사용하는 MAVLink 메시지 목록
│
└── logs/                       # 런타임 로그 (gitignore)
    └── .gitkeep
```

### 구조 설계 근거

| 결정 | 이유 |
|------|------|
| `config/` 분리 | 코드 재빌드 없이 현장에서 파라미터 수정 가능 |
| `tools/` 분리 | 튜닝·디버그 도구가 미션 코드에 섞이지 않음 |
| `tests/mock/` | Vision·FC를 Mock으로 교체해 Pi 없이 미션 로직 검증 |
| `common/Types.hpp` | 모듈 간 데이터 구조를 한 곳에서 관리, 의존성 최소화 |
| `scripts/` | Pi 배포 자동화, 팀원 누구나 동일한 절차로 배포 |
| 모듈당 hpp/cpp 분리 | 헤더만 보면 인터페이스 파악 가능, 팀 분업 용이 |

---

## 14. 빌드/실행 방향

### 빌드 (노트북에서 크로스 컴파일 또는 Pi에서 네이티브 빌드)

```bash
# 빌드 디렉토리 생성
mkdir build && cd build

# 구성 (Pi 네이티브 빌드 예시)
cmake .. -DCMAKE_BUILD_TYPE=Release

# 컴파일
make -j$(nproc)
```

### 실행

```bash
# Pi 위에서 실행
./uav_onboard --config ../config/
```

### 단위 테스트

```bash
cd build
cmake .. -DBUILD_TESTS=ON
make -j$(nproc)
ctest --output-on-failure
```

### Pi 배포

```bash
# 노트북에서 실행
./scripts/deploy.sh <PI_IP>
```

---

## 15. 개발 우선순위

아래 순서로 구현하여, 각 단계에서 독립적으로 검증 가능하도록 한다.

| 단계 | 작업 | 검증 방법 |
|------|------|-----------|
| 1 | `common/Types.hpp` — 공통 데이터 구조 정의 | 코드 리뷰 |
| 2 | `logging/Logger` — 파일 로거 | 단위 테스트 |
| 3 | `vision/LineDetector` — 라인/교차점 검출 | `tools/line_detector_tuner` |
| 4 | `vision/ArucoDetector` — ArUco 검출 | 실제 마커로 벤치 테스트 |
| 5 | `mission/GridMap`, `MarkerMap` — 좌표/마커 관리 | 단위 테스트 |
| 6 | `mission/StateMachine` — 상태머신 기본 전환 | Mock 입력 단위 테스트 |
| 7 | `autopilot/MavlinkSerial` — UART MAVLink 송수신 | Pixhawk 연결 벤치 테스트 |
| 8 | `control/ControlModule` — 명령 변환 | Mock FC로 단위 테스트 |
| 9 | `telemetry/` — GCS UDP 송수신 | GCS 연결 통합 테스트 |
| 10 | `safety/SafetyMonitor` — Failsafe 전체 | 시나리오 테스트 |
| 11 | `main.cpp` 통합 | 실내 벤치 비행 테스트 |

---

## 16. 향후 확장 가능성

| 항목 | 방향 |
|------|------|
| 카메라 추가 | `vision/` 내 카메라 추상화 인터페이스 도입으로 다중 카메라 지원 |
| 고급 경로 계획 | `PathPlanner`를 인터페이스로 추상화, A* 등 교체 가능 |
| 시뮬레이터 연동 | `AutopilotInterface`를 Mock으로 교체해 SITL 연결 가능 |
| ROS2 이식 | 모듈 경계가 명확하므로 ROS2 노드로 개별 이식 용이 |
| 상향 카메라 | Vision 모듈에 카메라 소스 추상화 후 추가 |
| 로그 분석 도구 | `tools/`에 로그 파서·시각화 도구 추가 |
| OTA 업데이트 | `scripts/deploy.sh` 확장으로 무선 바이너리 업데이트 가능 |

---

*최종 수정: 2026-04-24 | 작성: 윤민석*
