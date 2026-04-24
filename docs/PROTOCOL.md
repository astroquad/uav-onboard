# PROTOCOL.md — UAV 온보드 ↔ GCS 통신 프로토콜

> **버전**: v1.0 (JSON 기반 초기 버전)  
> **이 문서는 `uav-onboard`와 `uav-gcs` 양쪽에 동일하게 위치하며, 두 팀이 공통으로 참조하는 Single Source of Truth입니다.**  
> `uav-onboard/docs/PROTOCOL.md` ≡ `uav-gcs/docs/PROTOCOL.md`

---

## 1. 개요

### 1.1 목적

Raspberry Pi 온보드 프로그램(uav-onboard)과 노트북 GCS(uav-gcs) 사이에서 주고받는 **telemetry 메시지**와 **command 메시지**의 형식을 정의한다.

### 1.2 설계 원칙

| 원칙 | 내용 |
|------|------|
| **전방 호환** | 수신 측은 알 수 없는 필드를 조용히 무시한다. 절대 크래시되지 않는다. |
| **버전 명시** | 모든 메시지에 `protocol_version` 필드를 포함한다. |
| **시간 추적** | 모든 메시지에 `timestamp_ms` 필드를 포함한다 (UTC Unix 시각, 밀리초). |
| **타입 식별** | 모든 메시지에 `type` 필드를 포함한다. |
| **확장 가능** | JSON → FlatBuffers / MessagePack / Protobuf 전환을 고려한 구조로 설계한다. |

### 1.3 전송 계층 요약

| 채널 | 방향 | 전송 방식 | 기본 포트 |
|------|------|-----------|-----------|
| Telemetry | 온보드 → GCS | UDP | 14550 |
| Command | GCS → 온보드 | TCP | 14551 |
| Video Stream | 온보드 → GCS | UDP (MJPEG) | 5600 |

> 포트 및 IP는 각 프로젝트의 `config/network.toml`에서 설정한다.

---

## 2. 공통 메시지 구조

### 2.1 모든 메시지에 반드시 포함되는 필드

```json
{
  "protocol_version": 1,
  "type": "<MESSAGE_TYPE>",
  "seq": 12345,
  "timestamp_ms": 1714000000000
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `protocol_version` | `int` | 프로토콜 버전 번호. 현재 `1`. |
| `type` | `string` | 메시지 종류 식별자 (아래 목록 참조). |
| `seq` | `uint32` | 송신 측 단조 증가 시퀀스 번호. 패킷 손실 감지에 사용. |
| `timestamp_ms` | `int64` | UTC Unix 시각 (밀리초). 송신 시각 기준. |

### 2.2 버전 호환 정책

- 수신 측은 `protocol_version`이 자신의 지원 버전보다 **높으면** 경고 로그를 남기고 파싱을 계속 시도한다.
- 수신 측은 `protocol_version`이 자신의 지원 버전보다 **낮으면** 최선으로 파싱하되, 없는 필드는 기본값 처리한다.
- 수신 측은 **알 수 없는 `type`**은 조용히 드롭한다 (에러 없음).
- 수신 측은 **알 수 없는 필드**는 조용히 무시한다 (에러 없음).

---

## 3. Telemetry 메시지 (온보드 → GCS, UDP)

### 3.1 메시지 타입 목록

| `type` 값 | 전송 주기 | 설명 |
|-----------|-----------|------|
| `TELEMETRY` | 1 Hz | 드론 전반 상태 (미션, 격자, 마커, 비전, 드론, 안전) 통합 패킷 |
| `MISSION_STATE_CHANGE` | 이벤트 | 상태머신 전환 발생 시 |
| `MARKER_FOUND` | 이벤트 | 새 마커 발견 시 |
| `SAFETY_EVENT` | 이벤트 | Safety/Failsafe 이벤트 발생 시 |
| `CMD_ACK` | 이벤트 | GCS 명령에 대한 응답 |
| `LOG` | 이벤트 | 온보드 로그 메시지 (중요 이벤트 한정) |

---

### 3.2 `TELEMETRY` — 주기적 통합 상태 패킷

1 Hz로 주기적으로 전송한다. 하위 객체별로 최신 상태를 모두 담는다.

```json
{
  "protocol_version": 1,
  "type": "TELEMETRY",
  "seq": 1001,
  "timestamp_ms": 1714000000000,

  "mission": {
    "state": "GRID_EXPLORE",
    "elapsed_ms": 42300
  },

  "grid_pose": {
    "row": 2,
    "col": 3,
    "heading_deg": 90.0
  },

  "marker_map": [
    { "id": 1, "row": 0, "col": 1, "found_timestamp_ms": 1714000010000 },
    { "id": 3, "row": 2, "col": 3, "found_timestamp_ms": 1714000038000 }
  ],

  "vision": {
    "line_offset": -12.5,
    "line_angle": 1.2,
    "intersection_score": 0.87,
    "marker_id": -1,
    "marker_offset_x": 0.0,
    "marker_offset_y": 0.0
  },

  "drone": {
    "altitude_m": 1.2,
    "battery_voltage": 11.8,
    "battery_pct": 72,
    "armed": true,
    "flight_mode": "GUIDED",
    "failsafe": false
  },

  "safety": {
    "line_lost": false,
    "gcs_link_lost": false,
    "pixhawk_link_lost": false,
    "low_battery": false
  }
}
```

#### 필드 상세

**`mission` 객체**

| 필드 | 타입 | 설명 |
|------|------|------|
| `state` | `string` | 현재 MissionState enum 이름 (아래 값 목록 참조) |
| `elapsed_ms` | `int64` | 미션 시작 후 경과 시간 (밀리초) |

`state` 허용 값: `IDLE`, `TAKEOFF`, `LINE_FOLLOW_TO_GRID`, `GRID_EXPLORE`, `REVISIT_MARKERS`, `RETURN_HOME`, `LAND`, `MISSION_COMPLETE`, `EMERGENCY_LAND`

**`grid_pose` 객체**

| 필드 | 타입 | 설명 |
|------|------|------|
| `row` | `int` | 현재 격자 행 좌표 (0-indexed) |
| `col` | `int` | 현재 격자 열 좌표 (0-indexed) |
| `heading_deg` | `float` | 현재 heading (도, 0=North, 시계방향) |

**`marker_map` 배열**

발견된 마커 목록. 발견된 마커가 없으면 빈 배열 `[]`.

| 필드 | 타입 | 설명 |
|------|------|------|
| `id` | `int` | ArUco 마커 ID |
| `row` | `int` | 발견된 격자 행 |
| `col` | `int` | 발견된 격자 열 |
| `found_timestamp_ms` | `int64` | 최초 발견 UTC 시각 (밀리초) |

**`vision` 객체**

| 필드 | 타입 | 설명 |
|------|------|------|
| `line_offset` | `float` | 라인 중심과 화면 중심의 수평 오프셋 (픽셀, 양수=오른쪽) |
| `line_angle` | `float` | 라인 기울기 (도) |
| `intersection_score` | `float` | 교차점 신뢰도 (0.0~1.0) |
| `marker_id` | `int` | 현재 프레임에서 검출된 마커 ID (-1이면 없음) |
| `marker_offset_x` | `float` | 마커 중심 수평 위치 (정규화, -1.0~1.0) |
| `marker_offset_y` | `float` | 마커 중심 수직 위치 (정규화, -1.0~1.0) |

**`drone` 객체**

| 필드 | 타입 | 설명 |
|------|------|------|
| `altitude_m` | `float` | 현재 고도 (미터, AGL 기준) |
| `battery_voltage` | `float` | 배터리 전압 (V) |
| `battery_pct` | `int` | 배터리 잔량 (%, 0~100) |
| `armed` | `bool` | Pixhawk armed 상태 |
| `flight_mode` | `string` | ArduPilot 현재 모드 (`GUIDED`, `LAND`, `RTL` 등) |
| `failsafe` | `bool` | ArduPilot failsafe 활성 여부 |

**`safety` 객체**

| 필드 | 타입 | 설명 |
|------|------|------|
| `line_lost` | `bool` | 라인 추종 실패 중 여부 |
| `gcs_link_lost` | `bool` | GCS 통신 두절 여부 |
| `pixhawk_link_lost` | `bool` | Pixhawk heartbeat 두절 여부 |
| `low_battery` | `bool` | 배터리 저전압 임계값 이하 여부 |

---

### 3.3 `MISSION_STATE_CHANGE` — 상태머신 전환 이벤트

```json
{
  "protocol_version": 1,
  "type": "MISSION_STATE_CHANGE",
  "seq": 1002,
  "timestamp_ms": 1714000005000,
  "from_state": "TAKEOFF",
  "to_state": "LINE_FOLLOW_TO_GRID",
  "reason": "target altitude reached"
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `from_state` | `string` | 이전 MissionState |
| `to_state` | `string` | 새 MissionState |
| `reason` | `string` | 전환 이유 (사람이 읽을 수 있는 설명, 선택적) |

---

### 3.4 `MARKER_FOUND` — 새 마커 발견 이벤트

```json
{
  "protocol_version": 1,
  "type": "MARKER_FOUND",
  "seq": 1015,
  "timestamp_ms": 1714000038000,
  "marker_id": 3,
  "grid_row": 2,
  "grid_col": 3
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `marker_id` | `int` | 발견된 ArUco 마커 ID |
| `grid_row` | `int` | 발견된 격자 행 |
| `grid_col` | `int` | 발견된 격자 열 |

---

### 3.5 `SAFETY_EVENT` — Safety/Failsafe 이벤트

```json
{
  "protocol_version": 1,
  "type": "SAFETY_EVENT",
  "seq": 1020,
  "timestamp_ms": 1714000060000,
  "event": "LINE_LOST",
  "detail": "line not detected for 2.0s",
  "action_taken": "HOVER"
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `event` | `string` | 이벤트 종류 (아래 값 목록 참조) |
| `detail` | `string` | 사람이 읽을 수 있는 상세 설명 |
| `action_taken` | `string` | 온보드가 취한 조치 (선택적) |

`event` 허용 값:

| 값 | 의미 |
|----|------|
| `LINE_LOST` | 라인 추종 실패 타임아웃 |
| `GCS_LINK_LOST` | GCS와의 통신 두절 |
| `PIXHAWK_LINK_LOST` | Pixhawk heartbeat 두절 |
| `LOW_BATTERY` | 배터리 저전압 임계값 이하 |
| `MISSION_TIMEOUT` | 전체 미션 시간 초과 |
| `EMERGENCY_CMD_RECEIVED` | GCS로부터 비상착륙 명령 수신 |

---

### 3.6 `CMD_ACK` — 명령 응답

GCS로부터 command를 수신한 후 온보드가 응답한다.

```json
{
  "protocol_version": 1,
  "type": "CMD_ACK",
  "seq": 1025,
  "timestamp_ms": 1714000070000,
  "ack_seq": 5,
  "result": "OK",
  "detail": ""
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `ack_seq` | `uint32` | 응답 대상 command 메시지의 `seq` |
| `result` | `string` | `"OK"` 또는 `"REJECTED"` |
| `detail` | `string` | 거부 이유 등 부가 설명 (선택적) |

---

### 3.7 `LOG` — 온보드 로그 메시지

중요한 이벤트에 한해 GCS로 로그를 전송한다. 일반 디버그 로그는 파일에만 기록한다.

```json
{
  "protocol_version": 1,
  "type": "LOG",
  "seq": 1030,
  "timestamp_ms": 1714000080000,
  "level": "WARN",
  "message": "intersection_score below threshold at (2, 3)"
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `level` | `string` | `"DEBUG"`, `"INFO"`, `"WARN"`, `"ERROR"` 중 하나 |
| `message` | `string` | 로그 메시지 내용 |

---

## 4. Command 메시지 (GCS → 온보드, TCP)

### 4.1 Command 타입 목록

| `type` 값 | 설명 |
|-----------|------|
| `start_mission` | 미션 시작 (IDLE → TAKEOFF) |
| `abort_mission` | 미션 중단 후 RETURN_HOME |
| `emergency_land` | 즉시 EMERGENCY_LAND |
| `set_marker_count` | 탐색할 마커 총 개수 설정 |
| `request_status` | 즉시 TELEMETRY 패킷 1회 요청 |

---

### 4.2 `start_mission` — 미션 시작

```json
{
  "protocol_version": 1,
  "type": "start_mission",
  "seq": 1,
  "timestamp_ms": 1714000000000
}
```

- 온보드가 `IDLE` 상태가 아닐 경우 `REJECTED`로 ACK한다.

---

### 4.3 `abort_mission` — 미션 중단

```json
{
  "protocol_version": 1,
  "type": "abort_mission",
  "seq": 2,
  "timestamp_ms": 1714000100000
}
```

- 온보드는 현재 상태를 중단하고 `RETURN_HOME`으로 전환한다.
- 이미 `RETURN_HOME`, `LAND`, `MISSION_COMPLETE`, `EMERGENCY_LAND` 상태면 무시한다.

---

### 4.4 `emergency_land` — 즉시 비상 착륙

```json
{
  "protocol_version": 1,
  "type": "emergency_land",
  "seq": 3,
  "timestamp_ms": 1714000200000
}
```

- **최우선 처리**. 어떤 상태에서도 즉시 `EMERGENCY_LAND`로 전환한다.
- GCS는 ACK 수신 전까지 최대 3회 재전송한다.

---

### 4.5 `set_marker_count` — 탐색 마커 수 설정

```json
{
  "protocol_version": 1,
  "type": "set_marker_count",
  "seq": 4,
  "timestamp_ms": 1714000000000,
  "params": {
    "marker_count": 5
  }
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `params.marker_count` | `int` | 이번 미션에서 탐색할 마커 총 개수 (1 이상) |

- `IDLE` 상태에서만 수락한다. 그 외 상태에서는 `REJECTED`로 ACK한다.

---

### 4.6 `request_status` — 즉시 상태 요청

```json
{
  "protocol_version": 1,
  "type": "request_status",
  "seq": 5,
  "timestamp_ms": 1714000000000
}
```

- 온보드는 즉시 `TELEMETRY` 패킷을 1회 전송하고 `CMD_ACK`를 보낸다.
- 주기 텔레메트리가 없는 연결 초기에 상태를 빠르게 동기화하기 위해 사용한다.

---

## 5. 시퀀스 다이어그램

### 5.1 정상 미션 시작 흐름

```
GCS                             온보드
 │                                 │
 │── set_marker_count (seq=1) ────►│
 │◄─ CMD_ACK (ack_seq=1, OK) ─────│
 │                                 │
 │── start_mission (seq=2) ───────►│
 │◄─ CMD_ACK (ack_seq=2, OK) ─────│
 │◄─ MISSION_STATE_CHANGE ────────│  (IDLE → TAKEOFF)
 │◄─ TELEMETRY (1Hz) ─────────────│
 │◄─ TELEMETRY (1Hz) ─────────────│
 │◄─ MARKER_FOUND ────────────────│  (마커 발견 시)
 │◄─ MISSION_STATE_CHANGE ────────│  (→ MISSION_COMPLETE)
 │                                 │
```

### 5.2 비상 착륙 흐름

```
GCS                             온보드
 │                                 │
 │── emergency_land (seq=10) ─────►│
 │◄─ CMD_ACK (ack_seq=10, OK) ────│
 │◄─ MISSION_STATE_CHANGE ────────│  (any → EMERGENCY_LAND)
 │◄─ SAFETY_EVENT ────────────────│  (EMERGENCY_CMD_RECEIVED)
 │                                 │
```

### 5.3 GCS 명령 거부 흐름

```
GCS                             온보드
 │                                 │
 │── start_mission (seq=20) ──────►│  (이미 GRID_EXPLORE 중)
 │◄─ CMD_ACK (ack_seq=20,          │
 │           result=REJECTED,      │
 │           detail="not IDLE") ──│
 │                                 │
```

---

## 6. 오류 처리 지침

### 6.1 온보드 (수신 측: command)

| 상황 | 처리 |
|------|------|
| JSON 파싱 실패 | 패킷 드롭, 로그 남김, ACK 없음 |
| 필수 필드 누락 (`type`, `protocol_version` 등) | 패킷 드롭, 로그 남김 |
| 알 수 없는 `type` | 패킷 드롭, 로그 없음 (정상 동작) |
| 알 수 없는 필드 | 무시 (정상 동작) |
| 버전 불일치 | 경고 로그 후 파싱 계속 |
| 상태 불일치 명령 | `REJECTED` ACK 반환 |

### 6.2 GCS (수신 측: telemetry)

| 상황 | 처리 |
|------|------|
| JSON 파싱 실패 | 패킷 드롭, 내부 오류 로그 |
| 필수 필드 누락 | 패킷 드롭, 내부 오류 로그 |
| 알 수 없는 `type` | 패킷 드롭, 조용히 무시 |
| 알 수 없는 필드 | 조용히 무시 |
| 버전 불일치 | UI에 경고 표시, 파싱 계속 |
| seq 역전 (패킷 손실) | 손실 카운터 증가, 최신 패킷 처리 |
| telemetry 2초 미수신 | UI에 "연결 끊김" 표시 |

---

## 7. 향후 바이너리 전환 고려사항

현재 v1은 JSON 기반이다. 성능이 부족할 경우 아래 순서로 전환한다.

| 단계 | 방식 | 비고 |
|------|------|------|
| v1 | JSON | 현재. 개발·디버그 편의성 최우선 |
| v2 | MessagePack | JSON 호환 바이너리, 라이브러리 교체 최소화 |
| v3 | FlatBuffers / Protobuf | 최고 성능, schema 관리 필요 |

**전환 시 호환 전략**:

- `protocol_version` 필드로 버전 구분 → 양측이 구버전 포맷도 처리 가능하도록 유지
- 포맷 변경 시 반드시 이 문서(`PROTOCOL.md`)를 먼저 업데이트하고, 양쪽 레포에 동기화한다.
- 전환 기간 동안 config로 `encoding: json | msgpack | flatbuffers` 선택 가능하도록 설계한다.

---

## 8. 포트 및 네트워크 설정

포트 및 IP는 코드에 하드코딩하지 않고, 각 프로젝트의 `config/network.toml`에서 관리한다.

**온보드 `config/network.toml` 예시**:

```toml
[gcs]
ip = "192.168.1.100"
telemetry_port = 14550    # UDP 송신
command_port   = 14551    # TCP 수신
video_port     = 5600     # UDP 영상 송신

[telemetry]
send_interval_ms = 1000   # TELEMETRY 패킷 주기
```

**GCS `config/network.toml` 예시**:

```toml
[onboard]
ip = "192.168.1.50"
telemetry_port = 14550    # UDP 수신
command_port   = 14551    # TCP 송신
video_port     = 5600     # UDP 영상 수신

[connection]
telemetry_timeout_ms = 2000   # 이 시간 이상 미수신 시 "연결 끊김" 표시
cmd_ack_timeout_ms   = 1000   # ACK 대기 시간
cmd_retry_count      = 3      # emergency_land 재전송 횟수
```

---

## 9. 변경 이력

| 버전 | 날짜 | 내용 |
|------|------|------|
| v1.0 | 2026-04-24 | 초안 작성 (JSON 기반, 6개 telemetry + 5개 command 타입) |

---

*최종 수정: 2026-04-24 | 작성: 윤민석*
