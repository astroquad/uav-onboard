# Astroquad Onboard-GCS Protocol

Version: v1.8
Last updated: 2026-05-21
This file must stay identical in `uav-onboard/docs/PROTOCOL.md` and `uav-gcs/docs/PROTOCOL.md`.

`protocol_version` in JSON remains integer `1`. The document version changes
when fields or operating rules are clarified.

## 1. Channels

| Channel | Direction | Transport | Default port | Status |
|---|---|---|---:|---|
| Telemetry | onboard -> GCS | UDP JSON | 14550 | implemented |
| Command | GCS -> onboard | TCP JSON | 14551 | planned |
| Video stream | onboard -> GCS | UDP MJPEG chunks | 5600 | implemented |
| GCS discovery | GCS -> LAN broadcast | UDP text beacon | 5601 | implemented |

Ports and destination addresses are configured in each repo's
`config/network.toml`. Telemetry and video are best-effort; packet loss must
not block onboard mission logic.

## 2. Compatibility Rules

- Every JSON packet includes `protocol_version`, `type`, `seq`, and
  `timestamp_ms`.
- Receivers ignore unknown fields and drop malformed JSON without terminating.
- Runtime source selection (`fake`, Gazebo camera, Raspberry Pi camera) does
  not change telemetry JSON or MJPEG chunk format.
- Debug video is never mission-critical. Onboard may drop old video frames and
  continue processing the latest camera frame.
- GCS overlay rendering uses only onboard metadata. GCS does not run marker,
  line, or intersection detection locally.
- `vision.grid_node` is an event-style field, but `astroquad-onboard` and its
  `grid_mission_node` compatibility target resend the latest committed node
  every frame for UDP-loss tolerance. GCS deduplicates by node id and coordinate.
- `vision.drone_position` carries fractional progress from the last committed
  grid node. The current GCS parser accepts it; the current ASCII grid map still
  renders the heading arrow at the latest committed node.

## 3. Telemetry JSON

Telemetry packets are sent from onboard to GCS over UDP. Vision/debug and
mission staging executables usually send one telemetry packet per processed
camera frame so GCS can match metadata to the optional MJPEG frame id.

Required top-level fields:

```json
{
  "protocol_version": 1,
  "type": "TELEMETRY",
  "seq": 120,
  "timestamp_ms": 1777199000000
}
```

### 3.1 Current TELEMETRY Shape

```json
{
  "protocol_version": 1,
  "type": "TELEMETRY",
  "seq": 120,
  "timestamp_ms": 1777199000000,
  "mission": {
    "state": "IDLE",
    "elapsed_ms": 0
  },
  "system": {
    "board_model": "Raspberry Pi 4 Model B Rev 1.5",
    "os_release": "Raspberry Pi OS GNU/Linux 12 (bookworm)",
    "uptime_s": 1234.0,
    "cpu_temp_c": 58.2,
    "throttled_raw": "throttled=0x0",
    "cpu_load_1m": 0.72,
    "mem_available_kb": 3120000,
    "wifi_signal_dbm": -48.0,
    "wifi_tx_bitrate_mbps": 72.2
  },
  "camera": {
    "status": "streaming",
    "sensor_model": "imx519",
    "camera_index": 0,
    "width": 960,
    "height": 720,
    "fps": 12.0,
    "configured_fps": 12.0,
    "measured_capture_fps": 11.8,
    "frame_seq": 358,
    "autofocus_mode": "manual",
    "lens_position": -1.0,
    "exposure_mode": "sport",
    "shutter_us": 0,
    "gain": 0.0,
    "awb": "auto"
  },
  "vision": {
    "line_detected": true,
    "line_offset": -12.3,
    "line_angle": 91.4,
    "line": {
      "detected": true,
      "raw_detected": true,
      "filtered": true,
      "held": false,
      "rejected_jump": false,
      "tracking_point_px": { "x": 467.7, "y": 396.0 },
      "raw_tracking_point_px": { "x": 468.1, "y": 396.0 },
      "centroid_px": { "x": 470.0, "y": 405.0 },
      "center_offset_px": -12.3,
      "raw_center_offset_px": -11.9,
      "angle_deg": 91.4,
      "raw_angle_deg": 92.0,
      "confidence": 0.76,
      "contour_px": []
    },
    "intersection_detected": false,
    "intersection_score": 0.0,
    "intersection": {
      "valid": true,
      "detected": false,
      "type": "straight",
      "raw_type": "straight",
      "stable": true,
      "held": false,
      "center_px": { "x": 480.0, "y": 390.0 },
      "raw_center_px": { "x": 480.0, "y": 390.0 },
      "score": 0.72,
      "raw_score": 0.70,
      "branch_mask": 5,
      "branch_count": 2,
      "stable_frames": 3,
      "radius_px": 50.0,
      "selected_mask_index": 0,
      "branches": [
        {
          "direction": "front",
          "present": true,
          "score": 0.9,
          "endpoint_px": { "x": 480.0, "y": 280.0 },
          "angle_deg": -90.0
        }
      ]
    },
    "intersection_decision": {
      "state": "cruise",
      "action": "continue",
      "accepted_type": "straight",
      "best_observed_type": "straight",
      "event_ready": false,
      "turn_candidate": false,
      "required_turn": false,
      "front_available": true,
      "node_recorded": false,
      "cooldown_active": false,
      "accepted_branch_mask": 5,
      "window_frames": 6,
      "age_ms": 416,
      "confidence": 0.82,
      "center_px": { "x": 480.0, "y": 390.0 },
      "center_y_norm": 0.54,
      "approach_phase": "turn_zone",
      "overshoot_risk": false,
      "too_late_to_turn": false,
      "branches": [
        {
          "direction": "front",
          "present_frames": 6,
          "max_score": 0.94,
          "average_score": 0.82
        }
      ],
      "node": {
        "valid": false,
        "id": 0,
        "local_coord": { "x": 0, "y": 0 },
        "topology": "unknown",
        "arrival_heading": "unknown",
        "camera_branch_mask": 0,
        "grid_branch_mask": 0,
        "first_node": false,
        "origin_local_only": true
      }
    },
    "grid_node": {
      "valid": true,
      "id": 3,
      "local_coord": { "x": 0, "y": -2 },
      "topology": "T",
      "arrival_heading": "north",
      "camera_branch_mask": 7,
      "grid_branch_mask": 7,
      "first_node": false,
      "origin_local_only": true
    },
    "drone_position": {
      "valid": true,
      "cell_progress": 0.42,
      "grid_offset_x": 0.0,
      "grid_offset_y": -0.42
    },
    "marker_detected": true,
    "marker_id": 2,
    "marker_count": 1,
    "markers": [
      {
        "id": 2,
        "center_px": { "x": 502.4, "y": 382.6 },
        "corners_px": [
          { "x": 474.1, "y": 354.2 },
          { "x": 531.5, "y": 355.0 },
          { "x": 530.8, "y": 412.9 },
          { "x": 473.4, "y": 411.7 }
        ],
        "orientation_deg": 0.7
      }
    ]
  },
  "grid": {
    "row": -2,
    "col": 0,
    "heading_deg": 0.0
  },
  "debug": {
    "processing_latency_ms": 12.4,
    "read_frame_ms": 56.1,
    "jpeg_decode_ms": 3.4,
    "aruco_latency_ms": 8.1,
    "line_latency_ms": 2.2,
    "intersection_latency_ms": 1.3,
    "intersection_decision_latency_ms": 0.05,
    "telemetry_build_ms": 0.3,
    "telemetry_send_ms": 0.1,
    "video_submit_ms": 0.1,
    "video_send_ms": 3.8,
    "capture_fps": 11.8,
    "processing_fps": 11.7,
    "debug_video_send_fps": 5.0,
    "video_chunk_pacing_us": 150,
    "cpu_temp_c": 62.5,
    "telemetry_bytes": 980,
    "video_jpeg_bytes": 24576,
    "video_sent_frames": 357,
    "video_dropped_frames": 2,
    "video_skipped_frames": 12,
    "video_chunks_sent": 4284,
    "video_send_failures": 0,
    "video_chunk_count": 21,
    "line_mask_count": 1,
    "line_contours_found": 4,
    "line_candidates_evaluated": 3,
    "line_roi_pixels": 141312,
    "line_selected_contour_points": 48,
    "note": "grid_mission"
  }
}
```

### 3.2 Optional Mission Object

The serializer supports a richer `mission` object, but the current
`GcsTelemetryPublisher` path used by `vision_debug_node`, `line_follow_node`,
`astroquad-onboard`, and `grid_mission_node` does not yet populate it. For
those executables, the console log and `debug.note` identify the active
runtime, while GCS mission state remains minimal.

Reserved richer shape:

```json
{
  "mission": {
    "state": "SNAKE_FORWARD",
    "control_intent": "fwd_blind",
    "phase_elapsed_ms": 1250,
    "target_altitude_m": 2.0,
    "altitude_off_pad_confirmed": true,
    "grid": {
      "x": 0,
      "y": -2,
      "heading": "north",
      "snake_dir": "right",
      "valid": true
    },
    "vertiport": {
      "verified": true,
      "marker_id": 23
    },
    "markers_found": [
      {
        "id": 2,
        "grid": [0, -2],
        "grid_valid": true,
        "orientation_deg": 0.7
      }
    ],
    "markers_expected": 4,
    "snake_complete": false,
    "last_safety_event": ""
  }
}
```

## 4. Field Notes

| Field | Meaning |
|---|---|
| `camera.frame_seq` | Matches the MJPEG video `frame_id` for the same camera frame. |
| `vision.line.*` | Filtered line-tracking result. `tracking_point_px.x` is the lateral control reference. |
| `vision.line.contour_px` | Simplified selected contour. Intersections may appear as cross-shaped contours. |
| `vision.intersection.*` | Stabilized per-frame topology from the line mask. `straight` is valid but not a turn node. |
| `vision.intersection_decision.*` | Sliding-window decision layer over intersection classification. It emits node/turn readiness hints, not autopilot commands. |
| `vision.grid_node` | Latest committed grid node event. In `astroquad-onboard`/`grid_mission_node`, this is intentionally resent every frame. |
| `vision.grid_node.local_coord` | Local exploration coordinates only; official competition origin conversion is not implemented. |
| `vision.drone_position` | Fractional progress from the last committed node, derived from short-window local estimate displacement. |
| `vision.markers[]` | ArUco observations in the current frame. Marker commitment/stability is onboard mission logic, not a GCS decision. |
| `grid.row`, `grid.col` | Legacy convenience fields mirrored from valid `vision.grid_node`; default `-1` when unknown. |
| `debug.note` | Runtime source such as `vision_debug`, `line_follow`, or `grid_mission`. |

## 5. Video Stream

Video uses UDP datagrams containing a 28-byte `AQV1` header followed by a JPEG
payload chunk.

Header layout, big-endian:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | magic bytes `AQV1` |
| 4 | 2 | header size, currently `28` |
| 6 | 2 | flags, currently `0` |
| 8 | 4 | `frame_id` |
| 12 | 2 | `chunk_index` |
| 14 | 2 | `chunk_count` |
| 16 | 4 | `payload_size` |
| 20 | 8 | `timestamp_ms` |

Rules:

- Maximum payload per datagram is 1200 bytes.
- GCS displays a JPEG only after all chunks for the same `frame_id` arrive.
- Incomplete frames are dropped.
- GCS keeps the last complete frame through temporary UDP drops.
- GCS does not display a video latency estimate because onboard and laptop
  clocks are not assumed synchronized.

## 6. GCS Discovery Beacon

GCS broadcasts the following text payload to UDP port 5601 once per second:

```text
AQGCS1 video_port=5600
```

When onboard is configured with `gcs.ip = "255.255.255.255"` or `0.0.0.0`,
video/debug tools listen for this beacon before streaming. If a beacon is
received, onboard uses the sender IP and advertised video port for unicast
debug video. If discovery fails, onboard falls back to configured destination
IP and port.

## 7. Autopilot Integration Notes

Primary control mode is ArduPilot `GUIDED` with MAVLink
`SET_POSITION_TARGET_LOCAL_NED` body-frame velocity setpoints. SITL uses UDP
MAVLink; real ArduPilot serial mission support uses serial or USB serial through
`SerialMavlinkTransport`.

Current boundaries:

- `line_follow_node` supports SITL UDP and guarded real ArduPilot serial paths.
- `mavlink_probe` and `mavlink_motor_test` are no-arm/props-removed bench tools.
- `astroquad-onboard` supports SITL UDP and guarded real ArduPilot serial paths;
  `grid_mission_node` builds the same entrypoint as a compatibility/staging
  alias. Real arm/takeoff requires `--allow-arm-takeoff`.
- Command channel fields for GCS-originated mission control remain planned.

## 8. Command Channel

The command channel is planned but not implemented. Reserved command message
types:

- `start_mission`
- `abort_mission`
- `emergency_land`
- `set_marker_count`
- `request_status`
- `set_control_backend`

Command messages will use JSON with the same common top-level fields and should
receive a `CMD_ACK` telemetry response when implemented.

## 9. Current Executables

| Executable | Repo | Purpose |
|---|---|---|
| `astroquad-onboard` | onboard | Current grid arena snake-mission SITL and guarded serial runtime. |
| `grid_mission_node` | onboard | Compatibility/staging alias for `astroquad-onboard`. |
| `uav-onboard-telem` | onboard | Basic telemetry bring-up sender / development probe. |
| `vision_debug_node` | onboard | Camera/vision telemetry and optional raw MJPEG debug video. |
| `line_follow_node` | onboard | Short line-follow mission staging for SITL and guarded ArduPilot serial tests. |
| `mavlink_probe` | onboard | No-arm MAVLink/local-estimate/parameter probe. |
| `mavlink_motor_test` | onboard | Props-removed low-throttle motor command check. |
| `video_streamer` | onboard | Raw MJPEG transport smoke tool. |
| `astroquad-gcs` | GCS | Current GCS UI with telemetry/video/log windows and GCS-side overlays. |
| `uav-gcs-telem` | GCS | Telemetry-only console receiver / development probe. |
| `uav-gcs-video` | GCS | Raw MJPEG video viewer. |

## 10. Change Log

| Version | Date | Change |
|---|---|---|
| v1.0 | 2026-04-24 | Initial JSON telemetry, command, and video protocol draft. |
| v1.1 | 2026-04-27 | Added marker telemetry and GCS discovery beacon. |
| v1.2 | 2026-04-27 | Added line telemetry and best-effort debug video rules. |
| v1.3 | 2026-04-27 | Added line stabilizer diagnostics and video queue counters. |
| v1.4 | 2026-04-28 | Added video chunk/send counters and Pi CPU temperature telemetry. |
| v1.5 | 2026-04-28 | Added Pi 4 + IMX519 camera/system telemetry and FPS counters. |
| v1.6 | 2026-04-30 | Added structured intersection telemetry and GCS overlay/log support. |
| v1.7 | 2026-05-01 | Added intersection decision and local grid-node telemetry. |
| v1.7 | 2026-05-14 | No schema change; documented runtime source profiles. |
| v1.8 | 2026-05-21 | Added `vision.drone_position`, clarified committed-node resend semantics, documented grid mission staging and current mission telemetry limits. |
