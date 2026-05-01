# Astroquad Onboard-GCS Protocol

Version: v1.7
Last updated: 2026-05-01
This file must stay identical in `uav-onboard/docs/PROTOCOL.md` and `uav-gcs/docs/PROTOCOL.md`.

## 1. Channels

| Channel | Direction | Transport | Default port | Status |
|---|---|---|---:|---|
| Telemetry | onboard -> GCS | UDP JSON | 14550 | implemented |
| Command | GCS -> onboard | TCP JSON | 14551 | planned |
| Video stream | onboard -> GCS | UDP MJPEG chunks | 5600 | implemented |
| GCS discovery | GCS -> LAN broadcast | UDP text beacon | 5601 | implemented |

Ports and destination addresses are configured in each repo's `config/network.toml`.

## 2. Compatibility Rules

- Every JSON message includes `protocol_version`, `type`, `seq`, and `timestamp_ms`.
- Receivers ignore unknown fields.
- Receivers should drop malformed JSON without terminating the process.
- `protocol_version` is currently `1`.
- UDP telemetry and video are best-effort. Missing packets must not block mission logic.
- Debug video streaming is never mission-critical. If video sending falls behind, onboard drops old frames and keeps the latest frame/result for vision and telemetry.

## 3. Telemetry JSON

Telemetry packets are sent from onboard to GCS over UDP. During vision debugging, onboard sends one telemetry packet per processed camera frame so GCS can match marker and line metadata to the video frame.

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
    "lens_position": 0.67,
    "exposure_mode": "sport",
    "shutter_us": 0,
    "gain": 0.0,
    "awb": "auto"
  },
  "vision": {
    "line_detected": false,
    "line_offset": 0.0,
    "line_angle": 0.0,
    "line": {
      "detected": false,
      "raw_detected": false,
      "filtered": false,
      "held": false,
      "rejected_jump": false,
      "tracking_point_px": { "x": 0.0, "y": 0.0 },
      "raw_tracking_point_px": { "x": 0.0, "y": 0.0 },
      "centroid_px": { "x": 0.0, "y": 0.0 },
      "center_offset_px": 0.0,
      "raw_center_offset_px": 0.0,
      "angle_deg": 0.0,
      "raw_angle_deg": 0.0,
      "confidence": 0.0,
      "contour_px": []
    },
    "intersection_detected": false,
    "intersection_score": 0.0,
    "intersection": {
      "valid": false,
      "detected": false,
      "type": "none",
      "raw_type": "none",
      "stable": false,
      "held": false,
      "center_px": { "x": 0.0, "y": 0.0 },
      "raw_center_px": { "x": 0.0, "y": 0.0 },
      "score": 0.0,
      "raw_score": 0.0,
      "branch_mask": 0,
      "branch_count": 0,
      "stable_frames": 0,
      "radius_px": 0.0,
      "selected_mask_index": -1,
      "branches": [
        {
          "direction": "front",
          "present": false,
          "score": 0.0,
          "endpoint_px": { "x": 0.0, "y": 0.0 },
          "angle_deg": -90.0
        }
      ]
    },
    "intersection_decision": {
      "state": "cruise",
      "action": "continue",
      "accepted_type": "none",
      "best_observed_type": "none",
      "event_ready": false,
      "turn_candidate": false,
      "required_turn": false,
      "front_available": false,
      "node_recorded": false,
      "cooldown_active": false,
      "accepted_branch_mask": 0,
      "window_frames": 0,
      "age_ms": 0,
      "confidence": 0.0,
      "center_px": { "x": 0.0, "y": 0.0 },
      "center_y_norm": 0.0,
      "approach_phase": "far",
      "overshoot_risk": false,
      "too_late_to_turn": false,
      "branches": [
        {
          "direction": "front",
          "present_frames": 0,
          "max_score": 0.0,
          "average_score": 0.0
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
      "valid": false,
      "id": 0,
      "local_coord": { "x": 0, "y": 0 },
      "topology": "unknown",
      "arrival_heading": "unknown",
      "camera_branch_mask": 0,
      "grid_branch_mask": 0,
      "first_node": false,
      "origin_local_only": true
    },
    "marker_detected": true,
    "marker_id": 7,
    "marker_count": 1,
    "markers": [
      {
        "id": 7,
        "center_px": { "x": 312.4, "y": 188.6 },
        "corners_px": [
          { "x": 280.1, "y": 150.2 },
          { "x": 344.5, "y": 151.0 },
          { "x": 343.8, "y": 222.9 },
          { "x": 279.4, "y": 221.7 }
        ],
        "orientation_deg": 0.7
      }
    ]
  },
  "grid": {
    "row": -1,
    "col": -1,
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
    "note": "vision_debug_node"
  }
}
```

### 3.2 Field Notes

| Field | Type | Meaning |
|---|---|---|
| `system.board_model` | string | Onboard Linux board model, usually from device tree. |
| `system.os_release` | string | Onboard OS pretty name when available. |
| `system.uptime_s` | number | Onboard OS uptime in seconds. |
| `system.cpu_temp_c` | number | Pi CPU temperature in Celsius when available. |
| `system.throttled_raw` | string | Raw `vcgencmd get_throttled` output, for example `throttled=0x0`. |
| `system.cpu_load_1m` | number | 1-minute Linux load average. |
| `system.mem_available_kb` | `uint64` | Linux `MemAvailable` from `/proc/meminfo`. |
| `system.wifi_signal_dbm` | number | Wi-Fi signal level from `/proc/net/wireless` when available. |
| `system.wifi_tx_bitrate_mbps` | number | Wi-Fi tx bitrate from `iw dev wlan0 link` when available. |
| `camera.sensor_model` | string | Configured sensor model, currently expected to be `imx519` for the Pi 4 upgrade. |
| `camera.camera_index` | int | rpicam camera index passed with `--camera`. |
| `camera.frame_seq` | `uint32` | Must match the MJPEG video `frame_id` for the same camera frame. |
| `camera.timestamp_ms` | `int64` | Not currently a nested field. Use top-level `timestamp_ms` as frame capture time. |
| `camera.configured_fps` | number | Configured rpicam capture FPS. |
| `camera.measured_capture_fps` | number | Rolling measured onboard capture/read FPS. |
| `camera.autofocus_mode` | string | Configured rpicam autofocus mode. |
| `camera.lens_position` | number | Configured manual lens position/dioptre when used. |
| `camera.exposure_mode` | string | Configured rpicam exposure mode. |
| `camera.shutter_us` | int | Configured fixed shutter in microseconds; `0` means automatic/default. |
| `camera.gain` | number | Configured analogue gain; `0.0` means automatic/default. |
| `camera.awb` | string | Configured auto white balance mode. |
| `vision.line_detected` | `bool` | Legacy summary field. True when the detailed line object is detected. |
| `vision.line_offset` | number | Legacy summary field. Same value as `vision.line.center_offset_px`. |
| `vision.line_angle` | number | Legacy summary field. Same value as `vision.line.angle_deg`. |
| `vision.line.detected` | `bool` | True when a usable line contour/tracking point was found. |
| `vision.line.raw_detected` | `bool` | True when the current raw detector frame found a candidate before smoothing/hold logic. |
| `vision.line.filtered` | `bool` | True when the reported line has passed through the stabilizer. |
| `vision.line.held` | `bool` | True when the stabilizer is temporarily holding the previous line because the current raw frame was missing or rejected. |
| `vision.line.rejected_jump` | `bool` | True when the current raw candidate was suppressed as a sudden offset/angle jump. |
| `vision.line.tracking_point_px` | object | Representative line point used for tracing. GCS draws it as a green point. |
| `vision.line.raw_tracking_point_px` | object | Raw detector tracking point before EMA/hold filtering. |
| `vision.line.centroid_px` | object | Contour centroid fallback/diagnostic point. |
| `vision.line.center_offset_px` | number | Horizontal offset from image center to `tracking_point_px`. |
| `vision.line.raw_center_offset_px` | number | Raw offset before stabilizer filtering. |
| `vision.line.angle_deg` | number | Image-plane line angle. |
| `vision.line.raw_angle_deg` | number | Raw detector angle before stabilizer filtering. |
| `vision.line.confidence` | number | 0.0 to 1.0 confidence estimate from contour quality. |
| `vision.line.contour_px` | array | Simplified connected line candidate contour/polyline in image pixel coordinates. GCS draws it in magenta, including cross-shaped intersections when they are part of the selected contour. |
| `vision.intersection_detected` | `bool` | Legacy summary field. True for stabilized `+`, `T`, or `L` intersection types. False for `straight`, `none`, or `unknown`. |
| `vision.intersection_score` | number | Stabilized intersection confidence score. |
| `vision.intersection.valid` | `bool` | True when the intersection classifier has a usable classification result for the frame or a short held result. |
| `vision.intersection.detected` | `bool` | True when the stabilized type is a turn/branching intersection: `+`, `T`, or `L`. |
| `vision.intersection.type` | string | Stabilized type: `none`, `unknown`, `straight`, `L`, `T`, or `+`. |
| `vision.intersection.raw_type` | string | Current-frame raw classifier type before temporal smoothing. |
| `vision.intersection.stable` | `bool` | True after the same type has survived the stabilizer for the required frame count. |
| `vision.intersection.held` | `bool` | True when the stabilizer is briefly holding the previous type because the raw frame is missing or weak. |
| `vision.intersection.center_px` | object | Stabilized center point used by GCS to draw the intersection overlay. |
| `vision.intersection.raw_center_px` | object | Raw center point before temporal smoothing. |
| `vision.intersection.raw_score` | number | Current-frame raw confidence before temporal smoothing. |
| `vision.intersection.branch_mask` | int | Bit mask for present branches in front/right/back/left order. |
| `vision.intersection.branch_count` | int | Number of branches classified as present. |
| `vision.intersection.stable_frames` | int | Number of consecutive frames supporting the current stabilized type. |
| `vision.intersection.radius_px` | number | Approximate source-image center sampling radius used for debug overlay context. |
| `vision.intersection.selected_mask_index` | int | Index of the line polarity mask that produced the selected classification. |
| `vision.intersection.branches[]` | array | Per-direction branch observations. Directions are `front`, `right`, `back`, and `left`; each entry includes `present`, `score`, `endpoint_px`, and `angle_deg`. |
| `vision.intersection_decision.state` | string | Mission/debug decision state: `cruise`, `candidate`, `node_record`, `turn_confirm`, `turn_ready`, or `cooldown`. |
| `vision.intersection_decision.action` | string | Debug action hint. This is not a Pixhawk command. Current values include `continue`, `prepare_turn`, and `hold`. |
| `vision.intersection_decision.accepted_type` | string | Short-window accepted topology after branch evidence aggregation. |
| `vision.intersection_decision.best_observed_type` | string | Highest single-frame topology observed in the decision window. |
| `vision.intersection_decision.event_ready` | `bool` | True when this frame produced a grid node event. |
| `vision.intersection_decision.turn_candidate` | `bool` | True when side evidence and front-branch/policy gating make a turn candidate. |
| `vision.intersection_decision.required_turn` | `bool` | True when the decision layer believes a turn may be required; control is still future work. |
| `vision.intersection_decision.front_available` | `bool` | True when recent evidence still supports a branch in the camera-forward direction. |
| `vision.intersection_decision.center_y_norm` | number | Intersection center Y normalized by frame height, used for approach phase and late/overshoot flags. |
| `vision.intersection_decision.approach_phase` | string | Image-space phase: `far`, `approaching`, `turn_zone`, `late`, or `passed`. |
| `vision.intersection_decision.overshoot_risk` | `bool` | True when a required turn candidate is already past the allowed turn zone. |
| `vision.intersection_decision.branches[]` | array | Per-direction decision evidence with `present_frames`, `max_score`, and `average_score`. |
| `vision.intersection_decision.node` | object | Grid node event attached to this decision when available. |
| `vision.grid_node` | object | Latest grid node event for this frame. `local_coord` is local-only until official competition origin conversion is implemented. |
| `vision.marker_detected` | `bool` | True when `markers` is not empty. |
| `vision.marker_id` | `int` | First detected marker id, kept for backward compatibility. `-1` means none. |
| `vision.marker_count` | `int` | Number of entries in `vision.markers`. |
| `vision.markers[].center_px` | object | Marker center in image pixel coordinates. |
| `vision.markers[].corners_px` | array | Four marker corners in image pixel coordinates, ordered as OpenCV returns them. |
| `vision.markers[].orientation_deg` | number | Image-plane angle from corner 0 to corner 1. |
| `debug.processing_latency_ms` | number | Onboard decode and detector processing latency after a camera frame has been read. |
| `debug.read_frame_ms` | number | Time spent waiting for/reading one MJPEG camera frame. |
| `debug.jpeg_decode_ms` | number | Time spent decoding the camera JPEG for onboard detectors. |
| `debug.aruco_latency_ms` | number | ArUco detector latency for the frame. |
| `debug.line_latency_ms` | number | Line detector latency for the frame. |
| `debug.intersection_latency_ms` | number | Intersection classifier latency for the frame. |
| `debug.intersection_decision_latency_ms` | number | Short-window decision and grid-node update latency for the frame. |
| `debug.telemetry_build_ms` | number | Most recently measured telemetry JSON serialization latency. |
| `debug.telemetry_send_ms` | number | Most recently measured UDP telemetry send call latency. |
| `debug.video_submit_ms` | number | Most recently measured latest-frame video queue submit latency. |
| `debug.video_send_ms` | number | Most recently measured onboard UDP MJPEG chunk send latency in the video worker. |
| `debug.capture_fps` | number | Rolling measured camera read FPS. |
| `debug.processing_fps` | number | Rolling measured detector loop FPS. |
| `debug.debug_video_send_fps` | number | Configured best-effort debug video send cap. |
| `debug.video_chunk_pacing_us` | int | Configured delay between UDP video chunks. |
| `debug.cpu_temp_c` | number | Raspberry Pi CPU temperature in Celsius when available; `0.0` when unavailable. |
| `debug.telemetry_bytes` | `uint64` | Most recently serialized telemetry payload size. |
| `debug.video_jpeg_bytes` | `uint64` | Raw camera JPEG byte size for this frame. |
| `debug.video_sent_frames` | `uint64` | Number of frames sent by the onboard video worker. |
| `debug.video_dropped_frames` | `uint64` | Number of stale frames replaced before the video worker could send them. |
| `debug.video_skipped_frames` | `uint64` | Number of camera frames intentionally not submitted to video because debug video send FPS is capped. |
| `debug.video_chunks_sent` | `uint64` | Total number of UDP video chunks sent by the onboard video worker. |
| `debug.video_send_failures` | `uint64` | Number of failed UDP video frame send attempts in the onboard video worker. |
| `debug.video_chunk_count` | `int` | Number of UDP chunks in the most recently sent MJPEG frame. |
| `debug.line_mask_count` | `int` | Number of threshold masks evaluated by the line detector. |
| `debug.line_contours_found` | `int` | Number of line candidate contours found before filtering. |
| `debug.line_candidates_evaluated` | `int` | Number of ranked contours actually scored. |
| `debug.line_roi_pixels` | `int` | Pixel count of the resized ROI processed by the line detector. |
| `debug.line_selected_contour_points` | `int` | Number of simplified contour points sent in telemetry. |

GCS overlay rendering uses only onboard vision metadata. It does not run marker, line, or intersection detection locally.

## 4. Video Stream

Video uses UDP datagrams containing a 28-byte `AQV1` header followed by a JPEG payload chunk.

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
- GCS reassembles a JPEG frame only after all chunks for the same `frame_id` arrive.
- Incomplete frames are dropped.
- GCS keeps displaying the last complete frame if the next UDP frame is incomplete.
- `timestamp_ms` is the onboard capture timestamp. It is useful for telemetry
  correlation when clocks are synchronized. GCS does not display a video
  latency estimate because the laptop and Pi clocks are not assumed to be
  synchronized and the debug video path is best-effort.

## 5. GCS Discovery Beacon

GCS broadcasts the following text payload to UDP port 5601 once per second:

```text
AQGCS1 video_port=5600
```

When onboard is configured with `gcs.ip = "255.255.255.255"` or `0.0.0.0`, video/debug tools listen for this beacon before streaming. If a beacon is received, onboard uses the sender IP as the GCS destination and the advertised `video_port` as the video destination port.

This avoids manual laptop IP entry for normal same-LAN testing. If discovery fails, onboard falls back to the configured destination IP and port.

## 6. Command Channel

The command channel is planned but not implemented in this milestone. Reserved command message types:

- `start_mission`
- `abort_mission`
- `emergency_land`
- `set_marker_count`
- `request_status`

Command messages will use JSON with the same common top-level fields and will receive a `CMD_ACK` telemetry response.

## 7. Current Executables

| Executable | Repo | Purpose |
|---|---|---|
| `uav_onboard` | onboard | Basic telemetry bring-up sender. |
| `video_streamer` | onboard | Raw MJPEG streaming smoke tool. |
| `vision_debug_node` | onboard | Pi camera capture, ArUco/line/intersection detection, raw video send, vision telemetry send. |
| `uav_gcs` | GCS | Basic telemetry receiver. |
| `uav_gcs_video` | GCS | Raw MJPEG video viewer. |
| `uav_gcs_vision_debug` | GCS | MJPEG video viewer with vision log window and GCS-side marker/line overlay. |

## 8. Change Log

| Version | Date | Change |
|---|---|---|
| v1.0 | 2026-04-24 | Initial JSON telemetry, command, and video protocol draft. |
| v1.1 | 2026-04-27 | Added `vision.markers[]`, frame-synchronized marker telemetry, `debug.aruco_latency_ms`, and GCS discovery beacon details. |
| v1.2 | 2026-04-27 | Added `vision.line`, `debug.line_latency_ms`, GCS line overlay metadata, and explicit best-effort debug video rules. |
| v1.3 | 2026-04-27 | Added line stabilizer state, raw line diagnostics, latency breakdown, video queue counters, and line detector workload counters. |
| v1.3 | 2026-04-28 | No schema change; high-altitude line detector/stabilizer tuning continues to use the existing `vision.line.*` and `debug.line_*` fields. |
| v1.4 | 2026-04-28 | Added debug video chunk/send/skip counters and optional Pi CPU temperature telemetry for diagnosing frame drops and thermal throttling. |
| v1.5 | 2026-04-28 | Added Raspberry Pi 4 + IMX519 camera/system telemetry, capture/processing FPS, debug video pacing config, and video send failure counters. |
| v1.5 | 2026-04-29 | No schema change; synchronized example defaults to the performance profile: 960x720 camera, 5 FPS opt-in debug video, and 150us chunk pacing. |
| v1.6 | 2026-04-30 | Added structured `vision.intersection`, branch ray metadata, `debug.intersection_latency_ms`, and GCS intersection overlay/log support. |
| v1.7 | 2026-05-01 | Added `vision.intersection_decision`, `vision.grid_node`, branch evidence telemetry, local grid node coordinates, and `debug.intersection_decision_latency_ms`. |
