# Astroquad Onboard-GCS Protocol

Version: v1.2
Last updated: 2026-04-27
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
  "camera": {
    "status": "streaming",
    "width": 640,
    "height": 480,
    "fps": 15.0,
    "frame_seq": 358
  },
  "vision": {
    "line_detected": false,
    "line_offset": 0.0,
    "line_angle": 0.0,
    "line": {
      "detected": false,
      "tracking_point_px": { "x": 0.0, "y": 0.0 },
      "centroid_px": { "x": 0.0, "y": 0.0 },
      "center_offset_px": 0.0,
      "angle_deg": 0.0,
      "confidence": 0.0,
      "contour_px": []
    },
    "intersection_detected": false,
    "intersection_score": 0.0,
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
    "aruco_latency_ms": 8.1,
    "line_latency_ms": 2.2,
    "note": "vision_debug_node"
  }
}
```

### 3.2 Field Notes

| Field | Type | Meaning |
|---|---|---|
| `camera.frame_seq` | `uint32` | Must match the MJPEG video `frame_id` for the same camera frame. |
| `camera.timestamp_ms` | `int64` | Not currently a nested field. Use top-level `timestamp_ms` as frame capture time. |
| `vision.line_detected` | `bool` | Legacy summary field. True when the detailed line object is detected. |
| `vision.line_offset` | number | Legacy summary field. Same value as `vision.line.center_offset_px`. |
| `vision.line_angle` | number | Legacy summary field. Same value as `vision.line.angle_deg`. |
| `vision.line.detected` | `bool` | True when a usable line contour/tracking point was found. |
| `vision.line.tracking_point_px` | object | Representative line point used for tracing. GCS draws it as a green point. |
| `vision.line.centroid_px` | object | Contour centroid fallback/diagnostic point. |
| `vision.line.center_offset_px` | number | Horizontal offset from image center to `tracking_point_px`. |
| `vision.line.angle_deg` | number | Image-plane line angle. |
| `vision.line.confidence` | number | 0.0 to 1.0 confidence estimate from contour quality. |
| `vision.line.contour_px` | array | Simplified contour/polyline in image pixel coordinates. GCS draws it in magenta. |
| `vision.marker_detected` | `bool` | True when `markers` is not empty. |
| `vision.marker_id` | `int` | First detected marker id, kept for backward compatibility. `-1` means none. |
| `vision.marker_count` | `int` | Number of entries in `vision.markers`. |
| `vision.markers[].center_px` | object | Marker center in image pixel coordinates. |
| `vision.markers[].corners_px` | array | Four marker corners in image pixel coordinates, ordered as OpenCV returns them. |
| `vision.markers[].orientation_deg` | number | Image-plane angle from corner 0 to corner 1. |
| `debug.processing_latency_ms` | number | Total onboard frame processing latency measured in the vision debug loop. |
| `debug.aruco_latency_ms` | number | ArUco detector latency for the frame. |
| `debug.line_latency_ms` | number | Line detector latency for the frame. |

GCS overlay rendering uses only onboard vision metadata. It does not run marker or line detection locally.

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
- `timestamp_ms` is the capture timestamp and is used for latency display and telemetry fallback matching.

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
| `vision_debug_node` | onboard | Pi camera capture, ArUco/line detection, raw video send, vision telemetry send. |
| `uav_gcs` | GCS | Basic telemetry receiver. |
| `uav_gcs_video` | GCS | Raw MJPEG video viewer. |
| `uav_gcs_vision_debug` | GCS | MJPEG video viewer with vision log window and GCS-side marker/line overlay. |

## 8. Change Log

| Version | Date | Change |
|---|---|---|
| v1.0 | 2026-04-24 | Initial JSON telemetry, command, and video protocol draft. |
| v1.1 | 2026-04-27 | Added `vision.markers[]`, frame-synchronized marker telemetry, `debug.aruco_latency_ms`, and GCS discovery beacon details. |
| v1.2 | 2026-04-27 | Added `vision.line`, `debug.line_latency_ms`, GCS line overlay metadata, and explicit best-effort debug video rules. |
