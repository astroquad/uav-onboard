# uav-onboard

Onboard software for the indoor UAV search mission.

This project owns camera processing, mission state management, MAVLink control,
GCS communication, safety handling, and headless logging.

## Layout

- `config/`: runtime TOML configuration
- `src/`: mission application source
- `tools/`: local development and debugging utilities
- `test_data/`: captured images, videos, and logs for repeatable tests
- `tests/`: unit tests
- `scripts/`: build, deploy, and run helpers
- `docs/`: design notes and protocol reference
- `logs/`: runtime logs

## Build

On a clean Raspberry Pi OS Lite 64-bit install, set up build dependencies first:

```bash
bash scripts/setup_rpi_dependencies.sh
```

Optional development tools:

```bash
bash scripts/setup_rpi_dependencies.sh --with-dev-tools
```

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Camera Bring-Up

```bash
./build/camera_preview --device 0 --frames 30 --save test_data/images/camera_smoke.jpg
```

Expected output includes the captured frame size, measured FPS, and the saved
image path.

For Raspberry Pi CSI cameras, first verify the libcamera/rpicam path:

```bash
rpicam-hello --list-cameras
rpicam-still -t 1000 --nopreview -o test_data/images/camera_smoke.jpg
```

## Video Stream Bring-Up

Start `uav_gcs_video` on the laptop first, then run the onboard streamer.
With the default broadcast address in `config/network.toml`, `video_streamer`
listens for the GCS discovery beacon and then streams to the discovered laptop
IP by unicast:

```bash
./build/video_streamer --source rpicam --config config
```

If discovery is blocked by the network, override the destination explicitly:

```bash
./build/video_streamer --source rpicam --config config --gcs-ip <laptop-ip>
```

For a local sender smoke test without the Pi camera:

```bash
./build/video_streamer --source test-pattern --gcs-ip 127.0.0.1 --port 5600 --count 30
```

`video_streamer` sends MJPEG frames over UDP in small chunks. It is a best-effort
debug stream; dropped frames are acceptable so video output does not become part
of the future mission-critical vision loop.

## Tests

Configure with tests enabled:

```bash
cmake -S . -B build-tests -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

Current focused tests cover telemetry JSON generation for `vision.line`.

## Vision Debug: ArUco and Line Tracing

Start `uav_gcs_vision_debug` on the laptop first. Then run this on the
Raspberry Pi:

```bash
./build/vision_debug_node --config config
```

This node captures Pi camera MJPEG frames, decodes each frame once, runs enabled
onboard detectors, sends vision metadata as telemetry, and sends the original
camera JPEG to GCS. It does not draw overlays on the Raspberry Pi.

Current detectors:

- ArUco marker detection.
- Line tracing MVP using resized ROI local-contrast masking, lookahead-band
  projection, contour scoring, and onboard EMA/hold filtering for sudden
  offset jumps.

The debug video path is best-effort and non-critical. If GCS video streaming
falls behind, old video frames are dropped and the vision loop keeps working on
the latest camera frame/result. Future mission decision and control output must
stay on the critical path; GCS video is only an observation aid. The default
debug stream now captures at 12 FPS, submits video at 10 FPS, and paces UDP
chunks to reduce Wi-Fi burst loss.

Useful options:

```bash
./build/vision_debug_node --config config --count 100
./build/vision_debug_node --config config --gcs-ip <laptop-ip>
./build/vision_debug_node --config config --no-video
./build/vision_debug_node --config config --no-telemetry
./build/vision_debug_node --config config --aruco-only
./build/vision_debug_node --config config --line-only
./build/vision_debug_node --config config --disable-line
./build/vision_debug_node --config config --disable-aruco
./build/vision_debug_node --config config --line-only --line-mode auto
./build/vision_debug_node --config config --line-only --line-mode light_on_dark
./build/vision_debug_node --config config --line-only --line-mode dark_on_light
./build/vision_debug_node --config config --line-only --line-threshold 160
./build/vision_debug_node --config config --line-only --line-roi-top 0.08 --line-lookahead 0.55
```

Line mode guidance:

- `light_on_dark`: default for the current outdoor practice grid. Use when the
  track line is brighter than the floor.
- `auto`: tests both bright-line and dark-line masks and chooses the
  best line-shaped candidate.
- `dark_on_light`: use when the track line is darker than the floor.
- `--line-threshold 0`: use automatic Otsu thresholding. A positive value uses
  that fixed grayscale threshold. With the default `local_contrast` mask,
  `line.local_contrast_threshold` is used instead.
- `--line-roi-top`: image ratio to ignore at the top. The default `0.08`
  keeps most of the upper frame visible; raise it only if the camera sees
  horizon or non-floor clutter.
- `--line-lookahead`: vertical image ratio for the green tracking point. The
  default `0.55` places it close to the frame center instead of the old lower
  debug point.

The line contour sent to GCS is the simplified connected contour selected by
the detector. The green tracking point is calculated from a narrow lookahead
band inside that contour rather than from the whole contour centroid, so broad
floor blobs and cross-shaped contours are less likely to pull the control point
away from the actual line center.

Latency and stability defaults are configured in `config/vision.toml`:

- `line.process_width = 480`: line detection keeps more high-altitude line
  pixels than the previous 320px setting while still running on a resized ROI.
- `line.mask_strategy = "local_contrast"`: detects bright line structures by
  comparing each pixel to its local background instead of relying only on a
  global grayscale threshold.
- `line.lookahead_band_ratio = 0.06`: the tracking point is measured from a
  short horizontal band around the configured lookahead Y coordinate.
- `line.morph_open_kernel = 1`, `line.morph_close_kernel = 7`: local-contrast
  masks are closed more aggressively than they are opened so a bright line does
  not split into two edge contours.
- `line.line_run_merge_gap_px = 16`: nearby projection runs are merged when
  measuring the tracking line width.
- `line.max_candidates = 8`: only the largest ranked contours are scored.
- `line.filter_enabled = true`: raw line offset/angle spikes are masked with
  EMA smoothing, short hold, and multi-frame reacquire logic.
- `line.filter_max_offset_velocity_ratio = 0.08`: accepted measurements are
  still rate-limited so one noisy frame cannot move the green point too far.
- `line.filter_max_angle_jump_deg = 90.0`: angle-only jumps are treated
  loosely because cross/grid contours can make `fitLine` angle noisy while the
  tracking offset remains stable.
- `video.send_fps = 10`, `video.chunk_pacing_us = 150`: debug video is
  decimated and paced separately from the detector frame loop.
- The GCS vision log shows read/decode/ArUco/line/JSON/send/video timings,
  video chunk/skip counters, CPU temperature when available, contour counts,
  queue drops, and raw-vs-filtered line state.

For image-file detector smoke tests:

```bash
./build/aruco_detector_tester --config config --image test_data/images/marker.jpg
./build/line_detector_tuner --config config --image test_data/images/line_sample.jpg
./build/line_detector_tuner --config config --image test_data/images/line_sample.jpg --mode auto
./build/line_detector_tuner --config config --image test_data/images/line_sample.jpg --mask local_contrast --process-width 480 --band 0.06
./build/line_detector_tuner --config config --image test_data/images/line_sample.jpg --local-threshold 10 --morph-open 1 --morph-close 7 --merge-gap 16
./build/line_detector_tuner --config config --image test_data/images/line_sample.jpg --mode light_on_dark --threshold 160 --roi-top 0.08 --lookahead 0.55
```

Expected GCS result:

- ArUco markers are overlaid by GCS as marker boxes, center points, direction
  arrows, and labels.
- Detected line contour/border is overlaid by GCS in magenta.
- The line tracking point is overlaid by GCS in green.
- The onboard JPEG stream remains raw camera video with no drawn overlay.

## Telemetry Bring-Up

Start `uav-gcs` on the laptop first, then run:

```bash
./build/uav_onboard --config config --count 10
```

By default, telemetry is sent to the IPv4 local broadcast address:

```toml
[gcs]
ip = "255.255.255.255"
telemetry_port = 14550
```

This avoids editing the onboard config whenever the laptop IP changes. Some
guest Wi-Fi networks block local broadcast; if GCS receives nothing there, use
the laptop's explicit IPv4 address with `--gcs-ip` or in `network.toml`.

For a quick local loopback test, override the destination:

```bash
./build/uav_onboard --config config --gcs-ip 127.0.0.1 --count 10
```

Use `--count 0` or omit `--count` to send telemetry until interrupted.

## Bring-Up Order

1. On the laptop, build and start `uav-gcs` telemetry receiver.
2. On the Raspberry Pi, run `bash scripts/setup_rpi_dependencies.sh`.
3. On the Raspberry Pi, build `uav-onboard`.
4. On the Raspberry Pi, validate the camera with `camera_preview`.
5. On the laptop, start `uav_gcs_video`.
6. On the Raspberry Pi, run `video_streamer` and confirm it prints
   `discovered GCS video receiver at <ip>:5600`.
7. On the Raspberry Pi, run `uav_onboard` and confirm telemetry appears in GCS.
