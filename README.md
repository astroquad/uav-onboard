# uav-onboard

Onboard software for the indoor UAV search mission.

This project owns camera processing, mission state management, MAVLink control,
GCS communication, safety handling, and headless logging.

Current target hardware is Raspberry Pi 4 + IMX519-78 16MP AF CSI camera.
The previous Raspberry Pi Zero 2 W/ZeroCam path is kept conceptually supported,
but new camera defaults and bring-up notes assume the Pi 4 IMX519 setup.

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

For Raspberry Pi 4 + IMX519-78, verify the rpicam/libcamera path first:

```bash
rpicam-hello --version
rpicam-hello --list-cameras
rpicam-still -t 1000 --nopreview -o test_data/images/imx519_smoke.jpg
rpicam-vid -t 5000 --nopreview --codec mjpeg --width 640 --height 480 --framerate 12 -o /tmp/imx519_test.mjpeg
```

Focus smoke tests:

```bash
rpicam-still -t 1000 --nopreview --autofocus-mode continuous -o test_data/images/focus_continuous.jpg
rpicam-still -t 1000 --nopreview --autofocus-mode auto -o test_data/images/focus_auto.jpg
rpicam-still -t 1000 --nopreview --autofocus-mode manual --lens-position 0.67 -o test_data/images/focus_manual_067.jpg
```

Mission-like runs should prefer stable camera settings over continuous visual
beauty. Continuous autofocus can hunt while the UAV is moving; manual lens
position or one-shot autofocus should be compared at the actual flight height.

The older OpenCV camera smoke tool is still available, but CSI cameras on
Raspberry Pi OS are validated primarily through `rpicam-*`:

```bash
./build/camera_preview --device 0 --frames 30 --save test_data/images/camera_smoke.jpg
```

Expected output includes the captured frame size, measured FPS, and the saved
image path.

For any Raspberry Pi CSI camera, the same rpicam checks remain the source of truth:

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

Current focused tests cover telemetry JSON generation, line/intersection
stabilization, and synthetic intersection classification when OpenCV is
available.

## Vision Debug: ArUco and Line Tracing

Start `uav_gcs_vision_debug` on the laptop first. Then run this on the
Raspberry Pi:

```bash
./build/vision_debug_node --config config
```

This node captures Pi camera MJPEG frames, decodes each frame once, runs enabled
onboard detectors, sends vision metadata as telemetry, and sends the original
camera JPEG to GCS. It does not draw overlays on the Raspberry Pi.

The onboard side still sends only metadata for ArUco markers and line tracing.
All marker boxes, line contours, labels, and tracking-point overlays are drawn
by GCS. Keep this separation: onboard CPU time is reserved for future mission
logic and MAVLink control, and debug video may be disabled entirely for final
mission runs.

Current detectors:

- ArUco marker detection.
- Line tracing MVP using the shared resized ROI mask builder. The current
  default `white_fill` strategy detects wide white tape/paper as filled blobs,
  then uses an anchor-band projection, contour scoring, and onboard EMA/hold
  filtering for sudden offset jumps. `local_contrast` remains available as a
  comparison/tuning strategy.
- Intersection classification using the same line mask, largest-blob center
  candidates, branch ray scoring, angle-based `L` vs `straight` checks, and
  onboard temporal smoothing for `+`, `T`, `L`, and `straight`.
- Intersection decision telemetry that aggregates recent branch evidence,
  records `L`/`T`/`+` as local grid node events, and reports turn candidates,
  approach phase, and overshoot risk without sending Pixhawk control commands.

The debug video path is opt-in, best-effort, and non-critical. By default the
onboard process captures camera frames only to compute ArUco/line metadata and
system telemetry. If GCS video streaming is explicitly enabled and falls behind,
old video frames are dropped and the vision loop keeps working on the latest
camera frame/result. Future mission decision and control output must stay on
the critical path; GCS video is only an observation aid. The Pi 4 + IMX519
default captures at 960x720 12 FPS with MJPEG quality 45 to keep onboard
decode, detector, and optional UDP video overhead conservative.
GCS video receiver discovery is skipped unless debug video is enabled.

Mission-style metadata-only examples:

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
./build/vision_debug_node --config config --aruco-only --camera-quality 90 --lens-position 1.0
```

GCS raw camera/overlay visual tuning examples:

```bash
./build/vision_debug_node --config config --video
./build/vision_debug_node --config config --video --fps 12
./build/vision_debug_node --config config --line-only --line-mode light_on_dark --video
./build/vision_debug_node --config config --line-only --line-mode light_on_dark --video --gcs-ip <laptop-ip>
./build/vision_debug_node --config config --aruco-only --video --camera-quality 90 --lens-position 1.0
```

If the GCS camera window says `waiting for video stream...`, check the onboard
startup line. `video: off` means the run is telemetry-only and the GCS log will
show `video_sent=0`, `chunks_last=0`, and `last_bytes=0`. Add `--video` for
visual debugging.

Debug video send FPS is independent from camera capture FPS. With no CLI
override, opt-in GCS video uses `debug_video.send_fps = 5`. Use `--fps 12` for
debug sessions where the GCS view should track the 12 FPS vision loop more
closely. Passing `--fps` enables debug video unless `--no-video` is also given,
which is treated as an option conflict.

Line mode guidance:

- `light_on_dark`: default for the current outdoor practice grid. Use when the
  track line is brighter than the floor.
- `auto`: tests both bright-line and dark-line masks and chooses the
  best line-shaped candidate.
- `dark_on_light`: use when the track line is darker than the floor.
- `--line-threshold 0`: use automatic Otsu thresholding. A positive value uses
  that fixed grayscale threshold for global threshold masks. With the default
  `white_fill` mask, `line.white_v_min`, `line.white_s_max`, and fill morphology
  are the primary tuning values.
- `--line-roi-top`: image ratio to ignore at the top. The default `0.08`
  keeps most of the upper frame visible; raise it only if the camera sees
  horizon or non-floor clutter.
- `--line-lookahead`: vertical image ratio for the green tracking point. The
  default `0.55` remains the telemetry lookahead reference. The current GCS
  overlay renders the red line-center point at the camera-center Y row and
  draws only the horizontal lateral error from the image center.

The line contour sent to GCS is the simplified connected contour selected by
the detector. The tracking X is calculated from a narrow anchor/lookahead band
inside that contour rather than from the whole contour centroid, so broad floor
blobs and cross-shaped contours are less likely to pull the control point away
from the actual line center. GCS intentionally displays that X as a red point on
the camera-center Y row with a green horizontal offset line for later lateral
control tuning.

Intersection classification is sent separately from the line tracking point.
The GCS overlay now keeps only compact cyan intersection type text and yellow
present-branch rays for the upper/current approach region, while the log shows
raw/stabilized type, branch scores, hold state, and classifier latency.
`straight` is reported as a valid type but does not set
`intersection_detected`, because it is not a turn/branching intersection.

`vision.intersection_decision` is a mission/debug layer above the stabilized
classifier. It uses a short branch-evidence window to accept topology, keeps
`T -> +` false upgrades guarded by `high_confidence_score`, records local grid
node events, and reports `front_available`, `turn_candidate`, `center_y_norm`,
`approach_phase`, and `overshoot_risk`. These fields are telemetry only; they
do not command the flight controller.

Latency and stability defaults are configured in `config/vision.toml`:

- `camera.sensor_model = "imx519"`: Pi 4 + IMX519-78 is the current default
  camera target.
- `camera.width = 960`, `camera.height = 720`, `camera.fps = 12`: capture keeps
  more marker pixels than the original Zero 2 W 640x480 baseline while keeping
  the detector loop conservative on Pi 4.
- `camera.jpeg_quality = 45`: MJPEG compression is intentionally moderate
  because the competition marker is large enough for detection at the planned
  2m altitude, and lower JPEG size reduces decode and optional UDP video cost.
- `camera.autofocus_mode = "manual"`, `camera.lens_position = 0.67`: initial
  mission-like focus default near 1.5m. For bench ArUco tests, override with
  `--lens-position <1/distance_m>` or edit `config/vision.toml`.
- `camera.exposure = "sport"`: initial setting to reduce motion blur.
- `line.process_width = 480`: line detection keeps more high-altitude line
  pixels than the previous 320px setting while still running on a resized ROI.
- `line.mask_strategy = "white_fill"`: detects wide bright/white line structures
  as filled blobs using low-saturation/high-value masking, then closes/dilates
  the mask so broad tape or paper is not reduced to thin edge strips.
- `line.white_v_min = 145`, `line.white_s_max = 90`: default HSV gates for the
  current white-line practice captures.
- `line.fill_close_kernel = 11`, `line.fill_dilate_kernel = 3`: white-fill
  morphology defaults used to connect broad line regions before contour
  selection.
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
- `line.intersection_threshold = 0.8`: minimum branch ray score used by the
  intersection classifier to decide whether a front/right/back/left branch is
  present.
- `intersection_decision.cruise_window_frames = 6`: short evidence window used
  to accept `L`/`T`/`+` node events without stopping for seconds at every node.
- `intersection_decision.min_branch_score = 0.72`: lower decision-layer branch
  evidence threshold; `intersection_decision.high_confidence_score = 0.85`
  prevents weak fourth-branch evidence from upgrading `T` to `+`.
- `intersection_decision.record_node_once_frames = 18`: lockout to avoid
  recording the same node repeatedly while the vehicle is still over it.
- `intersection_decision.turn_zone_y_min/max` and `late_zone_y`: image-space
  gating for turn readiness and overshoot risk telemetry.
- `debug_video.enabled = false`: GCS MJPEG streaming is disabled by default so
  normal onboard runs send metadata only.
- `debug_video.send_fps = 5`, `debug_video.chunk_pacing_us = 150`: when debug
  video is enabled for visual tuning, it is decimated and paced separately from
  the detector frame loop so GCS observation does not compete with mission work.
- The GCS vision log shows read/decode/ArUco/line/intersection/JSON/send/video timings,
  capture/processing FPS, video chunk/skip/failure counters, Pi board/OS/load/
  memory/throttling/Wi-Fi state, camera focus/exposure config, contour counts,
  queue drops, raw-vs-filtered line state, and raw-vs-stabilized intersection
  state.

For image-file detector smoke tests:

```bash
./build/aruco_detector_tester --config config --image test_data/images/marker.jpg
./build/line_detector_tuner --config config --image test_data/images/line_sample.jpg
./build/line_detector_tuner --config config --image test_data/images/line_sample.jpg --mode auto
./build/line_detector_tuner --config config --image test_data/images/line_sample.jpg --mask local_contrast --process-width 480 --band 0.06
./build/line_detector_tuner --config config --image test_data/images/line_sample.jpg --mask white_fill --white-v-min 95 --white-s-max 130 --fill-close 17 --output test_data/logs/line_sample
./build/line_detector_tuner --config config --image test_data/images/line_sample.jpg --local-threshold 10 --morph-open 1 --morph-close 7 --merge-gap 16
./build/line_detector_tuner --config config --image test_data/images/line_sample.jpg --mode light_on_dark --threshold 160 --roi-top 0.08 --lookahead 0.55
./build/grid_image_smoke --config config --image test_data/images/grid_sample.png --output test_data/logs/grid_smoke --scenario sample
```

When `--output <dir>` is passed, `line_detector_tuner` writes `mask_0.png`,
`line_overlay.png`, and `summary.txt` so tuning runs can be reviewed without
changing the input image.

`grid_image_smoke` writes `sections.csv`, `snake_full_field.csv`,
`snake_from_entry.csv`, `snake_from_entry_grid_log/*.txt`, and annotated crop
PNGs. The snake smoke path detects the grid entry side/row/column, rotates each
crop into the current camera heading before detection, and renders the same
incremental ASCII grid shape that GCS receives from `GridMapTracker`. This
matches the flight assumption that the drone yaws at turns and then continues
moving forward. Edge nodes use centered padded crops so image-boundary clipping
does not move the node away from the camera-frame center.

Expected GCS result:

- ArUco markers are overlaid by GCS as marker boxes, center points, direction
  arrows, and labels.
- Detected line contour/border is overlaid by GCS in magenta.
- The current line center is overlaid by GCS as a red point fixed on the
  camera-center Y row, with the lateral error shown as a green horizontal line.
- Detected intersection type is overlaid compactly in cyan, with present branch
  rays in yellow for the upper/current approach region.
- The GCS vision log appends `[grid-map]` ASCII output as local grid nodes are
  visited during snake exploration.
- If the opt-in onboard JPEG stream is enabled, it remains raw camera video with
  no drawn overlay.

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
4. On the Raspberry Pi, validate the camera with `rpicam-hello`, `rpicam-still`,
   and an MJPEG `rpicam-vid` smoke test.
5. On the laptop, start `uav_gcs_video`.
6. On the Raspberry Pi, run `video_streamer` and confirm it prints
   `discovered GCS video receiver at <ip>:5600`.
7. On the Raspberry Pi, run `uav_onboard` and confirm telemetry appears in GCS.
