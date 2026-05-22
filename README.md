# uav-onboard

Onboard software for the Astroquad indoor/grid UAV search mission.

Current target hardware is Raspberry Pi 4 + IMX519 CSI camera + Pixhawk1.
The codebase now has three practical runtime layers:

- `vision_debug_node`: camera/vision/GCS telemetry bring-up.
- `line_follow_node`: short line-follow mission staging for SITL and guarded
  real Pixhawk1 tests.
- `grid_mission_node`: current grid-arena snake mission staging for SITL.

The final product target is still `uav_onboard`, but today that executable is a
basic telemetry bring-up sender. Mission development should keep sharing the
libraries already factored out of the tools instead of copying detector code
between executables.

## Layout

- `config/`: network, vision, mission, safety, runtime, Pixhawk parameter files
- `src/`: reusable onboard libraries
- `tools/`: staging executables and smoke tools
- `tests/`: CTest targets
- `scripts/`: dependency and SITL launcher helpers
- `sim/gazebo/`: Astroquad-owned Gazebo worlds/models/textures
- `docs/`: protocol reference
- `logs/`, `test_data/`: runtime/captured artifacts

## Main Executables

| Executable | Role |
|---|---|
| `uav_onboard` | Basic telemetry sender; final composition root target. |
| `vision_debug_node` | Camera source + ArUco/line/intersection processing + GCS telemetry + optional MJPEG video. |
| `line_follow_node` | Auto takeoff, short line follow, marker hover/landing staging. Supports SITL UDP and guarded Pixhawk1 serial paths. |
| `grid_mission_node` | Current full grid-arena snake mission staging. SITL is the active arm/takeoff target. |
| `mavlink_probe` | No-arm Pixhawk heartbeat/local estimate/RC/battery/parameter probe. |
| `mavlink_motor_test` | Props-removed low-throttle motor command check. |
| `video_streamer` | Raw MJPEG transport smoke tool. |
| `line_detector_tuner`, `aruco_detector_tester`, `grid_image_smoke`, `marker_grid_replay` | Offline vision/regression tools. |

## Build

On Raspberry Pi OS Lite 64-bit:

```bash
bash scripts/setup_rpi_dependencies.sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

MAVLink headers are required for the autopilot tools. CMake first looks for
ArduPilot-generated headers and then downloads MAVLink `c_library_v2` into the
build tree when needed. Offline builds can pass:

```bash
cmake -S . -B build -DMAVLINK_INCLUDE_DIR=/path/to/include
```

## Camera Bring-Up

Current camera defaults live in `config/vision.toml`:

```toml
[camera]
sensor_model = "imx519"
width = 960
height = 720
fps = 12
mode = "2328:1748:10:P"
jpeg_quality = 45
focus_absolute = 1984
focus_device = "/dev/v4l-subdev1"
exposure = "sport"

[debug_video]
enabled = false
send_fps = 5
jpeg_quality = 40
chunk_pacing_us = 150
```

Validate the camera path before running mission code:

```bash
rpicam-hello --version
rpicam-hello --list-cameras
rpicam-still -t 1000 --nopreview -o test_data/images/imx519_smoke.jpg
rpicam-vid -t 5000 --nopreview --codec mjpeg --width 960 --height 720 --framerate 12 -o /tmp/imx519_test.mjpeg
```

The current IMX519 setup uses the V4L2 focus motor path
`camera.focus_absolute`/`camera.focus_device`; `--lens-position` may be ignored
if libcamera reports no AF algorithm for the module.

## Vision Debug

Start `uav_gcs_vision_debug` on the laptop first, then run:

```bash
./build/vision_debug_node --config config --line-only --line-mode light_on_dark
./build/vision_debug_node --config config --line-only --line-mode dark_on_light --video --fps 12
```

Onboard sends raw camera JPEG only when `--video` is enabled. Marker boxes,
line contours, intersection labels, and grid-map text are drawn by GCS from
telemetry metadata.

Current vision path:

- ArUco marker detection with stabilizer/hold support.
- Shared `LineMaskBuilder` for `white_fill`, `dark_fill`, and local-contrast
  strategies.
- Line detector chooses ranked contours, then measures tracking X from an
  anchor/lookahead projection band rather than the whole contour centroid.
- Marker-like square occlusion masks reduce false line candidates around
  ArUco markers when bright-line modes are active.
- Intersection detector/classifier reports `straight`, `L`, `T`, and `+`
  branch evidence.
- `IntersectionDecisionEngine` keeps a short evidence window, emits
  `node_record`/`turn_ready` decisions, and guards weak `T -> +` upgrades.

Line mode guide:

- `light_on_dark`: bright/white line on darker floor.
- `dark_on_light`: black/dark line on bright floor or Gazebo grid arena.
- `auto`: evaluate both polarities and pick the best line-shaped candidate.

## Gazebo SITL

Run Windows GCS:

```powershell
cd astroquad\uav-gcs
.\build\uav_gcs_vision_debug.exe --config config
```

### Line-Tracing World

```bash
bash ~/astroquad/uav-onboard/scripts/line_tracing_test.sh

WINDOWS_GCS_IP="$(ip route | awk '/default/ {print $3; exit}')"
cd ~/astroquad/uav-onboard
./build/line_follow_node --config config --target sitl --vision gazebo \
  --line-mode dark_on_light --video --gcs-ip "$WINDOWS_GCS_IP"
```

### Grid Arena World

The grid arena is the current full-mission SITL course:

- 3m x 3m vertiport box at the origin.
- Vertiport ArUco marker ID 23.
- 5 x 8 grid cells, 3m cell size, black lines on white ground.
- Four 50cm ArUco markers on white pads:
  ID 1 at `(11.5, 3.0)`, ID 2 at `(17.5, 9.0)`,
  ID 3 at `(23.5, 6.0)`, ID 4 at `(26.5, 12.0)`.

Launcher:

```bash
bash ~/astroquad/uav-onboard/scripts/grid_arena_test.sh
```

Current grid mission command:

```bash
WINDOWS_GCS_IP="$(ip route | awk '/default/ {print $3; exit}')"

cd ~/astroquad/uav-onboard
./build/grid_mission_node \
  --config config \
  --target sitl \
  --vision gazebo \
  --world grid \
  --line-mode dark_on_light \
  --marker-count 4 \
  --revisit-order desc\
  --video \
  --gcs-ip "$WINDOWS_GCS_IP"
```

`--world grid` selects `config/runtime.sitl.grid.toml`, including the grid
arena Gazebo camera topic. CLI `--gazebo-topic` still overrides it.

Grid mission state flow:

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
  -> REVISIT_INIT / REVISIT_FORWARD / REVISIT_MARKER_HOVER (optional)
  -> LAND
  -> DONE
```

Any active state can enter `EMERGENCY_LAND` on heartbeat loss, mission timeout,
altitude ceiling, max-intersection, hop-distance, entry timeout, or abort.

Current snake strategy:

- Take off over the vertiport.
- Hold the vertiport marker centered while yawing right by `+90 deg`.
- Fly yaw-frozen `EntryForward`; the new grid arena has no line from the
  vertiport to the first grid intersection.
- After the first L/T/+ is seen, enter `EntryCenterOrigin` and center that
  intersection before publishing local `(0,0)`.
- Between nodes, use LOCAL_NED distance from the last committed node as the
  hop reference.
- Run line following only in a short mid-cell align window
  (`hop_align_start_m` to `hop_align_end_m`); otherwise fly forward with yaw
  locked.
- At nodes, dwell briefly so branch evidence settles. Marker IDs are committed
  only after the sliding marker window sees the same non-vertiport ID often
  enough without mixed IDs.
- At boundaries, `SnakePlanner` latches the first valid turn direction and
  alternates left/right strictly. If the expected alternation branch is absent,
  the snake mission completes and lands rather than backtracking.

The current grid mission can revisit found grid markers after snake completion
with `--revisit-order asc` or `--revisit-order desc`. Use `--revisit-order none`
to keep the older behaviour and land after the snake completes. Official
coordinate conversion and return-to-start are still future work.

## Pixhawk1 Bench And Line-Follow

No-arm local-estimate probe:

```bash
./build/mavlink_probe --config config --target pixhawk1 \
  --duration-ms 12000 --strict-local-estimate
```

No-arm RC gate:

```bash
./build/mavlink_probe --config config --target pixhawk1 \
  --duration-ms 30000 --strict-rc
```

No-arm line-follow serial smoke:

```bash
./build/line_follow_node --config config --target pixhawk1 \
  --mavlink-smoke --smoke-duration-ms 5000 --no-telemetry
```

Props-removed motor command check:

```bash
./build/mavlink_motor_test --config config --target pixhawk1 \
  --motor 1 --percent 5 --seconds 1 --props-removed
```

Real line-follow arming requires explicit acknowledgement:

```bash
./build/line_follow_node --config config --target pixhawk1 \
  --vision rpicam --line-mode dark_on_light --video --allow-arm-takeoff
```

Do not run real serial mission paths until RC takeover, battery telemetry,
MTF-01 optical-flow/range local estimate, motor order, prop direction, and a
manual hover have been verified. `grid_mission_node --target pixhawk1` is not
armed for real grid mission flight yet; use `--no-arm` only for smoke.

## Offline Vision Tools

```bash
./build/aruco_detector_tester --config config --image test_data/images/marker.jpg
./build/line_detector_tuner --config config --image test_data/images/line_sample.jpg --mode auto
./build/line_detector_tuner --config config --image test_data/images/line_sample.jpg --mask white_fill --output test_data/logs/line_sample
./build/grid_image_smoke --config config --image test_data/images/grid_sample.png --output test_data/logs/grid_smoke --scenario sample
```

`grid_image_smoke` writes CSVs, annotated crops, and incremental ASCII grid logs
that match the GCS local-grid assumption: the drone yaws at turns and then keeps
moving forward in camera frame.

## Telemetry Bring-Up

Start `uav_gcs` on the laptop first:

```bash
./build/uav_onboard --config config --count 10
./build/uav_onboard --config config --gcs-ip 127.0.0.1 --count 10
```

The default GCS IP is broadcast:

```toml
[gcs]
ip = "255.255.255.255"
telemetry_port = 14550
```

Some networks block local broadcast; use `--gcs-ip <laptop-ip>` when needed.
