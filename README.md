# uav-onboard

Onboard software for the Astroquad indoor/grid UAV search mission.

Current target hardware (2026-07 upgrade, ~2.1 kg TOW):

- Airframe: Holybro S500 V2 frame kit + companion mounting plate
- Flight controller: Holybro Pixhawk 6C Mini (ArduCopter)
- Companion: Raspberry Pi 5 (4 GB) with Active Cooler
- Camera: Raspberry Pi Global Shutter Camera (IMX296 mono, fixed-focus CS-mount)
- Optical flow / rangefinder: MTF-01 (MAVLink mode)
- Propulsion: Sunnysky X2212 980KV (CW/CCW) + Tarot 9450 folding props +
  GT-Drone 35A BLHeli_S OPTO ESCs
- Power: 4S LiPo (Pollyttronics PT-B8000-FX35, XT90S) + Matek BEC12S-PRO
- RC: ExpressLRS CRSF (BETAFPV Nano 2400 RX)

The previous platform (Raspberry Pi 4 + IMX519 + Pixhawk 1/fmuv2, 3S/1.5 kg)
is retired; configs referencing it are marked SUPERSEDED. The FC parameter
target for this build is `config/pixhawk6c_indoor_flow.params`.
The codebase now has three practical runtime layers:

- `vision_debug_node`: camera/vision/GCS telemetry bring-up.
- `line_follow_node`: short line-follow mission staging for SITL and guarded
  real ArduPilot serial tests.
- `astroquad-onboard`: current grid-arena snake mission runtime for SITL and
  guarded ArduPilot serial tests.

`grid_mission_node`, the byte-identical compatibility alias, was removed in
2026-07 — `astroquad-onboard` is the single mission runtime. The old
telemetry bring-up sender is now `uav-onboard-telem`.

## Layout

- `config/`: network, vision, mission, safety, runtime, ArduPilot parameter files
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
| `astroquad-onboard` | Current full grid-arena snake mission runtime. Supports SITL UDP and guarded ArduPilot serial paths. |
| `uav-onboard-telem` | Basic telemetry sender / development probe. |
| `vision_debug_node` | Camera source + ArUco/line/intersection processing + GCS telemetry + optional MJPEG video. |
| `line_follow_node` | Auto takeoff, short line follow, marker hover/landing staging. Supports SITL UDP and guarded ArduPilot serial paths. |
| `mavlink_probe` | No-arm MAVLink heartbeat/local estimate/RC/battery/parameter probe. |
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
sensor_model = "imx296"
width = 1456
height = 1088
fps = 12
jpeg_quality = 45
exposure = "sport"
shutter_us = 0    # lock at the venue (see vision.toml comments)
gain = 0.0

[debug_video]
enabled = false
send_fps = 0       # 0 = every processed frame; >0 caps the rate
jpeg_quality = 60
chunk_pacing_us = 150
send_width = 728   # downscale for the GCS link (0 = camera JPEG untouched)
send_height = 0    # 0 keeps the aspect ratio
```

Debug video is downscaled to 728x544 and re-encoded at `jpeg_quality` on the
sender's worker thread (roughly 2-3 Mbit/s at the full ~12 fps processing
rate); the vision/mission loop never pays for the resize or encode. GCS
overlays stay aligned because telemetry coordinates remain in camera pixel
space and the GCS scales them. Set `send_width`/`send_height` to 0 to forward
the full-resolution camera JPEG unchanged.

`send_fps = 0` forwards every processed frame (matches the vision rate). Set
a positive value, or pass `--fps <n>`, to cap the send rate for a constrained
link. The cap is clamped to the camera FPS (12), so `--fps 15` still sends 12.

The IMX296 is a mono global-shutter sensor with a fixed-focus CS-mount lens:
there are no autofocus/AWB controls, frames are decoded grayscale end-to-end,
and the lens choice sets the field of view — measure the mounted lens FOV
before tuning altitude- or pixel-width-dependent vision parameters.

Validate the camera path before running mission code:

```bash
rpicam-hello --version
rpicam-hello --list-cameras
rpicam-still -t 1000 --nopreview -o test_data/images/imx296_smoke.jpg
rpicam-vid -t 5000 --nopreview --codec mjpeg --width 1456 --height 1088 --framerate 12 -o /tmp/imx296_test.mjpeg
```

## Network Destinations (Known Hosts)

The Tailscale IPs are effectively static, so `config/network.toml` ships with
`gcs.ip = "gcs-laptop"` and no run command needs a literal IP. Known names
live in `src/common/KnownHosts.hpp` (kept byte-identical with the copy in
`uav-gcs`):

| Name | Address | Host |
|---|---|---|
| `gcs-laptop` | `100.85.239.73` | Lenovo GCS laptop (Tailscale) |
| `pi5` | `100.101.84.47` | Raspberry Pi 5 (Tailscale) |
| `broadcast` | `255.255.255.255` | LAN/WSL beacon discovery |

`--gcs-ip <ip|name>` accepts either a known name or a literal IP and
overrides the config. On a plain LAN without Tailscale, pass
`--gcs-ip broadcast` (or a literal IP) to restore beacon discovery.

Over Tailscale, both devices must be logged in (`tailscale status`) and the
Windows laptop needs the one-time firewall setup from
`uav-gcs/scripts/setup_windows_firewall.ps1` — Windows otherwise drops
inbound UDP on the Tailscale interface silently.

Telemetry larger than 1200 bytes is chunked (`AQT1`, protocol v1.11) so no
datagram exceeds the 1280-byte tunnel MTU; deploy onboard and GCS together
when updating across this protocol change.

## Astroquad GCS

Start `astroquad-gcs` on the laptop first (use `uav-gcs-video` only as a raw
transport smoke tool — it has no telemetry receiver and never draws
line/intersection/marker overlays), then run:

```bash
./build/vision_debug_node --config config --line-mode light_on_dark --video
./build/vision_debug_node --config config --line-only --line-mode light_on_dark
```

Onboard sends debug video only when `--video` is enabled. Marker boxes,
line contours, intersection labels, and grid-map text are drawn by GCS from
telemetry metadata.
`astroquad-onboard`, `line_follow_node`, and
`vision_debug_node` all accept `--fps <n>` to override the debug-video send
FPS; the publisher clamps it to the configured camera FPS (12), so
`--fps 15` silently becomes 12.

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
- `dark_on_light`: black/dark line on bright floor.
- `auto`: evaluate both polarities and pick the best line-shaped candidate.

## Gazebo SITL

Clone-only is not enough on a fresh WSL machine. The launchers also need:

- Gazebo Harmonic (`gz sim`, Gazebo Sim 8) and Gazebo dev libraries.
- ArduPilot SITL (`sim_vehicle.py`, MAVProxy, ArduCopter build tree).
- The `ardupilot_gazebo` plugin built locally and exposed through
  `GZ_SIM_SYSTEM_PLUGIN_PATH` / `GZ_SIM_RESOURCE_PATH`.
- Windows-side `uav-gcs` cloned, built, and running separately.

Fresh WSL setup:

```bash
cd ~/astroquad/uav-onboard
bash scripts/setup_gazebo_sitl.sh
source ~/.bashrc
```

The setup script installs Gazebo Harmonic and Linux build dependencies, clones
`~/ardupilot` and `~/ardupilot_gazebo` if they are missing, runs the ArduPilot
Ubuntu prerequisite installer, builds the Gazebo plugin, writes the required
environment variables to `~/.bashrc`, and builds `uav-onboard`.

Useful overrides:

```bash
ONBOARD_DIR=~/astroquad/uav-onboard \
ARDUPILOT_DIR=~/ardupilot \
ARDUPILOT_GAZEBO_DIR=~/ardupilot_gazebo \
BUILD_ONBOARD=1 \
bash scripts/setup_gazebo_sitl.sh
```

Set `UPDATE_EXISTING=1` only when you want the script to `git pull --ff-only`
existing `ardupilot` / `ardupilot_gazebo` checkouts.

Primary upstream references:

- Gazebo Ubuntu packages: <https://gazebosim.org/docs/latest/install_ubuntu/>
- ArduPilot SITL with Gazebo: <https://ardupilot.org/dev/docs/sitl-with-gazebo.html>
- ArduPilot Gazebo plugin: <https://github.com/ArduPilot/ardupilot_gazebo>

Run Windows GCS:

```powershell
cd astroquad\uav-gcs
cmake -S . -B build
cmake --build build --config Release
.\build\astroquad-gcs.exe --config config
```

If the generator places executables under `build\Release`, run:

```powershell
.\build\Release\astroquad-gcs.exe --config config
```

### Line-Tracing World

WSL/SITL flows keep an explicit `--gcs-ip` override because the Windows host
IP seen from WSL is not the Tailscale address in `config/network.toml`:

```bash
bash ~/astroquad/uav-onboard/scripts/line_tracing_test.sh

WINDOWS_GCS_IP="$(ip route | awk '/default/ {print $3; exit}')"
cd ~/astroquad/uav-onboard
./build/line_follow_node --config config --target sitl --vision gazebo \
  --line-mode light_on_dark --video --gcs-ip "$WINDOWS_GCS_IP"
```

### Grid Arena World

The grid arena is the current full-mission SITL course:

- 3m x 3m vertiport box at the origin.
- Vertiport ArUco marker ID 23.
- 5 x 8 grid cells, 3m cell size, white lines on grass.
- Four 50cm ArUco markers:
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
./build/astroquad-onboard \
  --config config \
  --target sitl \
  --vision gazebo \
  --world grid \
  --line-mode light_on_dark \
  --marker-count 4 \
  --revisit-order desc \
  --video \
  --gcs-ip "$WINDOWS_GCS_IP"
```

Descending marker revisit order is the default, so `--revisit-order desc` can
be omitted. Use this only when you want ascending order:

```bash
./build/astroquad-onboard --config config --target sitl --vision gazebo --world grid \
  --line-mode light_on_dark --marker-count 4 --revisit-order asc --video \
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
  -> REVISIT_INIT / REVISIT_FORWARD / REVISIT_MARKER_HOVER
  -> RETURN_HOME_INIT / RETURN_HOME_FORWARD / RETURN_HOME_ALIGN_ORIGIN
  -> RETURN_VERTIPORT_FORWARD / RETURN_VERTIPORT_MARKER_HOVER
  -> LAND
  -> MISSION_COMPLETE
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
- Fly forward with yaw locked while borrowing only lateral line-centering when
  a confident straight corridor is visible.
- At nodes, dwell briefly so branch evidence settles. Marker IDs are committed
  only after the sliding marker window sees the same non-vertiport ID often
  enough without mixed IDs.
- At boundaries, `SnakePlanner` latches the first valid turn direction and
  alternates left/right strictly. If the expected alternation branch is absent,
  the snake mission completes and lands rather than backtracking.

The current grid mission always revisits found grid markers after snake
completion. Omit `--revisit-order` for descending order, or pass
`--revisit-order asc` for ascending order. After revisit it returns to `(0,0)`,
faces grid south, flies back toward the latched vertiport marker, centers on
that marker, lands, and publishes mission completion telemetry.

## ArduPilot Serial Bench And Missions

No-arm local-estimate probe:

```bash
./build/mavlink_probe --config config --target ardupilot_serial \
  --duration-ms 12000 --strict-local-estimate
```

No-arm RC gate:

```bash
./build/mavlink_probe --config config --target ardupilot_serial \
  --duration-ms 30000 --strict-rc
```

No-arm line-follow serial smoke:

```bash
./build/line_follow_node --config config --target ardupilot_serial \
  --mavlink-smoke --smoke-duration-ms 5000 --no-telemetry
```

Props-removed motor command check:

```bash
./build/mavlink_motor_test --config config --target ardupilot_serial \
  --motor 1 --percent 5 --seconds 1 --props-removed
```

Line-follow default GUIDED + body-NED velocity path:

```bash
./build/line_follow_node --config config --target ardupilot_serial \
  --vision rpicam --line-mode light_on_dark --video \
  --control-backend guided_velocity \
  --allow-arm-takeoff
```

`guided_velocity` is the default, so `--control-backend guided_velocity` may be
omitted. This path requests GUIDED, arms, performs MAVLink takeoff, then sends
body-frame NED velocity setpoints. On real serial targets it requires RC input
and an optical-flow/range local estimate before arm/takeoff.

Line-follow only ALT_HOLD + RC override experiment path:

```bash
./build/line_follow_node --config config --target ardupilot_serial \
  --vision rpicam --line-mode light_on_dark --video \
  --control-backend alt_hold_rc_override --alt-hold-auto-takeoff \
  --allow-rc-override --allow-arm-takeoff \
  --unsafe-assume-rc-present
```

This backend is intentionally limited to `line_follow_node`. With
`--alt-hold-auto-takeoff`, it requests ALT_HOLD, arms, climbs with
`RC_CHANNELS_OVERRIDE` throttle, settles at neutral throttle, then sends small
roll/pitch/yaw override inputs for line following. The `[rc_override]`
marker servo settings are only used by this backend; they give marker centering
separate roll/pitch scaling so line following can stay gentle while marker
braking is stronger. On exit, takeover, or landing it releases the override.
Use this no-arm smoke first with props
removed and Mission Planner's radio view open:

```bash
./build/line_follow_node --config config --target ardupilot_serial \
  --rc-override-smoke --allow-rc-override
```

If the real RC receiver works through ArduPilot but MAVLink `RC_CHANNELS` is
not visible to onboard, `--unsafe-assume-rc-present` bypasses only the onboard
RC gate. ArduPilot pre-arm checks still apply, and the transmitter mode switch
must be verified to take over before using this flag.

For a short video-only line-follow sanity check, omit `--mavlink-smoke` and do
not pass `--alt-hold-auto-takeoff`; stop with Ctrl+C after confirming frames:

```bash
./build/line_follow_node --config config --target ardupilot_serial \
  --vision rpicam --line-mode light_on_dark --video --fps 12 \
  --control-backend alt_hold_rc_override \
  --allow-rc-override --unsafe-assume-rc-present
```

During `line_follow_node`, Ctrl+C/SIGTERM is handled as a safety land request:
the node releases RC override, requests LAND mode, and requests disarm only
after touchdown is likely. Use the transmitter takeover/disarm path first in an
emergency; Ctrl+C depends on the Raspberry Pi process and MAVLink link still
being alive.

Real grid-mission arming also requires explicit acknowledgement. The
`ardupilot_serial` defaults to `runtime.ardupilot_serial.toml` and `rpicam`; override the
serial endpoint with `--autopilot serial:///dev/serial0:115200` if needed:

```bash
./build/astroquad-onboard --config config --target ardupilot_serial \
  --line-mode light_on_dark --marker-count 4 --revisit-order asc \
  --video --allow-arm-takeoff
```

By default the downscaled stream is sent at the full processing rate
(~12 fps). Pass `--fps <n>` to cap it (e.g. `--fps 6`) on a constrained
link; video/telemetry stay strictly best-effort either way.

Do not run real serial mission paths until RC takeover, battery telemetry,
MTF-01 optical-flow/range local estimate, motor order, prop direction, and a
manual hover have been verified. Both real mission nodes require a fresh
MAVLink RC signal and optical-flow local estimate before arm/takeoff; use
`--unsafe-assume-rc-present` only for deliberate RC-gate debugging.

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

Start `uav-gcs-telem` on the laptop first:

```bash
./build/uav-onboard-telem --config config --count 10
./build/uav-onboard-telem --config config --gcs-ip 127.0.0.1 --count 10
```

The default GCS destination is the laptop's Tailscale address via the known
host name (see "Network Destinations"):

```toml
[gcs]
ip = "gcs-laptop"
telemetry_port = 14550
```

Use `--gcs-ip broadcast` for LAN beacon discovery, or `--gcs-ip <ip|name>`
for any other destination.
