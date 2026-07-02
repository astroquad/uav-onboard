# Astroquad Gazebo Assets

This directory contains Astroquad-owned Gazebo worlds, models, and textures for
vision, line-follow, and grid-mission SITL testing.

## Line-Tracing Test World

World:

```text
worlds/line_tracing_test_world.sdf
```

Launcher:

```bash
bash ~/astroquad/uav-onboard/scripts/line_tracing_test.sh
```

The line-tracing world uses `iris_with_downward_camera` and
`line_tracing_course`. It provides a grass-textured ground plane, a 10cm white
line, and 50cm ArUco marker textures for detector and line-follow smoke tests.

Default camera topic:

```text
/world/line_tracing_test_world/model/iris_with_downward_camera/link/downward_camera_link/sensor/downward_camera/image
```

## Grid Arena Test World

World:

```text
worlds/grid_arena_test_world.sdf
```

Launcher:

```bash
bash ~/astroquad/uav-onboard/scripts/grid_arena_test.sh
```

The grid arena is the current full mission SITL course:

- 3m x 3m x 0.7m vertiport box centered at world origin.
- Vertiport top texture with ArUco ID 23.
- 5 x 8 grid cells, 3m cell size, white 10cm grid lines on grass.
- 50cm ArUco grid markers:
  ID 1 at world `(11.5, 3.0)`, ID 2 at `(17.5, 9.0)`,
  ID 3 at `(23.5, 6.0)`, ID 4 at `(26.5, 12.0)`.
- Iris starts on the vertiport top with yaw 90deg.

Default grid camera topic from `config/runtime.sitl.grid.toml`:

```text
/world/grid_arena_test_world/model/iris_with_downward_camera/link/downward_camera_link/sensor/downward_camera/image
```

Current mission command:

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
  --video \
  --gcs-ip "$WINDOWS_GCS_IP"
```

`--world grid` loads the grid runtime profile and normally removes the need for
manual `--gazebo-topic`. Use `--gazebo-topic` only when testing a custom world
or fixture.

## Static Vision Fixtures

Detector-only fixtures without ArduPilot SITL:

```text
worlds/astroquad_line_camera_fixture.sdf
worlds/astroquad_marker_center_fixture.sdf
```

Fixture topics:

```text
/world/astroquad_line_camera_fixture/model/astroquad_static_downward_camera/link/downward_camera_link/sensor/downward_camera/image
/world/astroquad_marker_center_fixture/model/astroquad_static_downward_camera/link/downward_camera_link/sensor/downward_camera/image
```

Example:

```bash
GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}" \
GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$PWD/sim/gazebo/models:$PWD/sim/gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}" \
  gz sim -s -v2 -r sim/gazebo/worlds/astroquad_marker_center_fixture.sdf

./build/vision_debug_node --config config --target sitl --vision gazebo \
  --gazebo-topic /world/astroquad_marker_center_fixture/model/astroquad_static_downward_camera/link/downward_camera_link/sensor/downward_camera/image \
  --count 5 --no-telemetry --no-video
```

## Models And Textures

| Path | Role |
|---|---|
| `models/iris_with_downward_camera` | Wrapper around ArduPilot Iris with downward camera. |
| `models/line_tracing_course` | Line-tracing smoke course and marker textures. |
| `models/astroquad_grid_course` | Current vertiport + grid arena course. |
| `models/astroquad_static_downward_camera` | Fixed camera model for deterministic detector fixtures. |

Gazebo downward-camera zoom is controlled in:

```text
models/iris_with_downward_camera/model.sdf
```

Change `<horizontal_fov>`: smaller values zoom in, larger values zoom out. FOV
edits require restarting Gazebo but not rebuilding C++.
