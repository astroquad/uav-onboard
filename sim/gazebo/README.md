# Astroquad Gazebo Vision World

This directory contains Astroquad-owned Gazebo assets for the current SITL vision step.

Run from `uav-onboard` with ArduPilot Gazebo models and Astroquad models on the resource path:

```bash
GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}" \
GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$PWD/sim/gazebo/models:$PWD/sim/gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}" \
  gz sim -v4 -r sim/gazebo/worlds/line_tracing_test_world.sdf
```

The line-tracing test world uses the local `iris_with_downward_camera` wrapper around the existing ArduPilot Iris model and includes `line_tracing_course`, which provides a white ground plane, a high-contrast 10 cm black line, and a 50 cm x 50 cm ArUco ID 1 marker texture at 3 m forward on the line. The default Gazebo camera topic is configured in `config/vision.toml`.

`worlds/astroquad_line_camera_fixture.sdf` and `worlds/astroquad_marker_center_fixture.sdf` are detector smoke fixtures. They use `astroquad_static_downward_camera` at 1.2 m AGL so line and marker vision can be checked without starting ArduPilot SITL. The marker fixture uses the same 50 cm x 50 cm ID 1 marker texture as the flight course.

The active ID 1 texture is `models/line_tracing_course/materials/textures/aruco_id1.png`. The user-provided `/home/mseoky/test_aruco_marker/aruco2.png` was verified as OpenCV `DICT_4X4_50` ID 1 and copied into that canonical texture name.

Fixture topics:

```text
/world/astroquad_line_camera_fixture/model/astroquad_static_downward_camera/link/downward_camera_link/sensor/downward_camera/image
/world/astroquad_marker_center_fixture/model/astroquad_static_downward_camera/link/downward_camera_link/sensor/downward_camera/image
```

The original `/home/mseoky/test_aruco_marker` files are kept as raw assets. Their verified OpenCV IDs are:

```text
aruco2.png -> DICT_4X4_50 ID 1
aruco3.png -> DICT_4X4_50 ID 2
aruco4.png -> DICT_4X4_50 ID 3
aruco1.png -> DICT_4X4_50 ID 4
Vmarker.png -> future vertiport visual, not an ArUco fixture marker
```
