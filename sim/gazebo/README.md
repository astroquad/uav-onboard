# Astroquad Gazebo Vision World

This directory contains Astroquad-owned Gazebo assets for the current SITL vision step.

Run from `uav-onboard` with ArduPilot Gazebo models and Astroquad models on the resource path:

```bash
GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}" \
GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$PWD/sim/gazebo/models:$PWD/sim/gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}" \
  gz sim -v4 -r sim/gazebo/worlds/astroquad_iris_vision.sdf
```

The main world uses the local `iris_with_downward_camera` wrapper around the existing ArduPilot Iris model and includes `astroquad_vision_course`, which provides a dark ground plane, a high-contrast 10 cm white line, and a 50 cm x 50 cm ArUco ID 1 marker texture at 3 m forward on the line. The default Gazebo camera topic is configured in `config/vision.toml`.

`worlds/astroquad_line_camera_fixture.sdf` and `worlds/astroquad_marker_center_fixture.sdf` are detector smoke fixtures. They use `astroquad_static_downward_camera` at 1.2 m AGL so line and marker vision can be checked without starting ArduPilot SITL. The marker fixture uses the same 50 cm x 50 cm ID 1 marker texture as the flight course.

The active ID 1 texture is `models/astroquad_vision_course/materials/textures/aruco_id1.png`. The user-provided `/home/mseoky/test_aruco_marker/aruco2.png` was verified as OpenCV `DICT_4X4_50` ID 1 and copied into that canonical texture name.
