# Underwater Simulation

ROS2 Humble + Gazebo tunnel environment for underwater robot simulation.

The tunnel model is centered at the world origin, runs along the x axis, is 30 m long, and has a 5 m inner diameter. A robot spawned at `(0, 0, 0)` starts inside the tunnel and can move along `+X`.

The circular tunnel wall is built from 64 box segments instead of a hollow cylinder. Each segment has matching `visual` and `collision` geometry, with 0.3 m wall thickness and a concrete material. This keeps the collision model useful for later DVL, sonar, camera, and ray-based simulation.

Six dim spot lights are placed near the tunnel roof at 5 m spacing. A reference ground plane sits below the tunnel and does not intrude into the internal motion space.

The world also spawns a robot model at the origin:

- body: 1.0 m long, 0.25 m diameter cylinder, `base_link` at the geometric center
- frame convention: `+X` forward, `+Y` left, `+Z` up
- sensors: `camera_left_link`, `camera_right_link`, `imu_link`, `mag_link`, `depth_link`, `dvl_link`, `sonar_up_link`, `sonar_down_link`, `sonar_left_link`, `sonar_right_link`
- visual mesh: `robot_visual_link`, fixed to `base_link`, used only for appearance; visible thrusters come from the BlueROV mesh

The Gazebo model is in `models/pipe_robot/model.sdf`. The matching ROS description is in `urdf/pipe_robot.urdf.xacro`.

The model intentionally separates appearance from physics:

- `robot_visual_link` contains only a visual mesh.
- `base_link` keeps the simplified 1.0 m x 0.25 m body collision and inertial values.
- no separate simplified thruster links are added because the current BlueROV mesh already includes thruster geometry.
- no collision or inertial values are generated from high-poly meshes.

## BlueROV2 Heavy visual mesh

The preferred external appearance is a BlueROV2 Heavy CAD/mesh or an existing BlueROV2 Gazebo mesh. Keep it visual-only so SLAM, sensor frames, and GraphManager interfaces do not change.

Suggested sources:

- Blue Robotics BlueROV2 / Heavy configuration product page: https://bluerobotics.com/store/rov/bluerov2/
- Existing BlueROV2 Gazebo/ROS model files: https://github.com/patrickelectric/bluerov_ros_playground/tree/master/model
- Direct candidate mesh paths in that repository:
  - `model/BlueRov2.dae`
  - `model/BlueRov2.stl`

Place the downloaded mesh here:

```text
underwater_simulation/models/pipe_robot/meshes/bluerov2/BlueRov2.dae
```

Then update `models/pipe_robot/model.sdf`:

```xml
<uri>model://pipe_robot/meshes/bluerov2/BlueRov2.dae</uri>
<scale>...</scale>
```

For the ROS/Xacro description, override these arguments as needed:

```text
mesh_filename:=package://underwater_simulation/models/pipe_robot/meshes/bluerov2/BlueRov2.dae
mesh_scale:=...
mesh_xyz:=...
mesh_rpy:=...
```

Adjust `mesh_rpy` so the vehicle forward direction is ROS `+X`, and adjust `mesh_scale` so the overall length is close to 1.0 m. The bundled `meshes/rov_visual.obj` stays as a fallback because the external BlueROV2 mesh may not be present on a fresh checkout; this prevents Gazebo from failing to open.

The launch file starts Gazebo classic through the `gazebo` executable and does not require the ROS `gazebo_ros` package. If launch prints that `gazebo` is not found, install/source Gazebo classic first.

```bash
cd /home/xiaoyang/water_robot/auv_ws
colcon build --packages-select underwater_simulation
source install/setup.bash
ros2 launch underwater_simulation tunnel_world.launch.py
```

Package layout:

```text
underwater_simulation/
|-- worlds/
|   `-- tunnel.world
|-- models/
|   `-- tunnel/
|       |-- model.config
|       `-- model.sdf
|   `-- pipe_robot/
|       |-- model.config
|       |-- meshes/
|       |   `-- rov_visual.obj
|       `-- model.sdf
|-- urdf/
|   `-- pipe_robot.urdf.xacro
`-- launch/
    `-- tunnel_world.launch.py
```
