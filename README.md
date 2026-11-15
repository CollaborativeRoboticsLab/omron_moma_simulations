# Omron MoMa Gazebo simulation

ROS 2 Jazzy / Gazebo Harmonic starting point for the TM12x on the LD250. This
package reuses the AMR simulation worlds and the existing combined
`moma_description` Xacro. The default launch is a **static geometry preview**.
An experimental fixed-base arm simulation is available separately.
An experimental mobile-base mode combines the drive simulation with arm control.

## Build

With `omron_amr_simulation`, `omron_amr`, `omron_moma`, `tmr_ros2`,
`grasping_omron_moma`, `grasping`, and `grippers` in the workspace and their ROS
dependencies installed:

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to omron_moma_simulation
```

## Run

```bash
source install/setup.bash
ros2 launch omron_moma_simulation preview.launch.py
```

The `gripper` argument selects the tool configuration:

```bash
source install/setup.bash
ros2 launch omron_moma_simulation preview.launch.py gripper:=soft_two_fingers
```

The default is `gripper:=none`. The soft two-finger configuration reuses the
grasping stack's combined description, including the tool mount and camera.

To preview the MoMa in the Gazebo Fuel warehouse instead of the default empty
world:

```bash
ros2 launch omron_moma_simulation preview.launch.py gripper:=soft_two_fingers world:=warehouse
```

The warehouse layout is local; the first launch downloads its models from Gazebo
Fuel. Set `x`, `y`, `z` (meters) and `yaw` (radians) to choose the robot's spawn
pose. Three open racks near `(-2.7, 0)` hold 20 movable colored boxes. Their
positions are fixed by default; use `randomize_boxes:=true` to shuffle them,
and optionally `box_seed:=42` for a repeatable layout. See the AMR simulation
README for object details and world attribution.

For a separate grasp-testing world with five tables and dynamic objects:

```bash
ros2 launch omron_moma_simulation preview.launch.py gripper:=soft_two_fingers world:=grasp_test
```

Each table holds a cube, cylinder, sphere, block, and bottle. Set
`randomize_objects:=true` to shuffle their positions on each table, and
optionally `object_seed:=42` for a repeatable layout. The static robot and
gripper cannot pick them up yet. See the AMR simulation README for object details.

Following are the supported gripper configurations and their corresponding descriptions(external package dependencies)

| Gripper Configuration | Description | External Package Dependencies |
|---------------------|-------------|-------------------------------|
| none                | No gripper attached | None |
| soft_two_fingers    | Soft two-finger gripper with tool mount and camera | `grasping_omron_moma`, `grasping`, `grippers` |


In the default static preview, the gripper and camera do not simulate actuation
or sensor output. Future configurations can be added to the
gripper-to-description mapping in the launch file.

Pass `gui:=false` for a headless server. This launches only Gazebo, the clock
bridge, and description/state publishers; it does not start the physical arm or
base drivers, MoveIt, or Nav2. See the AMR simulation repository for the base
model and interface assessment.

## Experimental fixed-base control

```bash
ros2 launch omron_moma_simulation preview.launch.py world:=grasp_test gripper:=soft_two_fingers fixed_base_control:=true
```

This mode anchors the base and starts `gz_ros2_control`, a joint-state
broadcaster, and the arm trajectory controller at
`/tmr_arm_with_base_controller/follow_joint_trajectory`. The base cannot drive,
and sensors are disabled unless explicitly enabled. Use Gazebo joint states as
the single authoritative source; do not run the fake TM driver or hardware
drivers against the same joints.

For `soft_two_fingers`, the simulation also exposes `/gripper_command`. It only
accepts position goals with `max_effort: 0`. One simulated finger currently
remains at its limit, so nonzero-width commands can abort and grasping is not
yet supported. Arm and gripper inertias, collision geometry, controller limits,
and contact behavior still need validation before relying on physical results.

## Experimental mobile-base control

```bash
ros2 launch omron_moma_simulation preview.launch.py mobile_base_control:=true
```

This starts the arm controller and the AMR simulation's `cmd_vel`, `stop`, `odom`
and `odom -> base_link` interfaces. It cannot be combined with
`fixed_base_control:=true`. The default static preview and fixed-base mode remain
unchanged. See the AMR simulation README for command limits, timeout behavior,
approximate wheel geometry and safety limitations. Arm and base commands have
been tested separately, not as coordinated mobile manipulation. Sensors, Nav2
and MoveIt integration remain untested, and the soft gripper limitation above
also applies to this mode.

To enable both LD250 lidar scans and the simulated dock:

```bash
ros2 launch omron_moma_simulation preview.launch.py mobile_base_control:=true gripper:=soft_two_fingers lidar:=true docking:=true
```

The simulated dock provides `/undock_robot` and a limited `/dock_robot` action.
The AMR simulation README documents action examples, dock placement, supported
goals and limitations. Scans are simulated; charging, dock sensing and hardware
safety interlocks are not.

## Experimental wrist camera

```bash
ros2 launch omron_moma_simulation preview.launch.py gripper:=soft_two_fingers fixed_base_control:=true camera:=true world:=grasp_test
```

`camera:=true` also works with `mobile_base_control:=true` and may be combined
with `lidar:=true`. The camera is attached to the D435 wrist mount and publishes:

- `/camera/camera/color/image_raw` and `/camera/camera/color/camera_info`
- `/camera/camera/depth/image_rect_raw` and `/camera/camera/depth/camera_info`
- `/camera/camera/depth/points`

Color images use `rgb8`; depth images use `32FC1` in meters. Images and camera
info use the corresponding `camera_color_optical_frame` and
`camera_depth_optical_frame`. The point cloud uses `camera_depth_frame`, whose
axes are forward, left, up. The color image topic matches the default vision
driver configuration.

The 640x480, 15 Hz cameras use approximate fields of view and range limits, not
device calibration. Depth is not aligned to color; infrared, IMU, device control
and realistic noise are not simulated. Gazebo's rendering system must
work in the environment, including for headless use. Do not start the physical
camera driver on the same topics when using the simulation.

## Arm simulation viability

`tm_use_simulation:=true` in the existing TM bringup selects the driver's fake
robot, not a Gazebo backend. When built with MoveIt support, that fake driver
executes trajectory points in its own state and publishes joint states. It is
useful for testing ROS-side planning and interfaces without hardware, but it
does not move the Gazebo model or simulate contacts. Without MoveIt support, the
driver's simulation behavior differs, so check the built driver configuration.

The combined Xacro includes `libgazebo_ros_control.so`, a ROS 1 Gazebo Classic
plugin, and legacy transmissions. The simulation launch removes the plugin and
uses Jazzy-compatible `gz_ros2_control` in the controlled modes. MoveIt, Nav2
and coordinated mobile manipulation remain untested.

The package maintainer field is a placeholder to be decided before publishing the
repository.