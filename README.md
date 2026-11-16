# Omron MoMa Gazebo simulations

ROS 2 Jazzy / Gazebo Harmonic simulations for the TM12x on the LD250. The
workspace is split into `omron_moma_custom` for the soft two-finger gripper and
`omron_moma_onrobot` for OnRobot RG2, RG6, and VGC10 tools. Both reuse the AMR
simulation worlds and the combined MoMa description. Static previews and
experimental fixed-base and mobile-base modes are available.

## Build

With `omron_amr_simulation`, `omron_amr`, `omron_moma`, `tmr_ros2`, both
`grasping_omron_moma_custom` and `grasping_omron_moma_onrobot`, `grasping`,
`grippers`, `onrobot_grippers`, and `mobile_manipulator` in the workspace and
their ROS dependencies installed:

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to omron_moma_custom omron_moma_onrobot
```

## Run

```bash
source install/setup.bash
ros2 launch omron_moma_custom simulation.launch.py
```

The `gripper` argument selects the tool configuration:

```bash
source install/setup.bash
ros2 launch omron_moma_custom simulation.launch.py gripper:=soft_two_fingers
```

The default is `gripper:=none`. The soft two-finger configuration reuses the
grasping stack's combined description, including the tool mount and camera.

To preview the MoMa in the Gazebo Fuel warehouse instead of the default empty
world:

```bash
ros2 launch omron_moma_custom simulation.launch.py gripper:=soft_two_fingers world:=warehouse
```

The warehouse layout is local; the first launch downloads its models from Gazebo
Fuel. Set `x`, `y`, `z` (meters) and `yaw` (radians) to choose the robot's spawn
pose. Three open racks near `(-2.7, 0)` hold 20 movable colored boxes. Their
positions are fixed by default; use `randomize_boxes:=true` to shuffle them,
and optionally `box_seed:=42` for a repeatable layout. See the AMR simulation
README for object details and world attribution.

For a separate grasp-testing world with five tables and dynamic objects:

```bash
ros2 launch omron_moma_custom simulation.launch.py gripper:=soft_two_fingers world:=grasp_test
```

Each table holds a cube, cylinder, sphere, block, and bottle. Set
`randomize_objects:=true` to shuffle their positions on each table, and
optionally `object_seed:=42` for a repeatable layout. The static robot and
gripper cannot pick them up yet. See the AMR simulation README for object details.

Following are the supported gripper configurations and their corresponding descriptions(external package dependencies)

| Gripper Configuration | Description | External Package Dependencies |
|---------------------|-------------|-------------------------------|
| none                | No gripper attached | None |
| soft_two_fingers    | Soft two-finger gripper with tool mount and camera | `grasping_omron_moma_custom`, `grasping`, `grippers` |

## OnRobot simulation

The separate `omron_moma_onrobot` package supports `none`, `rg2`, `rg6`, and
`vgc10`. RG2 and RG6 use the OnRobot Gazebo descriptions, position controller,
and simulated `GripperCommand` action server. VGC10 uses contact sensors and
Gazebo detachable joints to model grasping and release for objects included in
the `grasp_test` and `warehouse` worlds. This approximates vacuum grasping; it
does not model vacuum pressure, seal quality, payload limits, or leaks.

```bash
ros2 launch omron_moma_onrobot simulation.launch.py gripper:=rg2 fixed_base_control:=true
ros2 launch omron_moma_onrobot simulation.launch.py gripper:=rg6 mobile_base_control:=true
ros2 launch omron_moma_onrobot simulation.launch.py gripper:=vgc10 fixed_base_control:=true world:=grasp_test
ros2 launch omron_moma_onrobot moveit.launch.py gripper:=vgc10
```

The RG action servers are available at `/onrobot_rg2/gripper_command` or
`/onrobot_rg6/gripper_command`. VGC10 exposes `/onrobot_vg/command` using
`onrobot_msgs/action/VGCommand`; suction goals are rejected unless a supported
world object is currently touching a VGC10 cup. Object attachment is an
experimental fixed-joint approximation, not a calibrated suction model.
Do not run the physical OnRobot drivers against these simulated action topics.
OnRobot mobile bringup is available with
`ros2 launch omron_moma_onrobot mobile_manipulator.launch.py gripper:=rg2`; it
does not currently start the custom package's mobile-manipulator task server.


In the default static preview, the gripper and camera do not simulate actuation
or sensor output. Future configurations can be added to the
gripper-to-description mapping in the launch file.

Pass `gui:=false` for a headless server. This launches only Gazebo, the clock
bridge, and description/state publishers; it does not start the physical arm or
base drivers, MoveIt, or Nav2. See the AMR simulation repository for the base
model and interface assessment.

## Experimental fixed-base control

```bash
ros2 launch omron_moma_custom simulation.launch.py world:=grasp_test gripper:=soft_two_fingers fixed_base_control:=true
```

This mode anchors the base and starts `gz_ros2_control`, a joint-state
broadcaster, and the arm trajectory controller at
`/tmr_arm_with_base_controller/follow_joint_trajectory`. The base cannot drive,
and sensors are disabled unless explicitly enabled. Use Gazebo joint states as
the single authoritative source; do not run the fake TM driver or hardware
drivers against the same joints.

For `soft_two_fingers`, the simulation also exposes `/gripper_command`. It only
accepts position goals with `max_effort: 0`. Fixed-base control uses a single
actuated slider and a mimic joint with the Bullet Featherstone physics backend.
The simulated gripper model and action adapter are provided by `gripper_gazebo`.
Repeated open and close goals succeeded without an object. Arm and gripper
inertias, collision geometry, controller limits, contact, and grasping still need
validation before relying on physical results.

## Experimental mobile-base control

```bash
ros2 launch omron_moma_custom simulation.launch.py mobile_base_control:=true
```

This starts the arm controller and the AMR simulation's `cmd_vel`, `stop`, `odom`
and `odom -> base_link` interfaces. It cannot be combined with
`fixed_base_control:=true`. The default static preview and fixed-base mode remain
unchanged. See the AMR simulation README for command limits, timeout behavior,
approximate wheel geometry and safety limitations. Arm and base commands have
been tested separately, not as coordinated mobile manipulation. Mobile mode uses
DART and independently controlled finger joints. A 1 mm simulated joint-limit
margin avoids a sticking finger; direct joint commands can exceed the nominal
physical limits. Unloaded opening and closing have been tested before and after
driving, but object contact and lifting remain unverified.

To enable both LD250 lidar scans and the simulated dock:

```bash
ros2 launch omron_moma_custom simulation.launch.py mobile_base_control:=true gripper:=soft_two_fingers lidar:=true docking:=true
```

The simulated dock provides `/undock_robot` and a limited `/dock_robot` action.
The AMR simulation README documents action examples, dock placement, supported
goals and limitations. Scans are simulated; charging, dock sensing and hardware
safety interlocks are not.

## Experimental wrist camera

```bash
ros2 launch omron_moma_custom simulation.launch.py gripper:=soft_two_fingers fixed_base_control:=true camera:=true world:=grasp_test
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

## Experimental MoveIt and mobile manipulation

```bash
ros2 launch omron_moma_custom moveit.launch.py gui:=false
ros2 launch omron_moma_custom mobile_manipulator.launch.py gui:=false
```

Run these commands separately. The first starts a fixed-base simulation and
MoveIt; use `mobile:=true` for a mobile base, or `gripper:=none` for the bare
arm. Use `start_sim:=false` only with a matching simulator already running.
The second starts the mobile robot, lidar, SLAM, Nav2, MoveIt, scene manager,
arm motion and mobile-manipulator task services. It defaults to the warehouse;
`world`, `gui`, `camera`, `docking`, and config-file arguments can be changed.
It does not start physical drivers or automatically send a task. The optional
`run_task_client:=true` starts the existing configured test client, which may
command motion; `task_mode` defaults to `arm_only_ik`.

An arm joint target was planned and executed through MoveIt in simulation.
Coordinated mobile manipulation is not yet functional: the existing task server
can time out while waiting for its Cartesian feasibility backend. The backend
accepts `arm_only_ik` and `arm_only_plan`, not `mobile_manipulator`. Short Nav2
goals succeeded for the simulated AMR; see the AMR simulation README for lidar
and SLAM limitations. The MoveIt planning scene does not automatically
represent Gazebo world objects. The simulated gripper is not yet reliable for
grasping, and none of these interfaces are a hardware safety system.

## Arm simulation viability

`tm_use_simulation:=true` in the existing TM bringup selects the driver's fake
robot, not a Gazebo backend. When built with MoveIt support, that fake driver
executes trajectory points in its own state and publishes joint states. It is
useful for testing ROS-side planning and interfaces without hardware, but it
does not move the Gazebo model or simulate contacts. Without MoveIt support, the
driver's simulation behavior differs, so check the built driver configuration.

The combined Xacro includes `libgazebo_ros_control.so`, a ROS 1 Gazebo Classic
plugin, and legacy transmissions. The simulation launch removes the plugin and
uses Jazzy-compatible `gz_ros2_control` in the controlled modes. Coordinated
mobile manipulation remains unvalidated.

The package maintainer field is a placeholder to be decided before publishing the
repository.