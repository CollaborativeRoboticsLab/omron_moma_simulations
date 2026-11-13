# Omron MoMa Gazebo simulation

ROS 2 Jazzy / Gazebo Harmonic starting point for the TM12x on the LD250. This
package reuses the AMR simulation world and the existing combined
`moma_description` Xacro. The current launch is a **static geometry preview**:
neither the base nor the arm is simulated or controllable.

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


It is a static preview: the gripper joints and camera do not yet simulate
actuation, contact, or sensor output. Future configurations can be added to the
gripper-to-description mapping in the launch file.

Pass `gui:=false` for a headless server. This launches only Gazebo, the clock
bridge, and description/state publishers; it does not start the physical arm or
base drivers, MoveIt, or Nav2. See the AMR simulation repository for the base
model and interface assessment.

## Arm simulation viability

`tm_use_simulation:=true` in the existing TM bringup selects the driver's fake
robot, not a Gazebo backend. When built with MoveIt support, that fake driver
executes trajectory points in its own state and publishes joint states. It is
useful for testing ROS-side planning and interfaces without hardware, but it
does not move the Gazebo model or simulate contacts. Without MoveIt support, the
driver's simulation behavior differs, so check the built driver configuration.

The combined Xacro includes `libgazebo_ros_control.so`, a ROS 1 Gazebo Classic
plugin, and legacy transmissions. The preview removes that plugin and makes the
model static. For physical arm simulation, replace it in a simulation-specific
description with Jazzy-compatible `gz_ros2_control`, configure a joint trajectory
controller and joint state broadcaster, and ensure the action name and joint
names match MoveIt. Use Gazebo joint states as the single authoritative source;
do not run the fake TM driver and a Gazebo controller against the same joints.
Review arm and mounting collision geometry, inertias, self-collision, controller
limits and the base/arm frame tree before enabling dynamics. Integration with
the simulated base, sensors and Nav2 remains to be implemented and tested.

The package maintainer field is a placeholder to be decided before publishing the
repository.