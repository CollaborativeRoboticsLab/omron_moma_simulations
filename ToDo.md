# Simulation To Do

## Separate gripper simulations

- [x] Define the package and launch boundaries for the custom soft two-finger
	gripper and the OnRobot RG2, RG6, and VGC10; share common MoMa simulation
	components without duplicating tool-specific configuration.
- [x] Inventory each gripper's existing descriptions, mounts, MoveIt configs,
	controllers, interfaces, and dependencies; identify missing Gazebo support.
- [x] Separate the existing soft two-finger configuration into the custom
	simulation package, preserving static, fixed-base, and mobile-base modes.
- [x] Add an RG2 Gazebo configuration with the correct model, mount, joint
	limits, control interface, and MoveIt integration.
- [x] Add an RG6 Gazebo configuration with the correct model, mount, joint
	limits, control interface, and MoveIt integration.
- [x] Add a VGC10 Gazebo configuration with the correct model, mount, vacuum
	control interface, and simulated attachment and release behavior.
- [ ] Configure tool-specific collision geometry, inertias, contacts, and
	controller parameters; calibrate against the physical tools where possible.
- [x] Update simulation, MoveIt, and mobile-manipulator launches and package
	dependencies to select the correct gripper and reject unsupported options.
- [x] Add automated tests for each gripper's description, launch selection,
	controller interfaces, and MoveIt configuration, including regression tests
	for the existing soft two-finger and no-gripper modes.
- [ ] Validate opening and closing or vacuum activation and release, object
	contact, grasping and lifting in fixed and mobile modes for each gripper.
- [x] Update build and launch documentation with per-gripper examples,
	dependencies, supported modes, and known simulation limitations.

## Existing simulation work

- [ ] Validate gripper contact and lifting, and calibrate the mobile DART
	joint-limit workaround; unloaded opening and closing work in fixed and mobile
	modes, but grasping remains unverified.
- [ ] Fix the mobile-manipulator task server's feasibility-backend timeout and
	validate coordinated arm and base tasks; the backend currently accepts
	`arm_only_ik` and `arm_only_plan`, not `mobile_manipulator`.
- [ ] Validate lidar filtering against close external obstacles, costmaps,
	longer Nav2 routes, and the combined MoMa lidar frames.
- [ ] Synchronize Gazebo objects with the MoveIt planning scene.
- [ ] Calibrate simulated masses, inertias, contacts, sensors, and controller
	limits; validate behavior against the physical robot.
- [ ] Improve the wrist camera model and calibration, including depth/color
	alignment, noise, and currently unsupported device features.
- [ ] Expand dock simulation beyond pre-aligned motion to include sensing,
	charging, and safety interlocks. Simulation is not a hardware safety system.
