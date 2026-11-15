# Simulation To Do

- [ ] Fix the stuck `gripper_planar_4` finger and validate repeated opening,
	closing, object contact, and lifting. Grasping is currently unreliable.
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
