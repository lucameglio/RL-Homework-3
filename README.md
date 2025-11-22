# PX4 Drone Simulation with ROS2 and Gazebo

## Launching the Drone in Gazebo

1. Navigate to the PX4 Autopilot folder inside your ROS2 workspace:

```bash
cd ros2_ws/src/PX4-Autopilot
```

2. Build and launch the custom Gazebo quadrotor:

```bash
make px4_sitl gz_custom_quad
```

3. In another terminal, run the DDS bridge:

```bash
cd ros2_ws/src
./DDS_run.sh
```

4. Open QGroundControl to connect to the simulated drone.

## Enabling Auto-Landing

To enable automatic landing via ROS2:

```bash
ros2 run force_land force_land
```

## Enabling Offboard Control

To enable offboard control mode via ROS2:

```bash
ros2 run px4_ros_com offboard_control
```

## Executing a Trajectory

To execute the predefined or user-defined trajectory using the offboard RL node:

```bash
ros2 run offboard_rl trajectory
```

* When prompted, you can either enter custom waypoints manually or press Enter to use the default trajectory.

## Notes

* Make sure all ROS2 nodes are running in the same workspace.
* For keyboard input of custom waypoints, follow the format:

```
x y z yaw; x y z yaw; ...
```

* The drone uses the NED coordinate frame (z-axis is negative for altitude).

