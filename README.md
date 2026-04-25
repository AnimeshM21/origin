# Cartesian Motion with Vertical Lift

## Objective

Simulate and control smooth Cartesian motions of a 6-DOF robotic arm (UR10e)
mounted on a vertical prismatic joint (a lift), enabling it to reach points in
a 3D workspace.  Ensure constant-velocity Cartesian motion on a 2D plane at
varying heights, using trapezoidal velocity profiles.

## Architecture

```
src/
├── ur_description/          # Official UR10e robot description (vendored)
└── prismatic_7dof_sim/      # Custom ROS 2 ament_python package
    ├── config/
    │   └── waypoints.yaml           # XYZ waypoints at 3 Z-heights
    ├── launch/
    │   ├── sim.launch.py            # Gazebo + bridge + RViz
    │   └── cartesian_demo.launch.py # Adds controller node on top
    ├── model/
    │   └── prismatic_ur10e.urdf.xacro
    ├── prismatic_7dof_sim/          # Python package
    │   ├── kdl_builder.py           # URDF → KDL chain (replaces kdl_parser_py)
    │   ├── trapezoidal_profile.py   # Bang-cruise-bang velocity profile
    │   ├── cartesian_waypoint_tracker.py  # Main controller node
    │   └── velocity_plotter.py            # matplotlib plot generation
    ├── rviz/
    │   └── view_robot.rviz
    └── worlds/
        └── arm_world.sdf            # Floor + compliance wall
```

## Approach

### 1 — URDF: Prismatic Lift + UR10e

The URDF attaches a vertical prismatic joint (0 – 1.0 m range, Z-axis) between
the world and the UR10e base.  The kinematic chain is:

```
world → lift_base → [prismatic lift_joint] → lift_carriage →
  lift_mount → UR10e (6 revolute joints) → tool0
```

This gives 7 actuated degrees of freedom.

Each joint has a `JointPositionController` Ignition plugin that exposes a
`cmd_pos` topic for position-level control from ROS 2.

### 2 — Simulation (ROS 2 Humble + Ignition Fortress)

- **Gazebo Sim**: `JointStatePublisher` publishes joint states;
  `JointPositionController` accepts position commands per joint.
- **ros_gz_bridge**: Bidirectional for joint states (Gazebo → ROS 2) and
  unidirectional for position commands (ROS 2 → Gazebo) on 7 `cmd_pos` topics.
- **RViz 2**: Visualises the robot model and TF tree in real time.

### 3 — Cartesian Waypoint Tracking

A ROS 2 Python node (`cartesian_waypoint_tracker`) drives the arm through
a list of (x, y, z) waypoints:

1. **KDL chain** is built from the URDF XML at runtime using a custom builder
   (`kdl_builder.py`), since `kdl_parser_py` is not available as a pip package.
2. **Trapezoidal velocity profile** is computed for each segment in Cartesian
   space — smooth acceleration → constant velocity → smooth deceleration.
3. **Resolved-rate IK**: At each 50 Hz control tick:
   - The trapezoidal profile yields the desired scalar speed along the segment.
   - A proportional position-error feedback term corrects drift.
   - The combined 3-D Cartesian velocity is mapped to joint velocities via
     the Jacobian pseudo-inverse (`ChainIkSolverVel_pinv`).
   - The lift joint naturally handles Z-height changes because its Jacobian
     column is `[0, 0, 1, 0, 0, 0]ᵀ` (pure Z translation).
4. Joint velocities are integrated and published as position commands at 50 Hz.

### 4 — Velocity Plotting

After all waypoints are reached, the node generates three matplotlib plots:

- `joint_velocities.png` — velocity of each of the 7 joints over time
- `ee_velocities.png` — end-effector vx, vy, vz components and scalar speed
  (should show clear trapezoidal profiles)
- `ee_trajectory.png` — 3-D plot of the end-effector path with waypoint markers

All plots are saved to the configured output directory (default: `/tmp/prismatic_plots/`).

## Quick Start

```bash
# 1. Build
source /opt/ros/humble/setup.bash
cd ~/Desktop/origin
colcon build --symlink-install
source install/setup.bash

# 2. Run the full demo (simulation + controller + plotting)
ros2 launch prismatic_7dof_sim cartesian_demo.launch.py

# 3. Or run sim and controller separately:
ros2 launch prismatic_7dof_sim sim.launch.py
# In another terminal:
ros2 run prismatic_7dof_sim cartesian_waypoint_tracker

# 4. View the generated plots
ls /tmp/prismatic_plots/
eog /tmp/prismatic_plots/ee_velocities.png
```

## Dependencies

| Package | Purpose |
|---------|---------|
| ROS 2 Humble | Middleware |
| Ignition Fortress | Physics simulation |
| ros_gz_bridge | Gazebo ↔ ROS 2 topic bridging |
| PyKDL | Kinematics (FK, Jacobian, IK) |
| urdf_parser_py | URDF XML parsing |
| matplotlib | Velocity plots |
| numpy | Numerical computation |

## Configuration

Edit `config/waypoints.yaml` to change waypoints or velocity profile parameters.

Key parameters:
- `cart_v_max` — maximum end-effector Cartesian speed (m/s)
- `cart_a_max` — acceleration / deceleration (m/s²)
