#!/usr/bin/env python3
"""Cartesian Waypoint Tracker."""

from __future__ import annotations

import math
from collections import OrderedDict
from typing import Dict, List, Optional, Tuple

import numpy as np
import PyKDL as kdl

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from .kdl_builder import chain_from_urdf_string
from .trapezoidal_profile import TrapezoidalProfile
from .velocity_plotter import (
    plot_ee_trajectory,
    plot_ee_velocity,
    plot_joint_velocities,
)


# Constants

# Joint names in kinematic chain order
ALL_JOINT_NAMES: List[str] = [
    "lift_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Bridge topic template
_CMD_TOPIC = "/model/prismatic_ur10e/joint/{jname}/cmd_pos"

# Ready joint configuration
READY_JOINTS: List[float] = [
    0.0,                          # lift_joint
    math.pi / 2.0 - 0.3,         # shoulder_pan_joint   (~1.27 rad)
    -math.pi / 2.0,              # shoulder_lift_joint  (−90°)
    math.pi / 4.0,               # elbow_joint          (+45°)
    -math.pi / 2.0,              # wrist_1_joint        (−90°)
    -math.pi / 2.0,              # wrist_2_joint        (−90°)
    0.0,                          # wrist_3_joint
]

# Default waypoints (x, y, z)
DEFAULT_WAYPOINTS: List[Tuple[float, float, float]] = [
    # Low plane
    (0.60,  0.20, 0.45),
    (0.60, -0.20, 0.45),
    (0.80, -0.20, 0.45),
    (0.80,  0.20, 0.45),
    # Mid plane
    (0.60,  0.20, 0.75),
    (0.60, -0.20, 0.75),
    (0.80, -0.20, 0.75),
    (0.80,  0.20, 0.75),
    # High plane
    (0.70,  0.00, 1.05),
    (0.50,  0.20, 1.05),
    (0.50, -0.20, 1.05),
]

# Trapezoidal profile parameters
CART_V_MAX = 0.10       # m/s
CART_A_MAX = 0.15       # m/s²

# Control loop
CONTROL_HZ = 50
DT = 1.0 / CONTROL_HZ

# IK tuning
POSITION_GAIN = 1.5     # proportional gain for position error correction
MAX_CORRECTION_VEL = 0.15  # m/s — clamp on correction term magnitude
MAX_REVOLUTE_VEL = 1.0  # rad/s — clamp per revolute joint velocity
MAX_LIFT_VEL = 0.25     # m/s  — clamp for prismatic lift (URDF limit = 0.30)


class CartesianWaypointTracker(Node):
    """Drives a prismatic-lift + UR10e through Cartesian waypoints."""

    def __init__(self):
        super().__init__("cartesian_waypoint_tracker")

        # Parameters
        self.declare_parameter("cart_v_max", CART_V_MAX)
        self.declare_parameter("cart_a_max", CART_A_MAX)
        self.declare_parameter("output_dir", "/tmp/prismatic_plots")
        self.declare_parameter("waypoints_file", "")  # path to waypoints.yaml

        self._cart_v_max = self.get_parameter("cart_v_max").value
        self._cart_a_max = self.get_parameter("cart_a_max").value
        self._output_dir = self.get_parameter("output_dir").value
        _waypoints_file  = self.get_parameter("waypoints_file").value

        # URDF to KDL chain
        self._urdf_xml: Optional[str] = None
        self._chain: Optional[kdl.Chain] = None
        self._chain_joint_names: List[str] = []
        self._fk: Optional[kdl.ChainFkSolverPos_recursive] = None
        self._jac_solver: Optional[kdl.ChainJntToJacSolver] = None
        self._ik_vel: Optional[kdl.ChainIkSolverVel_pinv] = None

        self._rd_sub = self.create_subscription(
            String,
            "/robot_description",
            self._on_robot_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.get_logger().info("Waiting for /robot_description …")

        # Joint state subscriber
        self._current_positions: Dict[str, float] = {}
        self._js_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        # Joint command publishers
        self._cmd_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        for jname in ALL_JOINT_NAMES:
            topic = _CMD_TOPIC.format(jname=jname)
            self._cmd_pubs[jname] = self.create_publisher(Float64, topic, 10)

        # Recording buffers
        self._rec_timestamps: List[float] = []
        self._rec_joint_pos: Dict[str, List[float]] = {n: [] for n in ALL_JOINT_NAMES}
        self._rec_joint_vel: Dict[str, List[float]] = {n: [] for n in ALL_JOINT_NAMES}
        self._rec_ee_pos: Dict[str, List[float]] = {"x": [], "y": [], "z": []}
        self._rec_ee_vel: Dict[str, List[float]] = {
            "vx": [], "vy": [], "vz": [], "speed": [],
        }

        # State machine
        self._state = "WAIT_URDF"
        self._waypoints = self._load_waypoints(_waypoints_file)
        self._wp_idx = 0
        self._ready_start_pos: Optional[Dict[str, float]] = None

        # Segment state
        self._seg_start_xyz: Optional[np.ndarray] = None
        self._seg_target_xyz: Optional[np.ndarray] = None
        self._seg_direction: Optional[np.ndarray] = None
        self._seg_distance: float = 0.0
        self._seg_profile: Optional[TrapezoidalProfile] = None
        self._seg_t0 = None

        # Current commanded positions (tracked for integration)
        self._cmd_positions: Dict[str, float] = {}
        self._tick_count = 0

        # Main control timer
        self._timer = self.create_timer(DT, self._control_tick)

    # Callbacks

    def _on_robot_description(self, msg):
        if self._urdf_xml is not None:
            return
        self._urdf_xml = msg.data
        self.get_logger().info("Received robot_description — building KDL chain")
        self._chain, self._chain_joint_names = chain_from_urdf_string(
            self._urdf_xml, root_link="world", tip_link="tool0"
        )
        self._fk = kdl.ChainFkSolverPos_recursive(self._chain)
        self._jac_solver = kdl.ChainJntToJacSolver(self._chain)
        self._ik_vel = kdl.ChainIkSolverVel_pinv(self._chain)
        self.get_logger().info(
            f"KDL chain: {self._chain.getNrOfSegments()} segments, "
            f"{self._chain.getNrOfJoints()} joints: {self._chain_joint_names}"
        )

    def _on_joint_state(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in ALL_JOINT_NAMES or name in self._chain_joint_names:
                self._current_positions[name] = msg.position[i]

    # Waypoint loading

    def _load_waypoints(self, waypoints_file: str) -> List[Tuple[float, float, float]]:
        """Load waypoints from file."""
        if waypoints_file:
            try:
                import yaml
                with open(waypoints_file, "r") as fh:
                    cfg = yaml.safe_load(fh)
                raw = cfg.get("waypoints", [])
                if raw:
                    wps = [tuple(float(v) for v in w) for w in raw]
                    self.get_logger().info(
                        f"Loaded {len(wps)} waypoints from '{waypoints_file}'"
                    )
                    return wps
                self.get_logger().warn(
                    f"'waypoints' key missing or empty in '{waypoints_file}' "
                    "— falling back to DEFAULT_WAYPOINTS"
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to load waypoints from '{waypoints_file}': {exc} "
                    "— falling back to DEFAULT_WAYPOINTS"
                )
        else:
            self.get_logger().info(
                "No waypoints_file parameter set — using DEFAULT_WAYPOINTS"
            )
        return list(DEFAULT_WAYPOINTS)

    # FK and Jacobian helpers

    def _get_chain_jnt_array(self) -> Optional[kdl.JntArray]:
        """Build a KDL JntArray from current joint states in chain order."""
        if self._chain is None:
            return None
        n = self._chain.getNrOfJoints()
        q = kdl.JntArray(n)
        for i, name in enumerate(self._chain_joint_names):
            if name not in self._current_positions:
                return None
            q[i] = self._current_positions[name]
        return q

    def _fk_position(self, q: kdl.JntArray) -> kdl.Frame:
        """Forward kinematics — returns end-effector frame."""
        frame = kdl.Frame()
        self._fk.JntToCart(q, frame)
        return frame

    def _compute_jacobian(self, q: kdl.JntArray) -> np.ndarray:
        """6×N Jacobian at configuration *q*."""
        n = self._chain.getNrOfJoints()
        jac = kdl.Jacobian(n)
        self._jac_solver.JntToJac(q, jac)
        J = np.zeros((6, n))
        for i in range(6):
            for j in range(n):
                J[i, j] = jac[i, j]
        return J

    # Main control loop

    def _control_tick(self):
        if self._state == "WAIT_URDF":
            if self._chain is not None and len(self._current_positions) >= 7:
                self.get_logger().info(
                    "Chain + joint states ready — moving to READY pose"
                )
                self._state = "GO_READY"
                self._go_ready_start = self.get_clock().now()
                self._ready_start_pos = dict(self._current_positions)
            return

        if self._state == "GO_READY":
            self._move_to_ready()
            return

        if self._state == "TRACKING":
            self._track_waypoint()
            return

        # DONE — do nothing
        return

    # GO_READY

    def _move_to_ready(self):
        """Interpolate joints to READY pose."""
        elapsed = (self.get_clock().now() - self._go_ready_start).nanoseconds * 1e-9
        ramp_time = 5.0
        alpha = min(elapsed / ramp_time, 1.0)

        for i, jname in enumerate(ALL_JOINT_NAMES):
            start = self._ready_start_pos.get(jname, 0.0)
            goal = READY_JOINTS[i]
            cmd = start + alpha * (goal - start)
            self._publish_cmd(jname, cmd)
            self._cmd_positions[jname] = cmd

        if elapsed >= ramp_time + 2.0:
            # Settled — print EE position and start tracking
            q = self._get_chain_jnt_array()
            if q is not None:
                ee = self._fk_position(q)
                self.get_logger().info(
                    f"Ready pose achieved.  EE at "
                    f"({ee.p.x():.3f}, {ee.p.y():.3f}, {ee.p.z():.3f})"
                )
            self.get_logger().info(
                f"Starting waypoint tracking — {len(self._waypoints)} waypoints"
            )
            self._state = "TRACKING"
            self._start_next_segment()

    # TRACKING

    def _start_next_segment(self):
        """Prepare profile for next segment."""
        if self._wp_idx >= len(self._waypoints):
            self._finish()
            return

        target = self._waypoints[self._wp_idx]
        q = self._get_chain_jnt_array()
        if q is None:
            self.get_logger().warn("No joint states yet — retrying")
            return

        ee = self._fk_position(q)
        self._seg_start_xyz = np.array([ee.p.x(), ee.p.y(), ee.p.z()])
        self._seg_target_xyz = np.array(target)

        delta = self._seg_target_xyz - self._seg_start_xyz
        self._seg_distance = float(np.linalg.norm(delta))

        if self._seg_distance > 1e-4:
            self._seg_direction = delta / self._seg_distance
        else:
            self._seg_direction = np.zeros(3)

        self._seg_profile = TrapezoidalProfile(
            distance=self._seg_distance,
            v_max=self._cart_v_max,
            a_max=self._cart_a_max,
        )
        self._seg_t0 = self.get_clock().now()

        self.get_logger().info(
            f"WP {self._wp_idx + 1}/{len(self._waypoints)}: "
            f"({self._seg_start_xyz[0]:.3f}, {self._seg_start_xyz[1]:.3f}, "
            f"{self._seg_start_xyz[2]:.3f}) → "
            f"({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})  "
            f"d={self._seg_distance:.3f}m  t={self._seg_profile.t_total:.2f}s"
        )

    def _track_waypoint(self):
        """Resolved-rate control tick."""
        if self._seg_profile is None:
            return

        q = self._get_chain_jnt_array()
        if q is None:
            return

        elapsed = (self.get_clock().now() - self._seg_t0).nanoseconds * 1e-9

        # Segment complete check
        if elapsed >= self._seg_profile.t_total + 0.8:
            self._wp_idx += 1
            self._start_next_segment()
            return

        # Desired Cartesian state
        frac, _ = self._seg_profile.evaluate(elapsed)
        desired_pos = self._seg_start_xyz + frac * (
            self._seg_target_xyz - self._seg_start_xyz
        )
        desired_speed = self._seg_profile.velocity_at(elapsed)
        desired_vel = self._seg_direction * desired_speed   # 3-D

        # Position-error feedback
        ee = self._fk_position(q)
        actual_pos = np.array([ee.p.x(), ee.p.y(), ee.p.z()])
        pos_error = desired_pos - actual_pos
        correction = POSITION_GAIN * pos_error               # 3-D

        # Clamp correction magnitude
        corr_mag = float(np.linalg.norm(correction))
        if corr_mag > MAX_CORRECTION_VEL:
            correction = correction * (MAX_CORRECTION_VEL / corr_mag)

        # Combined linear velocity command
        v_cmd = desired_vel + correction

        # Build twist
        twist = kdl.Twist(
            kdl.Vector(v_cmd[0], v_cmd[1], v_cmd[2]),
            kdl.Vector(0.0, 0.0, 0.0),
        )

        # Pseudo-inverse IK
        n = self._chain.getNrOfJoints()
        qdot = kdl.JntArray(n)
        self._ik_vel.CartToJnt(q, twist, qdot)

        self._tick_count += 1
        if self._tick_count % 100 == 0:  # Log every 2 seconds at 50Hz
            v_list = [f"{self._chain_joint_names[i]}: {qdot[i]:.4f}" for i in range(n)]
            self.get_logger().info(f"IK velocities: {', '.join(v_list)}")

        # Clamp and integrate
        # Integrate from commanded positions
        for i in range(n):
            jname = self._chain_joint_names[i]
            vel = float(qdot[i])

            # Velocity limits
            if jname == "lift_joint":
                vel = max(-MAX_LIFT_VEL, min(MAX_LIFT_VEL, vel))
            else:
                vel = max(-MAX_REVOLUTE_VEL, min(MAX_REVOLUTE_VEL, vel))

            # Integrate
            last_cmd = self._cmd_positions.get(jname, float(q[i]))
            new_pos = last_cmd + vel * DT

            # Clamp lift to [0, 1]
            if jname == "lift_joint":
                new_pos = max(0.0, min(1.0, new_pos))

            self._publish_cmd(jname, new_pos)
            self._cmd_positions[jname] = new_pos

        # Record data
        self._record(q, qdot)

    # Data recording

    def _record(self, q: kdl.JntArray, qdot: kdl.JntArray):
        t_abs = self.get_clock().now().nanoseconds * 1e-9
        self._rec_timestamps.append(t_abs)

        n = self._chain.getNrOfJoints()
        for i, name in enumerate(self._chain_joint_names):
            if name in self._rec_joint_pos:
                self._rec_joint_pos[name].append(float(q[i]))
                self._rec_joint_vel[name].append(float(qdot[i]))

        # FK for end-effector position
        ee = self._fk_position(q)
        self._rec_ee_pos["x"].append(ee.p.x())
        self._rec_ee_pos["y"].append(ee.p.y())
        self._rec_ee_pos["z"].append(ee.p.z())

        # End-effector velocity via Jacobian × qdot
        J = self._compute_jacobian(q)
        qdot_np = np.array([float(qdot[i]) for i in range(n)])
        v_ee = J @ qdot_np  # 6-D twist
        self._rec_ee_vel["vx"].append(float(v_ee[0]))
        self._rec_ee_vel["vy"].append(float(v_ee[1]))
        self._rec_ee_vel["vz"].append(float(v_ee[2]))
        self._rec_ee_vel["speed"].append(float(np.linalg.norm(v_ee[:3])))

    # Publishing

    def _publish_cmd(self, joint_name: str, position: float):
        msg = Float64()
        msg.data = float(position)
        if joint_name in self._cmd_pubs:
            self._cmd_pubs[joint_name].publish(msg)

    # Completion and plotting

    def _finish(self):
        self._state = "DONE"
        self.get_logger().info(
            "\n═══════════════════════════════════════════════════\n"
            "  All waypoints reached — generating plots …\n"
            "═══════════════════════════════════════════════════"
        )

        try:
            p1 = plot_joint_velocities(
                self._rec_timestamps,
                self._rec_joint_vel,
                output_dir=self._output_dir,
            )
            self.get_logger().info(f"  ✓ {p1}")

            p2 = plot_ee_velocity(
                self._rec_timestamps,
                self._rec_ee_vel,
                output_dir=self._output_dir,
            )
            self.get_logger().info(f"  ✓ {p2}")

            p3 = plot_ee_trajectory(
                self._rec_ee_pos,
                self._waypoints,
                output_dir=self._output_dir,
            )
            self.get_logger().info(f"  ✓ {p3}")
        except Exception as e:
            self.get_logger().error(f"Plotting failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

        self.get_logger().info(
            f"\nPlots saved to {self._output_dir}/\n"
            "Node will remain alive for inspection.  Ctrl-C to exit."
        )
        self._timer.cancel()


# Entry point

def main(args=None):
    rclpy.init(args=args)
    node = CartesianWaypointTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
