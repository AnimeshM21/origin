"""Velocity Plotter."""

from __future__ import annotations

import os
from typing import Dict, List, Optional

import matplotlib
matplotlib.use("Agg")  # non-interactive backend — safe for headless
import matplotlib.pyplot as plt
import numpy as np


def _make_output_dir(directory: str) -> str:
    os.makedirs(directory, exist_ok=True)
    return directory


def plot_joint_velocities(
    timestamps: List[float],
    joint_velocities: Dict[str, List[float]],
    output_dir: str = "/tmp/prismatic_plots",
    filename: str = "joint_velocities.png",
) -> str:
    """Plot each joint's velocity over time and save to *output_dir*."""
    _make_output_dir(output_dir)
    t = np.array(timestamps) - timestamps[0]  # relative time

    fig, axes = plt.subplots(
        len(joint_velocities), 1,
        figsize=(12, 2.5 * len(joint_velocities)),
        sharex=True,
    )
    if len(joint_velocities) == 1:
        axes = [axes]

    colors = plt.cm.viridis(np.linspace(0.15, 0.85, len(joint_velocities)))

    for ax, (name, vel), color in zip(axes, joint_velocities.items(), colors):
        v = np.array(vel[: len(t)])
        ax.plot(t[: len(v)], v, color=color, linewidth=1.2)
        ax.set_ylabel(f"{name}\n[{'m/s' if 'lift' in name else 'rad/s'}]",
                       fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color="gray", linewidth=0.5)

    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("Joint Velocities", fontsize=14, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    path = os.path.join(output_dir, filename)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_ee_velocity(
    timestamps: List[float],
    ee_velocities: Dict[str, List[float]],
    output_dir: str = "/tmp/prismatic_plots",
    filename: str = "ee_velocities.png",
) -> str:
    """Plot end-effector Cartesian velocity components and magnitude."""
    _make_output_dir(output_dir)
    t = np.array(timestamps) - timestamps[0]

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    # Linear velocity components
    ax = axes[0]
    for key, color, label in [
        ("vx", "#e74c3c", "vx"),
        ("vy", "#2ecc71", "vy"),
        ("vz", "#3498db", "vz"),
    ]:
        if key in ee_velocities:
            v = np.array(ee_velocities[key][: len(t)])
            ax.plot(t[: len(v)], v, color=color, linewidth=1.2, label=label)
    ax.set_ylabel("Velocity [m/s]")
    ax.set_title("End-Effector Cartesian Velocity Components")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    # Speed magnitude
    ax = axes[1]
    if "speed" in ee_velocities:
        s = np.array(ee_velocities["speed"][: len(t)])
        ax.plot(t[: len(s)], s, color="#9b59b6", linewidth=1.5, label="|v|")
        ax.fill_between(t[: len(s)], 0, s, alpha=0.15, color="#9b59b6")
    ax.set_ylabel("Speed [m/s]")
    ax.set_xlabel("Time [s]")
    ax.set_title("End-Effector Speed (trapezoidal profile)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    fig.suptitle("End-Effector Velocities", fontsize=14, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    path = os.path.join(output_dir, filename)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_ee_trajectory(
    positions: Dict[str, List[float]],
    waypoints: Optional[List] = None,
    output_dir: str = "/tmp/prismatic_plots",
    filename: str = "ee_trajectory.png",
) -> str:
    """3-D scatter plot of the end-effector trajectory."""
    _make_output_dir(output_dir)
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    x = np.array(positions.get("x", []))
    y = np.array(positions.get("y", []))
    z = np.array(positions.get("z", []))

    ax.plot(x, y, z, color="#3498db", linewidth=1.5, label="Trajectory")

    if waypoints:
        wx = [w[0] for w in waypoints]
        wy = [w[1] for w in waypoints]
        wz = [w[2] for w in waypoints]
        ax.scatter(wx, wy, wz, color="#e74c3c", s=80, marker="^",
                   label="Waypoints", zorder=5)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("End-Effector 3-D Trajectory", fontsize=14, fontweight="bold")
    ax.legend()

    path = os.path.join(output_dir, filename)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path
