"""Build a PyKDL Chain from a URDF XML string."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import PyKDL as kdl
from urdf_parser_py.urdf import URDF


# Public API

def chain_from_urdf_string(
    urdf_xml: str,
    root_link: str = "world",
    tip_link: str = "tool0",
) -> Tuple[kdl.Chain, List[str]]:
    """Return (kdl_chain, joint_names) spanning root_link to tip_link."""
    robot = URDF.from_xml_string(urdf_xml)

    # Build a quick child-lookup:  parent_link  →  [(joint, child_link), …]
    children: dict[str, list] = {}
    for jnt in robot.joints:
        children.setdefault(jnt.parent, []).append(jnt)

    # Walk from root_link to tip_link (BFS / DFS path search)
    path = _find_path(children, root_link, tip_link)
    if path is None:
        raise ValueError(
            f"No kinematic path from '{root_link}' to '{tip_link}' in the URDF."
        )

    chain = kdl.Chain()
    joint_names: List[str] = []

    for jnt in path:
        kdl_joint, kdl_frame = _urdf_joint_to_kdl(jnt)
        segment = kdl.Segment(jnt.child, kdl_joint, kdl_frame)
        chain.addSegment(segment)
        if jnt.type != "fixed":
            joint_names.append(jnt.name)

    return chain, joint_names


# Internal helpers

def _find_path(children, start, goal, visited=None):
    """DFS from *start* to *goal*, returning the list of joints on the path."""
    if visited is None:
        visited = set()
    visited.add(start)
    for jnt in children.get(start, []):
        if jnt.child == goal:
            return [jnt]
        if jnt.child not in visited:
            sub = _find_path(children, jnt.child, goal, visited)
            if sub is not None:
                return [jnt] + sub
    return None


def _urdf_joint_to_kdl(jnt) -> Tuple[kdl.Joint, kdl.Frame]:
    """Convert a single ``urdf_parser_py`` joint into KDL Joint + Frame."""
    # Origin frame
    origin = jnt.origin
    if origin is not None:
        xyz = origin.xyz or [0.0, 0.0, 0.0]
        rpy = origin.rpy or [0.0, 0.0, 0.0]
    else:
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]

    frame = kdl.Frame(
        kdl.Rotation.RPY(rpy[0], rpy[1], rpy[2]),
        kdl.Vector(xyz[0], xyz[1], xyz[2]),
    )

    # Joint
    axis = jnt.axis if jnt.axis is not None else [0.0, 0.0, 1.0]
    axis_kdl = kdl.Vector(axis[0], axis[1], axis[2])

    if jnt.type == "fixed":
        kdl_joint = kdl.Joint(jnt.name, kdl.Joint.Fixed)
    elif jnt.type == "revolute" or jnt.type == "continuous":
        kdl_joint = kdl.Joint(
            jnt.name,
            kdl.Vector(0, 0, 0),  # origin in segment frame
            axis_kdl,             # rotation axis
            kdl.Joint.RotAxis,
        )
    elif jnt.type == "prismatic":
        kdl_joint = kdl.Joint(
            jnt.name,
            kdl.Vector(0, 0, 0),
            axis_kdl,
            kdl.Joint.TransAxis,
        )
    else:
        raise ValueError(f"Unsupported URDF joint type: {jnt.type}")

    return kdl_joint, frame
