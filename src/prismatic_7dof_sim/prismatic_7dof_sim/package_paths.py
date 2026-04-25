import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = "prismatic_7dof_sim"


def get_share_directory() -> Path:
    return Path(get_package_share_directory(PACKAGE_NAME))


def get_model_file() -> Path:
    return get_share_directory() / "model" / "prismatic_ur10e.urdf.xacro"


def get_rviz_file() -> Path:
    return get_share_directory() / "rviz" / "view_robot.rviz"


def get_world_file() -> Path:
    return get_share_directory() / "worlds" / "arm_world.sdf"


def build_gazebo_resource_path() -> str:
    resource_roots = [os.environ.get("GZ_SIM_RESOURCE_PATH", "")]
    for package_name in (PACKAGE_NAME, "ur_description"):
        share_directory = Path(get_package_share_directory(package_name))
        resource_roots.append(str(share_directory.parent))
    return os.pathsep.join(path for path in resource_roots if path)
