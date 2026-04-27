import os

from setuptools import setup

package_name = "prismatic_7dof_sim"

def package_data_files(*directories):
    data_files = []
    for directory in directories:
        for path, dirnames, filenames in os.walk(directory):
            dirnames[:] = [dirname for dirname in dirnames if dirname != "__pycache__"]
            files = [
                os.path.join(path, filename)
                for filename in filenames
                if not filename.endswith((".pyc", ".pyo"))
            ]
            if files:
                data_files.append((os.path.join("share", package_name, path), files))
    return data_files

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ]
    + package_data_files("launch", "model", "rviz", "worlds", "config"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Animesh Mishra",
    maintainer_email="animeshmishra211@gmail.com",
    description="Simulation environment for a UR10e mounted on a vertical prismatic lift.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cartesian_waypoint_tracker = prismatic_7dof_sim.cartesian_waypoint_tracker:main",
        ],
    },
)
