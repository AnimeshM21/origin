import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from prismatic_7dof_sim import (
    build_gazebo_resource_path,
    get_model_file,
    get_rviz_file,
    get_world_file,
)


def generate_launch_description():
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    gz_args = LaunchConfiguration("gz_args")
    use_rviz = LaunchConfiguration("use_rviz")
    ur_type = LaunchConfiguration("ur_type")

    resource_paths = build_gazebo_resource_path()
    world_file = str(get_world_file())
    rviz_file = str(get_rviz_file())
    xacro_file = str(get_model_file())

    robot_description = {
        "robot_description": Command(["xacro", " ", xacro_file, " ", "ur_type:=", ur_type])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name",
                    "prismatic_ur10e",
                    "-topic",
                    "robot_description",
                ],
                output="screen",
            )
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/arm_world/model/prismatic_ur10e/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            # Joint position command bridges  (ROS 2 → Gazebo, unidirectional)
            "/model/prismatic_ur10e/joint/lift_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/model/prismatic_ur10e/joint/shoulder_pan_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/model/prismatic_ur10e/joint/shoulder_lift_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/model/prismatic_ur10e/joint/elbow_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/model/prismatic_ur10e/joint/wrist_1_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/model/prismatic_ur10e/joint/wrist_2_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/model/prismatic_ur10e/joint/wrist_3_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
        ],
        remappings=[
            (
                "/world/arm_world/model/prismatic_ur10e/joint_state",
                "joint_states",
            ),
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("ur_type", default_value="ur10e"),
            DeclareLaunchArgument("gz_args", default_value=f"-r {world_file}"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_paths),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_paths),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            bridge,
            rviz,
        ]
    )
