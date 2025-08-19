# mm_bringup/launch/sim.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

#  Use these modules on Humble
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution,TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- robot_description (URDF from xacro) ---
    xacro_exec = FindExecutable(name="xacro")
    robot_xacro = PathJoinSubstitution([FindPackageShare("mm_description"), "urdf", "mm_robot.xacro"])

    robot_description = {"robot_description": ParameterValue(Command([xacro_exec, TextSubstitution(text=" "), robot_xacro,TextSubstitution(text=" "),TextSubstitution(text="use_ros2_control:=false")]),  value_type=str,)}

    # --- robot_state_publisher ---
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # --- Gazebo (ros_gz_sim) ---
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items(),
    )

    # --- Spawn the robot from /robot_description ---
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", "mm", "-topic", "robot_description"],
    )

    # --- Controller spawners ---
    cfg_dir = PathJoinSubstitution([FindPackageShare("mm_bringup"), "config"])

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", PathJoinSubstitution([cfg_dir, "jsb.yaml"]),
        ],
    )

    spawner_base = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "base_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", PathJoinSubstitution([cfg_dir, "base_controller.yaml"]),
        ],
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", PathJoinSubstitution([cfg_dir, "arm_controller.yaml"]),
        ],
    )

    return LaunchDescription([gz_launch, rsp, spawn, spawner_jsb, spawner_base, spawner_arm])
