# mm_moveit/launch/bringup.launch.py
import os, yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# We need Command + FindExecutable + TextSubstitution to call xacro correctly
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- Build URDF string from your xacro ---
    xacro = FindExecutable(name="xacro")                  # resolves to /opt/ros/humble/bin/xacro
    robot_xacro = PathJoinSubstitution(
        [FindPackageShare("mm_description"), "urdf", "mm_robot.xacro"]
    )
    # IMPORTANT: insert an explicit space token between 'xacro' and the path
    robot_description_content = Command([xacro, TextSubstitution(text=" "), robot_xacro])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # --- Build SRDF string (from your mm.srdf.xacro) ---
    srdf_xacro = PathJoinSubstitution(
        [FindPackageShare("mm_moveit"), "config", "mm.srdf.xacro"]
    )
    robot_description_semantic_content = Command([xacro, TextSubstitution(text=" "), srdf_xacro])
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # --- Kinematics for group "manipulator" ---
    mm_share = get_package_share_directory("mm_moveit")
    with open(os.path.join(mm_share, "config", "kinematics.yaml"), "r") as f:
        robot_description_kinematics = {"robot_description_kinematics": yaml.safe_load(f)}

    # --- MoveIt controller plugin & mapping ---
    moveit_controller_manager = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    }
    with open(os.path.join(mm_share, "config", "moveit_controllers.yaml"), "r") as f:
        moveit_simple_params = {
            "moveit_simple_controller_manager": {"ros__parameters": yaml.safe_load(f)}
        }

    # --- move_group needs URDF/SRDF/kinematics + controller mapping ---
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            moveit_controller_manager,
            moveit_simple_params,
        ],
    )

    # --- RViz ALSO needs URDF/SRDF (otherwise Planning Group is grey) ---
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        # Give RViz the same robot_description + SRDF (+ optional kinematics)
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        # Optional: preload MotionPlanning panel config if you have one
        # arguments=["-d", PathJoinSubstitution([FindPackageShare("mm_moveit"), "config", "moveit.rviz"])],
    )

    return LaunchDescription([move_group, rviz])

