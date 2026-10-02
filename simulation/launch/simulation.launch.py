# Author: Luca Obwegs
"""Launch Gazebo Harmonic, the robot model, ROS/Gazebo bridges, and controller."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from balancing_robot_sim.generate_sdf import generate


def launch_setup(context, *_args, **_kwargs):
    share = Path(get_package_share_directory("balancing_robot_sim"))
    config = Path(LaunchConfiguration("config").perform(context))
    if not config.is_absolute():
        config = share / config
    model_file = Path(tempfile.gettempdir()) / "balancing_robot" / "model.sdf"
    generate(config, share / "models" / "balancing_robot" / "model.sdf.in", model_file)
    gz_share = Path(get_package_share_directory("ros_gz_sim"))

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {share / 'worlds' / 'balancing_world.sdf'}"}.items(),
    )
    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-world", "balancing_world", "-file", str(model_file)],
                output="screen",
            )
        ],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/model/balancing_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/balancing_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        output="screen",
    )
    controller = Node(
        package="balancing_robot_sim",
        executable="balance_controller",
        parameters=[
            {"config": str(config), "controller": LaunchConfiguration("controller")}
        ],
        output="screen",
    )
    return [gz, spawn, bridge, TimerAction(period=4.0, actions=[controller])]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("controller", default_value="lqr"),
            DeclareLaunchArgument(
                "config", default_value="config/mechanical.json"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
