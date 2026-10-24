import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("ican_inspection_sim")
    sim_prefix = get_package_prefix("ican_inspection_sim")
    world_path = os.path.join(sim_share, "worlds", "inspection_task1.world")
    camera_rviz_config = os.path.join(sim_share, "rviz", "uav_camera.rviz")
    gui = LaunchConfiguration("gui")
    ground_demo = LaunchConfiguration("ground_demo")
    camera_rviz = LaunchConfiguration("camera_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI client."),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("camera_rviz", default_value="true", description="Start RViz with the UAV camera image."),
            DeclareLaunchArgument(
                "ground_demo",
                default_value="false",
                description="Start a simple mecanum waypoint demo for the amphibious drone land mode.",
            ),
            SetEnvironmentVariable(
                "GAZEBO_PLUGIN_PATH",
                os.path.join(sim_prefix, "lib") + ":" + os.environ.get("GAZEBO_PLUGIN_PATH", ""),
            ),
            ExecuteProcess(
                cmd=[
                    "gzserver",
                    world_path,
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    "-s",
                    "libgazebo_ros_force_system.so",
                ],
                output="screen",
            ),
            TimerAction(
                period=4.0,
                actions=[
                    ExecuteProcess(
                        cmd=["gzclient"],
                        output="screen",
                        condition=IfCondition(gui),
                    )
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="uav_base_to_camera_tf",
                arguments=[
                    "--x", "0.390",
                    "--y", "0",
                    "--z", "0.130",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "uav/base_link",
                    "--child-frame-id", "uav/camera_link",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(camera_rviz),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="uav_base_to_laser_tf",
                arguments=[
                    "--x", "0",
                    "--y", "0",
                    "--z", "-0.070",
                    "--roll", "0",
                    "--pitch", "1.5708",
                    "--yaw", "0",
                    "--frame-id", "uav/base_link",
                    "--child-frame-id", "uav/laser_link",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=["rviz2", "-d", camera_rviz_config],
                        output="screen",
                        condition=IfCondition(camera_rviz),
                    )
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "run", "ican_inspection_sim", "ground_mecanum_demo.py"],
                        output="screen",
                        condition=IfCondition(ground_demo),
                    )
                ],
            ),
        ]
    )
