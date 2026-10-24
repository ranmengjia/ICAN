import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory("ican_inspection_navigation")
    sim_share = get_package_share_directory("ican_inspection_sim")

    default_params = os.path.join(navigation_share, "config", "nav2_ground_params.yaml")
    default_livox_params = os.path.join(navigation_share, "config", "livox_to_scan.yaml")
    default_map = os.path.join(sim_share, "maps", "inspection_task1_map.yaml")
    default_rviz = os.path.join(navigation_share, "rviz", "nav2_ground.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("rviz")
    use_livox = LaunchConfiguration("use_livox")
    livox_points_topic = LaunchConfiguration("livox_points_topic")
    livox_params_file = LaunchConfiguration("livox_params_file")

    lifecycle_nodes = [
        "map_server",
        "amcl",
        "planner_server",
        "controller_server",
        "smoother_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_livox",
                default_value="false",
                description="Convert Livox PointCloud2 to /scan for ground Nav2.",
            ),
            DeclareLaunchArgument("livox_points_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("livox_params_file", default_value=default_livox_params),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                arguments=[
                    "--x", "0.08",
                    "--y", "0",
                    "--z", "0.08",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "base_footprint",
                    "--child-frame-id", "laser_link",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_ground_camera_tf",
                arguments=[
                    "--x", "0.16",
                    "--y", "0",
                    "--z", "0.11",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "base_footprint",
                    "--child-frame-id", "camera_link",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_uav_tf",
                arguments=[
                    "--x", "0",
                    "--y", "0",
                    "--z", "0",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "base_footprint",
                    "--child-frame-id", "uav/base_link",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                condition=IfCondition(use_livox),
                remappings=[("cloud_in", livox_points_topic), ("scan", "/scan")],
                parameters=[livox_params_file, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[params_file, {"yaml_filename": map_file, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
            ExecuteProcess(
                cmd=["rviz2", "-d", default_rviz],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
