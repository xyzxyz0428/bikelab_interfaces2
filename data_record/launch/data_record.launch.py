from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    record_topics = [
        "/ubx_nav_pvt",
        "/ubx_rxm_rtcm",
        "/ntrip_client/rtcm",
        "/rslidar_points_200",
        "/rslidar_points_201",
        "/rslidar_points_202",
        "/camera/image_raw",
        "/camera/camera_info",
        "/adc_data",
        "/bike_speed_data",
        "/power_meter_data",
        "/imu",
        "/tf_static",
        "/tf",
    ]
    topics_str = " ".join(record_topics)

    tf_yaml_default = PathJoinSubstitution(
        [FindPackageShare("data_record"), "config", "static_frames.yaml"]
    )

    static_tf_node = Node(
        package="data_record",
        executable="static_tf_from_yaml",
        name="static_tf_from_yaml",
        output="screen",
        parameters=[{"tf_yaml_path": LaunchConfiguration("tf_yaml")}],
    )

    rosbag_process = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            [
                "ros2 bag record -o ",
                LaunchConfiguration("output_root"),
                "/$(date +%Y%m%d_%H%M%S) ",
                topics_str,
            ],
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("record")),
    )

    delayed_rosbag = TimerAction(period=1.5, actions=[rosbag_process])

    return LaunchDescription([
        DeclareLaunchArgument("tf_yaml", default_value=tf_yaml_default),
        DeclareLaunchArgument("record", default_value="true"),
        DeclareLaunchArgument("output_root", default_value="/mnt/bikelab_data"),

        static_tf_node,
        delayed_rosbag,
    ])