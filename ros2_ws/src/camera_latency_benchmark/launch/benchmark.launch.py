from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_composition = LaunchConfiguration("use_composition")
    intra_process = LaunchConfiguration("intra_process")
    transport = LaunchConfiguration("transport")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
    publish_topic = LaunchConfiguration("publish_topic")
    subscribe_topic = LaunchConfiguration("subscribe_topic")
    qos_depth = LaunchConfiguration("qos_depth")
    qos_reliability = LaunchConfiguration("qos_reliability")
    shared_memory = LaunchConfiguration("shared_memory")
    stats_every = LaunchConfiguration("stats_every")
    stats_window = LaunchConfiguration("stats_window")
    csv_path = LaunchConfiguration("csv_path")

    common_params = {
        "width": width,
        "height": height,
        "fps": fps,
        "output_topic": publish_topic,
        "qos_depth": qos_depth,
        "qos_reliability": qos_reliability,
    }

    probe_params = {
        "transport": transport,
        "input_topic": subscribe_topic,
        "qos_depth": qos_depth,
        "qos_reliability": qos_reliability,
        "stats_every": stats_every,
        "stats_window": stats_window,
        "csv_path": csv_path,
    }

    cyclonedds_config = (
        get_package_share_directory("camera_latency_benchmark")
        + "/config/cyclonedds_shm.xml"
    )

    shm_env = [
        SetEnvironmentVariable(
            name="RMW_IMPLEMENTATION",
            value="rmw_cyclonedds_cpp",
            condition=IfCondition(shared_memory),
        ),
        SetEnvironmentVariable(
            name="CYCLONEDDS_URI",
            value="file://" + cyclonedds_config,
            condition=IfCondition(shared_memory),
        ),
    ]

    container = ComposableNodeContainer(
        name="camera_latency_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="camera_latency_benchmark",
                plugin="camera_latency_benchmark::FakeCameraNode",
                name="fake_camera",
                parameters=[common_params],
                extra_arguments=[{"use_intra_process_comms": intra_process}],
            ),
            ComposableNode(
                package="camera_latency_benchmark",
                plugin="camera_latency_benchmark::LatencyProbeNode",
                name="latency_probe",
                parameters=[probe_params],
                extra_arguments=[{"use_intra_process_comms": intra_process}],
            ),
        ],
        output="screen",
        condition=IfCondition(use_composition),
    )

    standalone_fake = Node(
        package="camera_latency_benchmark",
        executable="fake_camera_node",
        name="fake_camera",
        parameters=[common_params],
        output="screen",
        condition=UnlessCondition(use_composition),
    )

    standalone_probe = Node(
        package="camera_latency_benchmark",
        executable="latency_probe_node",
        name="latency_probe",
        parameters=[probe_params],
        output="screen",
        condition=UnlessCondition(use_composition),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_composition", default_value="false"),
            DeclareLaunchArgument("intra_process", default_value="false"),
            DeclareLaunchArgument("transport", default_value="raw"),
            DeclareLaunchArgument("width", default_value="640"),
            DeclareLaunchArgument("height", default_value="480"),
            DeclareLaunchArgument("fps", default_value="30.0"),
            DeclareLaunchArgument("publish_topic", default_value="image"),
            DeclareLaunchArgument("subscribe_topic", default_value="image"),
            DeclareLaunchArgument("qos_depth", default_value="10"),
            DeclareLaunchArgument("qos_reliability", default_value="best_effort"),
            DeclareLaunchArgument("shared_memory", default_value="false"),
            DeclareLaunchArgument("stats_every", default_value="100"),
            DeclareLaunchArgument("stats_window", default_value="1000"),
            DeclareLaunchArgument("csv_path", default_value=""),
            *shm_env,
            container,
            standalone_fake,
            standalone_probe,
        ]
    )
