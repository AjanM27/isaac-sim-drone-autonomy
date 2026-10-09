"""3D LiDAR-only RTAB-Map tuned for the simulated indoor warehouse."""

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context: LaunchContext, *args, **kwargs):
    voxel_size = float(LaunchConfiguration("voxel_size").perform(context))
    reset_database = LaunchConfiguration("reset_database").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )

    shared = {
        "use_sim_time": True,
        "frame_id": "slam_base",
        "qos": 1,
        "wait_for_transform": 0.5,
        "Icp/PointToPlane": "true",
        "Icp/Iterations": "10",
        "Icp/VoxelSize": str(voxel_size),
        "Icp/Epsilon": "0.001",
        "Icp/PointToPlaneK": "20",
        "Icp/PointToPlaneRadius": "0",
        "Icp/MaxTranslation": "2.0",
        "Icp/MaxCorrespondenceDistance": str(voxel_size * 10.0),
        "Icp/Strategy": "1",
        "Icp/OutlierRatio": "0.7",
        "Icp/CorrespondenceRatio": "0.01",
    }

    odometry = Node(
        package="rtabmap_odom",
        executable="icp_odometry",
        name="icp_odometry",
        output="screen",
        parameters=[
            shared,
            {
                # Isaac publishes the OS1 render product at 10 Hz nominally,
                # but quantization against the 100 Hz physics clock produces
                # occasional 11.11-12.5 Hz intervals. Keep the rejection
                # threshold above that measured maximum without disabling the
                # missing-update watchdog entirely.
                "expected_update_rate": 15.0,
                # Isaac currently publishes a completed Cartesian XYZ scan,
                # without per-point timestamps. Deskewing must stay disabled.
                "deskewing": False,
                "odom_frame_id": "icp_odom",
                "Odom/ScanKeyFrameThr": "0.4",
                "OdomF2M/ScanSubtractRadius": str(voxel_size),
                "OdomF2M/ScanMaxSize": "15000",
                "OdomF2M/BundleAdjustment": "false",
            },
        ],
        remappings=[
            ("scan_cloud", "/slam/lidar/points"),
            ("odom", "/slam/odom"),
            ("odom_info", "/slam/odom_info"),
            ("imu", "/slam/imu_not_used"),
        ],
    )

    slam_arguments = ["-d"] if reset_database else []
    slam = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        arguments=slam_arguments,
        parameters=[
            shared,
            {
                "database_path": "/maps/rtabmap.db",
                "subscribe_depth": False,
                "subscribe_rgb": False,
                "subscribe_scan_cloud": True,
                "subscribe_odom_info": True,
                "map_frame_id": "map",
                "odom_sensor_sync": True,
                "RGBD/ProximityMaxGraphDepth": "0",
                "RGBD/ProximityPathMaxNeighbors": "1",
                "RGBD/AngularUpdate": "0.05",
                "RGBD/LinearUpdate": "0.05",
                "RGBD/CreateOccupancyGrid": "true",
                "Mem/NotLinkedNodesKept": "false",
                "Mem/STMSize": "30",
                "Reg/Strategy": "1",
                # Build a true 3D local grid from LiDAR and deliberately keep
                # floor returns because this is an aerial robot.
                "Grid/FromDepth": "false",
                "Grid/Sensor": "0",
                "Grid/3D": "true",
                "Grid/GroundIsObstacle": "true",
                "Grid/CellSize": str(voxel_size),
                "Grid/RangeMax": "12.0",
                # Build RTAB-Map's probabilistic OctoMap as the canonical
                # compact global occupancy representation. The explorer keeps
                # a 30 cm leaf cache only for inexpensive local A* queries.
                "Grid/RayTracing": "true",
            },
        ],
        remappings=[
            ("scan_cloud", "/slam/lidar/points"),
            ("odom", "/slam/odom"),
            ("odom_info", "/slam/odom_info"),
            ("imu", "/slam/imu_not_used"),
        ],
    )

    return [odometry, slam]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "voxel_size",
                default_value="0.15",
                description="Indoor ICP and 3D map voxel size in metres.",
            ),
            DeclareLaunchArgument(
                "reset_database",
                default_value="true",
                description="Start with a clean map database for this simulation run.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
