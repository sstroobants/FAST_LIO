import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )
    # FAST-LIO publishes everything (point cloud, odometry, path, its own TF)
    # under frame_id "camera_init" (see laserMapping.cpp), which this fork
    # never gravity-aligns — camera_init is literally wherever the Livox was
    # pointing at the first IMU sample, i.e. tilted by the sensor's fixed
    # mount rotation (scripts/fastlio_to_px4_visual_odom.py corrects this for
    # what PX4 receives, but doesn't touch what FAST-LIO itself publishes).
    # This static transform makes "level_init" a gravity-level parent of
    # camera_init, so RViz (Fixed Frame: level_init, set in fastlio.rviz) shows
    # the point cloud/odometry/path all correctly leveled, up to the same
    # arbitrary-but-constant yaw offset that was already present before the
    # mount was tilted. qx/qy/qz/qw below = Ry(+30 deg), i.e. the SAME
    # rotation as iris_livox.sdf's mount pose / _LIDAR_MOUNT_PITCH_RAD in the
    # bridge script — keep all three in sync if the mount changes.
    level_frame_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='level_init_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0.25881904510252074',
            '--qz', '0', '--qw', '0.96592582628906831',
            '--frame-id', 'level_init', '--child-frame-id', 'camera_init',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    # FAST-LIO's "body" frame (odomAftMapped.child_frame_id, see
    # laserMapping.cpp) is really the Livox sensor's own frame (state.rot),
    # not the vehicle's — that's what state.rot always was, mount tilt or not.
    # With the mount now pitched 30 deg down, "body"'s arrow/axes will
    # correctly point ~30 deg downward even when the drone is perfectly
    # level; that's the sensor's true physical orientation, not a bug. This
    # second static transform adds "vehicle_body" as a child of the (dynamic)
    # "body" frame, undoing the mount rotation (Ry(-30 deg) = the inverse of
    # the level_init rotation above), so a display pointed at "vehicle_body"
    # shows the actual drone attitude instead. Add an RViz "TF" or "Axes"
    # display with reference frame "vehicle_body" to see it (not wired into
    # fastlio.rviz by default, since the Odometry display's arrow is tied to
    # the /Odometry topic's own embedded child_frame_id and can't be
    # redirected via RViz config alone).
    vehicle_body_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='vehicle_body_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '-0.25881904510252074',
            '--qz', '0', '--qw', '0.96592582628906831',
            '--frame-id', 'body', '--child-frame-id', 'vehicle_body',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(level_frame_tf_node)
    ld.add_action(vehicle_body_tf_node)
    ld.add_action(rviz_node)

    return ld
