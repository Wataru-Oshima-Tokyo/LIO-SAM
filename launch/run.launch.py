import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,RegisterEventHandler,GroupAction,OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression,Command,PathJoinSubstitution
from launch.event_handlers import OnProcessStart, OnProcessExit

def launch_setup(context, *args, **kwargs):
    fake_frame = "fake_velodyne_link"
    share_dir = get_package_share_directory('lio_sam')
    map_handler_dir = get_package_share_directory('map_handler')
    parameter_file = LaunchConfiguration('lio_parameter_file').perform(context)
    rviz_file = LaunchConfiguration('rviz_file').perform(context)
    parameter_file += ".yaml"
    rviz_file += ".rviz"
    use_sim_time = LaunchConfiguration('use_sim_time')
    lio_parameter_file = os.path.join(share_dir, 'config', parameter_file)
    rviz_config_file = os.path.join(share_dir, 'config', rviz_file)
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')

    velodyne_to_base_link =  Node(
        package='fake_frame',
        executable='fake_dynamic_tf_broadcaster',
        name='fake_base_link',
            parameters=[{'parent_link': "velodyne"},
                        {'child_link': "base_link"},
                        {"use_sim_time": use_sim_time}]    
    )


    static_world_to_map_node =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map']
    )


    static_map_to_odom_node =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )
    
    static_base_link_to_base_footprint =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'base_footprint']
    )

    velodyne_to_base_link_to_velodyne =  Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name="velodyne_base_link_to_velodyne",
        arguments='0.0 0.0 0.0 0.0 0.0 0.0 velodyne_base_link velodyne'.split(' '),
        output='screen',
        )


    lio_sam_nodes = GroupAction(
        actions=[
            Node(
                package='lio_sam',
                executable='lio_sam_imuPreintegration',
                name='lio_sam_imuPreintegration',
                parameters=[lio_parameter_file],
                output='screen'
            ),
            Node(
                package='lio_sam',
                executable='lio_sam_imageProjection',
                name='lio_sam_imageProjection',
                parameters=[lio_parameter_file],
                output='screen'
            ),
            Node(
                package='lio_sam',
                executable='lio_sam_featureExtraction',
                name='lio_sam_featureExtraction',
                parameters=[lio_parameter_file],
                output='screen'
            ),
            Node(
                package='lio_sam',
                executable='lio_sam_mapOptimization',
                name='lio_sam_mapOptimization',
                parameters=[lio_parameter_file, {"saveOdomDirectory": map_handler_dir}],
                output='screen'
            ),
        ]
    )


    delayed_base_link = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=static_world_to_map_node,
            on_start=[static_base_link_to_base_footprint],
        )
    )


    delayed_lio_sam_server =   RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=static_world_to_map_node,
            on_start=[lio_sam_nodes],
        )
    )

    delayed_fake_odom =   RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=static_world_to_map_node,
            on_start=[static_map_to_odom_node],
        )
    )

    delayed_fake_base_link =   RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=static_world_to_map_node,
            on_start=[velodyne_to_base_link],
        )
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    return [
        static_world_to_map_node,
        delayed_base_link,
        delayed_lio_sam_server,
        delayed_fake_odom,
        delayed_fake_base_link,
        rviz2,
        velodyne_to_base_link_to_velodyne
    ]

def generate_launch_description():
    
    params_declare = DeclareLaunchArgument(
        'lio_parameter_file',
        default_value='world_map',
        description='FPath to the ROS2 parameters file to use.')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='use sim time or not')

    rviz_arg = DeclareLaunchArgument(
        'rviz_file',
        default_value='rviz2',
        description='rviz file name')

    return LaunchDescription([
        params_declare,
        use_sim_time_arg,
        rviz_arg,
        OpaqueFunction(function=launch_setup)

    ])
