import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,RegisterEventHandler,GroupAction,OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression,Command,PathJoinSubstitution
from launch.event_handlers import OnProcessStart, OnProcessExit
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    share_dir = get_package_share_directory('lio_sam')
    sim = LaunchConfiguration('sim')
    map_name = LaunchConfiguration('map_name').perform(context)
    nav_param_file = map_name + "_nav.yaml"
    navigation_param = os.path.join(get_package_share_directory(
        'robot_navigation'), 'param', nav_param_file)
    nav_to_pose_xml = os.path.join(get_package_share_directory(
        'robot_navigation'), 'param', "nav_to_pose_and_pause_near_goal_obstacle.xml")
    nav_thr_pose_xml = os.path.join(get_package_share_directory(
        'robot_navigation'), 'param', "nav_thr_poses.xml")
    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    param_substitutions = {
        'default_nav_to_pose_bt_xml': nav_to_pose_xml,
        'default_nav_through_poses_bt_xml': nav_thr_pose_xml}

    configured_params = RewrittenYaml(
            source_file=navigation_param,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)


    navigation_node =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': configured_params
            }.items()
    )
    return [
        navigation_node
    ]

def generate_launch_description():




    sim_declare =  DeclareLaunchArgument(
            name='sim', 
            default_value='true',
            description='Enable use_sime_time to true'
    )

    map_name_declare =  DeclareLaunchArgument(
            name='map_name', 
            default_value='sh',
            description='Enable use_sime_time to true'
    )

    return LaunchDescription([
        sim_declare,
        map_name_declare,
        OpaqueFunction(function=launch_setup)
    ])
