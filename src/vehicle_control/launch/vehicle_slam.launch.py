import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ################################ Set paths to important files ################################
    pkg_share_path = get_package_share_directory('vehicle_control')

    urdf_model_path = PathJoinSubstitution(
        [pkg_share_path, 
         'urdf', 
         'robots', 
         LaunchConfiguration('model')]
    )
    
    rviz_config_path = PathJoinSubstitution(
        [pkg_share_path, 
         'rviz', 
         LaunchConfiguration('rviz_config_file')]
    )

    # Path to the Slam Toolbox launch file
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox_params_path = os.path.join(
        pkg_share_path,
        'config',
        'slam_toolbox_mapping.yaml'
    )

    gz_bridge_params_path = os.path.join(
        pkg_share_path,
        'config',
        'gz_bridge.yaml'
    )

    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    localization_params_path = os.path.join(
        pkg_share_path,
        'config',
        'amcl_localization.yaml'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    navigation_params_path = os.path.join(
        pkg_share_path,
        'config',
        'navigation.yaml'
    )
    
    # map_file_path = os.path.join(
    #     pkg_share_path,
    #     'maps',
    #     'my_map.yaml'
    # )
    ###############################################################################################

    ################################ Declare the launch arguments ################################
    declare_model_arg = DeclareLaunchArgument(
        name='model', 
        default_value='robot_3d.urdf.xacro',
        description='Name of the URDF description to load'
    )
    
    declare_use_rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        description='Whether to start RVIZ'
    )

    declare_rviz_config_file_arg = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value='vehicle_slam.rviz',
        description='Full path to the RVIZ config file to use'
    )

    declare_world_arg = DeclareLaunchArgument(
        name='world', 
        default_value='warehouse.sdf',
        description='Name of the Gazebo world file to load'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='True',
        description='Flag to enable use_sim_time'
    )

    declare_use_gz_sim_arg = DeclareLaunchArgument(
        name='gz_sim',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to use gazebo simulation'
    )
    ###############################################################################################

    ################################ Declare external lanuch files ################################
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_path, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        }.items()
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )


    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': localization_params_path,
                # 'map': map_file_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
        }.items()
    )
    ###############################################################################################
    
    ################################ Declare the launch arguments ################################
    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        condition=IfCondition(LaunchConfiguration('gz_sim')),
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0",
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gz_sim')),
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_model_path]),
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Node to bridge messages like /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration('gz_sim')),
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        parameters=[
            {'use_sim_time': True},
        ]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share_path, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
             ]
    )
    ###############################################################################################

    ###################################### Declare the nodes ######################################
    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(declare_model_arg)
    launchDescriptionObject.add_action(declare_use_rviz_arg)
    launchDescriptionObject.add_action(declare_rviz_config_file_arg)
    launchDescriptionObject.add_action(declare_world_arg)
    launchDescriptionObject.add_action(declare_use_sim_time_arg)
    launchDescriptionObject.add_action(declare_use_gz_sim_arg)

    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(slam_toolbox_launch)
    launchDescriptionObject.add_action(localization_launch)
    launchDescriptionObject.add_action(navigation_launch)

    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(ekf_node)

    return launchDescriptionObject
    ###############################################################################################
