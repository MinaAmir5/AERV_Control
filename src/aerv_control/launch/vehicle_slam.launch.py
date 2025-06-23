import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, LoadComposableNodes
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ComposableNode

    ##################################### External functions #####################################
def start_vehicle_control():
    """
    Starts the necessary controllers for the vehicle's operation in ROS 2.

    @return: A tuple containing ExecuteProcess actions for the joint state, forward velocity, 
             and forward position controllers.
    """
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'joint_state_broadcaster'],
        output='screen')

    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_velocity_controller'],
        output='screen')

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen')

    return (joint_state_controller,
            forward_velocity_controller,
            forward_position_controller)
    ###############################################################################################

def generate_launch_description():

    ################################ Set paths to important files ################################
    pkg_share_path = get_package_share_directory('aerv_control')

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
        'online_sync_launch.py'
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

    static_map_path = os.path.join(
        pkg_share_path,
        'maps',
        'depot.yaml'
    )

    vehicle_params_path = os.path.join(
        pkg_share_path, 
        'config',
        'parameters.yaml'
    )
    
    # map_file_path = os.path.join(
    #     pkg_share_path,
    #     'maps',
    #     'my_map.yaml'
    # )
    ###############################################################################################

    ####################################### Functions calls #######################################
    joint_state, forward_velocity, forward_position = start_vehicle_control()
    ###############################################################################################

    ################################ Declare the launch arguments ################################
    declare_model_arg = DeclareLaunchArgument(
        name='model', 
        default_value='vehicle_aerv13.xacro',
        description='Name of the URDF description to load'
    )
    
    declare_use_rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        description='Whether to start RVIZ'
    )

    declare_rviz_config_file_arg = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value='vehicle_slam2.rviz',
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

    declare_use_encoder_arg = DeclareLaunchArgument(
        name="use_encoder",
        default_value="true",
        choices=["true", "false"],
        description="Whether to use encoder"
    )

    declare_use_mqtt_server_arg = DeclareLaunchArgument(
        name="use_mqtt",
        default_value="false",
        choices=["true", "false"],
        description="Whether to use mqtt server"
    )

    declare_use_ultrasonic_arg = DeclareLaunchArgument(
        name="use_ultrasonic",
        default_value="false",
        choices=["true", "false"],
        description="whether to use ultrasonic sensors"
    )

    declare_use_app_arg = DeclareLaunchArgument(
        name="use_app",
        default_value="false",
        choices=["true", "false"],
        description="whether to use mobile application"
    )
    ###############################################################################################

    ################################ Declare external lanuch files ################################
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_path, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('gz_sim'))
    )

    websocket_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        )
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
                'map': static_map_path,
                'params_file': localization_params_path,
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
            # "-x", "280.0", "-y", "-140.0", "-z", "3.0", "-Y", "2.5",
            "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0",

        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # robot_description_content = ParameterValue(Command(['xacro ', urdf_model_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gz_sim')),
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_model_path]),
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
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
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    ekf_encoder_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration("use_encoder")),
        parameters=[
            os.path.join(pkg_share_path, 'config', 'ekf_encoder_local.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time'),}
        ],
        remappings=[("odometry/filtered", "odometry/local")],
    )

    ekf_encoder_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        condition=IfCondition(LaunchConfiguration("use_encoder")),
        parameters=[
            os.path.join(pkg_share_path, 'config', 'ekf_encoder_global.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[("odometry/filtered", "odometry/global")],
    )

    ekf_cmd_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration("use_encoder")),
        parameters=[
            os.path.join(pkg_share_path, 'config', 'ekf_cmd_local.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[("odometry/filtered", "odometry/local")],
    )

    ekf_cmd_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration("use_encoder")),
        parameters=[
            os.path.join(pkg_share_path, 'config', 'ekf_cmd_global.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[("odometry/filtered", "odometry/global")],
    )

    navsat_transforom_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        # condition=UnlessCondition(LaunchConfiguration("use_encoder")),
        parameters=[
            os.path.join(pkg_share_path, 'config', 'navsat.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('gps/fix', 'fix'),
            ('imu/data', 'imu/filtered'),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ('odometry/filtered', 'odometry/global'),
        ]
    )

    imu_filter_node = Node(
        package='aerv_control',
        executable='imu_filter_node',
        # condition=UnlessCondition(LaunchConfiguration("use_encoder"))
    )

    vehicle_controller_node = Node(
        package='aerv_control',
        executable='vehicle_controller',
        parameters=[vehicle_params_path],
        output='screen'
    )

    tf_baselink_basefootprint_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_tf",
        condition=UnlessCondition(LaunchConfiguration('gz_sim')),
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "base_link"
        ]
    )

    tf_basefootprint_odom_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_tf",
        condition=UnlessCondition(LaunchConfiguration('gz_sim')),
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--frame-id", "odom",
            "--child-frame-id", "base_footprint"
        ]
    )

    tf_odom_map_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_tf",
        condition=UnlessCondition(LaunchConfiguration('gz_sim')),
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "odom"
        ]
    )

    mqtt_server_node = Node(
        package='aerv_control',
        executable='mqtt_bridge_node',
        name='mqtt_bridge',
        condition=IfCondition(LaunchConfiguration("use_mqtt")),
        output='screen'
    )

    mode_switch_node = Node(
        package='aerv_control',
        executable='switch_node',
        name='switch_node',
        output='screen'
    )

    LaserScantoRange_converter_node = Node(
        package='aerv_control',
        executable='laser_to_range_node',
        condition=IfCondition(LaunchConfiguration("use_ultrasonic")),
        output='screen'
    )

    mqtt_app_node = Node(
        package='aerv_control',
        executable='mqtt_app_node',
        condition=IfCondition(LaunchConfiguration("use_app")),
        output='screen'
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
    launchDescriptionObject.add_action(declare_use_encoder_arg)
    launchDescriptionObject.add_action(declare_use_mqtt_server_arg)
    launchDescriptionObject.add_action(declare_use_ultrasonic_arg)
    launchDescriptionObject.add_action(declare_use_app_arg)

    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(websocket_launch)
    launchDescriptionObject.add_action(slam_toolbox_launch)
    # launchDescriptionObject.add_action(localization_launch)
    launchDescriptionObject.add_action(navigation_launch)

    launchDescriptionObject.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn_urdf_node,
                                        on_exit=[joint_state]
            )
    ))
    launchDescriptionObject.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(target_action=joint_state,
                                        on_exit=[forward_velocity,forward_position]
            )
    ))

    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(joint_state_publisher_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(vehicle_controller_node)
    launchDescriptionObject.add_action(ekf_cmd_local_node)
    launchDescriptionObject.add_action(ekf_cmd_global_node)
    launchDescriptionObject.add_action(ekf_encoder_local_node)
    # launchDescriptionObject.add_action(ekf_encoder_global_node)
    launchDescriptionObject.add_action(imu_filter_node)
    launchDescriptionObject.add_action(navsat_transforom_node)
    # launchDescriptionObject.add_action(tf_baselink_basefootprint_node)
    # launchDescriptionObject.add_action(tf_odom_map_node)
    launchDescriptionObject.add_action(tf_basefootprint_odom_node)
    launchDescriptionObject.add_action(mqtt_server_node)
    launchDescriptionObject.add_action(mode_switch_node)
    launchDescriptionObject.add_action(LaserScantoRange_converter_node)
    launchDescriptionObject.add_action(mqtt_app_node)

    return launchDescriptionObject
    ###############################################################################################
