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
        default_value='vehicle_sim.rviz',
        description='Full path to the RVIZ config file to use'
    )

    declare_world_arg = DeclareLaunchArgument(
        name='world', default_value='empty.sdf',
        description='Name of the Gazebo world file to load'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation (Gazebo) clock if true'
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
    ###############################################################################################

    ###################################### Declare the nodes ######################################
    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0",
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
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
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU", 
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",             
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked", 
            "/cam_1/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/cam_1/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",   
        ],
        parameters=[{'use_sim_time': True}]
    )
    ###############################################################################################

    ############################ Create the launch arguments and nodes ############################
    launchDescriptionObject = LaunchDescription()

    # Define the launch arguments
    launchDescriptionObject.add_action(declare_use_rviz_arg)
    launchDescriptionObject.add_action(declare_world_arg)
    launchDescriptionObject.add_action(declare_model_arg)
    launchDescriptionObject.add_action(declare_rviz_config_file_arg)
    launchDescriptionObject.add_action(declare_use_sim_time_arg)

    launchDescriptionObject.add_action(world_launch)

    # Add any actions
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(gz_bridge_node)

    return launchDescriptionObject
