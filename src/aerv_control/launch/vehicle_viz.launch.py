from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
  
def generate_launch_description():
 
    ################################ Set paths to important files ################################
    pkg_share_path = FindPackageShare('aerv_control')

    urdf_model_path = PathJoinSubstitution(
        [pkg_share_path, 
         'urdf', 
         'robots', 
         LaunchConfiguration('model')]
    )

    # urdf_model_path = '/home/mina/aerv_ws/install/aerv_control/share/aerv_control/urdf/Assem1.SLDASM.urdf'
    
    rviz_config_path = PathJoinSubstitution(
        [pkg_share_path, 
         'rviz', 
         LaunchConfiguration('rviz_config_file')]
    )
    ###############################################################################################

    ################################ Declare the launch arguments ################################
    declare_model_arg = DeclareLaunchArgument(
        'model', 
        default_value='vehicle_aerv12.xacro',
        description='Name of the URDF description to load'
    )
 
    declare_jsp_gui_arg = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
 
    declare_use_rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to start RVIZ'
    )

    declare_rviz_config_file_arg = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value='vehicle_viz.rviz',
        description='Full path to the RVIZ config file to use'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation (Gazebo) clock if true'
    )
    ###############################################################################################

    ###################################### Declare the nodes ######################################
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.

    # robot_description_content = ParameterValue(Command(['xacro ', urdf_model_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': Command(['xacro', ' ', urdf_model_path])
        }]
    )
 
    # Publish the joint state values for the non-fixed joints in the URDF file.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('jsp_gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
 
    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
 
    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    ###############################################################################################

    ############################ Create the launch arguments and nodes ############################
    launchDescriptionObject = LaunchDescription()
 
    # Define the launch arguments
    launchDescriptionObject.add_action(declare_model_arg)
    launchDescriptionObject.add_action(declare_jsp_gui_arg)
    launchDescriptionObject.add_action(declare_rviz_config_file_arg)
    launchDescriptionObject.add_action(declare_use_rviz_arg)
    launchDescriptionObject.add_action(declare_use_sim_time_arg)
 
    # Add any actions
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(joint_state_publisher_node)
    launchDescriptionObject.add_action(joint_state_publisher_gui_node)
    launchDescriptionObject.add_action(rviz_node)
 
    return launchDescriptionObject
    ###############################################################################################