import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, TextSubstitution

def generate_launch_description():

    ################################ Set paths to important files ################################
    pkg_share_path = get_package_share_directory('vehicle_control')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Add your own gazebo library path here
    gazebo_models_path = "/home/mina/gazebo_models"
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.pathsep + gazebo_models_path
    ###############################################################################################

    ################################ Declare the launch arguments ################################
    world_arg = DeclareLaunchArgument(
        'world', default_value='world.sdf',
        description='Name of the Gazebo world file to load'
    )

    declare_use_gz_sim_arg = DeclareLaunchArgument(
        name='gz_sim',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to use gazebo simulation'
    )
    ###############################################################################################

    ###################################### Declare the nodes ######################################
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            pkg_share_path,
            'worlds',
            LaunchConfiguration('world')
        ]),
        #TextSubstitution(text=' -r -v -v1 --render-engine ogre')],
        TextSubstitution(text=' -r -v -v1')],
        'on_exit_shutdown': 'true'}.items()
    )
    ###############################################################################################

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(declare_use_gz_sim_arg)

    launchDescriptionObject.add_action(gazebo_launch)

    return launchDescriptionObject
    ###############################################################################################
