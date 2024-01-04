import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    desc = LaunchDescription()

    ####### DATA INPUT ##########
    xacro_file = "cyberknife_position.urdf.xacro"

    package_description = "cyberknife"

    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.5]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "cyberknife"
    ####### DATA INPUT END ##########

    # Path to robot model XACRO File
    robot_desc_path = os.path.join(get_package_share_directory(
        package_description), "urdf", xacro_file)


    # Robot Description in XACRO Format
    robot_desc = xacro.process_file(robot_desc_path)

    # Robot Description in XML Format
    xml = robot_desc.toxml()


    # Launch Config for Simulation Time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot State Publisher Node
    robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': xml}],
        output="screen"
    )


    # Static TF Transform
    tf=Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['1', '0', '0', '0', '0', '0', '1', '/map',  '/dummy_link'  ],
    )

    desc.add_action(DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'))
    desc.add_action(robot_state_publisher)
    desc.add_action(tf)

    return desc