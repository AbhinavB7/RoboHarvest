from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os

import launch_ros.descriptions
import launch
import xacro

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    ####### DATA INPUT ##########
    urdf_file = 'robo.urdf'
    xacro_file = "robo.urdf.xacro"
    #xacro_file = "box_bot.xacro"
    package_description = "roboharvest"
    use_urdf = False
    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 1.0]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "robo"
    ####### DATA INPUT END ##########
    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_description).find(package_description)

    # default_rviz_config_path = os.path.join(pkg_share, 'rviz/display_default.rviz')

    default_rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("roboharvest"), "rviz", "display_default.rviz"]
    )

    if use_urdf:
        # print("URDF URDF URDF URDF URDF URDF URDF URDF URDF URDF URDF ==>")
        robot_desc_path = os.path.join(get_package_share_directory(
            package_description), "robot", urdf_file)
    else:
        # print("XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO ==>")
        robot_desc_path = os.path.join(get_package_share_directory(
            package_description), "urdf", xacro_file)

    robot_desc = xacro.process_file(robot_desc_path)
    xml = robot_desc.toxml()


 

    robot_desc_path = os.path.join(get_package_share_directory(
            package_description), "urdf", xacro_file) # Update this with the actual URDF file path
    
    # RVIZ Configuration
    rviz_config_dir = PathJoinSubstitution(
        [FindPackageShare("roboharvest"), "rviz", "display_default.rviz"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])
    
    # Publish Robot Desciption in String form in the topic /robot_description
    publish_robot_description = Node(
        package='roboharvest',
        executable='robot_description_publisher.py',
        name='robot_description_publisher',
        output='screen',
        arguments=['-xml_string', xml,
                   '-robot_description_topic', '/robot_description'
                   ]
    )

    # Robot State Publisher
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': xml}],
        output="screen"
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_gui=Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        )
    
    # Static TF Transform
    tf=Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['1', '0', '0', '0', '0', '0', '1', '/map',  '/dummy_link'  ],
    )
    
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                        description='Flag to enable use_sim_time'),
        publish_robot_description,
        joint_state_publisher_node,
        robot_state_publisher,
        rviz_node,
        joint_state_gui,
        # tf,
        
    ])
