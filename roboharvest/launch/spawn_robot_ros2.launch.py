import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit
import launch_ros
from launch_ros.actions import Node
import xacro
import random

# this is the function launch  system will look for


def generate_launch_description():

    ####### DATA INPUT ##########
    xacro_file = "robo.urdf.xacro"

    package_description = "roboharvest"

    # Position and orientation
    # [X, Y, Z]
    position = [-9.0, -6.0, 0.5]
    # position = [0.0, 0.0, 0.5]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "robo"
    ####### DATA INPUT END ##########

    # Path to robot model XACRO File
    robot_desc_path = os.path.join(get_package_share_directory(
        package_description), "urdf", xacro_file)


    # Robot Description in XACRO Format
    robot_desc = xacro.process_file(robot_desc_path)

    # Robot Description in XML Format
    xml = robot_desc.toxml()
    
    # Entity Name
    entity_name = robot_base_name+"-"+str(random.random())

   

    # Spawn ROBOT Set Gazebo (Does not spwan robot only communicates with the Gazebo Client)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

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

    # Launch Config for Simulation Time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot State Publisher Node
    robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': xml}],
        output="screen"
    )

    # Joint State Publisher Node

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )


    # Joint State Broadcaster Node
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )


    # Joint Velocity Controller Node
    robot_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    )

    # Joint Position Controller Node
    robot_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_postion_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_position_controller_spawner],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_velocity_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_velocity_controller_spawner],
        )
    )

    # Static TF Transform
    tf=Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['1', '0', '0', '0', '0', '0', '1', '/map',  '/dummy_link'  ],
    )

    # create and return launch description object
    return LaunchDescription(
        [   
            launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
            launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
            publish_robot_description,
            joint_state_publisher_node,
            robot_state_publisher,
            spawn_robot,
            joint_state_broadcaster_spawner,
            delay_robot_postion_controller_spawner_after_joint_state_broadcaster_spawner,
            delay_robot_velocity_controller_spawner_after_joint_state_broadcaster_spawner,
            # tf
        ]
    )