import os
import xacro
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

PACKAGE_NAME = 'my_package'

def get_ros2_nodes(*args):
    package_dir = get_package_share_directory(PACKAGE_NAME)
    robot_description_path_rm = os.path.join(package_dir, 'resource', 'rm.urdf')
    robot_description_path_armor = os.path.join(package_dir, 'resource', 'armor.urdf')

    # Control nodes
    rm_controller = Node(
        package=PACKAGE_NAME,
        executable='rm_robot_driver.py',
        namespace='rm',
        output='screen',
        parameters=[
            {'robot_description': robot_description_path_rm},
        ]
    )
    armor_controller = Node(
        package=PACKAGE_NAME,
        executable='armor_robot_driver.py',
        namespace='armor',
        output='screen',
        parameters=[
            {'robot_description': robot_description_path_armor},
        ]
    )

    return [

        # Launch the driver node once the URDF robot is spawned.
        # You might include other nodes to start them with the driver node.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                on_stdout=lambda event: get_webots_driver_node(
                    event, [rm_controller, armor_controller]
                ),
            )
        ),
    ]

def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    robot_description_path_rm = os.path.join(package_dir, 'resource', 'rm.urdf')
    robot_description_path_armor = os.path.join(package_dir, 'resource', 'armor.urdf')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'test.wbt']),
        ros2_supervisor=True
    )

    rm_robot_driver = WebotsController(
        robot_name='rm',
        namespace='rm',
        parameters=[
            {'robot_description': robot_description_path_rm},
        ],
        respawn=True
    )

    armor_robot_driver = WebotsController(
        robot_name='armor',
        namespace='armor',
        parameters=[
            {'robot_description': robot_description_path_armor},
        ],
        respawn=True
    )

    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        rm_robot_driver,
        armor_robot_driver,
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.UnregisterEventHandler(
                        event_handler=reset_handler.event_handler
                    ),
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
        reset_handler
    ]+ get_ros2_nodes())
