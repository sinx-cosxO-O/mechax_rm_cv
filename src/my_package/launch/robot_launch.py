import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description_path_rm = os.path.join(package_dir, 'resource', 'rm.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'test.wbt')
    )

    rm_robot_driver = WebotsController(
        robot_name='rm',
        parameters=[
            {'robot_description': robot_description_path_rm},
        ]
    )


    return LaunchDescription([
        webots,
        rm_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
