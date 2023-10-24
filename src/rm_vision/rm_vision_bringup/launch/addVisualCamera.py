import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import node_params, launch_params, robot_state_publisher, tracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown

    from launch import LaunchDescription

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('my_package'), 'launch'),
         '/robot_launch.py'])
      )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'armor_detector:='+launch_params['detector_log_level']], 
            on_exit=Shutdown(),
        )

    cam_detector = get_camera_detector_container(camera_node)

    # detector_node = Node(
    #     package='armor_detector',
    #     executable='armor_detector_node',
    #     emulate_tty=True,
    #     output='both',
    #     parameters=[node_params],
    #     arguments=['--ros-args', '--log-level',
    #                'armor_detector:='+launch_params['detector_log_level']],
    # )

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )

    return LaunchDescription([
        robot_state_publisher,
        cam_detector,
        # detector_node,
        delay_tracker_node,
    ])
