import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import node_params, launch_params, robot_state_publisher, tracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,    # 软件包
            plugin=plugin,      # 插件，相当于调用其中的哪个类
            name='camera_node', # 节点名称
            parameters=[node_params], # 参数列表，保存在config中
            #启用了节点间的进程内通信，这意味着节点之间可以更高效地进行通信
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            # 用于容纳和管理多个ROS节点的容器类，
            # 它允许将多个节点组合在一起并一起运行。
            # 创建了一个容器，包含相机节点和目标检测节点。
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                # 配置目标检测节点
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            # 配置节点的输出方式，可以是 "screen"、"log" 或 "both"。
            # 在这里，节点的输出被设置为 "both"，表示输出
            # 会同时显示在屏幕和日志中
            output='both',
            # 设置是否模拟终端。如果设置为 True，节点将模拟终端，以便在终端上显示输出
            emulate_tty=True,
            # 用于传递额外的ROS参数给容器和容器中的节点，在这里设置节点的日志级别
            ros_arguments=['--ros-args', '--log-level',
                           'armor_detector:='+launch_params['detector_log_level']], # 将不同的日志级别应用于不同的节点
            on_exit=Shutdown(),
        )

    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    mv_camera_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')

    if (launch_params['camera'] == 'hik'):
        cam_detector = get_camera_detector_container(hik_camera_node)
    elif (launch_params['camera'] == 'mv'):
        cam_detector = get_camera_detector_container(mv_camera_node)

    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=['--ros-args', '--log-level',
                       'serial_driver:='+launch_params['serial_log_level']],
    )

    # delay_serial_node 会以1.5秒的周期触发执行串口驱动节点，
    # 而 delay_tracker_node 会以2.0秒的周期触发执行追踪节点。
    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )

    return LaunchDescription([
        robot_state_publisher, # 可视化
        cam_detector,          # 相机+detector
        delay_serial_node,     # 串口通信
        delay_tracker_node,    # tracker
    ])
