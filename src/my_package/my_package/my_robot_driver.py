import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # 初始化6个电机
        self.__wheels = [self.__robot.getDevice('LFwheel'), 
                         self.__robot.getDevice('RFwheel'), 
                         self.__robot.getDevice('RBwheel'), 
                         self.__robot.getDevice('LBwheel'), 
                         self.__robot.getDevice('yaw'),
                         self.__robot.getDevice('pitch')]

        for wheel in self.__wheels:
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0.0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        for wheel in self.__wheels:
            wheel.setVelocity(forward_speed)

        # self.__left_motor.setVelocity(command_motor_left)
        # self.__right_motor.setVelocity(command_motor_right)
