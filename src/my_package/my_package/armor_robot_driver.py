import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        #初始化
        self.__leds = [self.__robot.getDevice('led1'), 
                         self.__robot.getDevice('led2')]

        for led in self.__leds:
            led.set(1)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        # ?存疑
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

