import rclpy
from geometry_msgs.msg import Twist, Pose, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster

import numpy as np
from scipy.spatial.transform import Rotation as R

L_X = 0.1
L_Y = 0.12
WHEEL_RADIUS = 0.04

class MecanumRobotDriver:
    def init(self, webots_node, properties):

        ## Robot node
        self.__robot = webots_node.robot        

        ## Configure Lidar
        self.__lidar = self.__robot.getDevice('lidar')
        self.__lidar.enable(32)
        self.__lidar.enablePointCloud()

        ## Configure position sensors (Encoders)
        self.__encoder1 = self.__robot.getDevice('encoder1')
        self.__encoder1.enable(32)

        self.__encoder2 = self.__robot.getDevice('encoder2')
        self.__encoder2.enable(32)

        self.__encoder3 = self.__robot.getDevice('encoder3')
        self.__encoder3.enable(32)

        self.__encoder4 = self.__robot.getDevice('encoder4')
        self.__encoder4.enable(32)

        ## Configure motors
        self.__front_left_motor = self.__robot.getDevice('motor1')
        self.__front_right_motor = self.__robot.getDevice('motor2')
        self.__back_left_motor = self.__robot.getDevice('motor3')
        self.__back_right_motor = self.__robot.getDevice('motor4')

        self.__front_left_motor.setPosition(float('inf'))
        self.__front_left_motor.setVelocity(0)

        self.__front_right_motor.setPosition(float('inf'))
        self.__front_right_motor.setVelocity(0)

        self.__back_left_motor.setPosition(float('inf'))
        self.__back_left_motor.setVelocity(0)

        self.__back_right_motor.setPosition(float('inf'))
        self.__back_right_motor.setVelocity(0)

        self.__target_twist = Twist()

        ## Node configuration
        rclpy.init(args=None)
        self.__node = rclpy.create_node('mecanum_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__joint_states_publisher = self.__node.create_publisher(JointState, 'wheels_encoders', 1)
        self.tf_broadcaster = TransformBroadcaster(self.__node)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        ## Set motor speeds
        V_x = self.__target_twist.linear.x
        V_y = self.__target_twist.linear.y
        thetap = self.__target_twist.angular.z

        command_motor_front_left = (V_x - V_y - (L_X + L_Y)*thetap) / WHEEL_RADIUS
        command_motor_front_right = (V_x + V_y + (L_X + L_Y)*thetap) / WHEEL_RADIUS
        command_motor_back_left = (V_x + V_y - (L_X + L_Y)*thetap) / WHEEL_RADIUS
        command_motor_back_right = (V_x - V_y + (L_X + L_Y)*thetap) / WHEEL_RADIUS

        self.__front_left_motor.setVelocity(command_motor_front_left) 
        self.__front_right_motor.setVelocity(command_motor_front_right) 
        self.__back_left_motor.setVelocity(command_motor_back_left)
        self.__back_right_motor.setVelocity(command_motor_back_right) 

        ## Publish joint states
        joint_states_header = Header()
        joint_states_header.stamp = self.__node.get_clock().now().to_msg()
        joint_states_header.frame_id = ''

        joint_states = JointState()
        joint_states.header = joint_states_header
        joint_states.name = ['motor1', 'motor2', 'motor3', 'motor4']
        joint_states.position = [
            float(self.__encoder1.getValue()),
            float(self.__encoder2.getValue()),
            float(self.__encoder3.getValue()),
            float(self.__encoder4.getValue())
        ]

        self.__joint_states_publisher.publish(joint_states)

