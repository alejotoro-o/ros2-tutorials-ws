import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster

HALF_DISTANCE_BETWEEN_WHEELS = 0.05
WHEEL_RADIUS = 0.03

class RobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        ## Motor Config
        self.__left_motor = self.__robot.getDevice('motor2')
        self.__right_motor = self.__robot.getDevice('motor1')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        ## Encoder Config
        self.__encoder1 = self.__robot.getDevice('encoder1')
        self.__encoder1.enable(32)

        self.__encoder2 = self.__robot.getDevice('encoder2')
        self.__encoder2.enable(32)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

        self.__joint_state_publisher = self.__node.create_publisher(JointState, 'wheels_encoders', 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        ## Robot control
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = -(forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = -(forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

        ## Publish joint states
        joint_states_header = Header()
        joint_states_header.stamp = self.__node.get_clock().now().to_msg()
        joint_states_header.frame_id = ''

        joint_states = JointState()
        joint_states.header = joint_states_header
        joint_states.name = ['motor1', 'motor2']
        joint_states.position = [
            float(self.__encoder1.getValue()),
            float(self.__encoder2.getValue())
        ]

        self.__joint_state_publisher.publish(joint_states)