import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster

import numpy as np
from scipy.spatial.transform import Rotation as R

L_X = 0.1
L_Y = 0.12
WHEEL_RADIUS = 0.04

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        self.q = np.zeros((3, 1))
        # self.x = 0
        # self.y = 0
        # self.theta = 0

        self.joint_states = JointState()
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0]
        self.odometry = Odometry()

        self.create_subscription(JointState, 'wheels_encoders', self.joint_states_callback, 1)
        self.prev_pos_motors = np.zeros(4)
        # self.prev_pos_motor_front_right = 0
        # self.prev_pos_motor_back_left = 0
        # self.prev_pos_motor_back_right = 0

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.dt = 0.01
        self.create_timer(self.dt, self.odometry_callback)

    def joint_states_callback(self, joint_states_msg):

        self.joint_states = joint_states_msg

    def get_robot_speed(self):

        vel_motors = (np.array(self.joint_states.position) - self.prev_pos_motors)/self.dt

        # vel_motor_front_left = (self.joint_states.position[0] - self.prev_pos_motor_front_left)/self.dt
        # vel_motor_front_right = (self.joint_states.position[1] - self.prev_pos_motor_front_right)/self.dt
        # vel_motor_back_left = (self.joint_states.position[2] - self.prev_pos_motor_back_left)/self.dt
        # vel_motor_back_right = (self.joint_states.position[3] - self.prev_pos_motor_back_right)/self.dt

        self.prev_pos_motors = self.joint_states.position

        # self.prev_pos_motor_front_left = self.joint_states.position[0]
        # self.prev_pos_motor_front_right = self.joint_states.position[1]
        # self.prev_pos_motor_back_left = self.joint_states.position[2]
        # self.prev_pos_motor_back_right = self.joint_states.position[3]

        d = (L_X + L_Y)
        J = (WHEEL_RADIUS/4)*np.array([[1, 1, 1, 1],
                                       [-1, 1, 1, -1],
                                       [-1/d, 1/d, -1/d, 1/d]])

        q_dot = np.dot(J, vel_motors.reshape((4, 1)))

        return q_dot


    def odometry_callback(self):

        q_dot = self.get_robot_speed()

        dq = q_dot*self.dt

        rot_m = np.array([[np.cos(self.q[2,0]), -np.sin(self.q[2,0]), 0],
                          [np.sin(self.q[2,0]), np.cos(self.q[2,0]), 0],
                          [0, 0, 1]])

        self.q = self.q + np.dot(rot_m, dq)

        # self.x = self.x + (d_x*np.cos(self.theta))
        # self.y = self.y + (d_x*np.sin(self.theta))
        # self.theta = self.theta + d_theta

        ## TF transform
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.q[0,0]
        t.transform.translation.y = self.q[1,0]
        t.transform.translation.z = 0.0

        r = R.from_euler("xyz", [0, 0, self.q[2,0]])
        quat = r.as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

        ## Odometry message
        self.odometry.header.stamp = self.get_clock().now().to_msg()
        self.odometry.header.frame_id = 'odom'

        self.odometry.child_frame_id = 'base_link'
        self.odometry.pose.pose.position.x = self.q[0,0]
        self.odometry.pose.pose.position.y = self.q[1,0]
        self.odometry.pose.pose.position.z = 0.0

        self.odometry.pose.pose.orientation.x = quat[0]
        self.odometry.pose.pose.orientation.y = quat[1]
        self.odometry.pose.pose.orientation.z = quat[2]
        self.odometry.pose.pose.orientation.w = quat[3]

        self.odometry.twist.twist.linear.x = q_dot[0,0]
        self.odometry.twist.twist.linear.y = q_dot[1,0]
        self.odometry.twist.twist.angular.z = q_dot[2,0]

        ## Covariance
        ## Position
        self.odometry.pose.covariance[0] = 0.01
        self.odometry.pose.covariance[7] = 0.01
        self.odometry.pose.covariance[14] = 0.1
        ## Twist
        self.odometry.pose.covariance[21] = 0.01
        self.odometry.pose.covariance[28] = 0.01
        self.odometry.pose.covariance[-1] = 0.01

        self.odom_publisher.publish(self.odometry)


def main(args=None):

    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)

    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()