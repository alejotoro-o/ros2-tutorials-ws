import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster

import numpy as np
from scipy.spatial.transform import Rotation as R

HALF_DISTANCE_BETWEEN_WHEELS = 0.06
#EPSILON = 0.01
WHEEL_RADIUS = 0.03

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        self.x = 0
        self.y = 0
        self.theta = 0

        self.joint_states = JointState()
        self.joint_states.position = [0.0,0.0]
        self.odometry = Odometry()

        self.create_subscription(JointState, 'wheels_encoders', self.joint_states_callback, 1)
        self.prev_pos_motor_left = 0
        self.prev_pos_motor_right = 0

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.dt = 0.01
        self.create_timer(self.dt, self.odometry_callback)

    def joint_states_callback(self, joint_states_msg):

        self.joint_states = joint_states_msg

    def get_robot_speed(self):

        vel_motor_left = (self.joint_states.position[0] - self.prev_pos_motor_left)/self.dt
        vel_motor_right = (self.joint_states.position[1] - self.prev_pos_motor_right)/self.dt

        self.prev_pos_motor_left = self.joint_states.position[0]
        self.prev_pos_motor_right = self.joint_states.position[1]

        v_x = (WHEEL_RADIUS/2)*(vel_motor_left + vel_motor_right)
        theta_dot = (WHEEL_RADIUS/(2*HALF_DISTANCE_BETWEEN_WHEELS))*(vel_motor_right - vel_motor_left)
        #theta_dot = (WHEEL_RADIUS/(2*(HALF_DISTANCE_BETWEEN_WHEELS + EPSILON)))*(vel_motor_right - vel_motor_left)

        return v_x, theta_dot


    def odometry_callback(self):

        v_x, theta_dot = self.get_robot_speed()

        d_x = v_x*self.dt
        d_theta = theta_dot*self.dt

        self.x = self.x + (d_x*np.cos(self.theta))
        self.y = self.y + (d_x*np.sin(self.theta))
        self.theta = self.theta + d_theta

        ## TF transform
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        r = R.from_euler("xyz", [0,0,self.theta])
        quat = r.as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        #self.tf_broadcaster.sendTransform(t)

        ## Odometry message
        self.odometry.header.stamp = self.get_clock().now().to_msg()
        self.odometry.header.frame_id = 'odom'

        self.odometry.child_frame_id = 'base_link'
        self.odometry.pose.pose.position.x = self.x
        self.odometry.pose.pose.position.y = self.y
        self.odometry.pose.pose.position.z = 0.0

        self.odometry.pose.pose.orientation.x = quat[0]
        self.odometry.pose.pose.orientation.y = quat[1]
        self.odometry.pose.pose.orientation.z = quat[2]
        self.odometry.pose.pose.orientation.w = quat[3]

        self.odometry.twist.twist.linear.x = v_x
        self.odometry.twist.twist.angular.z = theta_dot

        self.odom_publisher.publish(self.odometry)


def main(args=None):

    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)

    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()