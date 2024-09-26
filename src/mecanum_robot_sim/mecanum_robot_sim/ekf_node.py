import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster

import numpy as np
from scipy.spatial.transform import Rotation as R

class EKFNode(Node):

    def __init__(self):

        super().__init__('ekf_node')

        self.declare_parameter('dt', 0.01)
        self.declare_parameter('model_noise', [0.01,0.0,0.0,0.0,0.01,0.0,0.0,0.0,0.1])
        self.declare_parameter('sensor_noise', 0.01)

        self.q = np.zeros((3,1))

        self.Q = np.array(self.get_parameter('model_noise').get_parameter_value().double_array_value).reshape((3,3))
        self.R = self.get_parameter('sensor_noise').get_parameter_value().double_value
        self.P = 1e-9*np.identity(3)

        self.odometry = Odometry()
        self.imu = Imu()
        self.filtered_odometry = Odometry()

        self.create_subscription(Odometry, 'wheel/odometry', self.get_odometry_callback, 10)
        self.create_subscription(Imu, 'imu/data', self.get_imu_callback, 10)

        self.odom_publisher = self.create_publisher(Odometry, 'filtered_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.create_timer(self.dt, self.ekf_update_callback)

    def get_odometry_callback(self, odometry_msg):

        self.odometry = odometry_msg

    def get_imu_callback(self, imu_msg):

        self.imu = imu_msg

    def ekf_update_callback(self):

        ## 1. Model Prediction
        q_dot = np.array([[self.odometry.twist.twist.linear.x, self.odometry.twist.twist.linear.y, self.odometry.twist.twist.angular.z]]).T

        dq = q_dot*self.dt

        rot_m = np.array([[np.cos(self.q[2,0]), -np.sin(self.q[2,0]), 0],
                          [np.sin(self.q[2,0]), np.cos(self.q[2,0]), 0],
                          [0, 0, 1]])

        self.q = self.q + np.dot(rot_m, dq)

        r = R.from_quat([self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w])
        imu_theta = r.as_rotvec()[-1]

        X = self.q

        ## 2. Model Linealization
        F = np.array([[1.0,0.0,(-dq[0,0]*np.sin(self.q[2,0]) - dq[1,0]*np.cos(self.q[2,0]))],
                      [0.0,1.0,(dq[0,0]*np.cos(self.q[2,0]) - dq[1,0]*np.sin(self.q[2,0]))],
                      [0.0,0.0,1.0]])
        L = np.identity(3)
        H = np.array([[0.0,0.0,1.0]])
        M = np.array([[1.0]])

        ## 3. Correction
        self.P = np.dot(np.dot(F,self.P),F.T) + np.dot(np.dot(L,self.Q),L.T)

        K = np.dot(np.dot(self.P,H.T),np.linalg.inv((np.dot(np.dot(H,self.P),H.T) + np.dot(np.dot(M,self.R),M.T))))
        I = np.identity(3)
        self.P = np.dot(np.dot((I - np.dot(K,H)), self.P), (I - np.dot(K,H)).T) + np.dot(np.dot(K,self.R),K.T)

        y = imu_theta
        h = X[2,0]

        X = X + np.dot(K, (y - h))

        self.q[0,0] = X[0,0]
        self.q[1,0] = X[1,0]
        self.q[2,0] = X[2,0]

        ## TF transform
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.q[0,0]
        t.transform.translation.y = self.q[1,0]
        t.transform.translation.z = 0.0

        r = R.from_euler("xyz", [0,0,self.q[2,0]])
        quat = r.as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

        ## Odometry message
        self.filtered_odometry.header.stamp = self.get_clock().now().to_msg()
        self.filtered_odometry.header.frame_id = 'odom'

        self.filtered_odometry.child_frame_id = 'base_footprint'
        self.filtered_odometry.pose.pose.position.x = self.q[0,0]
        self.filtered_odometry.pose.pose.position.y = self.q[1,0]
        self.filtered_odometry.pose.pose.position.z = 0.0

        self.filtered_odometry.pose.pose.orientation.x = quat[0]
        self.filtered_odometry.pose.pose.orientation.y = quat[1]
        self.filtered_odometry.pose.pose.orientation.z = quat[2]
        self.filtered_odometry.pose.pose.orientation.w = quat[3]

        self.odom_publisher.publish(self.filtered_odometry)


def main(args=None):

    rclpy.init(args=args)
    ekf_node = EKFNode()
    rclpy.spin(ekf_node)

    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()