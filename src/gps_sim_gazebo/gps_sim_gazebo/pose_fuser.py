import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D


class PoseFuser(Node):
    """Funde posicion (odometria GPS) y orientacion (IMU) en /pose (Pose2D)."""

    def __init__(self):
        super().__init__('pose_fuser')

        # Estado: posicion GPS, yaw, flags
        self.gps_x = 0.0
        self.gps_y = 0.0
        self.has_gps = False
        self.yaw = 0.0
        self.has_yaw = False

        self.sub_gps = self.create_subscription(Odometry, '/odometry/gps', self.gps_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.pub = self.create_publisher(Pose2D, '/pose', 10)

        self.get_logger().info('PoseFuser started — /odometry/gps (position) + /imu (yaw) → /pose')

    def imu_callback(self, msg):
        """Cuaternion → yaw (world-frame)."""
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_yaw = True
        self.try_publish()

    def gps_callback(self, msg):
        """Extrae X, Y de odometria GPS."""
        self.gps_x = msg.pose.pose.position.x
        self.gps_y = msg.pose.pose.position.y
        self.has_gps = True
        self.try_publish()

    def try_publish(self):
        """Publica /pose solo si ambos datos disponibles."""
        if not self.has_gps or not self.has_yaw:
            return
        pose = Pose2D()
        pose.x = self.gps_x
        pose.y = self.gps_y
        # Envoltura theta a [-pi, pi]
        pose.theta = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.pub.publish(pose)


def main(args=None):
    """Punto de entrada del nodo."""
    rclpy.init(args=args)
    rclpy.spin(PoseFuser())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
