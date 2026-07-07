import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D


class NavsatToCartesian(Node):
    """Convierte lat/lon (NavSatFix) a coordenadas Cartesianas X, Y (Haversine)."""

    def __init__(self):
        super().__init__('navsat_to_cartesian')

        # Parametros: lat_ref, lon_ref, heading_deg
        self.declare_parameter('lat_ref', -23.6509)
        self.declare_parameter('lon_ref', -70.3975)
        self.declare_parameter('heading_deg', 0.0)

        self.lat_ref = math.radians(self.get_parameter('lat_ref').value)
        self.lon_ref = math.radians(self.get_parameter('lon_ref').value)
        heading_deg = self.get_parameter('heading_deg').value
        self.heading_rad = math.radians(heading_deg)

        # Radio medio terrestre [m]
        self.R = 6371000.0

        self.sub = self.create_subscription(NavSatFix, '/navsat', self.callback, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odometry/gps', 10)
        self.pub_pose = self.create_publisher(Pose2D, '/gps_pose', 10)

        self.get_logger().info(
            f'NavsatToCartesian started — ref=({math.degrees(self.lat_ref):.4f}, '
            f'{math.degrees(self.lon_ref):.4f}), heading={heading_deg} deg'
        )

    def callback(self, msg):
        """Valida status, Haversine, rotacion heading, publica Odometry + Pose2D."""
        if msg.status.status < 0:
            return

        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)

        dlat = lat - self.lat_ref
        dlon = lon - self.lon_ref
        lat_mid = (lat + self.lat_ref) / 2.0

        # Haversine: ex = R·dlon·cos(lat_mid), ny = R·dlat
        ex = self.R * dlon * math.cos(lat_mid)
        ny = self.R * dlat

        # Rotacion alineacion heading_rad
        ch = math.cos(-self.heading_rad)
        sh = math.sin(-self.heading_rad)
        x = ch * ex - sh * ny
        y = sh * ex + ch * ny

        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(odom)

        pose = Pose2D()
        pose.x = x
        pose.y = y
        pose.theta = 0.0
        self.pub_pose.publish(pose)


def main(args=None):
    """Punto de entrada del nodo."""
    rclpy.init(args=args)
    rclpy.spin(NavsatToCartesian())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
