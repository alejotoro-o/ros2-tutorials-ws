import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, Pose2D


class SetpointClient(Node):
    """Cliente interactivo de setpoint por stdin (X,Y | s | c)."""

    def __init__(self):
        super().__init__('setpoint_client')

        # Publicador setpoint + suscriptor pose + estado
        self.pub = self.create_publisher(Point, '/setpoint_pos', 10)
        self.pose_sub = self.create_subscription(
            Pose2D, '/pose', self.pose_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.pose_ready = False
        self.last_setpoint = None

        self.get_logger().info('SetpointClient iniciado')

    def pose_callback(self, msg: Pose2D):
        """Actualiza posicion actual."""
        self.current_x = msg.x
        self.current_y = msg.y
        self.pose_ready = True

    def publish_setpoint(self, x, y):
        """Publica Point en /setpoint_pos."""
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.pub.publish(msg)

    def input_loop(self):
        """Bucle stdin: parsea comandos y publica setpoints."""
        print('setpoint_client — Ingrese X,Y | s=stop | c=continue | Ctrl+C=salir')
        while rclpy.ok():
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                line = line.strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not line:
                continue

            # Stop: setpoint en posicion actual
            if line == 's':
                if self.pose_ready:
                    self.publish_setpoint(self.current_x, self.current_y)
                    print(f'Detenido en ({self.current_x:.2f}, {self.current_y:.2f})')
                else:
                    print('Posicion no disponible via /pose')
            # Continue: reenvia ultimo setpoint
            elif line == 'c':
                if self.last_setpoint:
                    x, y = self.last_setpoint
                    self.publish_setpoint(x, y)
                    print(f'Reanudando setpoint ({x:.2f}, {y:.2f})')
                else:
                    print('No hay setpoint previo')
            # X,Y: nuevo setpoint
            else:
                try:
                    parts = line.split(',')
                    if len(parts) != 2:
                        print('Formato invalido. Use: X,Y')
                        continue
                    x = float(parts[0])
                    y = float(parts[1])
                    self.last_setpoint = (x, y)
                    self.publish_setpoint(x, y)
                    print(f'Setpoint enviado: X={x:.2f}, Y={y:.2f}')
                except ValueError:
                    print('Formato invalido. Use: X,Y')

        self.get_logger().info('SetpointClient finalizado')


def main(args=None):
    """Punto de entrada — MultiThreadedExecutor + hilo daemon de stdin."""
    rclpy.init(args=args)
    node = SetpointClient()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=node.input_loop, daemon=True)
    thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
