import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Pose2D
import math


class PositionController(Node):
    """Nodo de control en lazo cerrado de posicion."""

    def __init__(self):
        super().__init__('position_controller')

        # Parametros configurables
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('kp_linear', 0.4)
        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_linear', 0.0)
        self.declare_parameter('kd_angular', 0.0)
        self.declare_parameter('max_vel_lin', 0.5)
        self.declare_parameter('max_vel_ang', 1.0)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('controller_type', 'pid')
        self.declare_parameter('angle_threshold', 0.2)
        self.declare_parameter('dead_zone', 0.1)
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('pure_pursuit_speed', 0.3)

        # Obtener los parametros y guardarlos en variable
        def p(name): return self.get_parameter(name).value
        self.goal_x = p('goal_x')
        self.goal_y = p('goal_y')
        self.kp_linear = p('kp_linear')
        self.kp_angular = p('kp_angular')
        self.ki_linear = p('ki_linear')
        self.ki_angular = p('ki_angular')
        self.kd_linear = p('kd_linear')
        self.kd_angular = p('kd_angular')
        self.max_vel_lin = p('max_vel_lin')
        self.max_vel_ang = p('max_vel_ang')
        self.goal_tol = p('goal_tolerance')
        self.controller_type = p('controller_type')
        self.angle_threshold = p('angle_threshold')
        self.dead_zone = p('dead_zone')
        self.lookahead_distance = p('lookahead_distance')
        self.pure_pursuit_speed = p('pure_pursuit_speed')

        # Estado interno
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.pose_ready = False
        self.goal_reached = False

        # Estado interno — PID
        self.error_lin_int = 0.0
        self.error_ang_int = 0.0
        self.prev_error_lin = 0.0
        self.prev_error_ang = 0.0
        self.last_time = None

        # Suscriptores
        self.pose_sub = self.create_subscription(
            Pose2D, '/pose', self.pose_callback, 10)
        self.setpoint_sub = self.create_subscription(
            Point, '/setpoint_pos', self.setpoint_callback, 10)

        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(Twist, '/control_error', 10)
        self.error_components_pub = self.create_publisher(Point, '/error_components', 10)

        self.get_logger().info(
            f'Controlador iniciado — tipo={self.controller_type} '
            f'meta=({self.goal_x:.2f}, {self.goal_y:.2f}) m')

    def pose_callback(self, msg: Pose2D):
        """Actualiza pose actual y dispara el calculo de control."""
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta
        self.pose_ready = True
        self.compute_and_publish_control()

    def setpoint_callback(self, msg: Point):
        """Actualiza la meta y reinicia el estado de control."""
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_reached = False
        self.error_lin_int = 0.0
        self.error_ang_int = 0.0
        self.get_logger().info(
            f'Nueva meta: ({self.goal_x:.2f}, {self.goal_y:.2f}) m')

    def compute_and_publish_control(self):
        """Calcula y publica el comando de velocidad."""
        if not self.pose_ready:
            return

        # dt para terminos derivativos
        now = self.get_clock().now()
        dt = 0.0
        if self.last_time is not None:
            dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # PASO 1: Error en componentes (dx, dy) y distancia euclidiana
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        # PASO 2: Angulo hacia el objetivo (heading deseado)
        desired_heading = math.atan2(dy, dx)
        heading_error = desired_heading - self.current_theta
        heading_error = math.atan2(math.sin(heading_error),
                                   math.cos(heading_error))

        # PASO 3: Publicar errores para diagnostico
        err_msg = Twist()
        err_msg.linear.x = distance
        err_msg.angular.z = heading_error
        self.error_pub.publish(err_msg)

        comp_msg = Point()
        comp_msg.x = dx
        comp_msg.y = dy
        self.error_components_pub.publish(comp_msg)

        # PASO 4: Criterio de parada
        twist = Twist()
        if distance < self.goal_tol:
            if not self.goal_reached:
                self.get_logger().info('Meta alcanzada!')
                self.goal_reached = True
            self.cmd_vel_pub.publish(twist)
            return

        # PASO 5: Ley de control segun controller_type
        if self.controller_type == 'pid':
            v_cmd, w_cmd = self._pid(distance, heading_error, dt)
        elif self.controller_type == 'turn_drive':
            v_cmd, w_cmd = self._turn_drive(distance, heading_error)
        elif self.controller_type == 'bang_bang':
            v_cmd, w_cmd = self._bang_bang(distance, heading_error)
        elif self.controller_type == 'pure_pursuit':
            v_cmd, w_cmd = self._pure_pursuit(distance, heading_error)
        else:
            self.get_logger().error(f'Tipo de control desconocido: '
                                    f'{self.controller_type}')
            return

        # PASO 6: Saturacion de velocidades
        v_cmd = max(-self.max_vel_lin, min(self.max_vel_lin, v_cmd))
        w_cmd = max(-self.max_vel_ang, min(self.max_vel_ang, w_cmd))

        # PASO 7: Publicar cmd_vel
        twist.linear.x = v_cmd
        twist.angular.z = w_cmd
        self.cmd_vel_pub.publish(twist)

    def _pid(self, distance, heading_error, dt):
        """PID con anti-windup. ki=kd=0 → P puro."""
        # Terminos derivativos
        if dt > 0.0:
            deriv_lin = (distance - self.prev_error_lin) / dt
            deriv_ang = (heading_error - self.prev_error_ang) / dt
        else:
            deriv_lin = 0.0
            deriv_ang = 0.0

        # Integral con anti-windup por saturacion
        self.error_lin_int += distance * dt
        self.error_ang_int += heading_error * dt

        lin_int_sat = max(-self.max_vel_lin / max(self.ki_linear, 1e-9),
                          min(self.max_vel_lin / max(self.ki_linear, 1e-9),
                              self.error_lin_int))
        ang_int_sat = max(-self.max_vel_ang / max(self.ki_angular, 1e-9),
                          min(self.max_vel_ang / max(self.ki_angular, 1e-9),
                              self.error_ang_int))
        self.error_lin_int = lin_int_sat
        self.error_ang_int = ang_int_sat

        # Ley PID
        v_cmd = (self.kp_linear * distance +
                 self.ki_linear * self.error_lin_int +
                 self.kd_linear * deriv_lin)
        w_cmd = (self.kp_angular * heading_error +
                 self.ki_angular * self.error_ang_int +
                 self.kd_angular * deriv_ang)

        self.prev_error_lin = distance
        self.prev_error_ang = heading_error

        return v_cmd, w_cmd

    def _turn_drive(self, distance, heading_error):
        """Gira primero, avanza solo cuando apunta al objetivo."""
        if abs(heading_error) > self.angle_threshold:
            v_cmd = 0.0
            w_cmd = self.kp_angular * heading_error
        else:
            v_cmd = self.kp_linear * distance
            w_cmd = self.kp_angular * heading_error
        return v_cmd, w_cmd

    def _bang_bang(self, distance, heading_error):
        """Velocidades on/off con zona muerta angular."""
        v_cmd = self.max_vel_lin if distance > self.goal_tol else 0.0
        if heading_error > self.dead_zone:
            w_cmd = self.max_vel_ang
        elif heading_error < -self.dead_zone:
            w_cmd = -self.max_vel_ang
        else:
            w_cmd = 0.0
        return v_cmd, w_cmd

    def _pure_pursuit(self, distance, heading_error):
        """Curvatura lookahead, velocidad lineal constante."""
        v_cmd = self.pure_pursuit_speed
        curvature = 2.0 * math.sin(heading_error) / self.lookahead_distance
        w_cmd = curvature * v_cmd
        return v_cmd, w_cmd


def main(args=None):
    """Punto de entrada del nodo."""
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
