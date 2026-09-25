#!/usr/bin/env python3
"""inference_node.py -- Run a trained PPO policy for navigation."""

import math
import os

from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rl_sim.hyperparams import (
    CMD_TOPIC,
    ENV_CONFIG,
    ODOM_TOPIC,
    POSE_TOPIC,
    ROBOT_CONFIG,
    SCAN_TOPIC,
    SETPOINT_TOPIC,
)
from sensor_msgs.msg import LaserScan


def _yaw_from_quaternion(qx, qy, qz, qw):
    """Extract yaw angle from a quaternion."""
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy ** 2 + qz ** 2),
    )


class RLInferenceNode(Node):
    """ROS 2 node that executes a trained PPO policy."""

    def __init__(self):
        super().__init__('rl_sim_inference')

        # -- Parameters --
        self.declare_parameter('model_path', 'ppo_rl_sim_navigation')
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)

        model_path = (
            self.get_parameter('model_path').get_parameter_value().string_value
        )
        self.goal_x = (
            self.get_parameter('goal_x').get_parameter_value().double_value
        )
        self.goal_y = (
            self.get_parameter('goal_y').get_parameter_value().double_value
        )

        # -- Load trained model --
        from stable_baselines3 import PPO

        zip_path = model_path if model_path.endswith('.zip') else model_path + '.zip'
        if not os.path.exists(zip_path):
            self.get_logger().error(f'Model not found: {zip_path}')
            raise FileNotFoundError(f'Model not found: {zip_path}')

        self.model = PPO.load(model_path.replace('.zip', ''))
        self.get_logger().info(f'Model loaded: {zip_path}')

        # -- Robot config --
        self.v_max = ROBOT_CONFIG['v_max']
        self.w_max = ROBOT_CONFIG['w_max']
        self.lidar_samples = ROBOT_CONFIG['lidar_samples']
        self.lidar_total = ROBOT_CONFIG['lidar_total']
        self.lidar_max = ROBOT_CONFIG['lidar_max']
        self.use_lateral = ENV_CONFIG['use_lateral']
        self._max_rho = 12.0

        # -- Sensor state --
        self.scan = None
        self.pose = None   # (x, y, yaw)
        self.odom = None   # (v, w)

        # -- Pub / Sub --
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._scan_cb, 10)
        self.create_subscription(Pose, POSE_TOPIC, self._pose_cb, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 10)
        self.create_subscription(Point, SETPOINT_TOPIC, self._setpoint_cb, 10)

        # -- Control loop timer --
        control_hz = ROBOT_CONFIG['control_hz']
        self.create_timer(1.0 / control_hz, self._control_loop)

        self.get_logger().info(
            f'Inference started -- goal=({self.goal_x:.2f}, {self.goal_y:.2f}), '
            f'freq={control_hz} Hz, lateral={self.use_lateral}'
        )

    # -- Callbacks -----------------------------------------------------------

    def _scan_cb(self, msg: LaserScan):
        self.scan = np.array(msg.ranges, dtype=np.float32)

    def _pose_cb(self, msg: Pose):
        yaw = _yaw_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.pose = (msg.position.x, msg.position.y, yaw)

    def _odom_cb(self, msg: Odometry):
        self.odom = (
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        )

    def _setpoint_cb(self, msg: Point):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.get_logger().info(
            f'Goal updated via /setpoint_pos: '
            f'({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

    # -- Helpers -------------------------------------------------------------

    def _distance_to_goal(self):
        if self.pose is None:
            return 0.0
        dx = self.goal_x - self.pose[0]
        dy = self.goal_y - self.pose[1]
        return math.sqrt(dx * dx + dy * dy)

    def _angle_to_goal(self):
        if self.pose is None:
            return 0.0
        dx = self.goal_x - self.pose[0]
        dy = self.goal_y - self.pose[1]
        bearing = math.atan2(dy, dx)
        alpha = bearing - self.pose[2]
        return math.atan2(math.sin(alpha), math.cos(alpha))

    def _build_observation(self):
        lidar = np.zeros(self.lidar_samples, dtype=np.float32)
        if self.scan is not None and len(self.scan) >= self.lidar_total:
            step = self.lidar_total // self.lidar_samples
            raw = self.scan[::step][: self.lidar_samples]
            raw = np.nan_to_num(raw, nan=self.lidar_max, posinf=self.lidar_max)
            lidar = np.clip(raw, 0.0, self.lidar_max) / self.lidar_max

        rho = self._distance_to_goal()
        alpha = self._angle_to_goal()

        v = 0.0
        w = 0.0
        if self.odom is not None:
            v = self.odom[0]
            w = self.odom[1]

        obs = np.zeros(41, dtype=np.float32)
        obs[:36] = lidar
        obs[36] = rho / self._max_rho
        obs[37] = math.sin(alpha)
        obs[38] = math.cos(alpha)
        obs[39] = np.clip(v / self.v_max, -1.0, 1.0)
        obs[40] = np.clip(w / self.w_max, -1.0, 1.0)
        return obs

    # -- Control loop --------------------------------------------------------

    def _control_loop(self):
        if self.scan is None or self.pose is None:
            return

        # Check if goal reached
        dist = self._distance_to_goal()
        if dist < ENV_CONFIG['goal_tolerance']:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                f'Goal reached: dist={dist:.3f} '
                f'< tol={ENV_CONFIG["goal_tolerance"]}'
            )
            return

        # Run inference
        obs = self._build_observation()
        action, _ = self.model.predict(obs, deterministic=True)

        # Decode action (same mapping as training env)
        if self.use_lateral:
            vx = ((float(action[0]) + 1.0) / 2.0) * self.v_max
            vy = float(action[1]) * self.v_max
            w = float(action[2]) * self.w_max
        else:
            vx = ((float(action[0]) + 1.0) / 2.0) * self.v_max
            vy = 0.0
            w = float(action[1]) * self.w_max

        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RLInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
