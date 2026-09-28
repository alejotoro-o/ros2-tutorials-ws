#!/usr/bin/env python3
"""rl_env.py -- Gymnasium + ROS 2 environment bridge for Webots."""

import math

from geometry_msgs.msg import Pose, Twist
import gymnasium as gym
from gymnasium import spaces
from interfaces.srv import ResetSim
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rl_sim.hyperparams import (
    CMD_TOPIC,
    ENV_CONFIG,
    OBSTACLES,
    ODOM_TOPIC,
    POSE_TOPIC,
    REWARD_CONFIG,
    ROBOT_CONFIG,
    SCAN_TOPIC,
)
from sensor_msgs.msg import LaserScan


def _yaw_from_quaternion(qx, qy, qz, qw):
    """Extract yaw angle from a quaternion."""
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy ** 2 + qz ** 2),
    )


class RLEnv(gym.Env, Node):
    """Gymnasium environment that bridges a Webots mecanum-robot sim."""

    metadata = {'render_modes': []}

    def __init__(self, node_name='rl_sim_env'):
        Node.__init__(self, node_name)

        # -- Action / observation spaces --
        self.use_lateral = ENV_CONFIG['use_lateral']
        action_dim = 3 if self.use_lateral else 2
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(action_dim,), dtype=np.float32,
        )
        # obs: 36 lidar + rho_norm + sin(alpha) + cos(alpha) + v_norm + w_norm
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=(41,), dtype=np.float32,
        )

        # -- Robot limits --
        self.v_max = ROBOT_CONFIG['v_max']
        self.w_max = ROBOT_CONFIG['w_max']
        self.lidar_samples = ROBOT_CONFIG['lidar_samples']
        self.lidar_total = ROBOT_CONFIG['lidar_total']
        self.lidar_max = ROBOT_CONFIG['lidar_max']
        self.control_step = 1.0 / ROBOT_CONFIG['control_hz']

        # -- MDP config --
        self.goal_tol = ENV_CONFIG['goal_tolerance']
        self.collision_d = ENV_CONFIG['collision_threshold']
        self.obstacle_margin = ENV_CONFIG['obstacle_margin']
        self.goal_margin = ENV_CONFIG['goal_margin']
        self.spawn_clearance = ENV_CONFIG['spawn_clearance']
        self.max_steps = ENV_CONFIG['max_steps']
        self._max_rho = 12.0  # normalisation factor for goal distance

        # -- Episode state --
        self.scan = None
        self.pose = None       # (x, y, yaw)
        self.odom = None       # (v, w)
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.prev_dist = 0.0
        self.steps = 0

        # -- Publishers / subscribers --
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._scan_cb, 10)
        self.create_subscription(Pose, POSE_TOPIC, self._pose_cb, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 10)

        # -- Reset service client --
        self.reset_client = self.create_client(ResetSim, '/reset_sim')
        self.get_logger().info('Waiting for /reset_sim service...')
        self.reset_client.wait_for_service()
        self.get_logger().info('/reset_sim service available.')

        self.get_logger().info(
            f'RLEnv started -- lidar {self.lidar_samples}/{self.lidar_total}, '
            f'v_max={self.v_max}, w_max={self.w_max}, '
            f'use_lateral={self.use_lateral}'
        )

    # -- Sensor callbacks ----------------------------------------------------

    def _scan_cb(self, msg: LaserScan):
        self.scan = np.array(msg.ranges, dtype=np.float32)

    def _pose_cb(self, msg):
        """Receive geometry_msgs/Pose from /webots_pose."""
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

    # -- Spin helpers --------------------------------------------------------

    def _spin_for(self, duration):
        """Process ROS callbacks for *duration* seconds."""
        start = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= duration:
                break

    def _spin_until_data(self, timeout=10.0):
        """Wait until all sensor buffers are populated, or timeout."""
        start = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if (self.scan is not None
                    and self.pose is not None
                    and self.odom is not None):
                return True
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= timeout:
                self.get_logger().warn('Timeout waiting for sensor data')
                return False
        return False

    # -- Goal helpers --------------------------------------------------------

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

    # -- Sampling ------------------------------------------------------------

    def _sample_goal(self):
        for _ in range(100):
            x = np.random.uniform(
                ENV_CONFIG['goal_x_min'], ENV_CONFIG['goal_x_max'])
            y = np.random.uniform(
                ENV_CONFIG['goal_y_min'], ENV_CONFIG['goal_y_max'])
            if not self._is_in_obstacle(x, y, self.goal_margin):
                return x, y
        return 0.0, 0.0

    def _is_in_obstacle(self, x, y, margin=None):
        """
        Return True if (x, y) lies within *margin* of an obstacle box.

        ``margin`` defaults to the static-map obstacle margin but callers can
        pass a larger value (e.g. for goal sampling).
        """
        if margin is None:
            margin = self.obstacle_margin
        for (xmin, ymin, xmax, ymax) in OBSTACLES:
            if (xmin - margin <= x <= xmax + margin
                    and ymin - margin <= y <= ymax + margin):
                return True
        return False

    def _sample_spawn_pose(self, max_attempts=30):
        """
        Sample a spawn pose with a real safety margin.

        A candidate must be clear of the static obstacle map *and* have a
        lidar reading above ``spawn_clearance``, which is well above the
        collision threshold used to terminate an episode.
        """
        for _ in range(max_attempts):
            x = np.random.uniform(
                ENV_CONFIG['spawn_x_min'], ENV_CONFIG['spawn_x_max'])
            y = np.random.uniform(
                ENV_CONFIG['spawn_y_min'], ENV_CONFIG['spawn_y_max'])
            if self._is_in_obstacle(x, y, self.obstacle_margin):
                continue
            yaw = np.random.uniform(-math.pi, math.pi)
            # Teleport and verify with the actual lidar
            self._teleport(x, y, yaw)
            if self.scan is not None and len(self.scan) > 0:
                min_dist = float(np.min(self.scan))
                if min_dist > self.spawn_clearance:
                    return x, y, yaw
        self.get_logger().warn(
            'Could not find a safe spawn, using arena centre')
        self._teleport(0.0, 0.0, 0.0)
        return 0.0, 0.0, 0.0

    def _teleport(self, x, y, yaw):
        """Teleport robot and wait for fresh sensor data."""
        req = ResetSim.Request()
        req.pose.x = float(x)
        req.pose.y = float(y)
        req.pose.theta = float(yaw)
        future = self.reset_client.call_async(req)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
        self.cmd_pub.publish(Twist())
        self._spin_for(0.3)
        self.scan = None
        self.pose = None
        self.odom = None
        self._spin_until_data(timeout=5.0)

    # -- Observation builder -------------------------------------------------

    def _build_observation(self):
        lidar = np.zeros(self.lidar_samples, dtype=np.float32)
        if self.scan is not None and len(self.scan) > 0:
            raw = np.array(self.scan, dtype=np.float32)
            indices = np.linspace(
                0, len(raw) - 1, self.lidar_samples, dtype=int)
            raw = raw[indices]
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

    # -- Reward --------------------------------------------------------------

    def _compute_reward(self, w):
        dist = self._distance_to_goal()
        min_scan = (
            float(np.min(self.scan))
            if self.scan is not None and len(self.scan) > 0
            else self.lidar_max
        )

        reward = 0.0
        terminated = False

        # Progress towards goal
        reward += REWARD_CONFIG['progress_weight'] * (self.prev_dist - dist)
        self.prev_dist = dist

        # Heading alignment -- small bonus for facing the goal
        alpha = self._angle_to_goal()
        reward += REWARD_CONFIG['heading_weight'] * math.cos(alpha)

        # Step penalty
        reward += REWARD_CONFIG['step_penalty']

        # Proximity to obstacles
        if min_scan < REWARD_CONFIG['proximity_threshold']:
            reward -= (
                REWARD_CONFIG['proximity_weight']
                * (REWARD_CONFIG['proximity_threshold'] - min_scan)
            )

        # Goal reached
        if dist < self.goal_tol:
            reward += REWARD_CONFIG['goal_reward']
            terminated = True

        # Collision
        if min_scan < self.collision_d:
            reward += REWARD_CONFIG['collision_penalty']
            terminated = True

        return reward, terminated

    # -- Gymnasium interface -------------------------------------------------

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Sample a safe spawn pose. This teleports the robot there and waits
        # for fresh sensor data, so there is no need to teleport again.
        sx, sy, _ = self._sample_spawn_pose()
        self.goal_x, self.goal_y = self._sample_goal()

        self.steps = 0
        self.prev_dist = self._distance_to_goal()
        obs = self._build_observation()

        self.get_logger().info(
            f'Episode started -- spawn=({sx:.2f}, {sy:.2f}), '
            f'goal=({self.goal_x:.2f}, {self.goal_y:.2f}), '
            f'dist={self.prev_dist:.2f}'
        )
        return obs, {}

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)

        # Decode action
        if self.use_lateral:
            vx = ((float(action[0]) + 1.0) / 2.0) * self.v_max
            vy = float(action[1]) * self.v_max
            w = float(action[2]) * self.w_max
        else:
            vx = ((float(action[0]) + 1.0) / 2.0) * self.v_max
            vy = 0.0
            w = float(action[1]) * self.w_max

        # Publish velocity command
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

        # Let simulation advance
        self._spin_for(self.control_step)

        # Build observation and compute reward
        obs = self._build_observation()
        reward, terminated = self._compute_reward(w)

        self.steps += 1
        truncated = self.steps >= self.max_steps

        info = {
            'distance': self._distance_to_goal(),
            'steps': self.steps,
        }

        return obs, float(reward), terminated, truncated, info

    def close(self):
        if not rclpy.ok():
            return
        self.cmd_pub.publish(Twist())
        self.destroy_node()
