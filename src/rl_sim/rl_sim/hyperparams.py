#!/usr/bin/env python3
"""hyperparams.py -- Central configuration for the rl_sim package."""

# -- ROS Topics ---------------------------------------------------------------
SCAN_TOPIC = '/scan'
POSE_TOPIC = '/webots_pose'
ODOM_TOPIC = '/odom'
CMD_TOPIC = '/cmd_vel'
SETPOINT_TOPIC = '/setpoint_pos'

# -- Robot physical limits ----------------------------------------------------
ROBOT_CONFIG = {
    'v_max': 0.3,              # m/s  -- max linear velocity
    'w_max': 1.0,              # rad/s -- max angular velocity
    'lidar_total': 1024,       # raw beams from Webots LiDAR
    'lidar_samples': 36,       # subsampled beams for observation
    'lidar_max': 8.0,          # m -- max range (matches Webots world)
    'control_hz': 10.0,        # Hz -- control loop frequency
}

# -- MDP / environment parameters ---------------------------------------------
ENV_CONFIG = {
    'goal_tolerance': 0.4,          # m -- success threshold
    'collision_threshold': 0.25,    # m -- collision threshold (min lidar)
    'max_steps': 500,               # max steps per episode
    'use_lateral': True,            # True: 3-DOF (vx,vy,w), False: 2-DOF
    # Goal bounds (10x6 m arena)
    'goal_x_min': -4.0,
    'goal_x_max': 4.0,
    'goal_y_min': -2.0,
    'goal_y_max': 2.0,
    # Spawn bounds
    'spawn_x_min': -3.5,
    'spawn_x_max': 3.5,
    'spawn_y_min': -2.0,
    'spawn_y_max': 2.0,
    # Safety margin around obstacles (m)
    'obstacle_margin': 0.25,
}

# -- Obstacle bounding boxes (x_min, y_min, x_max, y_max) -------------------
# Derived from worlds/env.wbt.  Includes obstacle_margin around each obstacle.
OBSTACLES = [
    # Oil barrels (top wall area)
    (-4.60, 2.01, -4.00, 2.61),
    (-3.78, 2.01, -3.18, 2.61),
    (-2.90, 2.01, -2.30, 2.61),
    # Wooden pallet (bottom-left)
    (-4.35, -2.70, -3.75, -2.10),
    # Metal storage box (top area, rotated 90 deg)
    (-1.21, 1.99, -0.61, 2.59),
    # Cardboard boxes (center cluster)
    (-2.89, -1.66, -2.29, -1.06),
    (-2.89, -0.93, -2.29, -0.33),
    (-2.14, -0.93, -1.54, -0.33),
    (-2.14, -1.65, -1.54, -1.05),
    # Internal walls
    (1.85, -0.55, 2.15, 2.95),
    (1.85, -2.98, 2.15, -1.73),
]

# -- Reward weights -----------------------------------------------------------
REWARD_CONFIG = {
    'progress_weight': 2.0,         # reward per meter closer to goal
    'goal_reward': 100.0,           # terminal reward on goal reached
    'collision_penalty': -100.0,    # terminal penalty on collision
    'step_penalty': -0.05,          # per-step cost (encourages efficiency)
    'proximity_weight': 0.5,        # penalty weight near obstacles
    'proximity_threshold': 0.5,     # m -- distance below which penalty applies
    'smoothness_weight': 0.02,      # penalty for sharp turns (w^2)
}

# -- PPO training hyperparameters ---------------------------------------------
TRAIN_CONFIG = {
    'learning_rate': 3e-4,
    'n_steps': 2048,
    'batch_size': 64,
    'gamma': 0.99,
    'gae_lambda': 0.95,
    'ent_coef': 0.01,
    'total_timesteps': 1_000_000,
    'save_path': 'ppo_rl_sim_navigation',
    'tensorboard_log': './ppo_rl_sim_tb/',
    'checkpoint_freq': 50000,
    'device': 'cpu',  # 'cpu' | 'cuda' | 'auto'
}
