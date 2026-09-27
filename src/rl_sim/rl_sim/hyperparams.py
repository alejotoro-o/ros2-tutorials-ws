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
    'v_max': 0.15,              # m/s  -- max linear velocity
    'w_max': 1.0,              # rad/s -- max angular velocity
    'lidar_total': 1024,       # raw beams from Webots LiDAR
    'lidar_samples': 36,       # subsampled beams for observation
    'lidar_max': 8.0,          # m -- max range (matches Webots world)
    'control_hz': 10.0,        # Hz -- control loop frequency
}

# -- MDP / environment parameters ---------------------------------------------
ENV_CONFIG = {
    'goal_tolerance': 0.4,          # m -- success threshold
    'collision_threshold': 0.35,    # m -- collision threshold (min lidar)
    'max_steps': 500,               # max steps per episode
    'use_lateral': False,           # True: 3-DOF (vx,vy,w), False: 2-DOF
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
    # Safety margins (m)
    'obstacle_margin': 0.35,    # static-map margin used for obstacle tests
    'goal_margin': 0.50,        # clearance required to accept a goal
    'spawn_clearance': 0.70,    # min lidar range required at a spawn
}

# -- Obstacle bounding boxes (x_min, y_min, x_max, y_max) -------------------
# True footprints derived from worlds/env.wbt and the Webots proto defaults.
# A safety margin (ENV_CONFIG['obstacle_margin'] / 'goal_margin') is applied
# on top of these boxes in RLEnv._is_in_obstacle().
OBSTACLES = [
    # Arena walls (RectangleArena floorSize 10x6, wallThickness 0.01)
    (-5.01, -3.01, -5.00, 3.01),   # left wall
    (5.00, -3.01, 5.01, 3.01),     # right wall
    (-5.01, -3.01, 5.01, -3.00),   # bottom wall
    (-5.01, 3.00, 5.01, 3.01),     # top wall
    # Oil barrels (radius 0.3, top wall area)
    (-4.60, 2.01, -4.00, 2.61),
    (-3.78, 2.01, -3.18, 2.61),
    (-2.90, 2.01, -2.30, 2.61),
    # WoodenPalletStack (default palletSize 1.2 x 0.8, bottom-left)
    (-4.66, -2.81, -3.44, -1.99),
    # Metal storage box (0.6 x 0.6, top area, rotated 90 deg)
    (-1.21, 1.99, -0.61, 2.59),
    # Cardboard boxes (0.6 x 0.6, center cluster)
    (-2.89, -1.66, -2.29, -1.06),
    (-2.89, -0.93, -2.29, -0.33),
    (-2.14, -0.93, -1.54, -0.33),
    (-2.14, -1.65, -1.54, -1.05),
    # Internal walls (Wall size 0.1 x length x 1)
    (1.95, -0.55, 2.05, 2.95),
    (1.95, -2.98, 2.05, -1.73),
]

# -- Reward weights -----------------------------------------------------------
REWARD_CONFIG = {
    'progress_weight': 5.0,         # reward per meter closer to goal
    'goal_reward': 100.0,           # terminal reward on goal reached
    'collision_penalty': -100.0,    # terminal penalty on collision
    'step_penalty': -0.01,          # per-step cost (encourages efficiency)
    'proximity_weight': 0.5,        # penalty weight near obstacles
    'proximity_threshold': 0.5,     # m -- distance below which penalty applies
    'heading_weight': 0.1,          # bonus for facing the goal
}

# -- PPO training hyperparameters ---------------------------------------------
TRAIN_CONFIG = {
    'learning_rate': 3e-4,
    'n_steps': 2048,
    'batch_size': 64,
    'gamma': 0.95,
    'gae_lambda': 0.95,
    'ent_coef': 0.1,
    # Wall-clock bound: the env advances in real time (~10 steps/s), so keep
    # this realistic. 300k steps is roughly 8 h on a typical machine.
    'total_timesteps': 300_000,
    'save_path': 'ppo_rl_sim_navigation',
    'tensorboard_log': './ppo_rl_sim_tb/',
    # NOTE: the values below are in TIMESTEPS (SB3 callback unit), NOT rollout
    # batches. Do not divide them by 'n_steps'.
    'checkpoint_freq': 20_000,
    'eval_freq': 10_000,
    'n_eval_episodes': 5,
    'early_stop_min_evals': 10,
    'early_stop_max_no_improvement_evals': 10,
    'device': 'cpu',  # 'cpu' | 'cuda' | 'auto'
    'policy_kwargs': {
        'net_arch': [128, 128],
    },
}
