#!/usr/bin/env python3
"""train_ppo.py -- Train a PPO agent for mecanum-robot navigation."""

import rclpy

from rl_sim.hyperparams import TRAIN_CONFIG
from rl_sim.rl_env import RLEnv
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    CheckpointCallback,
    EvalCallback,
    StopTrainingOnNoModelImprovement,
)
from stable_baselines3.common.monitor import Monitor


def main(args=None):
    rclpy.init(args=args)

    device = TRAIN_CONFIG.get('device', 'cpu')

    env = RLEnv()
    env = Monitor(env)

    model = PPO(
        policy='MlpPolicy',
        env=env,
        learning_rate=TRAIN_CONFIG['learning_rate'],
        n_steps=TRAIN_CONFIG['n_steps'],
        batch_size=TRAIN_CONFIG['batch_size'],
        gamma=TRAIN_CONFIG['gamma'],
        gae_lambda=TRAIN_CONFIG['gae_lambda'],
        ent_coef=TRAIN_CONFIG['ent_coef'],
        verbose=1,
        tensorboard_log=TRAIN_CONFIG['tensorboard_log'],
        device=device,
    )

    checkpoint_cb = CheckpointCallback(
        save_freq=TRAIN_CONFIG['checkpoint_freq'] // TRAIN_CONFIG['n_steps'],
        save_path='./checkpoints/',
        name_prefix='ppo_rl_sim',
    )

    stop_train_cb = StopTrainingOnNoModelImprovement(
        max_no_improvement_evals=10,
        min_evals=20,
        verbose=1,
    )

    eval_cb = EvalCallback(
        env,
        best_model_save_path='./best_model/',
        log_path='./eval_logs/',
        eval_freq=10000 // TRAIN_CONFIG['n_steps'],
        n_eval_episodes=5,
        deterministic=True,
        callback_after_eval=stop_train_cb,
    )

    model.learn(
        total_timesteps=TRAIN_CONFIG['total_timesteps'],
        callback=[checkpoint_cb, eval_cb],
    )

    model.save(TRAIN_CONFIG['save_path'])
    env.get_logger().info(
        f'Training complete. Model saved to '
        f'{TRAIN_CONFIG["save_path"]}.zip'
    )

    env.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
