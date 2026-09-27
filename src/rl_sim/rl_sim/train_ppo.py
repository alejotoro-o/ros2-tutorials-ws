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

    # Separate environment for evaluation so it does not disturb the training
    # environment's episode state.
    eval_env = Monitor(RLEnv(node_name='rl_sim_eval_env'))

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
        policy_kwargs=TRAIN_CONFIG.get('policy_kwargs', {}),
    )

    checkpoint_cb = CheckpointCallback(
        # save_freq is in TIMESTEPS (not rollout batches).
        save_freq=TRAIN_CONFIG['checkpoint_freq'],
        save_path='./checkpoints/',
        name_prefix='ppo_rl_sim',
    )

    stop_train_cb = StopTrainingOnNoModelImprovement(
        max_no_improvement_evals=(
            TRAIN_CONFIG['early_stop_max_no_improvement_evals']),
        min_evals=TRAIN_CONFIG['early_stop_min_evals'],
        verbose=1,
    )

    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path='./best_model/',
        log_path='./eval_logs/',
        # eval_freq is in TIMESTEPS (not rollout batches).
        eval_freq=TRAIN_CONFIG['eval_freq'],
        n_eval_episodes=TRAIN_CONFIG['n_eval_episodes'],
        deterministic=True,
        callback_after_eval=stop_train_cb,
    )

    model.learn(
        total_timesteps=TRAIN_CONFIG['total_timesteps'],
        callback=[checkpoint_cb, eval_cb],
    )

    model.save(TRAIN_CONFIG['save_path'])
    rclpy.logging.get_logger('rl_sim').info(
        f'Training complete. Model saved to '
        f'{TRAIN_CONFIG["save_path"]}.zip'
    )

    env.close()
    eval_env.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
