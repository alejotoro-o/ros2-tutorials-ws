#!/usr/bin/env python3
"""train_ppo.py -- Train a PPO agent for mecanum-robot navigation."""

import os

import rclpy

from rl_sim.hyperparams import TRAIN_CONFIG
from rl_sim.rl_env import RLEnv
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CheckpointCallback,
    EvalCallback,
    StopTrainingOnNoModelImprovement,
)
from stable_baselines3.common.monitor import Monitor


class EpisodeLogCallback(BaseCallback):
    """Write per-episode reward/length to TensorBoard as episodes finish."""

    def _on_step(self):
        for info in self.locals.get('infos', []):
            episode = info.get('episode')
            if episode is not None:
                self.logger.record('episode/reward', episode['r'])
                self.logger.record('episode/length', episode['l'])
                self.logger.dump(self.num_timesteps)
        return True


def main(args=None):
    rclpy.init(args=args)

    device = TRAIN_CONFIG.get('device', 'cpu')

    os.makedirs(TRAIN_CONFIG['train_root'], exist_ok=True)

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
        save_path=TRAIN_CONFIG['checkpoint_dir'],
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
        best_model_save_path=TRAIN_CONFIG['best_model_dir'],
        log_path=TRAIN_CONFIG['eval_log_dir'],
        # eval_freq is in TIMESTEPS (not rollout batches).
        eval_freq=TRAIN_CONFIG['eval_freq'],
        n_eval_episodes=TRAIN_CONFIG['n_eval_episodes'],
        deterministic=True,
        callback_after_eval=stop_train_cb,
    )

    episode_log_cb = EpisodeLogCallback()

    try:
        model.learn(
            total_timesteps=TRAIN_CONFIG['total_timesteps'],
            callback=[episode_log_cb, checkpoint_cb, eval_cb],
        )
    except KeyboardInterrupt:
        rclpy.logging.get_logger('rl_sim').warn(
            'Training interrupted -- saving the current model.')

    model.save(TRAIN_CONFIG['save_path'])
    rclpy.logging.get_logger('rl_sim').info(
        f'Model saved to {TRAIN_CONFIG["save_path"]}.zip'
    )

    env.close()
    eval_env.close()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
