#!/usr/bin/env python3
"""train_ppo.py -- Train a PPO agent for mecanum-robot navigation."""

import os

import rclpy
from rclpy.signals import SignalHandlerOptions

from rl_sim.hyperparams import TRAIN_CONFIG
from rl_sim.rl_env import RLEnv
from rl_sim.train_utils import load_eval_history, resolve_resume_path
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
    rclpy.init(args=args,
               signal_handler_options=SignalHandlerOptions.SIGTERM)
    log = rclpy.logging.get_logger('rl_sim')

    device = TRAIN_CONFIG.get('device', 'cpu')

    os.makedirs(TRAIN_CONFIG['train_root'], exist_ok=True)

    env = RLEnv()
    env = Monitor(env)

    # Separate environment for evaluation so it does not disturb the training
    # environment's episode state.
    eval_env = Monitor(RLEnv(node_name='rl_sim_eval_env'))

    resume_path = None
    if TRAIN_CONFIG.get('resume', False):
        resume_path = resolve_resume_path(
            TRAIN_CONFIG['save_path'],
            TRAIN_CONFIG['checkpoint_dir'],
            TRAIN_CONFIG.get('resume_from'))
        if resume_path is None:
            log.warn('resume=True but no model/checkpoint found; fresh run.')

    if resume_path is not None:
        model = PPO.load(
            resume_path,
            env=env,
            device=device,
            tensorboard_log=TRAIN_CONFIG['tensorboard_log'],
        )
        remaining = max(
            0, TRAIN_CONFIG['total_timesteps'] - model.num_timesteps)
        reset_num_timesteps = False
        log.info(
            f'Resuming from {resume_path} at {model.num_timesteps} steps '
            f'({remaining} remaining)'
        )
    else:
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
        remaining = TRAIN_CONFIG['total_timesteps']
        reset_num_timesteps = True

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

    if resume_path is not None and load_eval_history(
            eval_cb, TRAIN_CONFIG['eval_log_dir']):
        log.info('Restored previous evaluation history.')

    episode_log_cb = EpisodeLogCallback()

    if remaining > 0:
        try:
            model.learn(
                total_timesteps=remaining,
                callback=[episode_log_cb, checkpoint_cb, eval_cb],
                reset_num_timesteps=reset_num_timesteps,
                tb_log_name='PPO',
            )
        except KeyboardInterrupt:
            log.warn('Training interrupted -- saving the current model.')
    else:
        log.info('Training budget already reached; saving current model.')

    model.save(TRAIN_CONFIG['save_path'])
    log.info(f'Model saved to {TRAIN_CONFIG["save_path"]}.zip')

    env.close()
    eval_env.close()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
