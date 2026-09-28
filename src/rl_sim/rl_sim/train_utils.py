#!/usr/bin/env python3
"""train_utils.py -- Helpers for resuming PPO training."""

import glob
import os

import numpy as np


def latest_checkpoint(checkpoint_dir):
    """
    Return the newest PPO checkpoint in *checkpoint_dir*.

    Checkpoints are named ``ppo_rl_sim_<steps>_steps.zip``.

    :param checkpoint_dir: directory holding the checkpoint files
    :return: ``(path, steps)``; ``(None, -1)`` when none is found
    """
    pattern = os.path.join(checkpoint_dir, 'ppo_rl_sim_*_steps.zip')
    latest_path, latest_steps = None, -1
    for path in glob.glob(pattern):
        name = os.path.basename(path)
        try:
            steps = int(name[len('ppo_rl_sim_'):-len('_steps.zip')])
        except ValueError:
            continue
        if steps > latest_steps:
            latest_path, latest_steps = path, steps
    return latest_path, latest_steps


def load_eval_history(eval_cb, eval_log_dir):
    """
    Seed an ``EvalCallback`` with the results of a previous run.

    Without this, resuming truncates ``evaluations.npz`` and resets
    ``best_mean_reward``, which makes the first post-resume evaluation
    overwrite ``best_model.zip``.

    :param eval_cb: the ``EvalCallback`` instance to seed
    :param eval_log_dir: directory containing ``evaluations.npz``
    :return: ``True`` when a history file was loaded
    """
    npz_path = os.path.join(eval_log_dir, 'evaluations.npz')
    if not os.path.exists(npz_path):
        return False

    data = np.load(npz_path, allow_pickle=True)
    files = set(data.files)

    if 'timesteps' in files:
        eval_cb.evaluations_timesteps = data['timesteps'].tolist()
    if 'results' in files:
        results = data['results']
        eval_cb.evaluations_results = results.tolist()
        if results.ndim == 2 and results.size > 0:
            eval_cb.best_mean_reward = float(results.mean(axis=1).max())
    if 'ep_lengths' in files:
        eval_cb.evaluations_length = data['ep_lengths'].tolist()
    if 'successes' in files:
        eval_cb.evaluations_successes = data['successes'].tolist()
    return True


def resolve_resume_path(save_path, checkpoint_dir, resume_from=None):
    """
    Decide which model/checkpoint to resume from.

    Precedence: explicit *resume_from*, then the saved model
    (``<save_path>.zip``), then the newest checkpoint.

    :param save_path: model path without the ``.zip`` suffix
    :param checkpoint_dir: directory holding checkpoint files
    :param resume_from: optional explicit ``.zip`` path
    :return: path to load, or ``None`` when nothing is available
    """
    if resume_from:
        return resume_from if os.path.exists(resume_from) else None
    model_zip = save_path + '.zip'
    if os.path.exists(model_zip):
        return model_zip
    path, _ = latest_checkpoint(checkpoint_dir)
    return path
