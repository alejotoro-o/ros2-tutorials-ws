#!/usr/bin/env python3
"""Tests for the PPO resume helpers in train_utils."""

import numpy as np
import pytest

from rl_sim.train_utils import (
    latest_checkpoint,
    load_eval_history,
    resolve_resume_path,
)


class _DummyEvalCallback:
    """Minimal stand-in for stable_baselines3 EvalCallback."""

    def __init__(self):
        self.evaluations_timesteps = []
        self.evaluations_results = []
        self.evaluations_length = []
        self.evaluations_successes = []
        self.best_mean_reward = -np.inf


def test_latest_checkpoint_picks_highest_steps(tmp_path):
    """The checkpoint with the largest step count wins."""
    for steps in (5000, 15000, 10000):
        (tmp_path / f'ppo_rl_sim_{steps}_steps.zip').write_bytes(b'')
    (tmp_path / 'unrelated.zip').write_bytes(b'')

    path, steps = latest_checkpoint(str(tmp_path))

    assert steps == 15000
    assert path.endswith('ppo_rl_sim_15000_steps.zip')


def test_latest_checkpoint_empty(tmp_path):
    """An empty directory yields no checkpoint."""
    path, steps = latest_checkpoint(str(tmp_path))

    assert path is None
    assert steps == -1


def test_resolve_resume_path_precedence(tmp_path):
    """Explicit path beats model, model beats newest checkpoint."""
    checkpoints = tmp_path / 'checkpoints'
    checkpoints.mkdir()
    (checkpoints / 'ppo_rl_sim_5000_steps.zip').write_bytes(b'')
    save_path = str(tmp_path / 'model')

    # Only a checkpoint exists -> use it.
    resolved = resolve_resume_path(save_path, str(checkpoints))
    assert resolved.endswith('ppo_rl_sim_5000_steps.zip')

    # The saved model now takes precedence.
    model_zip = tmp_path / 'model.zip'
    model_zip.write_bytes(b'')
    assert resolve_resume_path(
        save_path, str(checkpoints)) == str(model_zip)

    # An explicit path takes precedence over everything.
    explicit = str(checkpoints / 'ppo_rl_sim_5000_steps.zip')
    assert resolve_resume_path(
        save_path, str(checkpoints), explicit) == explicit

    # A missing explicit path resolves to nothing.
    assert resolve_resume_path(
        save_path, str(checkpoints), 'missing.zip') is None


def test_load_eval_history_seeds_callback(tmp_path):
    """Prior evaluations are restored, including best_mean_reward."""
    np.savez(
        str(tmp_path / 'evaluations.npz'),
        timesteps=[2500, 5000],
        results=[[-100.0, -90.0], [-80.0, -70.0]],
        ep_lengths=[[10, 12], [20, 22]],
    )
    cb = _DummyEvalCallback()

    assert load_eval_history(cb, str(tmp_path)) is True
    assert cb.evaluations_timesteps == [2500, 5000]
    assert cb.evaluations_results == [[-100.0, -90.0], [-80.0, -70.0]]
    assert cb.evaluations_length == [[10, 12], [20, 22]]
    assert cb.best_mean_reward == pytest.approx(-75.0)


def test_load_eval_history_missing(tmp_path):
    """No history file leaves the callback untouched."""
    cb = _DummyEvalCallback()

    assert load_eval_history(cb, str(tmp_path)) is False
    assert cb.evaluations_timesteps == []
    assert cb.best_mean_reward == -np.inf
