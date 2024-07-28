# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

import os
import sys
from datetime import datetime
from pathlib import Path

import isaacgym
import numpy as np
import torch

from legged_gym.envs import off_policy_algos, task_registry
from legged_gym.utils import get_args
from legged_gym.utils.helpers import class_to_dict


def train(args):
    env, env_cfg = task_registry.make_env(name=args.task, args=args)

    is_on_policy = not any(algo in args.task for algo in off_policy_algos)
    ppo_runner, train_cfg = task_registry.make_alg_runner(
        env=env, name=args.task, args=args, is_on_policy=is_on_policy
    )

    if ppo_runner.log_dir is not None:
        import yaml

        with open(os.path.join(ppo_runner.log_dir, "train_cfg.yaml"), "w") as f:
            yaml.dump(class_to_dict(train_cfg), f)
        with open(os.path.join(ppo_runner.log_dir, "env_cfg.yaml"), "w") as f:
            yaml.dump(class_to_dict(env_cfg), f)
        with open(os.path.join(ppo_runner.log_dir, "args.yaml"), "w") as f:
            yaml.dump(vars(args), f)
        # save entire source code
        legged_gym_path = Path(__file__).resolve().parents[1]
        rsl_rl_path = Path(__file__).resolve().parents[2] / "rsl_rl"
        os.system(
            f"rsync -a --exclude='logs' --exclude='resources' --exclude='__pycache__' {legged_gym_path} {ppo_runner.log_dir}"
        )
        for path_to_copy in [
            rsl_rl_path,
            rsl_rl_path / "rsl_rl/algorithms/daydreamer.py",
            rsl_rl_path / "rsl_rl/runners/off_policy_runner.py",
        ]:
            os.system(f"rsync -a {path_to_copy} {ppo_runner.log_dir}")

    try:
        ppo_runner.learn(
            num_learning_iterations=train_cfg.runner.max_iterations,
            init_at_random_ep_len=True,
        )
    finally:
        if ppo_runner.log_dir is not None:
            print(
                f"log_dir: {Path(ppo_runner.log_dir).name}. full path: {ppo_runner.log_dir}"
            )


if __name__ == "__main__":
    args = get_args()
    train(args)
