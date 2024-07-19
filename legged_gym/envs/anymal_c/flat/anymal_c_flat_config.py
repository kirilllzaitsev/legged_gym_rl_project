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

from legged_gym.envs import (
    AnymalCRoughCfg,
    AnymalCRoughCfgDayDreamer,
    AnymalCRoughCfgPPO,
)


# env_cfg
class AnymalCFlatCfg(AnymalCRoughCfg):
    class env(AnymalCRoughCfg.env):
        num_observations = 48

    class terrain(AnymalCRoughCfg.terrain):
        mesh_type = "plane"
        measure_heights = False

    class asset(AnymalCRoughCfg.asset):
        self_collisions = 0  # 1 to disable, 0 to enable...bitwise filter

    class rewards(AnymalCRoughCfg.rewards):
        max_contact_force = 350.0

        class scales(AnymalCRoughCfg.rewards.scales):
            orientation = -5.0
            torques = -0.000025
            feet_air_time = 2.0
            # feet_contact_forces = -0.01

    class commands(AnymalCRoughCfg.commands):
        heading_command = False
        resampling_time = 4.0

        class ranges(AnymalCRoughCfg.commands.ranges):
            ang_vel_yaw = [-1.5, 1.5]

    class domain_rand(AnymalCRoughCfg.domain_rand):
        friction_range = [
            0.0,
            1.5,
        ]  # on ground planes the friction combination mode is averaging, i.e total friction = (foot_friction + 1.)/2.


# env_cfg
class AnymalCFlatCfgEasy(AnymalCRoughCfg):
    class env(AnymalCRoughCfg.env):
        num_observations = 48

    class terrain(AnymalCRoughCfg.terrain):
        mesh_type = "plane"
        measure_heights = False

    class asset(AnymalCRoughCfg.asset):
        self_collisions = 0  # 1 to disable, 0 to enable...bitwise filter

    class rewards(AnymalCRoughCfg.rewards):
        max_contact_force = 350.0

        class scales(AnymalCRoughCfg.rewards.scales):
            orientation = -5.0
            torques = -0.000025
            feet_air_time = 2.0
            tracking_lin_vel = 1.0
            tracking_ang_vel = 0.5
            # feet_contact_forces = -0.01

    class commands(AnymalCRoughCfg.commands):
        heading_command = True
        resampling_time = int(1e9)

        class ranges(AnymalCRoughCfg.commands.ranges):
            lin_vel_x = [0.0, 0.001]  # min max [m/s]
            lin_vel_y = [1.0, 1.0001]  # min max [m/s]
            ang_vel_yaw = [0, 0.0001]  # min max [rad/s]
            heading = [0.0, 0.0001]

    class domain_rand(AnymalCRoughCfg.domain_rand):
        randomize_friction = False
        randomize_base_mass = False
        push_robots = False

    class noise:
        add_noise = False
        noise_level = 1.0  # scales other values

        class noise_scales:
            dof_pos = 1.0
            dof_vel = 1.0
            lin_vel = 1.0
            ang_vel = 1.0
            gravity = 1.0
            height_measurements = 1.0


# train_cfg
class AnymalCFlatCfgPPO(AnymalCRoughCfgPPO):
    class policy(AnymalCRoughCfgPPO.policy):
        actor_hidden_dims = [128, 64, 32]
        critic_hidden_dims = [128, 64, 32]
        activation = "elu"  # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid

    class algorithm(AnymalCRoughCfgPPO.algorithm):
        entropy_coef = 0.01

    class runner(AnymalCRoughCfgPPO.runner):
        run_name = "ppo"
        experiment_name = "flat_anymal_c"
        load_run = -1
        max_iterations = 300


class AnymalCFlatCfgDayDreamer(AnymalCRoughCfgDayDreamer):

    class runner(AnymalCRoughCfgDayDreamer.runner):
        run_name = "dreamer"
        experiment_name = "flat_anymal_c"
        load_run = -1
        max_iterations = 300
