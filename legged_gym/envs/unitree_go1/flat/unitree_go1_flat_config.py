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


from legged_gym.envs.unitree_go1.mixed_terrains.unitree_go1_rough_config import (
    UnitreeGo1RoughCfg,
    UnitreeGo1RoughCfgDayDreamer,
    UnitreeGo1RoughCfgPPO,
)


class UnitreeGo1FlatCfg(UnitreeGo1RoughCfg):
    class env(UnitreeGo1RoughCfg.env):
        num_observations = 48

    class terrain(UnitreeGo1RoughCfg.terrain):
        mesh_type = "plane"
        measure_heights = False

    class commands(UnitreeGo1RoughCfg.commands):
        heading_command = False
        resampling_time = 10.0

        class ranges(UnitreeGo1RoughCfg.commands.ranges):
            ang_vel_yaw = [-1.0, 1.0]


class UnitreeGo1FlatCfgEasy(UnitreeGo1FlatCfg):
    class env(UnitreeGo1FlatCfg.env):
        num_observations = 48
        modality = "pixels"

    class terrain(UnitreeGo1FlatCfg.terrain):
        mesh_type = "plane"
        measure_heights = False

    class asset(UnitreeGo1FlatCfg.asset):
        self_collisions = 0  # 1 to disable, 0 to enable...bitwise filter

    class commands(UnitreeGo1FlatCfg.commands):
        heading_command = False
        resampling_time = int(1e9)

        class ranges(UnitreeGo1FlatCfg.commands.ranges):
            lin_vel_x = [0.0, 0.001]  # min max [m/s]
            lin_vel_y = [0.3, 0.30001]  # min max [m/s]
            # lin_vel_y = [1.0, 1.0001]  # min max [m/s]
            ang_vel_yaw = [0, 0.0001]  # min max [rad/s]
            heading = [0.0, 0.0001]

    class domain_rand(UnitreeGo1FlatCfg.domain_rand):
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


class UnitreeGo1FlatCfgDayDreamer(UnitreeGo1RoughCfgDayDreamer):

    class runner(UnitreeGo1RoughCfgDayDreamer.runner):
        run_name = "check"
        experiment_name = "unitree_go1"
        load_run = -1
        max_iterations = 300


class UnitreeGo1FlatCfgPPO(UnitreeGo1RoughCfgPPO):

    class runner(UnitreeGo1RoughCfgPPO.runner):
        run_name = "check"
        experiment_name = "unitree_go1"
        load_run = -1
        max_iterations = 300
