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

import math

from legged_gym.envs.base.legged_robot_config import (
    LeggedRobotCfg,
    LeggedRobotCfgDayDreamer,
    LeggedRobotCfgPPO,
)


# env_cfg
class UnitreeGo1RoughCfg(LeggedRobotCfg):
    class env(LeggedRobotCfg.env):
        num_envs = 4096
        num_actions = 12

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = "trimesh"

    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.4]  # x,y,z [m]
        default_joint_angles = {  # = target angles [rad] when action = 0.0
            # "LF_HAA": 0.0,
            # "LH_HAA": 0.0,
            # "RF_HAA": -0.0,
            # "RH_HAA": -0.0,
            # "LF_HFE": 0.4,
            # "LH_HFE": -0.4,
            # "RF_HFE": 0.4,
            # "RH_HFE": -0.4,
            # "LF_KFE": -0.8,
            # "LH_KFE": 0.8,
            # "RF_KFE": -0.8,
            # "RH_KFE": 0.8,
            # "LF_HAA" → "FL_hip_joint"
            # "LF_HFE" → "FL_thigh_joint"
            # "LF_KFE" → "FL_calf_joint"
            # "RF_HAA" → "FR_hip_joint"
            # "RF_HFE" → "FR_thigh_joint"
            # "RF_KFE" → "FR_calf_joint"
            # "LH_HAA" → "RL_hip_joint"
            # "LH_HFE" → "RL_thigh_joint"
            # "LH_KFE" → "RL_calf_joint"
            # "RH_HAA" → "RR_hip_joint"
            # "RH_HFE" → "RR_thigh_joint"
            # "RH_KFE" → "RR_calf_joint"
            "FL_hip_joint": 0.0,
            "RL_hip_joint": 0.0,
            "FR_hip_joint": -0.0,
            "RR_hip_joint": -0.0,
            "FL_thigh_joint": math.pi / 6,
            "RL_thigh_joint": math.pi / 6,
            "FR_thigh_joint": math.pi / 6,
            "RR_thigh_joint": math.pi / 6,
            "FL_calf_joint": -math.pi / 3,
            "RL_calf_joint": -math.pi / 3,
            "FR_calf_joint": -math.pi / 3,
            "RR_calf_joint": -math.pi / 3,
        }

    class control(LeggedRobotCfg.control):
        # PD Drive parameters:
        stiffness = {
            "hip_joint": 80.0,
            "thigh_joint": 80.0,
            "calf_joint": 80.0,
        }  # [N*m/rad]
        damping = {
            "hip_joint": 2.0,
            "thigh_joint": 2.0,
            "calf_joint": 2.0,
        }  # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        use_actuator_network = False
        actuator_net_file = (
            "{LEGGED_GYM_ROOT_DIR}/resources/actuator_nets/anydrive_v3_lstm.pt"
        )

    class asset(LeggedRobotCfg.asset):
        file = "{LEGGED_GYM_ROOT_DIR}/resources/robots/go1_description/urdf/go1.urdf"
        name = "go1"
        foot_name = "FOOT"
        penalize_contacts_on = ["SHANK", "THIGH"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1  # 1 to disable, 0 to enable...bitwise filter

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_base_mass = True
        added_mass_range = [-5.0, 5.0]

    class rewards(LeggedRobotCfg.rewards):
        base_height_target = 0.5
        max_contact_force = 500.0
        only_positive_rewards = True

        class scales(LeggedRobotCfg.rewards.scales):
            pass


# train_cfg
class UnitreeGo1RoughCfgDayDreamer(LeggedRobotCfgDayDreamer):
    class env(LeggedRobotCfg.env):
        num_envs = 64
        num_actions = 12

    class runner(LeggedRobotCfgDayDreamer.runner):
        run_name = "model_based"
        experiment_name = "rough_unitree_go1"
        load_run = -1
