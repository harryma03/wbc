# Copyright (c) 2025-2026, Junjie Zhu.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Articulation configuration for the wheeled TITATIT with Piper arm."""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import DelayedPDActuatorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

_ASSET_DIR = os.path.dirname(os.path.abspath(__file__))
TITATIT_PIPER_USD = os.path.join(_ASSET_DIR, "titatit_piper.usd")


TITATIT_PIPER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=TITATIT_PIPER_USD,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=1,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.40),
        joint_pos={
            "FL_hip_joint": 0.1,
            "FR_hip_joint": -0.1,
            "RL_hip_joint": 0.1,
            "RR_hip_joint": -0.1,
            "FL_thigh_joint": 0.8,
            "FR_thigh_joint": 0.8,
            "RL_thigh_joint": -0.8,
            "RR_thigh_joint": -0.8,
            "FL_calf_joint": -1.5,
            "FR_calf_joint": -1.5,
            "RL_calf_joint": 1.5,
            "RR_calf_joint": 1.5,
            ".*_foot_joint": 0.0,
            "joint[1-6]": 0.0,
            "gripper.*": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": DelayedPDActuatorCfg(
            joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
            effort_limit=55.0,
            velocity_limit=28.6,
            stiffness=80.0,
            damping=2.0,
            armature=0.01,
            min_delay=0,
            max_delay=4,
            friction=0.01,
        ),
        # Wheel actions are velocity targets. With zero stiffness, damping acts
        # as the velocity servo gain and the URDF effort limit caps wheel torque.
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=[".*_foot_joint"],
            effort_limit_sim=10.0,
            velocity_limit_sim=25.0,
            stiffness=0.0,
            damping=2.0,
            armature=0.01,
            friction=0.0,
        ),
        "arm": DelayedPDActuatorCfg(
            joint_names_expr=["joint[1-6]"],
            effort_limit=100.0,
            velocity_limit=5.0,
            stiffness={
                "joint1": 50.0,
                "joint2": 50.0,
                "joint3": 80.0,
                "joint4": 30.0,
                "joint5": 30.0,
                "joint6": 20.0,
            },
            damping={
                "joint1": 3.0,
                "joint2": 2.0,
                "joint3": 3.0,
                "joint4": 3.0,
                "joint5": 2.5,
                "joint6": 1.0,
            },
            armature=0.01,
            min_delay=0,
            max_delay=4,
            friction=0.01,
        ),
        # The current task does not expose a gripper action. This actuator keeps
        # the three imported gripper/mimic joints at their default positions.
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["gripper.*"],
            effort_limit_sim=10.0,
            velocity_limit_sim=3.0,
            stiffness=100.0,
            damping=5.0,
        ),
    },
)
