"""Stable flat-ground configuration for wheeled TITATIT + Piper."""

import math

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from LeggedManip_Lab.tasks.manager_based.leggedmanip_lab import mdp
from LeggedManip_Lab.tasks.manager_based.leggedmanip_lab.leggedmanip_lab_env_cfg import (
    LeggedManipLabEnvCfg,
    RewardsCfg,
)

from .common import (
    TitatitPiperActionsCfg,
    TitatitPiperObservationsCfg,
    configure_titatit_piper,
)


@configclass
class TitatitPiperFlatRewardsCfg(RewardsCfg):
    """Flat-task rewards adapted to rolling wheel contacts."""

    undesired_leg_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={
            "threshold": 1.0,
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", body_names=[".*_thigh.*", ".*_calf.*"]
            ),
        },
    )


@configclass
class TitatitPiperFlatEnvCfg(LeggedManipLabEnvCfg):
    actions: TitatitPiperActionsCfg = TitatitPiperActionsCfg()
    observations: TitatitPiperObservationsCfg = TitatitPiperObservationsCfg()
    rewards: TitatitPiperFlatRewardsCfg = TitatitPiperFlatRewardsCfg()

    def __post_init__(self) -> None:
        super().__post_init__()
        configure_titatit_piper(self)

        self.commands.ee_pose.debug_vis = False
        self.commands.base_velocity.debug_vis = False

        # Flat-task EE commands are entirely relative to link0.  Start around
        # the measured zero pose instead of immediately sampling the full box.
        self.commands.ee_pose.curriculum_enabled = True
        self.commands.ee_pose.ranges.pos_x = (0.15, 0.30)
        self.commands.ee_pose.ranges.pos_y = (-0.05, 0.05)
        self.commands.ee_pose.ranges.pos_z = (0.15, 0.30)
        self.commands.ee_pose.ranges.roll = (0.0, 0.0)
        self.commands.ee_pose.ranges.pitch = (0.0, 0.0)
        self.commands.ee_pose.ranges.yaw = (0.0, 0.0)
        self.commands.ee_pose.limit_ranges.pos_x = (0.10, 0.45)
        self.commands.ee_pose.limit_ranges.pos_y = (-0.18, 0.18)
        self.commands.ee_pose.limit_ranges.pos_z = (0.05, 0.35)
        self.commands.ee_pose.limit_ranges.roll = (0.0, 0.0)
        self.commands.ee_pose.limit_ranges.pitch = (0.0, 0.0)
        self.commands.ee_pose.limit_ranges.yaw = (0.0, 0.0)

        self.commands.base_velocity.curriculum_enabled = True
        self.commands.base_velocity.ranges.lin_vel_x = (-0.10, 0.30)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.20, 0.20)
        self.commands.base_velocity.limit_ranges.lin_vel_x = (-0.10, 0.60)
        self.commands.base_velocity.limit_ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.limit_ranges.ang_vel_z = (-0.35, 0.35)

        self.rewards.end_effector_position_tracking_exp.weight = 5.0
        self.rewards.end_effector_position_tracking_exp.params["std"] = math.sqrt(0.1)
        self.rewards.end_effector_orientation_tracking.weight = -0.75
        self.rewards.end_effector_orientation_tracking.params["reference_body_name"] = "link0"
        self.rewards.track_lin_vel_xy_exp.weight = 3.0
        self.rewards.track_ang_vel_z_exp.weight = 1.5
        self.rewards.track_base_height_exp.weight = 1.0
        self.rewards.track_base_height_exp.params["target_height"] = 0.30
        self.rewards.track_base_height_exp.params["std"] = 0.03
        self.rewards.flat_orientation_l2.weight = -1.5
        self.rewards.ang_vel_xy_l2.weight = -0.10
        self.rewards.action_rate_l2.weight = -0.02
        self.rewards.dof_pos_limits.weight = -2.0
        self.rewards.arm_deviation.weight = -0.01

        self.terminations.base_contact.params["threshold"] = 2.0
        self.terminations.bad_orientation.params["limit_angle"] = 0.60

        # configure_titatit_piper intentionally sets point-foot rewards to None.
        # The helper is None-safe, so removing any remaining zero terms is safe.
        self.disable_zero_weight_rewards()


@configclass
class TitatitPiperFlatEnvCfg_PLAY(TitatitPiperFlatEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()

        self.scene.num_envs = 1
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.commands.ee_pose.curriculum_enabled = False
        self.commands.base_velocity.curriculum_enabled = False
        self.curriculum.lin_vel_cmd_levels = None
        self.curriculum.ang_vel_cmd_levels = None
        self.curriculum.pos_cmd_levels = None

        self.commands.ee_pose.ranges.pos_x = (0.15, 0.30)
        self.commands.ee_pose.ranges.pos_y = (-0.05, 0.05)
        self.commands.ee_pose.ranges.pos_z = (0.15, 0.30)
        self.commands.ee_pose.ranges.roll = (0.0, 0.0)
        self.commands.ee_pose.ranges.pitch = (0.0, 0.0)
        self.commands.ee_pose.ranges.yaw = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_x = (-0.10, 0.30)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.20, 0.20)
