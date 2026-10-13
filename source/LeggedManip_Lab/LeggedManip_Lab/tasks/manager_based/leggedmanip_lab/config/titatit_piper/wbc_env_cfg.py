"""Stable whole-body-control configuration for wheeled TITATIT + Piper."""

import math

import isaaclab.terrains as terrain_gen
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from LeggedManip_Lab.tasks.manager_based.leggedmanip_lab import mdp
from LeggedManip_Lab.tasks.manager_based.leggedmanip_lab.config.go2_piper.wbc_env_cfg import (
    Go2PiperWBCEnvCfg,
)
from LeggedManip_Lab.tasks.manager_based.leggedmanip_lab.leggedmanip_lab_env_cfg import (
    RewardsCfg,
)

from .common import (
    TitatitPiperActionsCfg,
    TitatitPiperObservationsCfg,
    configure_titatit_piper,
)


TITATIT_LOCAL_FLAT_CFG = terrain_gen.TerrainGeneratorCfg(
    size=(180.0, 180.0),
    num_rows=1,
    num_cols=1,
    curriculum=False,
    use_cache=False,
    sub_terrains={"flat": terrain_gen.MeshPlaneTerrainCfg(proportion=1.0)},
)


@configclass
class TitatitPiperWBCRewardsCfg(RewardsCfg):
    """Robot-specific rewards for stable wheeled whole-body control."""

    # The exponential term supplies precision near the target.  This dense
    # term still supplies a useful gradient when the target is farther away.
    end_effector_position_tracking_l2 = RewTerm(
        func=mdp.position_command_error_l2,
        weight=-1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="end_effector"),
            "command_name": "ee_pose",
            "link0_name": "link0",
        },
    )

    # Wheels may contact the floor continuously, but upper leg links may not.
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
class TitatitPiperWBCEnvCfg(Go2PiperWBCEnvCfg):
    """Training configuration with stability prioritized over reward exploits."""

    actions: TitatitPiperActionsCfg = TitatitPiperActionsCfg()
    observations: TitatitPiperObservationsCfg = TitatitPiperObservationsCfg()
    rewards: TitatitPiperWBCRewardsCfg = TitatitPiperWBCRewardsCfg()

    def __post_init__(self) -> None:
        super().__post_init__()
        configure_titatit_piper(self)

        # The runner clips normalized actions to [-1, 1]. GO2's uniform
        # 0.25 scale would therefore restrict every arm target to +/-0.25 rad.
        # Keep the existing 18-position + 4-wheel ordering, but give the arm
        # an independent range. Values below are radians, relative to zero
        # for the arm; JointPositionAction applies clipping after its offset.
        self.actions.joint_pos.scale = {
            ".*_hip_joint": 0.25,
            ".*_thigh_joint": 0.25,
            ".*_calf_joint": 0.25,
            "joint1": math.radians(135.0),
            "joint2": math.radians(162.0),
            "joint3": math.radians(153.0),
            "joint[45]": math.radians(80.1),
            "joint6": math.radians(108.0),
        }
        # 90% of the USD angular endpoints (not Isaac Lab's centered soft
        # limit formula). Validated by scripts/tools/check_titatit_workspace.py.
        # Preserve the one-sided j2/j3 limits and zero-pose action offsets.
        self.actions.joint_pos.clip = {
            ".*_hip_joint": (-10.0, 10.0),
            ".*_thigh_joint": (-10.0, 10.0),
            ".*_calf_joint": (-10.0, 10.0),
            "joint1": (-math.radians(135.0), math.radians(135.0)),
            "joint2": (0.0, math.radians(162.0)),
            "joint3": (-math.radians(153.0), 0.0),
            "joint[45]": (-math.radians(80.1), math.radians(80.1)),
            "joint6": (-math.radians(108.0), math.radians(108.0)),
        }
        # Match the wheel actuator's configured velocity ceiling; the former
        # 12 rad/s target clip could restrict the expanded base-speed task.
        self.actions.wheel_vel.scale = 25.0
        self.actions.wheel_vel.clip = {".*": (-25.0, 25.0)}

        self.commands.ee_pose.debug_vis = False
        self.commands.base_velocity.debug_vis = False

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = TITATIT_LOCAL_FLAT_CFG
        self.scene.terrain.use_terrain_origins = False
        self.scene.terrain.env_spacing = self.scene.env_spacing

        # Mixed WBC convention: X/Y are expressed in link0, while Z is an
        # absolute world-frame height.  Start close to the measured zero pose.
        self.commands.ee_pose.curriculum_enabled = True
        self.commands.ee_pose.resampling_time_range = (8.0, 10.0)
        self.commands.ee_pose.workspace_max_radius = 0.60
        # The Piper gripper extends along end_effector +Z, whereas the generic
        # look-at command aligns +X with the target. Rotate tool +Z onto look-at
        # +X (a +90 degree rotation about Y in the tool frame).
        self.commands.ee_pose.tool_frame_offset = (
            math.sqrt(0.5),
            0.0,
            math.sqrt(0.5),
            0.0,
        )
        self.commands.ee_pose.ranges.pos_x = (0.25, 0.35)
        self.commands.ee_pose.ranges.pos_y = (-0.10, 0.10)
        self.commands.ee_pose.ranges.pos_z = (0.58, 0.74)
        self.commands.ee_pose.ranges.roll = (0.0, 0.0)
        self.commands.ee_pose.ranges.pitch = (0.0, 0.0)
        self.commands.ee_pose.ranges.yaw = (0.0, 0.0)
        # USD-chain IK audit at base height 0.30 m, with the same look-at
        # orientation and 0.60 m projection as the command generator.
        # Expand forward/laterally; avoid near-base targets with poor pose IK.
        # This is a sampled kinematic envelope, not collision certification.
        self.commands.ee_pose.limit_ranges.pos_x = (0.25, 0.55)
        self.commands.ee_pose.limit_ranges.pos_y = (-0.30, 0.30)
        self.commands.ee_pose.limit_ranges.pos_z = (0.50, 0.82)
        self.commands.ee_pose.limit_ranges.roll = (0.0, 0.0)
        self.commands.ee_pose.limit_ranges.pitch = (0.0, 0.0)
        self.commands.ee_pose.limit_ranges.yaw = (0.0, 0.0)

        # Fixed-axis wheels cannot track lateral velocity without skidding.
        # Learn forward/reverse and yaw first.
        self.commands.base_velocity.curriculum_enabled = True
        self.commands.base_velocity.resampling_time_range = (10.0, 10.0)
        self.commands.base_velocity.rel_standing_envs = 0.10
        self.commands.base_velocity.heading_command = False
        self.commands.base_velocity.ranges.lin_vel_x = (-0.20, 0.20)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.20, 0.20)
        self.commands.base_velocity.limit_ranges.lin_vel_x = (-1.0, 1.0)
        self.commands.base_velocity.limit_ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.limit_ranges.ang_vel_z = (-1.0, 1.0)

        # Keep manipulation, locomotion, and posture on comparable scales.
        # The previous EE weight of 15 made a heavily tilted base profitable.
        self.rewards.end_effector_position_tracking_exp.func = mdp.position_command_error_exp
        self.rewards.end_effector_position_tracking_exp.weight = 5.0
        self.rewards.end_effector_position_tracking_exp.params["std"] = 0.20
        self.rewards.end_effector_position_tracking_l2.weight = -1.5
        self.rewards.end_effector_orientation_tracking.weight = -4.0
        self.rewards.end_effector_orientation_tracking.params["reference_body_name"] = "link0"

        # Follow GO2's position-based curriculum, retaining smaller 2 cm
        # increments and the tighter 0.20 m reward kernel. A separate hard
        # orientation gate previously kept the workspace at its initial size.
        self.curriculum.pos_cmd_levels.params.update(
            {
                "success_threshold": 0.80,
                "position_step": 0.02,
                "orientation_reward_term_name": None,
                "orientation_error_threshold": None,
            }
        )
        # Let each base curriculum use its own velocity-tracking reward, as
        # in GO2; wheel-specific speed limits and zero lateral speed remain.
        manipulation_gate = {
            "position_reward_term_name": None,
            "orientation_reward_term_name": None,
            "orientation_error_threshold": None,
        }
        self.curriculum.lin_vel_cmd_levels.params.update(manipulation_gate)
        self.curriculum.ang_vel_cmd_levels.params.update(manipulation_gate)

        self.rewards.track_lin_vel_xy_exp.weight = 3.0
        self.rewards.track_ang_vel_z_exp.weight = 1.5

        self.rewards.track_base_height_exp.weight = 1.0
        self.rewards.track_base_height_exp.params["target_height"] = 0.30
        self.rewards.track_base_height_exp.params["std"] = 0.03
        self.rewards.flat_orientation_l2.weight = -1.5
        self.rewards.ang_vel_xy_l2.weight = -0.10

        self.rewards.action_rate_l2.weight = -0.03
        self.rewards.dof_pos_limits.weight = -2.0
        self.rewards.arm_deviation.weight = -0.01

        # Reject gross leaning earlier than the original one-radian threshold.
        self.terminations.base_contact.params["threshold"] = 2.0
        self.terminations.bad_orientation.params["limit_angle"] = 0.60


@configclass
class TitatitPiperWBCEnvCfg_PLAY(TitatitPiperWBCEnvCfg):
    """Deterministic evaluation on the initial, well-conditioned command set."""

    def __post_init__(self) -> None:
        super().__post_init__()

        self.scene.num_envs = 1
        self.scene.env_spacing = 2.5
        self.scene.terrain.env_spacing = self.scene.env_spacing
        self.observations.policy.enable_corruption = False

        # Make end-effector tracking directly visible during evaluation.
        self.commands.ee_pose.debug_vis = True
        self.commands.base_velocity.debug_vis = False
        self.commands.ee_pose.curriculum_enabled = False
        self.commands.base_velocity.curriculum_enabled = False

        # Remove curriculum terms in play mode so their disabled branches do
        # not overwrite these ranges with limit_ranges.
        self.curriculum.lin_vel_cmd_levels = None
        self.curriculum.ang_vel_cmd_levels = None
        self.curriculum.pos_cmd_levels = None

        self.commands.ee_pose.resampling_time_range = (4.0, 4.0)
        # Inherit the training start ranges. Test the full limit_ranges only
        # after training logs confirm that the curriculum has reached them.
        self.commands.ee_pose.ranges.roll = (0.0, 0.0)
        self.commands.ee_pose.ranges.pitch = (0.0, 0.0)
        self.commands.ee_pose.ranges.yaw = (0.0, 0.0)

        self.commands.base_velocity.resampling_time_range = (10.0, 10.0)
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
