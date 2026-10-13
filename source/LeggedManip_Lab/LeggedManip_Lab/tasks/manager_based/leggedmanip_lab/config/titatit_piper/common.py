"""Shared configuration overrides for wheeled TITATIT-Piper tasks."""

from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from LeggedManip_Lab.assets.titatit_piper.titatit_piper_articulation_cfg import TITATIT_PIPER_CFG
from LeggedManip_Lab.tasks.manager_based.leggedmanip_lab import mdp
from LeggedManip_Lab.tasks.manager_based.leggedmanip_lab.leggedmanip_lab_env_cfg import ActionsCfg, ObservationsCfg

LEG_AND_ARM_JOINTS = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "joint[1-6]",
]
WHEEL_JOINTS = ["FR_foot_joint", "FL_foot_joint", "RR_foot_joint", "RL_foot_joint"]


@configclass
class TitatitPiperActionsCfg(ActionsCfg):
    """Position actions for legs/arm and velocity actions for wheels."""

    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=LEG_AND_ARM_JOINTS,
        scale=0.25,
        use_default_offset=True,
        preserve_order=True,
        clip={".*": (-10.0, 10.0)},
    )
    wheel_vel = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=WHEEL_JOINTS,
        scale=12.0,
        use_default_offset=True,
        preserve_order=True,
        # JointAction applies ``scale`` before clipping the processed target.
        # The old [-1, 1] clip therefore limited the wheels to only 1 rad/s.
        clip={".*": (-12.0, 12.0)},
    )


@configclass
class TitatitPiperObservationsCfg(ObservationsCfg):
    """Observe wheel velocity, but omit unbounded wheel angles."""

    def __post_init__(self) -> None:
        joint_vel_names = LEG_AND_ARM_JOINTS + WHEEL_JOINTS
        self.policy.joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"joint_names": LEG_AND_ARM_JOINTS},
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        self.policy.joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"joint_names": joint_vel_names},
            noise=Unoise(n_min=-1.5, n_max=1.5),
            scale=0.05,
        )
        self.critic.joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"joint_names": LEG_AND_ARM_JOINTS},
        )
        self.critic.joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"joint_names": joint_vel_names},
            scale=0.05,
        )


def configure_titatit_piper(env_cfg) -> None:
    """Apply robot-specific scene, command, reward, and reset settings."""
    env_cfg.scene.robot = TITATIT_PIPER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # Arm workspace, expressed relative to the renamed arm base body ``link0``.
    if hasattr(env_cfg.commands.ee_pose, "root_name"):
        env_cfg.commands.ee_pose.root_name = "link0"
    if hasattr(env_cfg.commands.ee_pose, "link_name"):
        env_cfg.commands.ee_pose.link_name = "link0"
    env_cfg.commands.ee_pose.body_name = "end_effector"

    env_cfg.events.push_robot = None
    # Several source inertias contain large off-diagonal terms. The shared
    # component-wise diagonal randomizer can make those tensors non-positive
    # definite, which PhysX rejects. Mass/COM/gain randomization remains active.
    env_cfg.events.randomize_rigid_body_inertia = None
    env_cfg.rewards.track_base_height_exp.params["target_height"] = 0.30

    # Start with moderate reset/domain randomization.  The shared quadruped
    # values are unnecessarily aggressive for this arm-equipped platform.
    env_cfg.events.reset_robot_joints.params["position_range"] = (0.9, 1.1)
    env_cfg.events.add_mass.params["mass_distribution_params"] = (0.95, 1.05)
    env_cfg.events.base_com.params["com_range"] = {
        "x": (-0.02, 0.02),
        "y": (-0.02, 0.02),
        "z": (-0.005, 0.005),
    }
    env_cfg.events.randomize_actuator_gains.params["stiffness_distribution_params"] = (0.9, 1.1)
    env_cfg.events.randomize_actuator_gains.params["damping_distribution_params"] = (0.9, 1.1)

    # Rolling wheel contact is intentionally persistent. Foot gait/air-time and
    # generic foot-slide rewards from the quadruped task are invalid for wheels.
    env_cfg.rewards.feet_slide = None
    env_cfg.rewards.feet_air_time = None
    env_cfg.rewards.feet_long_air = None
    env_cfg.rewards.air_time_variance = None
    env_cfg.rewards.joint_mirror = None
