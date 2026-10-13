"""RSL-RL PPO configurations for wheeled TITATIT-Piper."""

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


def _model_cfg() -> RslRlMLPModelCfg:
    return RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0, std_type="log"),
    )


def _algorithm_cfg() -> RslRlPpoAlgorithmCfg:
    return RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=5e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class TitatitPiperFlatPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    # Bound the Gaussian policy output before it enters ActionManager.  This
    # also keeps the raw-action rate reward finite and well scaled.
    clip_actions = 1.0
    check_for_nan = True
    max_iterations = 10000
    save_interval = 1000
    experiment_name = "titatit_piper_flat"
    actor = _model_cfg()
    critic = _model_cfg()
    algorithm = _algorithm_cfg()


@configclass
class TitatitPiperWBCPPORunnerCfg(TitatitPiperFlatPPORunnerCfg):
    experiment_name = "titatit_piper_wbc"
