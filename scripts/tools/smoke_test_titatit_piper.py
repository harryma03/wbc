#!/usr/bin/env python3
"""Run a finite zero-action smoke test for a TITATIT-Piper task."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", default="TITATIT-PIPER-Flat", choices=["TITATIT-PIPER-Flat", "TITATIT-PIPER-WBC"])
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--steps", type=int, default=500)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

import LeggedManip_Lab.tasks  # noqa: E402, F401


def main() -> None:
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.observations.policy.enable_corruption = False
    env_cfg.commands.ee_pose.debug_vis = False
    env_cfg.commands.base_velocity.debug_vis = False
    env_cfg.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)

    env = gym.make(args_cli.task, cfg=env_cfg)
    env.reset()
    robot = env.unwrapped.scene["robot"]
    actions = torch.zeros(env.action_space.shape, device=env.unwrapped.device)
    min_height = torch.full((args_cli.num_envs,), torch.inf, device=env.unwrapped.device)
    max_height = torch.full((args_cli.num_envs,), -torch.inf, device=env.unwrapped.device)
    termination_count = 0

    with torch.inference_mode():
        for _ in range(args_cli.steps):
            observations, rewards, terminated, truncated, _ = env.step(actions)
            height = robot.data.root_pos_w[:, 2]
            min_height = torch.minimum(min_height, height)
            max_height = torch.maximum(max_height, height)
            termination_count += int(torch.count_nonzero(terminated | truncated).item())
            if not torch.isfinite(rewards).all():
                raise RuntimeError("Non-finite reward detected.")
            policy_obs = observations["policy"]
            if not torch.isfinite(policy_obs).all():
                raise RuntimeError("Non-finite policy observation detected.")

    print(f"Task: {args_cli.task}")
    print(f"Steps: {args_cli.steps}, environments: {args_cli.num_envs}")
    print(f"Action dimension: {env.action_space.shape[-1]}")
    print(f"Bodies ({len(robot.body_names)}): {robot.body_names}")
    print(f"Joints ({len(robot.joint_names)}): {robot.joint_names}")
    print(f"Root height range: [{min_height.min().item():.4f}, {max_height.max().item():.4f}] m")
    print(f"Terminations (including timeouts): {termination_count}")
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
