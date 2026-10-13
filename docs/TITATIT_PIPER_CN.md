# TITATIT-Piper：URDF 转 USD 与训练

源模型为 `titatit_piper/wheeled_titatit_rl_total.urdf`。它包含 12 个腿部关节、4 个轮关节、Piper 6 轴机械臂以及夹爪。

## 1. 转换 USD

原 URDF 同时使用了 `../meshes_mujoco/` 和 `package://agx_arm_description/...`，不能直接在当前目录解析。仓库内的转换脚本会在临时 URDF 中解析这些路径，并把机械臂基座和末端分别映射为公共任务使用的 `link0` 与 `end_effector`。它还会去掉无质量的占位根节点、保留语义固定刚体，并给空的夹爪主滑块补充极小惯量和几何；原 URDF 不会被修改。

在安装了 Isaac Sim / Isaac Lab 的 Python 环境中执行：

```bash
conda activate wlarm
python scripts/tools/convert_titatit_piper.py --headless
```

默认输出：

```text
source/LeggedManip_Lab/LeggedManip_Lab/assets/titatit_piper/titatit_piper.usd
```

如果 Isaac Lab 位于独立目录，也可使用它的启动脚本：

```bash
/path/to/IsaacLab/isaaclab.sh -p scripts/tools/convert_titatit_piper.py --headless
```

## 2. 检查任务

首次使用或代码更新后安装扩展：

```bash
python -m pip install -e source/LeggedManip_Lab
python scripts/list_envs.py --keyword TITATIT
```

先以少量环境做零动作和随机动作检查：

```bash
python scripts/tools/smoke_test_titatit_piper.py --task TITATIT-PIPER-Flat --num_envs 4 --steps 500 --headless
python scripts/random_agent.py --task TITATIT-PIPER-Flat --num_envs 4
```

## 3. 训练与回放

```bash
# 平地移动操作
python scripts/rsl_rl/train.py --task TITATIT-PIPER-Flat --num_envs 4096 --headless --max_iterations 5000

# 混合坐标系的全身控制
python scripts/rsl_rl/train.py --task TITATIT-PIPER-WBC --num_envs 4096 --headless --max_iterations 5000

# 加载最近一次 Flat checkpoint 回放
python scripts/rsl_rl/play.py --task TITATIT-PIPER-Flat-Play
```

轮关节使用速度动作（范围约为 `[-12, 12] rad/s`），腿部和机械臂使用位置动作。轮体持续接地，因此本任务关闭了四足步态中的腾空时间、足端滑动和关节镜像奖励。500 步零动作测试中基座稳定高度约为 `0.302 m`，所以高度奖励目标设为 `0.30 m`。关节刚度和轮速范围是安全起点，正式训练前仍应根据实机电机参数继续标定。
# GitHub 代码副本的模型依赖

训练日志和本地配置备份不上传。203 MB 的生成模型文件
`source/LeggedManip_Lab/LeggedManip_Lab/assets/titatit_piper/configuration/titatit_piper_base.usd`
也不包含在代码仓库中。新机器运行 TITATIT 任务前，必须从原机器复制此文件到同一路径，
或准备源 URDF 与网格文件后使用 `scripts/tools/convert_titatit_piper.py` 重新生成模型。
其余 USD 文件依赖该文件，单独克隆代码不能直接加载完整机器人。

