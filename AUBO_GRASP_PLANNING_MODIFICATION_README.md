# AUBO 视觉抓取规划失败修改说明

## 1. 目的

当前项目在料斗中央区域抓取较稳定，但目标位于较偏位置时，经常出现：

- 机械臂理论工作半径足够；
- 目标点视觉坐标正常；
- MoveIt 无法规划到目标；
- Cartesian Path `fraction` 不足；
- OMPL 偶尔也无法到达；
- 越靠近料斗边缘，规划失败概率越高。

本次修改的目标不是更换复杂规划器，而是重新梳理当前项目中 **OMPL 与 Cartesian Path 的职责分工**，并修正预抓取高度、目标姿态和工作空间约束。

---

# 2. 当前代码中的主要问题

## 2.1 长距离运动首先尝试 Cartesian Path

当前项目存在类似逻辑：

```python
def direct_or_ompl(...):
    try:
        trajectory = self.compute_cartesian(...)
    except:
        self.ompl_move_to(...)
```

也就是：

```text
Cartesian Path
      ↓
失败后
      ↓
OMPL
```

这对于中央目标可能可以正常工作，但对于靠边目标不合理。

Cartesian Path 会要求 TCP 在笛卡尔空间进行连续插值。在偏远位置，机械臂可能需要：

- 改变 shoulder 构型；
- 改变 elbow 构型；
- 调整 wrist；
- 绕开料斗边缘；
- 避免奇异点；
- 避免关节限位。

这些都不适合用长距离 Cartesian Path 强行完成。

### 修改要求

以下运动必须优先使用：

```text
OMPL / RRTConnect
```

包括：

- 当前姿态 → STAGING；
- STAGING → PICK_ENTRY；
- PICK_ENTRY 的水平定位；
- 抓取完成后的 TRANSFER；
- TRANSFER → PLACE_ENTRY。

Cartesian Path 只负责：

```text
短距离、近似纯 Z 方向的下降 / 抬升。
```

---

# 3. 必须修改的文件

主要检查和修改：

```text
src/aubo_vision_grasp/aubo_vision_grasp/visual_pick_to_bin.py

src/aubo_vision_grasp/aubo_vision_grasp/auto_suction_grasp.py
```

如果 Planning Scene 参数需要一起调整，还需要检查：

```text
src/aubo_vision_grasp/aubo_vision_grasp/setup_workcell_scene.py
```

另外检查 AUBO MoveIt 配置：

```text
aubo_moveit_config/config/kinematics.yaml
aubo_moveit_config/config/ompl_planning.yaml
```

---

# 4. 第一优先级修改：修正 pregrasp_offset

当前代码存在明显的不一致。

类似代码：

```python
# 抓取前在目标上方 12 cm 停一次
self.pregrasp_offset = 0.030

self.contact_offset = 0.020
```

如果：

```python
pregrasp_offset = 0.030
contact_offset = 0.020
```

那么 PREGRASP 实际只比 CONTACT 高：

```text
0.030 - 0.020 = 0.010 m
```

即：

```text
1 cm
```

这明显与“目标上方 12 cm”的设计不一致。

## 修改为

```python
self.pregrasp_offset = 0.120
self.contact_offset = 0.020
```

推荐的高度关系：

```text
PICK_ENTRY
    │
    │
    ▼
PREGRASP     object_top_z + 0.120
    │
    │ Cartesian
    ▼
CONTACT      object_top_z + 0.020
```

不要让机械臂先深入料斗底部，再开始计算预抓取路径。

---

# 5. 第二优先级修改：重新划分 OMPL / Cartesian

## 5.1 不建议继续使用当前逻辑

不要再使用：

```text
当前位置
   ↓
Cartesian
   ↓
PICK_SAFE
```

以及：

```text
PICK_SAFE
   ↓
Cartesian
   ↓
目标上方
```

这种做法。

## 5.2 推荐的完整抓取规划流程

修改为：

```text
CURRENT
   │
   │ OMPL / RRTConnect
   ▼
STAGING
   │
   │ OMPL / RRTConnect
   ▼
PICK_ENTRY
   │
   │ Cartesian，仅改变 Z
   ▼
PREGRASP
   │
   │ Cartesian，仅改变 Z
   ▼
CONTACT
   │
   │ suction_on
   ▼
PREGRASP
   │
   │ Cartesian
   ▼
PICK_ENTRY
   │
   │ OMPL / RRTConnect
   ▼
STAGING
   │
   │ OMPL
   ▼
TRANSFER
   │
   │ OMPL
   ▼
PLACE_ENTRY
   │
   │ Cartesian，仅改变 Z
   ▼
PLACE_CONTACT
   │
   │ suction_off
   ▼
PLACE_ENTRY
```

核心原则：

```text
长距离 / 绕障碍 / 换构型
→ OMPL / RRTConnect

短距离 / 垂直接近
→ Cartesian Path
```

---

# 6. 不要再让 PICK_SAFE 等于目标 XY + 固定 safe_z

当前代码存在类似：

```python
pick_safe_x = target_x
pick_safe_y = target_y
pick_safe_z = 0.300
```

即：

```text
PICK_SAFE = (target_x, target_y, 0.300)
```

这个设计存在问题。

对于很远的目标：

```text
target_y ≈ -1.30 m
```

如果再叠加：

```text
z = 0.30 m
```

该安全点可能比真实抓取点更难达到。

因此不要将：

```text
目标 XY
+
统一固定高 Z
```

直接作为安全点。

---

# 7. 增加固定 STAGING 点

推荐增加一个固定的中间安全位姿，例如：

```python
self.staging_pose = ...
```

该位置应该满足：

- 位于料斗口外或料斗口上方；
- 不靠近工作空间极限；
- 机械臂姿态自然；
- elbow 不处于极限；
- wrist 不接近奇异点；
- 从 HOME 到该点容易规划；
- 从该点到左右两侧目标都比较容易。

路径变成：

```text
HOME
 ↓
STAGING
 ↓
PICK_ENTRY
 ↓
抓取
 ↓
STAGING
 ↓
PLACE
```

STAGING 的具体坐标必须结合真实 AUBO 基座和料斗位置标定确定，不能直接照搬某个固定值。

---

# 8. 增加 PICK_ENTRY 层

不要让 OMPL 直接规划到接近物体底部的位置。

建议：

```python
pick_entry_z = ...
```

PICK_ENTRY 应位于：

```text
料斗口附近
或者
目标上方较高位置
```

然后：

```text
PICK_ENTRY
    ↓
Cartesian
    ↓
PREGRASP
    ↓
Cartesian
    ↓
CONTACT
```

这样可以减少：

- 手腕碰墙；
- forearm 碰料斗；
- IK 突然改变；
- 靠边位置轨迹失败。

---

# 9. Cartesian Path 必须保持当前 TCP orientation

当前项目中 Cartesian Path 使用了类似固定姿态：

```python
orientation.x = 1.0
orientation.y = 0.0
orientation.z = 0.0
orientation.w = 0.0
```

也就是说每次 Cartesian 都重新指定固定向下姿态。

这个做法在偏位置容易导致：

```text
OMPL 到达 PREGRASP
       ↓
得到一个可行的 wrist/yaw
       ↓
Cartesian 又要求另一个固定 orientation
       ↓
手腕发生大角度旋转
       ↓
IK branch 切换 / joint limit / fraction 失败
```

## 修改要求

OMPL 到达 `PICK_ENTRY` 后：

1. 获取当前 TCP pose；
2. 保存当前 orientation；
3. 后续 Cartesian descent 只修改 `z`；
4. `x / y / orientation` 保持不变。

伪代码：

```python
current_pose = get_current_tcp_pose()

target_pose = copy.deepcopy(current_pose)

target_pose.position.z = pregrasp_z

# 不重新设置 orientation
```

然后：

```python
target_pose.position.z = contact_z
```

同样保持 orientation 不变。

---

# 10. OMPL 的姿态约束不要过死

当前项目如果目标要求吸盘朝下，可以保留 roll / pitch 约束。

但是不要过早锁死所有姿态自由度。

例如可以继续允许：

```text
TCP Z 轴朝下
+
绕 TCP Z 轴允许一定 yaw 自由度
```

这样 OMPL 可以根据不同目标位置选择更合理的：

```text
shoulder
elbow
wrist yaw
```

组合。

但是一旦 OMPL 到达 `PICK_ENTRY`，后面的 Cartesian 应保持该姿态，不要再切换成另一套固定 quaternion。

---

# 11. 增加 IK 可达性预检查

在真正调用 MoveIt 规划之前，建议增加：

```text
/compute_ik
```

检查至少以下两个点：

```text
PICK_ENTRY
PREGRASP
CONTACT
```

推荐流程：

```text
视觉得到目标
      ↓
生成候选抓取 Pose
      ↓
IK check
      ↓
collision check
      ↓
可行
      ↓
进入规划
```

不可行时不要继续反复调用规划器。

日志应区分：

```text
IK_UNREACHABLE

IK_COLLISION

OMPL_PLAN_FAILED

CARTESIAN_FRACTION_LOW

JOINT_LIMIT

EXECUTION_FAILED
```

这样以后调试才能知道究竟是哪一层失败。

---

# 12. 检查工作空间过滤方式

当前视觉目标过滤不能只使用矩形：

```python
xmin < x < xmax
ymin < y < ymax
```

因为机械臂的工作空间并不是矩形。

建议保留矩形 ROI 用于过滤料斗之外的点，但再增加：

```text
IK feasibility check
```

最终判断：

```text
ROI 内
AND
IK 可解
AND
无碰撞
```

才允许抓取。

---

# 13. 料斗边缘安全距离过小

当前目标允许非常靠近料斗墙时，需要特别检查。

如果目标中心距离墙只有：

```text
20 ~ 30 mm
```

虽然 TCP 数学坐标可能在工作空间中，但实际：

```text
吸盘
法兰
wrist
forearm
```

可能已经与料斗发生碰撞。

所以需要区分：

```text
TCP 可到达
```

和：

```text
整条机械臂无碰撞可到达
```

这两个概念。

建议将视觉 ROI 的墙边安全 margin 调大。

初步可从：

```text
50 mm
```

甚至：

```text
80 mm
```

开始测试。

最终 margin 根据：

- 吸盘半径；
- TCP 到法兰距离；
- wrist 外径；
- 实际料斗倾角；

再确定。

不要直接把 20 mm 左右的区域认为是安全抓取区。

---

# 14. KDL IK 参数调整

AUBO MoveIt 当前如果使用 KDL：

```yaml
kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
```

需要检查：

```yaml
kinematics_solver_search_resolution
kinematics_solver_timeout
kinematics_solver_attempts
```

如果当前类似：

```yaml
kinematics_solver_timeout: 0.005
kinematics_solver_attempts: 3
```

可以先调整为：

```yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 10
```

注意：

**这不是第一优先级修改。**

必须先解决：

- 路径设计错误；
- pregrasp_offset；
- Cartesian 长距离使用；
- collision；
- orientation；

再调 IK timeout。

否则只是让一个本来不可达的目标计算更久。

---

# 15. OMPL Planner

当前阶段不建议更换规划器体系。

继续使用：

```text
OMPL
+
RRTConnect
```

即可。

建议显式设置：

```python
planner_id = "RRTConnectkConfigDefault"
```

如当前接口支持，同时可以适当增加：

```text
planning_time
planning_attempts
```

例如：

```text
planning_time = 3 ~ 5 s
planning_attempts = 5
```

具体 API 根据当前 MoveIt2 Python / action 接口实现。

暂时没有必要优先尝试：

```text
RRT*
PRM*
CHOMP
STOMP
```

因为如果失败原因是：

```text
GOAL_IN_COLLISION
NO_IK_SOLUTION
JOINT_LIMIT
```

换 planner 仍然无效。

---

# 16. 建议修改 direct_or_ompl()

当前函数名：

```python
direct_or_ompl()
```

容易继续沿用“先 Cartesian 后 OMPL”的错误逻辑。

建议拆成两个明确接口：

```python
move_ompl(...)
move_cartesian_z(...)
```

例如：

```python
def move_ompl(self, pose, stage_name):
    ...
```

以及：

```python
def move_cartesian_z(self, target_z, stage_name):
    ...
```

避免同一个接口自动选择规划方式。

规划方式应该由任务阶段明确决定。

---

# 17. 推荐状态机

建议抓取主状态机写清楚：

```python
STATE_HOME
STATE_STAGING
STATE_PICK_ENTRY
STATE_PREGRASP
STATE_CONTACT
STATE_SUCTION
STATE_LIFT
STATE_STAGING_RETURN
STATE_TRANSFER
STATE_PLACE_ENTRY
STATE_PLACE_CONTACT
STATE_RELEASE
STATE_RETREAT
STATE_DONE
```

规划器对应关系：

```text
HOME -> STAGING
OMPL

STAGING -> PICK_ENTRY
OMPL

PICK_ENTRY -> PREGRASP
Cartesian Z

PREGRASP -> CONTACT
Cartesian Z

CONTACT -> PREGRASP
Cartesian Z

PREGRASP -> PICK_ENTRY
Cartesian Z

PICK_ENTRY -> STAGING
OMPL

STAGING -> TRANSFER
OMPL

TRANSFER -> PLACE_ENTRY
OMPL

PLACE_ENTRY -> PLACE_CONTACT
Cartesian Z

PLACE_CONTACT -> PLACE_ENTRY
Cartesian Z
```

---

# 18. 必须增加的调试日志

每个目标至少输出：

```text
========== NEW TARGET ==========

Target xyz:
Object top z:

Distance from base:

PICK_ENTRY xyz:
PREGRASP xyz:
CONTACT xyz:

IK PICK_ENTRY:
IK PREGRASP:
IK CONTACT:

OMPL planning result:
Planning time:

Cartesian fraction:
Max joint step:

Collision:
Execution result:
===============================
```

如果规划失败，需要明确输出：

```text
FAILED_STAGE = PICK_ENTRY
FAILED_REASON = NO_IK
```

或者：

```text
FAILED_STAGE = DESCEND
FAILED_REASON = CARTESIAN_FRACTION_0.73
```

不能只输出：

```text
plan failed
```

---

# 19. 建议增加失败目标可视化

调试时可以通过 RViz Marker 发布：

```text
绿色球：
IK 可达 + 无碰撞

黄色球：
IK 可达但规划失败

红色球：
无 IK / 碰撞
```

这样在料斗中很容易看出：

```text
中央成功
边缘失败
```

到底形成什么空间分布。

---

# 20. 修改顺序

必须按以下顺序进行，避免同时修改太多参数后无法定位原因。

## 第 1 轮

只修改：

```text
pregrasp_offset = 0.120
```

并记录成功率。

---

## 第 2 轮

修改：

```text
长距离全部改 OMPL
Cartesian 只做 Z 向短距离运动
```

---

## 第 3 轮

增加：

```text
STAGING
PICK_ENTRY
```

---

## 第 4 轮

修改 Cartesian：

```text
保持 OMPL 到达时的当前 orientation
只改变 Z
```

---

## 第 5 轮

增加：

```text
IK pre-check
collision pre-check
```

---

## 第 6 轮

适当扩大料斗边缘 margin。

---

## 第 7 轮

最后再调：

```text
KDL timeout
IK attempts
OMPL planning_time
OMPL planning_attempts
```

---

# 21. 修改后的推荐逻辑

最终抓取流程应为：

```text
视觉目标
   │
   ▼
目标 ROI 检查
   │
   ▼
生成 PICK_ENTRY / PREGRASP / CONTACT
   │
   ▼
IK + Collision 预检查
   │
   ├── 不可行
   │      └── 跳过目标
   │
   ▼
OMPL: CURRENT → STAGING
   │
   ▼
OMPL: STAGING → PICK_ENTRY
   │
   ▼
保存当前 TCP orientation
   │
   ▼
Cartesian Z: PICK_ENTRY → PREGRASP
   │
   ▼
Cartesian Z: PREGRASP → CONTACT
   │
   ▼
Suction ON
   │
   ▼
Cartesian Z: CONTACT → PREGRASP
   │
   ▼
Cartesian Z: PREGRASP → PICK_ENTRY
   │
   ▼
OMPL: PICK_ENTRY → STAGING
   │
   ▼
OMPL: STAGING → PLACE_ENTRY
   │
   ▼
Cartesian Z: PLACE_ENTRY → PLACE_CONTACT
   │
   ▼
Suction OFF
   │
   ▼
Cartesian Z: RETREAT
```

---

# 22. 验收测试

修改完成后不要只测试中央目标。

至少准备：

```text
中央
左侧
右侧
前侧
后侧
左前角
右前角
左后角
右后角
```

每个区域至少测试：

```text
5 次
```

记录：

```text
IK success
OMPL planning success
Cartesian success
Execution success
Grasp success
```

建议生成表：

| Region | IK | OMPL | Cartesian | Execution | Grasp |
|---|---:|---:|---:|---:|---:|
| Center |  |  |  |  |  |
| Left |  |  |  |  |  |
| Right |  |  |  |  |  |
| Front |  |  |  |  |  |
| Rear |  |  |  |  |  |
| LF |  |  |  |  |  |
| RF |  |  |  |  |  |
| LR |  |  |  |  |  |
| RR |  |  |  |  |  |

---

# 23. 本次修改的核心结论

当前问题不要优先归因于：

```text
机械臂臂展不够
```

也不要优先通过：

```text
更换 RRT*
PRM
CHOMP
STOMP
```

解决。

当前更可能的问题是：

```text
1. pregrasp_offset 配置错误；
2. 长距离错误使用 Cartesian Path；
3. PICK_SAFE 本身可能比目标更难到；
4. 靠边目标的无碰撞 IK 空间很小；
5. OMPL 和 Cartesian 使用不同 orientation；
6. Cartesian 过程中发生 wrist 大角度旋转；
7. 视觉 ROI 不等于真实机械臂可达空间；
8. KDL timeout 较短。
```

优先把运动规划架构修改为：

```text
OMPL / RRTConnect
负责长距离运动和换构型

Cartesian
只负责最后短距离直线接近 / 抬升
```

这是本次修改最重要的一条。

---

# 24. 修改时不要破坏的功能

修改过程中必须保留：

```text
视觉目标坐标输入
TF 坐标转换
Planning Scene
碰撞检测
吸盘控制
抓取 / 放置状态机
现有成功的中央区域抓取
```

不要为了让偏位置规划成功而直接：

```python
avoid_collisions = False
```

也不要删除料斗碰撞模型。

如果关闭碰撞后“能规划”，这只能用于证明：

```text
问题可能来自 collision geometry
```

不能作为最终方案。

---

# 25. 推荐最终代码结构

建议最终拆分为：

```text
visual_pick_to_bin.py
    │
    ├── target filtering
    ├── target pose generation
    ├── state machine
    └── task orchestration

auto_suction_grasp.py
    │
    ├── move_ompl()
    ├── move_cartesian_z()
    ├── compute_ik()
    ├── check_pose_feasible()
    ├── get_current_tcp_pose()
    └── execute_trajectory()

setup_workcell_scene.py
    │
    └── workcell collision geometry
```

不要继续把：

```text
Cartesian fallback
OMPL fallback
IK
execution
```

混在一个函数里自动判断。

任务层明确决定每一个阶段使用哪个规划器，后续调试会容易很多。

---

## 最终目标

修改后，即使料斗边缘某些点由于真实机构或碰撞约束仍然不可抓，也应该能够明确判断：

```text
为什么不可抓
```

而不是统一表现为：

```text
MoveIt planning failed
```

理想结果是：

```text
目标可达
→ 自动找到合理关节构型完成抓取

目标不可达
→ 在规划前明确识别并跳过
```
