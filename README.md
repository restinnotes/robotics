# UR3e Arm Swing Imitation Learning

基于 MuJoCo 和 loco-mujoco 的机械臂摆臂模仿学习项目。

## 项目目标

训练 UR3e 机械臂模仿人类走路时的手臂自然摆动动作。

## 已实现功能

### 1. 轨迹数据提取
- 从 `loco-mujoco` 的 UnitreeH1 人形机器人走路数据集中提取手臂关节轨迹
- 数据文件: `data/walk_arm_direct.npz` (2000帧, 50Hz, 8关节)

### 2. 关节映射
将人形机器人手臂关节映射到 UR3e:
- `l_arm_shy` → `shoulder_pan_joint`
- `l_arm_shx` → `shoulder_lift_joint`
- `left_elbow` → `elbow_joint`
- `l_arm_shz` → `wrist_1_joint`

### 3. UR3e 模型修改
- **水平版**: `assets/universal_robots_ur3e/ur3e.xml`
- **竖直版**: `assets/universal_robots_ur3e/ur3e_vertical.xml` (更接近人类手臂姿态)
- 添加了 IMU 传感器 (加速度计、陀螺仪) 在 forearm 和 wrist

### 4. 直接控制验证
- `direct_control.py`: 基础正弦波摆臂测试
- `direct_control_imitation.py`: 使用提取的人类轨迹控制水平版 UR3e
- `direct_control_vertical.py`: 竖直版 UR3e 摆臂控制

### 5. 可视化对比
- 生成了人形机器人与 UR3e 的左右对比视频 (`outputs/side_by_side_v2.mp4`)

## 目录结构

```
robotics/
├── assets/                       # MuJoCo 模型文件
│   └── universal_robots_ur3e/
│       ├── ur3e.xml              # 水平版 UR3e
│       └── ur3e_vertical.xml     # 竖直版 UR3e
├── data/                         # 轨迹数据
│   ├── walk_arm_direct.npz       # 主要使用的走路摆臂数据
│   └── extract_trajectory_direct.py
├── outputs/                      # 生成的视频等输出
├── utils/                        # 工具脚本
│   ├── check_body.py             # 查看模型
│   ├── check_imu.py              # 测试 IMU 传感器
│   └── analyze_data.py           # 分析轨迹数据
├── archive/                      # 归档的旧文件
│   ├── train.py                  # (旧) SB3 训练脚本
│   ├── eval.py                   # (旧) SB3 评估脚本
│   └── ur3e_env.py               # (旧) Gym 环境
├── direct_control.py             # 基础控制测试
├── direct_control_imitation.py   # 人类轨迹模仿 (水平)
├── direct_control_vertical.py    # 人类轨迹模仿 (竖直)
└── loco-mujoco/                  # loco-mujoco 框架 (submodule)
```

## 快速开始

```bash
# 查看水平版 UR3e 摆臂
python direct_control_imitation.py

# 查看竖直版 UR3e 摆臂
python direct_control_vertical.py

# 查看模型
python utils/check_body.py
```

## 下一步计划

1. 创建基于 JAX 的训练环境
2. 实现关节跟踪奖励函数
3. 使用 PPO 训练竖直版 UR3e

## 依赖

- Python 3.8+
- MuJoCo
- NumPy
- loco-mujoco
