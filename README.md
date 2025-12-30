# UR3e Arm Swing & IMU Teleoperation Suite

基于 MuJoCo 的机械臂摆臂模仿学习与 IMU 遥操作项目。

## 🚀 项目阶段

### Phase 1 & 2: 基础模仿与 IMU 解算器 (Completed)
- 提取人类走路摆臂轨迹并映射至 UR3e。
- 实现 `IMUSolver`：只需初始校准，即可将双 IMU 四元数实时转化为关节角度。
- 验证误差 < 1度。

### Phase 3: Sim2Real 鲁棒性与 RL (Current Focus)
- **不作弊环境**: `ur3e_blind_env.py` 实现“虚拟人”驱动架构，Agent 仅通过 IMU 历史进行控制修正。
- **抗噪训练**: 引入高斯噪声、Bias 和漂移同步训练。
- **对比验证**: 建立竞技场，直观对比数学解算器与 RL 策略在恶劣传感器数据下的表现。

---

## 📁 目录结构

```
robotics/
├── ur3e_blind_env.py          # Phase 3 核心环境 (不作弊版)
├── train_blind.py             # Phase 3 RL 训练脚本
├── ur3e_imitation_env.py      # Phase 1 基础模仿环境
├── train_imitation.py         # Phase 1 训练脚本
├── simple_jax_ppo.py          # JAX PPO 算法实现
├── scripts/
│   ├── verification/          # 验证脚本 (对比竞技场, IMU 测试等)
│   └── basic_control/         # 早期基础控制演示
├── media/                     # 验证视频与对比录影
├── utils/                     # 噪声注入、IMU 解算等底层工具
├── assets/                    # UR3e MuJoCo 模型
└── data/                      # 轨迹数据集
```

---

## 📺 演示与对比 (Media)

视频存放于 `media/` 目录下：
- `humanoid_vs_ur3e.mp4`: H1 人形机器人原始轨迹与 UR3e 映射后动作的同步对比。
- `imu_solver_vs_gt.mp4`: 纯数学 IMU 解算器输出与 Ground Truth 轨迹的对比。
- `benchmark_moderate_v2.mp4`: 中等噪声下数学解算器 (🔴) 与 RL 策略 (🟢) 的对比。
- `benchmark_severe_v2.mp4`: 严苛噪声下数学解算器 (🔴) 与 RL 策略 (🟢) 的对比。

---

## 🛠️ 快速执行

```bash
# 1. 运行最新版的 RL 盲操评估
python train_blind.py --mode eval

# 2. 运行噪声对比竞技场
python scripts/verification/benchmark_arena.py --noise_level severe

# 3. 运行基础轨迹播放
python scripts/verification/trajectory_player.py
```

## 🛠️ 核心开发工具
- **噪声注入**: `utils/noise_injector.py`
- **IMU 解算**: `utils/imu_solver.py`
- **运动记录**: `scripts/verification/motion_recorder.py`
