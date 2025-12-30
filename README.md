# UR3e Arm Swing & IMU Teleoperation Suite

基于 MuJoCo 的机械臂摆臂模仿学习与 IMU 遥操作项目。

## 🚀 项目阶段

### Phase 1 & 2: 基础模仿与 IMU 解算器 (Completed)
- 提取人类走路摆臂轨迹并映射至 UR3e。
- 实现 `IMUSolver`：只需初始校准，即可将双 IMU 四元数实时转化为关节角度。
- 验证误差 < 1度。

### Phase 3: Sim2Real 鲁棒性与 RL (Current Focus)
- **不作弊环境**: `ur3e_blind_env.py` 实现"虚拟人"驱动架构，Agent 仅通过 IMU 历史进行控制修正。
- **抗噪训练**: 引入高斯噪声、Bias 和漂移同步训练。
- **校准随机化**: 支持 ±30° 传感器佩戴偏差随机化训练，模拟真实佩戴不准的情况。
- **对比验证**: 建立竞技场，直观对比数学解算器与 RL 策略在恶劣传感器数据下的表现。

### Phase 4: 手机遥操作与产品化 (New)
- **单手机驱动**: 支持使用手机 IMU (phyphox / Sensor Logger) 实时驱动机械臂仿真。
- **动作回放与触发**: 支持动作录制与回放，用于触发下游算法 (如计步、抬腕检测)。

---

## 📁 目录结构

```
robotics/
├── ur3e_blind_env.py          # Phase 3 核心环境 (支持校准随机化)
├── train_blind.py             # Phase 3 RL 训练脚本
├── ur3e_imitation_env.py      # Phase 1 基础模仿环境
├── train_imitation.py         # Phase 1 训练脚本
├── simple_jax_ppo.py          # JAX PPO 算法实现
├── scripts/
│   ├── phone_control_phyphox.py # 📱 手机遥操作 (推荐, 使用 phyphox)
│   ├── phone_imu_control.py     # 📱 手机遥操作 (支持 Sensor Logger)
│   ├── trajectory_analysis.py   # 📊 轨迹误差百分位分析
│   ├── plane_convergence_analysis.py # 📉 运动平面回归分析
│   ├── eval_multi_angle.py      # 🎥 多角度自动切换评估
│   ├── verify_calib_random.py   # 🔄 校准随机化稳定性验证
│   ├── verification/            # 验证脚本 (对比竞技场, 轨迹播放器等)
│   └── basic_control/           # 早期基础控制演示
├── media/                     # 验证视频与对比录影
├── models/                    # 训练好的模型权重
├── utils/                     # 噪声注入、IMU 解算器、噪声管道
├── assets/                    # UR3e MuJoCo 模型 (含竖直安装版本)
└── data/                      # 轨迹数据集 (.npz)
```

---

## 🎓 RL 训练与验证

### 校准随机化训练 (Calibration Randomization)
训练时加入随机偏置，使模型能自动纠正"戴歪"的传感器：
```bash
python train_blind.py --mode train --calib_deg 30
```

### 验证鲁棒性
在一个循环中测试 10 个具有不同随机偏置的 Episode，并自动切换视角：
```bash
python scripts/verify_calib_random.py --calib_deg 30
```

### 轨迹对比分析
计算 RL 输出与 Ground Truth 的分位数误差，验证是否回归到正确平面：
```bash
python scripts/trajectory_analysis.py --n_episodes 10 --calib_deg 30
```

---

## 📱 统一遥操作入口 (Unified Control)

**新版控制脚本** `scripts/robot_control.py` 支持 WiFi 和 BLE 两种模式，且均可连接仿真或真机。

### 1. WiFi 模式 (推荐 Phone)
配合 **phyphox** App 使用 (需开启 "斜面" -> "允许远程访问")：
```bash
# 控制仿真
python scripts/robot_control.py --source wifi --url http://192.168.1.31:8080 --target sim

# 控制真机 (需配置 IP)
python scripts/robot_control.py --source wifi --url http://192.168.1.31:8080 --target real --robot_ip 192.168.1.100
```

### 2. BLE 模式 (推荐 BHI3xx 板子)
配合 Bosch BHI360/260 传感器板子使用：

Step 1: 扫描设备地址
```bash
python scripts/ble_scan.py
# 记下板子的地址，例如 AA:BB:CC:DD:EE:FF
```

Step 2: 启动控制
```bash
python scripts/robot_control.py --source ble --address AA:BB:CC:DD:EE:FF --target sim
```

---

## 📺 验证脚本使用

### 1. 对比竞技场 (Benchmark Arena)
同屏显示两个机器人：🔴 数学解算器 vs 🟢 RL 策略

```bash
# 实时预览 (中等噪声)
python scripts/verification/benchmark_arena.py --noise_level moderate
```

### 2. 多角度自动评估
在评估模型时自动在三个视角间切换：
```bash
python scripts/eval_multi_angle.py --switch_interval 5
```

### 3. IMU 解算器验证
验证数学解算器的精度

```bash
python scripts/verification/test_imu_drive.py
```
