# UR3e Arm Swing & IMU Teleoperation Suite

基于 MuJoCo 的机械臂摆臂模仿学习与 IMU 遥操作项目。

## 🚀 项目阶段

## 📥 安装 (Installation)

1. 克隆仓库:
   ```bash
   git clone https://github.com/restinnotes/robotics.git
   cd robotics
   ```

2. 安装依赖:
   ```bash
   pip install -r requirements.txt
   ```

3. (可选) Linux用户:
   运行 setup 脚本配置 udev 权限:
   ```bash
   bash scripts/setup_linux.sh
   ```

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
├── scripts/
│   ├── basic_control/           # 早期基础控制演示
│   ├── training/                # RL 训练脚本 (train_blind.py 等)
│   ├── tests/                   # 单元测试与调试脚本
│   ├── utils/                   # 工具类脚本 (分析、绘图等)
│   ├── verification/            # 验证脚本 (对比竞技场, 轨迹播放器等)
│   ├── archive/                 # 归档的旧脚本
│   ├── setup_linux.sh           # Linux 环境安装脚本
│   ├── sensor_imu_control.py    # 核心：IMU 姿态解算与控制
│   └── arm_control_gui.py       # 核心：GUI 控制界面
├── ur3e_blind_env.py          # Phase 3 核心环境
├── ur3e_imitation_env.py      # Phase 1 基础模仿环境
├── utils/                     # 核心工具库 (IMU解算器、噪声模型等)
├── assets/                    # UR3e MuJoCo 模型
├── data/                      # 轨迹数据集
├── docs/                      # 文档
└── requirements.txt           # 项目依赖
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

### 1. WiFi 模式 (推荐 Phone -> Sim)
主要用于使用手机 APP (Phyphox) 控制 **仿真环境 (Simulation)**。手机模拟器方便前期调试算法。
```bash
# 控制仿真
python scripts/robot_control.py --source wifi --url http://192.168.1.31:8080 --target sim
```

### 2. BLE 模式 (推荐 BHI3xx Sensor -> Real)
主要用于使用真实的 **Bosch BHI360/260 传感器** 控制 **真机 (Real Robot)** 或高精度仿真。
```bash
# 扫描设备
python scripts/archive/ble_scan.py

# 启动控制 (连接真机)
python scripts/robot_control.py --source ble --address AA:BB:CC:DD:EE:FF --target real --robot_ip 192.168.1.100
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
