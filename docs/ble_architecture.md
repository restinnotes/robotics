# BLE 通讯架构设计文档

## 概述

本项目使用 BLE（蓝牙低功耗）从 **BHI3xx 传感器**读取四元数数据，并同时供给 **MuJoCo 仿真环境**和**真实 UR3e 机械臂**使用。

---

## 架构图

```
┌─────────────────┐         ┌─────────────────┐         ┌─────────────────┐
│   BHI3xx        │  BLE    │ BLEIMUReceiver  │  共享   │ IMUDataSource   │
│   传感器        ├────────►│ (Python)        ├────────►│ (抽象接口)      │
└─────────────────┘         └─────────────────┘         └─────────────────┘
                                                                 │
                                          ┌──────────────────────┼──────────────────────┐
                                          │                      │                      │
                                          ▼                      ▼                      ▼
                                 ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
                                 │  MuJoCo 仿真    │    │  真实 UR3e      │    │  其他应用...    │
                                 │  环境           │    │  机械臂         │    │                 │
                                 └─────────────────┘    └─────────────────┘    └─────────────────┘
```

---

## 核心模块说明

### 1. 抽象接口 `IMUDataSource`

**文件**: `utils/imu_data_source.py`

定义统一的数据获取接口，所有数据源都实现它：

```python
class IMUDataSource(ABC):
    def get_orientation() -> Rotation   # 获取当前姿态
    def calibrate()                      # 校准零位
    def is_connected() -> bool           # 检查连接状态
    def get_euler_degrees()              # 获取欧拉角 (便捷方法)
```

**设计目的**: 上层代码只依赖这个接口，不关心数据来自 BLE、WiFi 还是文件。

---

### 2. BLE 实现 `BLEIMUReceiver`

**文件**: `utils/ble_imu_receiver.py`

实现 `IMUDataSource` 接口，负责：

| 功能 | 说明 |
|------|------|
| BLE 连接 | 使用 `bleak` 库连接 BHI3xx 设备 |
| 数据订阅 | 订阅四元数 Characteristic 的通知 |
| 数据解析 | Bosch 格式: 4 × int16 (w,x,y,z)，除以 32768 归一化 |
| 校准 | 将当前姿态设为零位，后续返回相对旋转 |
| 重连 | 预留自动重连接口 |

**数据格式**:
```
┌────────┬────────┬────────┬────────┐
│ w(2B)  │ x(2B)  │ y(2B)  │ z(2B)  │  共 8 字节
└────────┴────────┴────────┴────────┘
     │
     ▼
  int16 / 32768.0 = 归一化四元数
```

---

### 3. WiFi 备选实现 `PhyphoxReceiver`

**文件**: `utils/phyphox_receiver.py`

使用手机 App (Phyphox) 通过 WiFi 发送 IMU 数据，同样实现 `IMUDataSource` 接口。

---

## 如何满足两个使用场景

### 场景 1: MuJoCo 仿真环境

```python
from utils.ble_imu_receiver import BLEIMUReceiver

# 1. 创建数据源
imu = BLEIMUReceiver("AA:BB:CC:DD:EE:FF")
await imu.connect()
imu.calibrate()

# 2. 在仿真循环中使用
while simulating:
    orientation = imu.get_orientation()

    # 将姿态写入仿真模型
    euler = orientation.as_euler('xyz')
    sim.data.qpos[joint_id] = euler[0]  # 示例

    sim.step()
```

**关键点**:
- 直接调用 `get_orientation()` 获取最新姿态
- 转换为关节角度写入模型
- 每帧调用，保持实时同步

---

### 场景 2: 真实 UR3e 机械臂

```python
from utils.ble_imu_receiver import BLEIMUReceiver

# 1. 共用同一个数据源
imu = BLEIMUReceiver("AA:BB:CC:DD:EE:FF")
await imu.connect()
imu.calibrate()

# 2. 在控制循环中使用
while controlling:
    orientation = imu.get_orientation()

    # 转换为机械臂关节角度
    euler = orientation.as_euler('xyz')
    target_joints = compute_ik(euler)

    # 发送给真实机械臂
    robot.movej(target_joints)
```

**关键点**:
- 与仿真使用**完全相同的代码**获取姿态
- 只是后处理不同（发送给真机而非写入仿真）

---

## 同时供给两端的实现方式

```python
# 单一数据源，两个消费者

imu_source = BLEIMUReceiver("AA:BB:CC:DD:EE:FF")
await imu_source.connect()
imu_source.calibrate()

# 仿真线程
def sim_loop():
    while True:
        orientation = imu_source.get_orientation()  # 读取
        update_simulation(orientation)

# 机械臂控制线程
def robot_loop():
    while True:
        orientation = imu_source.get_orientation()  # 同一份数据
        send_to_robot(orientation)

# 两个线程并行运行，共享同一个 imu_source
```

**数据流**:
```
BLE 数据 ──► BLEIMUReceiver._raw_quat (内部缓存)
                    │
                    ├──► sim_loop() 读取
                    │
                    └──► robot_loop() 读取
```

---

## 配置说明

### 需要确认的 UUID

当前代码使用占位符 UUID，需要替换为实际值：

```python
# utils/ble_imu_receiver.py

# TODO: 替换为你的 BHI3xx 固件中的实际 UUID
DEFAULT_SERVICE_UUID = "0000180a-0000-1000-8000-00805f9b34fb"
DEFAULT_QUATERNION_CHAR_UUID = "00002a5d-0000-1000-8000-00805f9b34fb"
```

**如何获取正确的 UUID**:
1. 使用手机 App (如 nRF Connect) 扫描 BHI3xx
2. 连接后查看 Services 和 Characteristics
3. 找到四元数数据对应的 UUID
4. 替换到代码中

---

## 依赖

```bash
pip install bleak scipy
```

- `bleak`: 跨平台 Python BLE 库
- `scipy`: 用于 `Rotation` 四元数运算

---

## 优势总结

| 特性 | 说明 |
|------|------|
| **统一接口** | 上层代码只依赖 `IMUDataSource`，数据源可自由切换 |
| **一次连接** | BLE 只连一次，避免重复开销 |
| **多端共享** | MuJoCo 和真机读取同一份数据，保持一致 |
| **易于扩展** | 新增数据源只需实现 `IMUDataSource` 接口 |
| **校准支持** | 内置零位校准，返回相对姿态 |

---

## 文件结构

```
utils/
├── imu_data_source.py      # 抽象接口定义
├── ble_imu_receiver.py     # BLE 实现 (BHI3xx, 无线)
├── usb_imu_receiver.py     # USB 实现 (BHI3xx, 有线直连, Linux)
└── phyphox_receiver.py     # WiFi 实现 (手机备选)
```

---

## USB 直连方案 (Linux Ubuntu)

### 适用场景
- USB 直接连接 BHI3xx 开发板
- 适合 Linux 环境下的应用
- 比 BLE 更稳定、低延迟

### 使用方法

```python
from utils.usb_imu_receiver import USBIMUReceiver

# 1. 创建接收器
receiver = USBIMUReceiver(
    port="/dev/ttyUSB0",  # 或 /dev/ttyACM0
    baudrate=115200,
    data_format="raw"  # 或 "framed"
)

# 2. 连接
if receiver.connect():
    receiver.calibrate()

    # 3. 使用
    while True:
        orientation = receiver.get_orientation()
        euler = receiver.get_euler_degrees()
        print(f"Roll: {euler[0]:.1f}, Pitch: {euler[1]:.1f}, Yaw: {euler[2]:.1f}")

# 4. 断开
receiver.disconnect()
```

### 串口权限设置

```bash
# 方法 1: 临时授权
sudo chmod 666 /dev/ttyUSB0

# 方法 2: 将用户添加到 dialout 组 (推荐，永久生效)
sudo usermod -a -G dialout $USER
# 重新登录后生效

# 扫描可用串口
python utils/usb_imu_receiver.py --scan
```

### 依赖安装

```bash
pip install pyserial scipy
```
