# BHI3xx Linux Ubuntu 使用指南

本文档介绍如何在 Linux Ubuntu 上配置、烧写和使用 BHI3xx 传感器开发板 (APP3.0 / APP3.1)。

## 1. 快速开始

### 1.1 安装依赖与权限
我们提供了一键安装脚本，用于安装系统依赖 (gcc, make, libusb, Python) 并配置 USB 权限 (udev rules)。

```bash
# 在项目根目录下运行
chmod +x scripts/setup_linux.sh
./scripts/setup_linux.sh
```

> **注意**: 脚本运行后，请**注销并重新登录**，以使 `dialout` 组权限生效。

### 1.2 编译 BHy2CLI 工具 (可选)
如果你需要使用命令行的 `bhy2cli` 工具来调试传感器：

```bash
chmod +x scripts/build_bhy2cli_linux.sh
./scripts/build_bhy2cli_linux.sh
```
编译完成后，二进制文件位于 `BHy2CLI/release/Linux/bin/bhy2cli`。

---

## 2. 固件烧写

在 Linux 上，我们使用标准开源工具 `dfu-util` 来烧写固件。

1. **进入 DFU 模式**:
   - 按住开发板上的 **BOOT** 按钮。
   - 按一下 **RESET** 按钮。
   - 松开 **BOOT** 按钮。
   - 此时开发板进入 DFU 模式 (通常指示灯会变化或熄灭)。

2. **运行烧写脚本**:
   ```bash
   chmod +x scripts/flash_firmware.sh
   ./scripts/flash_firmware.sh
   ```
   脚本会自动检测 DFU 设备并引导烧写 Bootloader 和固件。

---

## 3. 使用 Python 读取数据

### 3.1 USB 直连 (推荐)
这是最稳定、低延迟的方式，适合机器人控制。

```bash
# 扫描可用串口
python3 utils/usb_imu_receiver.py --scan

# 读取数据 (自动检测 /dev/ttyUSB* 或 /dev/ttyACM*)
python3 utils/usb_imu_receiver.py
```

### 3.2 BLE 无线连接
使用 `bleak` 库进行蓝牙连接。确保你的电脑支持蓝牙 4.0+。

```bash
# 扫描 BLE 设备
python3 utils/ble_imu_receiver.py

# (代码会自动扫描并连接第一个发现的 BHI3xx 设备)
```

---

## 4. 常见问题 (FAQ)

**Q: 运行 Python 脚本提示 "Permission denied: '/dev/ttyUSB0'"**
A: 你的用户没有访问串口的权限。请确保运行了 `./scripts/setup_linux.sh` 并**重新登录**了系统。或者手动运行: `sudo usermod -a -G dialout $USER`。

**Q: `flash_firmware.sh` 找不到设备**
A: 请确保先把开发板切换到 **DFU 模式**。普通运行模式下无法烧写 Bootloader。

**Q: BHy2CLI 编译失败**
A: 确保安装了 `build-essential`。检查 `BHy2CLI/Makefile` 是否被修改。
