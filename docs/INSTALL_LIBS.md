# 安装 MuJoCo Viewer 所需库

## 当前问题

窗口无法弹出，错误信息：
```
GLX: No GLXFBConfigs returned
GLX: Failed to find a suitable GLXFBConfig
ERROR: could not create window
```

## 需要安装的库

### 1. 安装 Mesa GLX 库 (必需)

```bash
sudo apt install libgl1-mesa-glx libgl1-mesa-dri
```

### 2. 安装 EGL 库 (推荐)

```bash
sudo apt install libegl1-mesa-dev libgles2-mesa-dev
```

### 3. 安装 OSMesa 库 (备选)

```bash
sudo apt install libosmesa6-dev
```

## 安装后测试

### 测试 1: 使用 xvfb (推荐)

```bash
xvfb-run -a python3 test_viewer_simple.py
```

### 测试 2: 使用 EGL 后端

```bash
MUJOCO_GL=egl xvfb-run -a python3 test_viewer_simple.py
```

### 测试 3: 使用 OSMesa 后端

```bash
MUJOCO_GL=osmesa python3 test_viewer_simple.py
```

## 一键安装命令

```bash
sudo apt install -y \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    libosmesa6-dev
```

## 运行实际脚本

安装库后，运行传感器控制脚本：

```bash
# 使用 xvfb
xvfb-run -a python3 scripts/sensor_imu_control.py

# 或使用项目脚本
./scripts/run_bhy2cli_with_xvfb.sh
```
