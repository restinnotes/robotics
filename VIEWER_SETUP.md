# MuJoCo Viewer 窗口设置指南

## 问题诊断

当前测试显示窗口无法弹出，主要原因是 **GLX 配置问题**。GLFW 需要 GLX 支持，但当前显示环境可能不支持。

## 已安装的库

✅ **Python 库**:
- MuJoCo 3.4.0
- NumPy, SciPy
- PyOpenGL, GLFW

✅ **系统库**:
- libglfw3
- Mesa OpenGL 库
- xvfb (X Virtual Framebuffer)

## 解决方案

### ⭐ 方案 1: 使用 xvfb (最简单，推荐)

xvfb 已经安装，可以直接使用：

```bash
# 测试 viewer
xvfb-run -a python3 test_viewer_simple.py

# 运行传感器控制脚本
xvfb-run -a python3 scripts/sensor_imu_control.py

# 或使用项目提供的脚本
./scripts/run_bhy2cli_with_xvfb.sh
```

**优点**: 不需要安装额外库，适用于无头服务器

### 方案 2: 安装 EGL 后端库

```bash
# 安装 EGL 相关库
sudo apt install libegl1-mesa-dev libgles2-mesa-dev

# 运行测试 (注意：可能需要配合 xvfb)
MUJOCO_GL=egl xvfb-run -a python3 test_viewer_simple.py
```

### 方案 3: 安装 OSMesa (软件渲染)

```bash
# 安装 OSMesa
sudo apt install libosmesa6-dev

# 运行测试
MUJOCO_GL=osmesa python3 test_viewer_simple.py
```

**注意**: OSMesa 在某些环境下可能仍有问题

## 快速测试命令

```bash
# 测试 1: 使用 EGL
MUJOCO_GL=egl python3 test_viewer.py

# 测试 2: 使用 xvfb
xvfb-run -a python3 test_viewer.py

# 测试 3: 使用 osmesa (需要先安装)
MUJOCO_GL=osmesa python3 test_viewer.py
```

## 运行实际控制脚本

如果窗口测试成功，可以运行实际的传感器控制脚本：

```bash
# 使用 xvfb 运行 (推荐)
xvfb-run -a python3 scripts/sensor_imu_control.py

# 或者使用项目提供的脚本
./scripts/run_bhy2cli_with_xvfb.sh

# 无图形界面模式 (如果窗口无法工作)
python3 scripts/sensor_imu_control.py --no-viewer  # 如果支持
```

## 常见问题

### 1. GLX 错误
```
GLX: No GLXFBConfigs returned
```
**解决方案**: 使用 xvfb 或设置 `MUJOCO_GL=egl`

### 2. OSMesa 错误
```
AttributeError: 'NoneType' object has no attribute 'glGetError'
```
**解决方案**: 安装 `libosmesa6-dev` 或使用其他后端

### 3. 无显示环境
**解决方案**: 使用 `xvfb-run` 或 `--no-viewer` 参数

## 检查依赖

运行依赖检查脚本：
```bash
python3 check_dependencies.py
```
