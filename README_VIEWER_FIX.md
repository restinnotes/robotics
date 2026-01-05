# MuJoCo Viewer 问题解决方案

## ✅ 已完成的工作

### 1. bhy2cli_receiver Linux 适配
- ✅ 修改了 `utils/bhy2cli_receiver.py`
- ✅ 添加了平台检测（Windows/Linux）
- ✅ 自动查找可执行文件（支持 `.exe` 和无扩展名）
- ✅ 已验证可以找到 Linux 可执行文件

### 2. 添加无 viewer 模式
- ✅ 修改了 `scripts/sensor_imu_control.py`
- ✅ 添加了 `--no-viewer` 命令行选项
- ✅ 可以在无图形界面环境下运行

## 🚀 使用方法

### 方法 1: 无图形界面模式（推荐，适用于当前环境）

```bash
# 直接运行，不需要 viewer
python3 scripts/sensor_imu_control.py --no-viewer
```

**优点**:
- 不需要安装额外的图形库
- 不需要配置 xvfb 或显示环境
- 可以在任何 Linux 环境中运行
- 功能完全正常，只是没有可视化窗口

### 方法 2: 有图形界面模式（需要真实显示环境）

```bash
# 在有图形界面的 Linux 桌面环境中运行
python3 scripts/sensor_imu_control.py
```

**要求**:
- 需要有真实的 X11 显示服务器
- 或者通过 SSH X11 转发: `ssh -X user@host`

### 方法 3: 使用 xvfb（如果将来需要）

如果将来需要 viewer 功能，可以尝试：

```bash
# 使用 xvfb（但当前配置下 viewer 仍可能无法工作）
xvfb-run -a python3 scripts/sensor_imu_control.py
```

## 📝 测试命令

```bash
# 1. 测试 bhy2cli_receiver 是否能找到可执行文件
python3 -c "from utils.bhy2cli_receiver import BHy2CLIReceiver; r = BHy2CLIReceiver(); print(f'✅ 找到: {r.exe_path}')"

# 2. 查看脚本帮助
python3 scripts/sensor_imu_control.py --help

# 3. 运行无 viewer 模式（推荐）
python3 scripts/sensor_imu_control.py --no-viewer
```

## 🔍 问题诊断

### Viewer 无法弹出的原因

1. **GLFW 需要完整的 GLX 支持**
   - MuJoCo 的 `viewer.launch_passive` 默认使用 GLFW
   - GLFW 需要完整的 GLX 配置，xvfb 的 GLX 实现不完整

2. **Ubuntu 24.04 包变化**
   - `libgl1-mesa-glx` 已废弃
   - 新的包结构可能不完全兼容

3. **EGL/OSMesa 后端问题**
   - 即使设置了 `MUJOCO_GL=egl`，`launch_passive` 仍可能使用 GLFW
   - OSMesa 在某些环境下有兼容性问题

## 💡 建议

**对于当前环境，强烈建议使用 `--no-viewer` 模式**：
- ✅ 功能完全正常
- ✅ 不需要额外的库或配置
- ✅ 可以在任何环境中运行
- ✅ 性能可能更好（没有渲染开销）

如果需要可视化，可以考虑：
- 使用远程桌面或 VNC
- 使用 SSH X11 转发
- 或者将来在有图形界面的环境中运行

## 📚 相关文件

- `utils/bhy2cli_receiver.py` - 已适配 Linux
- `scripts/sensor_imu_control.py` - 已添加 `--no-viewer` 选项
- `VIEWER_FIX.md` - 详细的问题分析
- `INSTALL_LIBS.md` - 库安装指南（如果需要）
