# MuJoCo Viewer 问题修复指南

## 问题总结

经过测试，发现以下问题：
1. ✅ `bhy2cli_receiver` 已成功适配 Linux，可执行文件查找正常
2. ❌ MuJoCo viewer 无法在当前的 xvfb 环境中创建窗口
3. ❌ GLFW 需要 GLX 支持，但 xvfb 的 GLX 实现不完整

## 根本原因

MuJoCo 的 `viewer.launch_passive` 默认使用 GLFW 后端，而 GLFW 需要完整的 GLX 支持。即使在 xvfb 中启用了 GLX 扩展，GLFW 仍然无法找到合适的 GLXFBConfig。

## 解决方案

### 方案 1: 修改脚本支持无 viewer 模式 (推荐)

修改控制脚本，添加 `--no-viewer` 选项，这样可以在无图形界面环境下运行：

```python
# 在脚本中添加
parser.add_argument('--no-viewer', action='store_true', help='无图形界面模式')

# 使用条件判断
if not args.no_viewer:
    with viewer.launch_passive(model, data) as v:
        # ... 控制循环
else:
    # 无 viewer 模式，直接运行控制循环
    while True:
        # ... 控制逻辑
        time.sleep(0.02)
```

### 方案 2: 在有图形界面的环境中运行

如果需要在有图形界面的 Linux 桌面环境中运行：

```bash
# 直接运行（需要真实的显示环境）
python3 scripts/sensor_imu_control.py
```

### 方案 3: 使用远程 X11 转发 (SSH)

如果通过 SSH 连接，可以使用 X11 转发：

```bash
# SSH 连接时启用 X11 转发
ssh -X user@host

# 然后运行脚本
python3 scripts/sensor_imu_control.py
```

### 方案 4: 使用 VNC 或远程桌面

设置 VNC 服务器或使用远程桌面，在有图形界面的环境中运行。

## 快速修复脚本

我已经创建了修复版的测试脚本，但 viewer 仍然无法工作。建议：

1. **修改现有脚本添加 `--no-viewer` 选项**
2. **或者在有图形界面的环境中运行**

## 测试命令

```bash
# 测试 bhy2cli_receiver (不需要 viewer)
python3 -c "from utils.bhy2cli_receiver import BHy2CLIReceiver; r = BHy2CLIReceiver(); print(f'找到: {r.exe_path}')"

# 如果有图形界面，直接运行
python3 scripts/sensor_imu_control.py

# 如果无图形界面，需要先修改脚本添加 --no-viewer 选项
```

## 总结

- ✅ **bhy2cli_receiver Linux 适配完成** - 代码已修改，可以正常工作
- ❌ **viewer 窗口问题** - 需要在有图形界面的环境中运行，或修改脚本支持无 viewer 模式
