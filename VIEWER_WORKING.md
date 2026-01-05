# Viewer 工作版本说明

## ✅ 可工作的代码版本

根据用户反馈，以下代码结构可以正常弹出图形界面：

```python
import mujoco
import mujoco.viewer
import time
import os

# 使用相对路径（从项目根目录运行）
xml_path = os.path.join("assets", "universal_robots_ur3e", "ur3e.xml")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.002)
```

## 🔑 关键点

1. **使用相对路径**：`os.path.join("assets", ...)` 而不是绝对路径
2. **从项目根目录运行**：确保在 `/home/bosch/Desktop/robotics` 目录下运行
3. **直接使用 `mujoco.viewer.launch_passive`**：不需要额外的配置

## 📝 已修复的脚本

已修复 `scripts/sensor_imu_control.py`：
- ✅ 添加了 `os.chdir(project_root)` 确保在项目根目录
- ✅ 使用相对路径加载模型文件
- ✅ 保持与可工作版本相同的代码结构

## 🚀 使用方法

### 方法 1: 有图形界面模式（推荐，如果环境支持）

```bash
# 确保在项目根目录
cd /home/bosch/Desktop/robotics

# 直接运行（会弹出图形界面）
python3 scripts/sensor_imu_control.py
```

### 方法 2: 无图形界面模式

```bash
# 如果图形界面无法工作，使用无 viewer 模式
python3 scripts/sensor_imu_control.py --no-viewer
```

## ⚠️ 注意事项

1. **运行目录很重要**：必须在项目根目录运行，或者脚本会自动切换到项目根目录
2. **图形界面环境**：需要有真实的 X11 显示服务器（不是 xvfb）
3. **如果 viewer 无法工作**：使用 `--no-viewer` 参数，功能完全正常

## 🔍 测试

```bash
# 测试可工作的版本
cd /home/bosch/Desktop/robotics
python3 test_user_code.py

# 测试修复后的脚本
python3 scripts/sensor_imu_control.py
```
