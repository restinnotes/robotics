#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版测试 - 测试 MuJoCo viewer 是否能创建
"""

import sys
import os

# 添加项目路径
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

print("=" * 60)
print("MuJoCo Viewer 测试")
print("=" * 60)

# 检查环境
print(f"\n环境变量:")
print(f"  DISPLAY: {os.environ.get('DISPLAY', '未设置')}")
print(f"  MUJOCO_GL: {os.environ.get('MUJOCO_GL', '未设置 (默认: glfw)')}")

try:
    import mujoco
    import mujoco.viewer
    print(f"\n✅ MuJoCo {mujoco.__version__} 导入成功")
except ImportError as e:
    print(f"\n❌ MuJoCo 导入失败: {e}")
    sys.exit(1)

# 检查模型文件
model_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e.xml")
if not os.path.exists(model_path):
    print(f"\n❌ 找不到模型文件: {model_path}")
    sys.exit(1)

print(f"✅ 找到模型文件")

try:
    print("\n正在加载模型...")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    print("✅ 模型加载成功")
    
    print("\n正在尝试创建 viewer...")
    print("(如果成功，窗口应该会弹出或创建)")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("✅ Viewer 创建成功!")
        print("Viewer 正在运行...")
        
        import time
        # 运行 2 秒
        start_time = time.time()
        step_count = 0
        while (time.time() - start_time) < 2.0:
            mujoco.mj_step(model, data)
            viewer.sync()
            step_count += 1
            time.sleep(0.01)
        
        print(f"✅ 测试完成! (运行了 {step_count} 步)")
        print("如果看到这条消息，说明 viewer 工作正常!")
    
except Exception as e:
    print(f"\n❌ 错误: {e}")
    import traceback
    traceback.print_exc()
    
    print("\n" + "=" * 60)
    print("解决方案:")
    print("=" * 60)
    print("\n1. 使用 xvfb (虚拟显示):")
    print("   xvfb-run -a python3 test_viewer_simple.py")
    print("\n2. 安装 EGL 库:")
    print("   sudo apt install libegl1-mesa-dev libgles2-mesa-dev")
    print("   MUJOCO_GL=egl python3 test_viewer_simple.py")
    print("\n3. 安装 OSMesa 库:")
    print("   sudo apt install libosmesa6-dev")
    print("   MUJOCO_GL=osmesa python3 test_viewer_simple.py")
    sys.exit(1)

print("\n" + "=" * 60)
print("✅ 测试通过! Viewer 可以正常工作")
print("=" * 60)
