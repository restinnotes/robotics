#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试 MuJoCo viewer 创建
"""

import os
import sys
import mujoco
import mujoco.viewer

# 打印环境变量
print("=" * 60)
print("环境变量检查:")
print(f"DISPLAY: {os.environ.get('DISPLAY', '未设置')}")
print(f"LIBGL_ALWAYS_SOFTWARE: {os.environ.get('LIBGL_ALWAYS_SOFTWARE', '未设置')}")
print(f"MUJOCO_GL: {os.environ.get('MUJOCO_GL', '未设置')}")
print("=" * 60)

# 加载模型
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
model_path = os.path.join(project_root, "assets", "universal_robots_ur3e", "ur3e.xml")

print(f"\n加载模型: {model_path}")
try:
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    print("✅ 模型加载成功")
except Exception as e:
    print(f"❌ 模型加载失败: {e}")
    sys.exit(1)

# 尝试创建 viewer
print("\n尝试创建 viewer...")
try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("✅ Viewer 创建成功!")
        print("Viewer 运行中，5秒后自动关闭...")
        import time
        time.sleep(5)
        print("✅ 测试完成")
except Exception as e:
    print(f"❌ Viewer 创建失败: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
