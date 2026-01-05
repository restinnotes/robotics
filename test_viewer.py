#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试 MuJoCo Viewer 窗口是否能正常弹出
"""

import sys
import os

# 添加项目路径
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

try:
    import mujoco
    import mujoco.viewer
    print("✅ MuJoCo 库导入成功")
except ImportError as e:
    print(f"❌ MuJoCo 库导入失败: {e}")
    print("\n请安装 MuJoCo:")
    print("  pip install mujoco")
    sys.exit(1)

# 检查模型文件
model_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e.xml")
if not os.path.exists(model_path):
    print(f"❌ 找不到模型文件: {model_path}")
    sys.exit(1)

print(f"✅ 找到模型文件: {model_path}")

try:
    print("正在加载模型...")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    print("✅ 模型加载成功")
    
    print("\n正在打开 MuJoCo Viewer 窗口...")
    print("如果窗口正常弹出，说明环境配置正确")
    print("按 ESC 或关闭窗口退出\n")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        import time
        start_time = time.time()
        while viewer.is_running() and (time.time() - start_time) < 10:
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.01)
    
    print("\n✅ 窗口测试完成！")
    
except Exception as e:
    print(f"❌ 运行出错: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
