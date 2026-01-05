#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试 MuJoCo viewer 使用 EGL 后端
"""

import sys
import os

# 强制设置 EGL 后端
os.environ['MUJOCO_GL'] = 'egl'

# 添加项目路径
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

print("=" * 60)
print("MuJoCo Viewer EGL 后端测试")
print("=" * 60)

print(f"\n环境变量:")
print(f"  DISPLAY: {os.environ.get('DISPLAY', '未设置')}")
print(f"  MUJOCO_GL: {os.environ.get('MUJOCO_GL', '未设置')}")

try:
    import mujoco
    from mujoco import viewer
    print(f"\n✅ MuJoCo {mujoco.__version__} 导入成功")
    print("✅ viewer 模块导入成功")
    
    # 检查可用的 GL 后端
    print("\n检查可用的 GL 后端...")
    try:
        from mujoco import egl
        print("  ✅ EGL 后端可用")
    except:
        print("  ❌ EGL 后端不可用")
    
    try:
        from mujoco import osmesa
        print("  ✅ OSMesa 后端可用")
    except:
        print("  ❌ OSMesa 后端不可用")
    
    try:
        from mujoco import glfw
        print("  ✅ GLFW 后端可用")
    except:
        print("  ❌ GLFW 后端不可用")
        
except ImportError as e:
    print(f"\n❌ MuJoCo 导入失败: {e}")
    sys.exit(1)

# 检查模型文件
model_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e.xml")
if not os.path.exists(model_path):
    print(f"\n❌ 找不到模型文件: {model_path}")
    sys.exit(1)

print(f"\n✅ 找到模型文件")

try:
    print("\n正在加载模型...")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    print("✅ 模型加载成功")
    
    print("\n正在尝试使用 EGL 后端创建 viewer...")
    
    # 尝试直接使用 EGL 上下文
    try:
        from mujoco.egl import GLContext
        print("  尝试使用 EGL.GLContext...")
        with GLContext(480, 320) as ctx:
            print("  ✅ EGL 上下文创建成功!")
            # 这里可以继续测试渲染
    except Exception as e:
        print(f"  ⚠️  EGL.GLContext 失败: {e}")
    
    # 尝试使用 launch_passive
    print("\n尝试使用 launch_passive (应该使用 EGL)...")
    with viewer.launch_passive(model, data) as v:
        print("✅ Viewer 创建成功!")
        print("Viewer 正在运行...")
        
        import time
        start_time = time.time()
        step_count = 0
        while (time.time() - start_time) < 2.0:
            mujoco.mj_step(model, data)
            v.sync()
            step_count += 1
            time.sleep(0.01)
        
        print(f"✅ 测试完成! (运行了 {step_count} 步)")
    
except Exception as e:
    print(f"\n❌ 错误: {e}")
    import traceback
    traceback.print_exc()
    
    print("\n" + "=" * 60)
    print("解决方案:")
    print("=" * 60)
    print("\n1. 安装 EGL 开发库:")
    print("   sudo apt install libegl1-mesa-dev libgles2-mesa-dev")
    print("\n2. 配置 xvfb 启用 GLX:")
    print("   xvfb-run -a -s '-screen 0 1024x768x24 +extension GLX' python3 test_viewer_egl.py")
    print("\n3. 或者使用无 viewer 模式运行脚本")
    sys.exit(1)

print("\n" + "=" * 60)
print("✅ 测试通过! EGL 后端工作正常")
print("=" * 60)
