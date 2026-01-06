#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
检查运行 MuJoCo viewer 所需的依赖
"""

import sys
import os

print("=" * 60)
print("依赖检查")
print("=" * 60)

# 1. 检查 Python 库
print("\n1. Python 库检查:")
libs = {
    'mujoco': 'MuJoCo 物理引擎',
    'numpy': 'NumPy',
    'scipy': 'SciPy',
}

for lib, desc in libs.items():
    try:
        mod = __import__(lib)
        version = getattr(mod, '__version__', '未知版本')
        print(f"  ✅ {desc}: {version}")
    except ImportError:
        print(f"  ❌ {desc}: 未安装")
        print(f"     安装命令: pip install {lib}")

# 2. 检查 OpenGL 相关
print("\n2. OpenGL 相关检查:")
try:
    import OpenGL
    print(f"  ✅ PyOpenGL: {OpenGL.__version__}")
except ImportError:
    print("  ❌ PyOpenGL: 未安装")
    print("     安装命令: pip install PyOpenGL")

try:
    import glfw
    print(f"  ✅ GLFW: {glfw.__version__}")
except ImportError:
    print("  ❌ GLFW: 未安装")
    print("     安装命令: pip install glfw")

# 3. 检查系统库
print("\n3. 系统库检查:")
import subprocess

system_libs = {
    'libgl1-mesa-glx': 'Mesa OpenGL 库',
    'libglfw3': 'GLFW3 库',
    'xvfb': 'X Virtual Framebuffer (用于无头显示)',
}

for lib, desc in system_libs.items():
    result = subprocess.run(['dpkg', '-l', lib], capture_output=True, text=True)
    if result.returncode == 0 and lib in result.stdout:
        print(f"  ✅ {desc}: 已安装")
    else:
        print(f"  ❌ {desc}: 未安装")
        print(f"     安装命令: sudo apt install {lib}")

# 4. 检查显示环境
print("\n4. 显示环境检查:")
display = os.environ.get('DISPLAY')
if display:
    print(f"  ✅ DISPLAY: {display}")
else:
    print("  ⚠️  DISPLAY: 未设置 (可能需要 xvfb)")

# 5. 检查 MuJoCo GL 后端
print("\n5. MuJoCo GL 后端:")
mujoco_gl = os.environ.get('MUJOCO_GL', '未设置')
print(f"  MUJOCO_GL: {mujoco_gl}")
print("  可选值: glfw, osmesa, egl")

# 6. 测试建议
print("\n" + "=" * 60)
print("建议:")
print("=" * 60)

if display:
    print("\n如果窗口无法弹出，尝试:")
    print("  1. 使用 xvfb (虚拟显示):")
    print("     sudo apt install xvfb")
    print("     xvfb-run -a python3 test_viewer.py")
    print("\n  2. 使用 osmesa (软件渲染):")
    print("     sudo apt install libosmesa6-dev")
    print("     MUJOCO_GL=osmesa python3 test_viewer.py")
    print("\n  3. 使用 --no-viewer 参数 (无图形界面):")
    print("     某些脚本支持 --no-viewer 参数")
else:
    print("\n检测到无显示环境，建议:")
    print("  1. 安装 xvfb:")
    print("     sudo apt install xvfb")
    print("     xvfb-run -a python3 test_viewer.py")
    print("\n  2. 或使用 --no-viewer 参数运行脚本")

print("\n" + "=" * 60)
