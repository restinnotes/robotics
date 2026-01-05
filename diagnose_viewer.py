#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
诊断 MuJoCo Viewer 问题
检查为什么 check_imu 可以工作，但其他脚本不行
"""

import os
import sys
import subprocess

print("=" * 60)
print("MuJoCo Viewer 诊断工具")
print("=" * 60)

# 1. 检查环境变量
print("\n1. 环境变量检查:")
print(f"   DISPLAY: {os.environ.get('DISPLAY', '未设置')}")
print(f"   XDG_SESSION_TYPE: {os.environ.get('XDG_SESSION_TYPE', '未设置')}")
print(f"   MUJOCO_GL: {os.environ.get('MUJOCO_GL', '未设置 (默认: glfw)')}")
print(f"   LIBGL_ALWAYS_SOFTWARE: {os.environ.get('LIBGL_ALWAYS_SOFTWARE', '未设置')}")

# 2. 检查 X 服务器
print("\n2. X 服务器检查:")
try:
    result = subprocess.run(['xdpyinfo'], capture_output=True, text=True, timeout=2)
    if result.returncode == 0:
        lines = result.stdout.split('\n')[:5]
        for line in lines:
            if line.strip():
                print(f"   {line}")
    else:
        print("   ❌ xdpyinfo 失败")
except Exception as e:
    print(f"   ❌ 无法运行 xdpyinfo: {e}")

# 3. 检查 GLX
print("\n3. GLX 检查:")
try:
    result = subprocess.run(['glxinfo'], capture_output=True, text=True, timeout=2)
    if result.returncode == 0:
        glx_lines = [l for l in result.stdout.split('\n') if 'OpenGL' in l or 'direct rendering' in l.lower()][:5]
        for line in glx_lines:
            print(f"   {line}")
    else:
        print("   ⚠️  glxinfo 不可用 (可能需要安装 mesa-utils)")
except FileNotFoundError:
    print("   ⚠️  glxinfo 未安装 (运行: sudo apt install mesa-utils)")
except Exception as e:
    print(f"   ⚠️  无法运行 glxinfo: {e}")

# 4. 检查 Python 库
print("\n4. Python 库检查:")
try:
    import mujoco
    print(f"   ✅ MuJoCo: {mujoco.__version__}")
except ImportError:
    print("   ❌ MuJoCo 未安装")

try:
    import glfw
    print(f"   ✅ GLFW: {glfw.__version__}")
    # 测试 GLFW 初始化
    if glfw.init():
        print("   ✅ GLFW 可以初始化")
        glfw.terminate()
    else:
        print("   ❌ GLFW 无法初始化")
except ImportError:
    print("   ❌ GLFW 未安装")
except Exception as e:
    print(f"   ⚠️  GLFW 测试失败: {e}")

# 5. 检查 MuJoCo viewer
print("\n5. MuJoCo Viewer 检查:")
try:
    from mujoco import viewer
    print("   ✅ viewer 模块可以导入")
    
    # 检查可用的后端
    backends = []
    try:
        from mujoco import egl
        backends.append("egl")
    except:
        pass
    
    try:
        from mujoco import osmesa
        backends.append("osmesa")
    except:
        pass
    
    try:
        from mujoco import glfw
        backends.append("glfw")
    except:
        pass
    
    print(f"   可用后端: {', '.join(backends) if backends else '无'}")
    
except ImportError as e:
    print(f"   ❌ viewer 模块导入失败: {e}")

# 6. 测试建议
print("\n" + "=" * 60)
print("诊断建议:")
print("=" * 60)

display = os.environ.get('DISPLAY')
if display:
    print(f"\n✅ 检测到显示环境: {display}")
    print("\n如果 check_imu 可以工作，但其他脚本不行，可能的原因:")
    print("1. check_imu 使用了不同的导入方式或配置")
    print("2. 运行时的环境变量不同")
    print("3. 需要安装某些缺失的库")
    print("\n建议:")
    print("1. 直接运行 check_imu 看看是否真的可以工作:")
    print("   python3 utils/check_imu.py")
    print("\n2. 如果 check_imu 可以工作，检查它的运行环境:")
    print("   env | grep -E 'DISPLAY|MUJOCO|GL'")
    print("\n3. 如果 check_imu 也不能工作，可能需要:")
    print("   sudo apt install mesa-utils")
    print("   然后运行: glxinfo | grep 'direct rendering'")
else:
    print("\n⚠️  未检测到显示环境")
    print("建议设置 DISPLAY 环境变量或使用 --no-viewer 模式")

print("\n" + "=" * 60)
