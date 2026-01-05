#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
六面校准工具
===========
通过六面校准生成 BSX 参数 bin 文件，用于重力方向校准

使用方法：
    python3 utils/perform_six_face_calibration.py

流程：
    1. 依次将传感器放在6个不同面上（正面、背面、左侧、右侧、顶面、底面）
    2. 每个面朝下时，传感器会自动进行校准
    3. 校准完成后，使用 getbsxparam 命令读取校准数据并保存到 bin 文件
    4. 之后可以使用 setbsxparam 命令加载这些 bin 文件
"""

import subprocess
import os
import sys
import time

def find_bhy2cli_exe():
    """查找 bhy2cli 可执行文件"""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(current_dir)
    bhy_root = os.path.join(project_root, "BHy2CLI")
    
    import platform
    is_windows = platform.system() == "Windows"
    exe_ext = ".exe" if is_windows else ""
    
    base_names = ["spi_bhy2cli", "i2c_bhy2cli", "bhy2cli"]
    for name in base_names:
        path = os.path.join(bhy_root, name + exe_ext)
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return os.path.abspath(path)
    
    raise FileNotFoundError("找不到 bhy2cli 可执行文件")

def perform_six_face_calibration():
    """执行六面校准并生成 bin 文件"""
    exe_path = find_bhy2cli_exe()
    calibration_folder = "calibration"
    
    if not os.path.exists(calibration_folder):
        os.makedirs(calibration_folder)
    
    print("="*60)
    print("六面校准工具")
    print("="*60)
    print("\n说明：")
    print("1. 此工具将引导你完成六面校准")
    print("2. 需要依次将传感器放在6个不同面上（朝下）")
    print("3. 每个面需要保持静止约3-5秒")
    print("4. 校准完成后会生成 acc_calib.bin 和 gyro_calib.bin 文件")
    print("="*60)
    
    input("\n按 Enter 开始...")
    
    # 六面校准：依次将传感器放在6个不同面上
    faces = [
        ("正面", "将传感器正面朝下放在桌面上"),
        ("背面", "将传感器背面朝下放在桌面上"),
        ("左侧面", "将传感器左侧面朝下放在桌面上"),
        ("右侧面", "将传感器右侧面朝下放在桌面上"),
        ("顶面", "将传感器顶面朝下放在桌面上"),
        ("底面", "将传感器底面朝下放在桌面上"),
    ]
    
    print("\n" + "="*60)
    print("开始六面校准")
    print("="*60)
    print("需要依次将传感器放在6个不同面上，每个面进行校准")
    print("每个面需要保持静止约3-5秒")
    print("="*60)
    
    for i, (face_name, instruction) in enumerate(faces, 1):
        print(f"\n[第 {i}/6 面] {face_name}")
        print(f"{instruction}")
        print("保持静止，准备好后按 Enter 开始校准此面...")
        input()
        
        print(f"正在校准 {face_name}（等待3秒，请保持静止）...")
        time.sleep(3)
        
        # 对每个面执行 staticcalib（FOC 校准）
        # 注意：可能需要保持连接状态，所以可能需要使用交互式方式
        cmd = [exe_path, "staticcalib"]
        print(f"执行: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30, cwd=os.path.dirname(exe_path))
        
        print(result.stdout)
        if "FOC Success" in result.stdout:
            print(f"✅ {face_name} 校准成功")
        else:
            print(f"⚠️  {face_name} 校准可能未完成，但继续...")
        
        if i < len(faces):
            print(f"\n{face_name} 校准完成，准备下一个面...")
    
    print("\n" + "="*60)
    print("✅ 所有6个面校准完成！")
    print("="*60)
    
    # 步骤2: 读取 BSX 参数并保存到 bin 文件
    print("\n[最后步骤] 读取 BSX 校准参数并保存到 bin 文件...")
    print("（这会读取所有6面校准的最终结果）")
    
    params = {
        "acc": ("0x201", os.path.join(calibration_folder, "acc_calib.bin")),
        "gyro": ("0x203", os.path.join(calibration_folder, "gyro_calib.bin"))
    }
    
    success_count = 0
    for name, (pid, path) in params.items():
        abs_path = os.path.abspath(path)
        cmd = [exe_path, "getbsxparam", pid, abs_path]
        
        print(f"\n正在读取 {name} 校准数据...")
        print(f"命令: {' '.join(cmd)}")
        
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=10,
                cwd=os.path.dirname(exe_path)
            )
            
            if os.path.exists(abs_path):
                file_size = os.path.getsize(abs_path)
                print(f"✅ {name} 校准数据已保存: {abs_path} ({file_size} bytes)")
                success_count += 1
            else:
                print(f"❌ {name} 文件未创建")
                if result.stdout:
                    print(f"输出: {result.stdout}")
                if result.stderr:
                    print(f"错误: {result.stderr}")
        except subprocess.TimeoutExpired:
            print(f"❌ {name} 读取超时")
        except Exception as e:
            print(f"❌ {name} 读取出错: {e}")
    
    print("\n" + "="*60)
    if success_count == len(params):
        print("✅ 六面校准完成！")
        print(f"✅ 已生成 {success_count} 个 bin 文件在 {calibration_folder}/ 目录")
        print("\n之后可以使用以下命令加载校准数据：")
        print(f"  {exe_path} setbsxparam 0x201 {os.path.abspath(params['acc'][1])}")
        print(f"  {exe_path} setbsxparam 0x203 {os.path.abspath(params['gyro'][1])}")
    else:
        print(f"⚠️  部分校准数据保存失败 ({success_count}/{len(params)})")
        print("请检查传感器连接和校准过程")
    print("="*60)

if __name__ == "__main__":
    try:
        perform_six_face_calibration()
    except KeyboardInterrupt:
        print("\n\n用户中断")
        sys.exit(1)
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
