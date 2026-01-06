"""
BHy2CLI 传感器对比测试脚本 (ID 34 vs 37)
=====================================
同时读取 ID 34 (Rotation Vector) 和 ID 37 (Game Rotation Vector)
用于直观对比磁力计对姿态的影响。
"""

import time
import argparse
import sys
import os
import threading

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.append(project_root)

from utils.bhy2cli_receiver import BHy2CLIReceiver

def main():
    print("启动双传感器对比...")
    print("ID 37: Game Rotation Vector (无磁，仅 IMU，平滑但有漂移)")
    print("ID 34: Rotation Vector      (有磁，绝对方向，受磁场干扰)")
    print("-" * 60)

    # 启动两个接收器
    # 注意：BHy2CLI 通常只允许一个进程独占，所以这里只能串行或者开两个 CLI 实例 (如果支持)
    # 实际上 bhy2cli 支持同时开启多个传感器流 "37:50 34:50"
    # 但我们的 BHy2CLIReceiver 类封装了单个 ID。

    # 简单的做法是修改 BHy2CLIReceiver 支持多 ID，或者直接运行 CLI 命令观察
    # 这里我们简单一点，直接用 subprocess 调用 CLI 打印原始数据流

    import subprocess
    exe_path = os.path.join(project_root, "BHy2CLI", "spi_bhy2cli.exe")

    # 构造命令: foc 3 (先校准) 然后开启 37 和 34
    cmd = [exe_path, "foc", "3", "-c", "37:50", "34:50"]

    print(f"执行命令: {' '.join(cmd)}")
    print("请观察 [SID: 34] 和 [SID: 37] 的数据输出")

    try:
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            cwd=os.path.dirname(exe_path)
        )

        for line in process.stdout:
            line = line.strip()
            if "SID: 37" in line or "SID: 34" in line:
                print(line)

    except KeyboardInterrupt:
        print("停止...")
        process.terminate()

if __name__ == "__main__":
    main()
