#!/bin/bash
# 使用 Xvfb 虚拟显示运行 bhy2cli_robot_control.py

# 检查是否安装了 xvfb
if ! command -v xvfb-run &> /dev/null; then
    echo "❌ 未找到 xvfb-run"
    echo ""
    echo "请安装 xvfb:"
    echo "  sudo apt install xvfb"
    echo ""
    echo "或者使用 --no-viewer 参数运行（无图形界面）:"
    echo "  python3 scripts/bhy2cli_robot_control.py --no-viewer"
    exit 1
fi

# 使用 xvfb-run 运行脚本
echo "🚀 使用 Xvfb 虚拟显示运行..."
xvfb-run -a -s "-screen 0 1024x768x24" python3 scripts/bhy2cli_robot_control.py "$@"
