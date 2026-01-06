#!/usr/bin/env python3
"""Temporary script to simplify the control logic."""
import re

with open(r'c:\Users\MECHREVO\Desktop\robotics\robotics\scripts\sensor_imu_arm_control.py', 'r', encoding='utf-8') as f:
    content = f.read()

# Replace the complex vector projection logic with simple Euler angles
# Pattern: from "if orientation:" to before "# 累积控制量"
pattern = r'if orientation:.*?# 累积控制量'
replacement = '''if orientation:
            # Simple Euler angle mapping (like human arm)
            euler = orientation.as_euler('zyx', degrees=False)
            yaw, pitch, roll = euler

            # Direct mapping with scale
            self.target_pan = yaw * self.scale

            # Lift: base is -1.57 (down), add pitch
            # Clamp to human-like range: 0 (horizontal) to -3.14 (pointing back)
            raw_lift = -1.57 + pitch * self.scale
            self.target_lift = np.clip(raw_lift, -3.14, 0.0)

            # 累积控制量'''

new_content = re.sub(pattern, replacement, content, flags=re.DOTALL)

# Also remove the now-unused delta/target accumulation logic
# Replace the accumulation block with direct application
old_accum = r'# 累积控制量.*?self\.target_lift = self\._target_lift'
new_accum = '''# Direct control (no accumulation needed with Euler angles)
            # target_pan and target_lift are already set above'''

new_content = re.sub(old_accum, new_accum, new_content, flags=re.DOTALL)

with open(r'c:\Users\MECHREVO\Desktop\robotics\robotics\scripts\sensor_imu_arm_control.py', 'w', encoding='utf-8') as f:
    f.write(new_content)

print("Control logic simplified successfully!")
