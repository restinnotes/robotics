#!/bin/bash
set -e

# setup_linux.sh
# Requires sudo privileges

echo ">>> Setting up Linux environment for BHI3xx IMU Project..."

# 1. Install System Dependencies
echo ">>> Installing system packages..."
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    libusb-1.0-0-dev \
    dfu-util \
    python3-pip \
    python3-venv \
    udev

# 2. Configure udev rules for Bosch devices
echo ">>> Configuring udev rules..."
# Create a temporary udev rule file
cat <<EOF > /tmp/99-bosch-bhi.rules
# Bosch Sensortec BHI3xx / APP3.0 / APP3.1
SUBSYSTEM=="usb", ATTRS{idVendor}=="108c", ATTRS{idProduct}=="ab38", MODE="0666", GROUP="dialout"
SUBSYSTEM=="usb", ATTRS{idVendor}=="108c", ATTRS{idProduct}=="ab2c", MODE="0666", GROUP="dialout"

# DFU Mode (Bootloader)
SUBSYSTEM=="usb", ATTRS{idVendor}=="108c", ATTRS{idProduct}=="ab3d", MODE="0666", GROUP="dialout"
EOF

# Move to rules directory
sudo mv /tmp/99-bosch-bhi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo ">>> udev rules installed. You may need to replug your device."

# 3. Add user to dialout group (for serial access)
if groups $USER | grep &>/dev/null 'dialout'; then
    echo ">>> User $USER is already in 'dialout' group."
else
    echo ">>> Adding user $USER to 'dialout' group..."
    sudo usermod -a -G dialout $USER
    echo ">>> NOTE: You must log out and back in for group changes to take effect."
fi

# 4. Install Python Dependencies
echo ">>> Installing Python dependencies..."
# Check for requirements.txt, if not create basic one
pip3 install bleak pyserial scipy mujoco numpy

echo ">>> Setup Complete!"
echo ">>> Please replug your BHI3xx device."
