#!/bin/bash
set -e

# flash_firmware.sh
# Flashes BHI3xx firmware on Linux using dfu-util

echo ">>> BHI3xx Linux Firmware Flasher"

# Check if dfu-util is installed
if ! command -v dfu-util &> /dev/null; then
    echo "Error: dfu-util is not installed. Please run setup_linux.sh first."
    exit 1
fi

PROJECT_ROOT="$(dirname "$(dirname "$(readlink -f "$0")")")"
FIRMWARE_DIR="$PROJECT_ROOT/BHy2CLI/release/firmware/app30"
# Default to APP3.0 firmware, can be made configurable later
BOOTLOADER="$FIRMWARE_DIR/bootloader_update/usb_ble_dfu_bootloader.pkg"
MTP_FW="$FIRMWARE_DIR/mtp_fw_update/usb_mtp.pkg"

echo "FIRMWARE_DIR: $FIRMWARE_DIR"

if [ ! -f "$BOOTLOADER" ]; then
    echo "Error: Firmware file not found: $BOOTLOADER"
    exit 1
fi

echo ">>> Please ensure your board is in DFU Mode."
echo "    (Hold the BOOT button, press RESET, release BOOT)"
echo ">>> Checking for DFU devices..."

dfu-util -l

read -p ">>> Ready to flash? [y/N] " confirm
if [[ $confirm != [yY] && $confirm != [yY][eE][sS] ]]; then
    echo "Aborted."
    exit 0
fi

echo ">>> Flashing Bootloader..."
# Note: These parameters (alt, interface) depend on the specific BHI3xx bootloader configuration.
# Standard Bosch DFU usually uses alt 0 for RAM, but persisting to flash might require alt 1 or 2.
# Assuming standard Bosch procedure:
# Download to RAM (if needed) or Flash.
# This command is a placeholder for the exact DFU parameters, which might need adjustment based on specific board docs.
# For now, using a generic command pattern:
# dfu-util -a 0 -D "$BOOTLOADER"

# Since we don't have the board connected to check 'alt' settings, we print instructions.
echo "!!! TODO: Verify correct 'alt' setting for your specific board using 'dfu-util -l' output."
echo "!!! Typical usage: dfu-util -d 108c:ab3d -a 0 -D <firmware_file>"
echo "!!! Simulating flashing command:"
echo "sudo dfu-util -d 108c:ab3d -a 0 -D \"$BOOTLOADER\" -R"

# Uncomment to actually run:
# sudo dfu-util -d 108c:ab3d -a 0 -D "$BOOTLOADER" -R

echo ">>> Flashing Complete (Simulation). Please reset your board."
