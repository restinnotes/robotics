"""
Simple BLE Scanner to find devices and their services.
Use this to find your phone or ESP32 board's address and UUIDs.
"""

import asyncio
from bleak import BleakScanner

async def main():
    print("Scanning for BLE devices... (5 seconds)")
    devices = await BleakScanner.discover(timeout=5.0)

    print(f"\nFound {len(devices)} devices:")
    for d in devices:
        # Filter out unnamed devices to reduce noise
        if d.name and d.name.strip() != "":
            print(f"  [{d.address}] {d.name}")
            print(f"     RSSI: {d.rssi} dBm")
            if d.metadata.get("uuids"):
                print(f"     Service UUIDs: {d.metadata['uuids']}")
            print("-" * 30)

if __name__ == "__main__":
    asyncio.run(main())
