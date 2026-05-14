from smbus2 import SMBus
import time

BUS = 5
ADDR = 0x29
bus = SMBus(BUS)

# Scan with read probe (like i2cdetect -y -r)
print("Scanning with read probe...")
devices = []
for addr in range(0x03, 0x77):
    try:
        bus.read_byte(addr)
        devices.append(hex(addr))
    except OSError:
        pass
print("Found devices:", devices)

# Minimal init: start ranging
# SYSTEM__MODE_START register is 0x000E
bus.write_byte_data(ADDR, 0x0E, 0x01)  # start ranging

time.sleep(0.05)

# Read distance result registers
print("Reading distances...")
for i in range(10):
    try:
        high = bus.read_byte_data(ADDR, 0x96)
        low = bus.read_byte_data(ADDR, 0x97)
        dist = (high << 8) | low
        print(f"Reading {i+1}: {dist} mm")
    except Exception as e:
        print("Error:", e)
    time.sleep(0.2)
