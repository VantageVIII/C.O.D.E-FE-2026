import smbus2
import time
from vl53l0x import VL53L0X  # pure Python driver

BUS_NUM = 5
MUX_ADDR = 0x70
channels = [0, 1, 4, 5]

bus = smbus2.SMBus(BUS_NUM)

def select_channel(ch):
    bus.write_byte(MUX_ADDR, 1 << ch)
    time.sleep(0.01)

for ch in channels:
    select_channel(ch)
    print(f"Channel {ch}:")
    try:
        sensor = VL53L0X(i2c_bus=BUS_NUM, i2c_address=0x29)
        sensor.start_ranging()
        dist = sensor.get_distance()
        print("Distance:", dist, "mm")
        sensor.stop_ranging()
    except Exception as e:
        print("Init failed:", e)

bus.close()
