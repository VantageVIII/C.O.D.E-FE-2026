import smbus2
import time

bus_num = 0   # /dev/i2c-0
addr = 0x29   # TCS34725 default address
bus = smbus2.SMBus(bus_num)

COMMAND_BIT = 0x80
ENABLE = 0x00
ATIME = 0x01
CONTROL = 0x0F
CDATA = 0x14  # Clear data low byte

# Init sensor
bus.write_byte_data(addr, COMMAND_BIT | ENABLE, 0x03)   # Power on + RGBC enable
bus.write_byte_data(addr, COMMAND_BIT | ATIME, 0xD5)    # Integration time ~101 ms
bus.write_byte_data(addr, COMMAND_BIT | CONTROL, 0x01)  # Gain = 4x
time.sleep(0.7)

def read_word(reg):
    low = bus.read_byte_data(addr, COMMAND_BIT | reg)
    high = bus.read_byte_data(addr, COMMAND_BIT | (reg+1))
    return (high << 8) | low

def get_rgb():
    c = read_word(CDATA)
    r = read_word(CDATA+2)
    g = read_word(CDATA+4)
    b = read_word(CDATA+6)

    if c == 0:  # avoid divide by zero
        return (0, 0, 0)

    # Normalize against clear channel and scale to 0–255
    r_std = int((r / c) * 255)
    g_std = int((g / c) * 255)
    b_std = int((b / c) * 255)

    # Clamp values
    r_std = max(0, min(255, r_std))
    g_std = max(0, min(255, g_std))
    b_std = max(0, min(255, b_std))

    return (r_std, g_std, b_std)

while True:
    rgb = get_rgb()
    print("Standard RGB:", rgb)
    time.sleep(1)
