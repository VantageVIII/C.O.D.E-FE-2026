import smbus2

bus = smbus2.SMBus(0)   # or 0 depending on your board
addr = 0x29
COMMAND_BIT = 0x80

try:
    chip_id = bus.read_byte_data(addr, COMMAND_BIT | 0x12)
    print("Chip ID:", hex(chip_id))
except OSError as e:
    print("I2C error:", e)
