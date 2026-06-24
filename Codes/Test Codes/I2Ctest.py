from smbus2 import SMBus

DEVICE_ADDR = 0x29  # replace with your address

bus = SMBus(0)  # bus number
try:
    bus.read_byte(DEVICE_ADDR)
    print("Device acknowledged!")
except OSError as e:
    print("No ACK:", e)
finally:
    bus.close()
