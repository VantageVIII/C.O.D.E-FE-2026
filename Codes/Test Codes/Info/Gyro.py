import serial
import struct

ser = serial.Serial('/dev/ttyS7', baudrate=9600, timeout=1)

print("Reading binary frames from DFRobot 6-Axis IMU...")

while True:
    # Read until we see header 0x55
    b = ser.read(1)
    if b == b'\x55':
        header = ser.read(1)
        if header == b'\x53':  # attitude packet
            data = ser.read(9)  # 9 bytes payload
            if len(data) == 9:
                # Unpack yaw, pitch, roll (2 bytes each, little-endian, scaled)
                roll = struct.unpack('<h', data[0:2])[0] / 32768.0 * 180
                pitch = struct.unpack('<h', data[2:4])[0] / 32768.0 * 180
                yaw = struct.unpack('<h', data[4:6])[0] / 32768.0 * 180
                print(f"Yaw={yaw:.2f}°, Pitch={pitch:.2f}°, Roll={roll:.2f}°")
