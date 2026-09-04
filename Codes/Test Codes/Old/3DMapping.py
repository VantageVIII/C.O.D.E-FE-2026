# rdk_server.py
import serial
import struct
import socket

ser = serial.Serial('/dev/ttyS7', baudrate=9600, timeout=1)

HOST = ''
PORT = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
sock.listen(1)

print("Waiting for laptop connection...")
conn, addr = sock.accept()
print(f"Connected by {addr}")

yaw = pitch = roll = 0
ax = ay = az = 0
gx = gy = gz = 0

while True:
    b = ser.read(1)
    if b == b'\x55':
        header = ser.read(1)

        if header == b'\x51':  # acceleration packet
            data = ser.read(9)
            if len(data) == 9:
                ax = struct.unpack('<h', data[0:2])[0] / 32768.0 * 16
                ay = struct.unpack('<h', data[2:4])[0] / 32768.0 * 16
                az = struct.unpack('<h', data[4:6])[0] / 32768.0 * 16

        elif header == b'\x52':  # gyro packet
            data = ser.read(9)
            if len(data) == 9:
                gx = struct.unpack('<h', data[0:2])[0] / 32768.0 * 2000
                gy = struct.unpack('<h', data[2:4])[0] / 32768.0 * 2000
                gz = struct.unpack('<h', data[4:6])[0] / 32768.0 * 2000

        elif header == b'\x53':  # angle packet
            data = ser.read(9)
            if len(data) == 9:
                roll = struct.unpack('<h', data[0:2])[0] / 32768.0 * 180
                pitch = struct.unpack('<h', data[2:4])[0] / 32768.0 * 180
                yaw = struct.unpack('<h', data[4:6])[0] / 32768.0 * 180

        msg = f"{ax:.3f},{ay:.3f},{az:.3f},{gx:.2f},{gy:.2f},{gz:.2f},{yaw:.2f},{pitch:.2f},{roll:.2f}\n"
        conn.sendall(msg.encode('utf-8'))
