import Hobot.GPIO as GPIO
import time
import serial
import struct
import smbus2

# -----------------------------
# GPIO Setup
# -----------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

IN1 = 29
IN2 = 31
LEDPin = 37
ServoPin = 32
ENA = 33
ButtonPin = 18
GPIO.setup([IN1, IN2, LEDPin, ENA, ServoPin], GPIO.OUT)
GPIO.output(LEDPin, GPIO.HIGH)
GPIO.setup(ButtonPin, GPIO.IN)

# -----------------------------
# Gyro Sensor Class
# -----------------------------
class GyroSensor:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyS7', baudrate=9600, timeout=0.01)
        self.base_yaw = None
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

    def update(self):
        while self.ser.in_waiting >= 11:
            header = self.ser.read(1)
            if header != b'\x55':
                continue
            packet_type = self.ser.read(1)
            data = self.ser.read(9)
            if len(data) != 9:
                continue

            if packet_type == b'\x53':  # Attitude packet
                self.roll = struct.unpack('<h', data[0:2])[0] / 32768.0 * 180
                self.pitch = struct.unpack('<h', data[2:4])[0] / 32768.0 * 180
                raw_yaw = struct.unpack('<h', data[4:6])[0] / 32768.0 * 180

                if self.base_yaw is None:
                    self.base_yaw = raw_yaw  # set starting orientation as 0°

                # Relative yaw (rotation from start)
                self.yaw = (raw_yaw - self.base_yaw) % 360
                if self.yaw > 180:
                    self.yaw -= 360  # keep range -180 to +180

# -----------------------------
# Colour Sensor Class
# -----------------------------
class ColorSensor:
    def __init__(self, i2c_bus=0, addr=0x29):
        self.bus = smbus2.SMBus(i2c_bus)
        self.addr = addr
        self.COMMAND_BIT = 0x80
        self.CDATA = 0x14

        # Init TCS34725
        self.bus.write_byte_data(self.addr, self.COMMAND_BIT | 0x00, 0x03)
        self.bus.write_byte_data(self.addr, self.COMMAND_BIT | 0x01, 0xD5)
        self.bus.write_byte_data(self.addr, self.COMMAND_BIT | 0x0F, 0x01)
        time.sleep(0.7)

    def read_word(self, reg):
        low = self.bus.read_byte_data(self.addr, self.COMMAND_BIT | reg)
        high = self.bus.read_byte_data(self.addr, self.COMMAND_BIT | (reg+1))
        return (high << 8) | low

    def get_rgb(self):
        c = self.read_word(self.CDATA)
        r = self.read_word(self.CDATA+2)
        g = self.read_word(self.CDATA+4)
        b = self.read_word(self.CDATA+6)

        if c == 0:
            return (0, 0, 0)

        r_std = int((r / c) * 255)
        g_std = int((g / c) * 255)
        b_std = int((b / c) * 255)

        return (max(0, min(255, r_std)),
                max(0, min(255, g_std)),
                max(0, min(255, b_std)))

# -----------------------------
# Main Loop Template
# -----------------------------
gyro = GyroSensor()
color = ColorSensor()

rotation_array = None
current_index = 0
repeat_count = 0
max_repeats = 3

print("Waiting for button press to start...")
while GPIO.input(ButtonPin) == GPIO.LOW:
    time.sleep(0.1)

print("Button pressed, starting sequence...")

while True:
    gyro.update()
    rgb = color.get_rgb()

    # Detect first colour trigger
    if rotation_array is None:
        r, g, b = rgb
        if r > 200 and g > 100 and b < 100:  # orange-ish
            rotation_array = [0, 90, 180, -90]  # clockwise
            print("\nClockwise rotation sequence selected")
        elif b > 200 and r < 100 and g < 150:  # blue-ish
            rotation_array = [0, -90, -180, 90]  # counterclockwise
            print("\nCounterclockwise rotation sequence selected")

    if rotation_array is not None and repeat_count < max_repeats:
        target_angle = rotation_array[current_index]
        error = target_angle - gyro.yaw

        # For now, just show calculations (no movement)
        print(f"Target={target_angle}° | Rotation={gyro.yaw:.2f}° | Error={error:.2f}° | RGB={rgb} | Repeat={repeat_count}", end="\r", flush=True)

        # Check if we've reached target (within tolerance)
        if abs(error) < 5:
            current_index += 1
            if current_index >= len(rotation_array):
                current_index = 0
                repeat_count += 1
            time.sleep(0.5)

    time.sleep(0.05)
