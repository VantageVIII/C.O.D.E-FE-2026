import Hobot.GPIO as GPIO
import time
import serial
import struct
import smbus2
import threading

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
# Movement Class
# -----------------------------
class Movement:
    current_angle = 0
    offset = 0
    _servo_thread_running = False
    pulse_ms = 1.5
    neutral_ms = 1.4

    @staticmethod
    def motor_forward(power=50, freq=200):
        period = 1.0 / freq
        high_time = (power / 100.0) * period
        low_time = period - high_time
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(ENA, GPIO.HIGH)
        time.sleep(high_time)
        GPIO.output(ENA, GPIO.LOW)
        time.sleep(low_time)

    @staticmethod
    def set_steering_angle(wheel_angle):
        corrected = wheel_angle + Movement.offset
        Movement.current_angle = max(-40, min(40, corrected))
        Movement.pulse_ms = Movement.neutral_ms + (Movement.current_angle / 40.0) * 0.5
        Movement.pulse_ms = max(1.0, min(2.0, Movement.pulse_ms))

    @staticmethod
    def _servo_loop():
        while Movement._servo_thread_running:
            high_time = Movement.pulse_ms / 1000.0
            frame = 0.02
            start = time.time()
            GPIO.output(ServoPin, GPIO.HIGH)
            time.sleep(high_time)
            GPIO.output(ServoPin, GPIO.LOW)
            elapsed = time.time() - start
            time.sleep(max(0, frame - elapsed))

    @staticmethod
    def start_servo():
        if not Movement._servo_thread_running:
            Movement._servo_thread_running = True
            threading.Thread(target=Movement._servo_loop, daemon=True).start()

    @staticmethod
    def stop_servo():
        Movement._servo_thread_running = False

    @staticmethod
    def brake():
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(ENA, GPIO.LOW)

# -----------------------------
# Gyro Sensor Class
# -----------------------------
class GyroSensor:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyS7', baudrate=9600, timeout=0.01)
        self.base_yaw = None
        self.yaw = 0.0

    def update(self):
        while self.ser.in_waiting >= 11:
            header = self.ser.read(1)
            if header != b'\x55':
                continue
            packet_type = self.ser.read(1)
            data = self.ser.read(9)
            if len(data) != 9:
                continue
            if packet_type == b'\x53':
                raw_yaw = struct.unpack('<h', data[4:6])[0] / 32768.0 * 180
                if self.base_yaw is None:
                    self.base_yaw = raw_yaw
                self.yaw = (raw_yaw - self.base_yaw) % 360
                if self.yaw > 180:
                    self.yaw -= 360

# -----------------------------
# Colour Sensor Class
# -----------------------------
class ColorSensor:
    def __init__(self, i2c_bus=0, addr=0x29):
        self.bus = smbus2.SMBus(i2c_bus)
        self.addr = addr
        self.COMMAND_BIT = 0x80
        self.CDATA = 0x14
        self.bus.write_byte_data(self.addr, self.COMMAND_BIT | 0x00, 0x03)
        self.bus.write_byte_data(self.addr, self.COMMAND_BIT | 0x01, 0xFF)
        self.bus.write_byte_data(self.addr, self.COMMAND_BIT | 0x0F, 0x03)
        time.sleep(0.7)

    def read_word(self, reg):
        low = self.bus.read_byte_data(self.addr, self.COMMAND_BIT | reg)
        high = self.bus.read_byte_data(self.addr, self.COMMAND_BIT | (reg+1))
        return (high << 8) | low

    def get_rgb(self):
        try:
            c = self.read_word(self.CDATA)
            r = self.read_word(self.CDATA+2)
            g = self.read_word(self.CDATA+4)
            b = self.read_word(self.CDATA+6)
        except OSError:
            return (0, 0, 0)

        if c == 0:
            return (r, g, b)

        r_std = int((r / c) * 255)
        g_std = int((g / c) * 255)
        b_std = int((b / c) * 255)
        return (r_std, g_std, b_std)

# -----------------------------
# Helper: tolerance check
# -----------------------------
def within_tolerance(value, target, tol=0.05):
    return abs(value - target) <= target * tol

# -----------------------------
# Main Loop
# -----------------------------
gyro = GyroSensor()
color = ColorSensor()

rotation_array = [0]
current_index = 0
lap_count = 0
max_laps = 3

print("Waiting for button press to start...")
while GPIO.input(ButtonPin) == GPIO.LOW:
    time.sleep(0.1)

print("Button pressed, starting forward movement...")
Movement.start_servo()

while True:
    gyro.update()
    rgb = color.get_rgb()
    r, g, b = rgb

    # Detect orientation once
    if rotation_array == [0]:
        if (within_tolerance(r, 142) and within_tolerance(g, 87) and within_tolerance(b, 63)):
            rotation_array = [0, 90, 180, 270]  # clockwise
            print("\nClockwise rotation sequence selected")
        elif (within_tolerance(r, 104) and within_tolerance(g, 105) and within_tolerance(b, 117)):
            rotation_array = [0, -90, -180, -270]  # anticlockwise
            print("\nCounterclockwise rotation sequence selected")

    target_angle = rotation_array[current_index]
    error = target_angle - gyro.yaw
    raw_angle = max(-40, min(40, error))

    Movement.set_steering_angle(raw_angle)
    Movement.motor_forward(50)

    print(f"Target={target_angle}° | Rotation={gyro.yaw:.2f}° | Error={error:.2f}° | RGB={rgb} | Lap={lap_count}")

    # Advance when same colour is seen again and close to target
    if abs(error) < 5:
        if (within_tolerance(r, 142) and within_tolerance(g, 87) and within_tolerance(b, 63)) or \
           (within_tolerance(r, 104) and within_tolerance(g, 105) and within_tolerance(b, 117)):
            current_index += 1
            if current_index >= len(rotation_array):
                current_index = 0
                lap_count += 1
                print(f"\nLap {lap_count} complete")

    if lap_count >= max_laps:
        Movement.brake()
        Movement.stop_servo()
        print("\nCourse complete. Car stopped.")
        break

    time.sleep(0.01)
