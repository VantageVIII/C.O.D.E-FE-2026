import Hobot.GPIO as GPIO
import time
import smbus2
import threading

# -----------------------------
# GPIO Setup
# -----------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

IN1 = 29
IN2 = 31
ENA = 33
GPIO.setup([IN1, IN2, ENA], GPIO.OUT)

# -----------------------------
# Colour Sensor
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
        low  = self.bus.read_byte_data(self.addr, self.COMMAND_BIT | reg)
        high = self.bus.read_byte_data(self.addr, self.COMMAND_BIT | (reg+1))
        return (high << 8) | low

    def get_rgb(self):
        try:
            c = self.read_word(self.CDATA)
            r = self.read_word(self.CDATA + 2)
            g = self.read_word(self.CDATA + 4)
            b = self.read_word(self.CDATA + 6)
        except OSError:
            return (0, 0, 0)
        if c == 0:
            return (r, g, b)
        return (int((r/c)*255), int((g/c)*255), int((b/c)*255))

# -----------------------------
# Motor PWM Thread
# -----------------------------
_motor_power = 0
_motor_running = False

def motor_loop(freq=200):
    global _motor_power, _motor_running
    period = 1.0 / freq
    while _motor_running:
        pwr = _motor_power
        if pwr > 0:
            high_time = (pwr/100.0) * period
            low_time  = period - high_time
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            GPIO.output(ENA, GPIO.HIGH)
            time.sleep(high_time)
            GPIO.output(ENA, GPIO.LOW)
            time.sleep(low_time)
        else:
            GPIO.output(ENA, GPIO.LOW)
            time.sleep(period)

def start_motor(power=40):
    global _motor_power, _motor_running
    _motor_power = power
    _motor_running = True
    threading.Thread(target=motor_loop, daemon=True).start()

def stop_motor():
    global _motor_running
    _motor_running = False
    GPIO.output(ENA, GPIO.LOW)

# -----------------------------
# Main Test
# -----------------------------
color = ColorSensor()

print("Driving forward for 0.5s at 40% power and logging RGB values quickly...")
start_motor(40)
start = time.time()
while time.time() - start < 0.5:
    rgb = color.get_rgb()
    print("RGB:", rgb)
    time.sleep(0.01)   # ~100 samples per second
stop_motor()
print("Test complete. Robot stopped.")
