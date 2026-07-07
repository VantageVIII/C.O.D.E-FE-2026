import Hobot.GPIO as GPIO
import time
import smbus2

# -----------------------------
# GPIO Setup
# -----------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

IN1 = 29
IN2 = 31
ENA = 33
LEDPin = 37
GPIO.setup([IN1, IN2, ENA, LEDPin], GPIO.OUT)
GPIO.output(LEDPin, GPIO.HIGH)  # keep LED on

# -----------------------------
# Movement Helpers
# -----------------------------
def motor_forward(power=50, freq=200, duration=0.5):
    """Run motor forward for 'duration' seconds at given power"""
    period = 1.0 / freq
    high_time = (power / 100.0) * period
    low_time = period - high_time
    start = time.time()
    while time.time() - start < duration:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(ENA, GPIO.HIGH)
        time.sleep(high_time)
        GPIO.output(ENA, GPIO.LOW)
        time.sleep(low_time)

def brake():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(ENA, GPIO.LOW)

# -----------------------------
# Colour Sensor Class
# -----------------------------
class ColorSensor:
    def __init__(self, i2c_bus=0, addr=0x29):
        self.bus = smbus2.SMBus(i2c_bus)
        self.addr = addr
        self.COMMAND_BIT = 0x80
        self.CDATA = 0x14
        # Power on and configure
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
            print("I2C read failed")
            return (0, 0, 0)

        if c == 0:
            return (r, g, b)

        r_std = int((r / c) * 255)
        g_std = int((g / c) * 255)
        b_std = int((b / c) * 255)
        return (r_std, g_std, b_std)

# -----------------------------
# Main Test
# -----------------------------
color = ColorSensor()

print("Driving forward for 0.5 seconds while printing colour values...")
start = time.time()
while time.time() - start < 0.5:
    # run one PWM cycle
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(ENA, GPIO.HIGH)
    time.sleep(0.0025)  # ~50% duty at 200 Hz
    GPIO.output(ENA, GPIO.LOW)
    time.sleep(0.0025)

    # read and print colour values
    rgb = color.get_rgb()
    print(f"RGB={rgb}")

brake()
print("Stopped.")
