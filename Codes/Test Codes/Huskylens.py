import Hobot.GPIO as GPIO
import time, threading
from huskylib import HuskyLensLibrary

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

IN1 = 29
IN2 = 31
LEDPin = 37
ServoPin = 32
ENA = 33
ButtonPin = 18

outputPins = [IN1, IN2, LEDPin, ENA, ServoPin]
GPIO.setup(outputPins, GPIO.OUT)
GPIO.output(LEDPin, GPIO.HIGH)
GPIO.setup(ButtonPin, GPIO.IN)

# -----------------------------
# Continuous Bit-Banged Motor PWM
# -----------------------------
class MotorThread(threading.Thread):
    def __init__(self, freq=200, duty=30):
        super().__init__()
        self.freq = freq
        self.duty = duty
        self.running = True

    def run(self):
        while self.running:
            period = 1.0 / self.freq
            high_time = (self.duty / 100.0) * period
            low_time = period - high_time
            GPIO.output(ENA, GPIO.HIGH)
            time.sleep(high_time)
            GPIO.output(ENA, GPIO.LOW)
            time.sleep(low_time)

    def update_duty(self, duty):
        self.duty = max(0, min(100, duty))

    def stop(self):
        self.running = False

# -----------------------------
# Movement Class
# -----------------------------
class Movement:
    motor_thread = None

    @staticmethod
    def start_motor(power, freq=200):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        Movement.motor_thread = MotorThread(freq=freq, duty=power)
        Movement.motor_thread.start()

    @staticmethod
    def update_motor(power):
        if Movement.motor_thread:
            Movement.motor_thread.update_duty(power)

    @staticmethod
    def stop_motor():
        if Movement.motor_thread:
            Movement.motor_thread.stop()
            Movement.motor_thread.join()
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(ENA, GPIO.LOW)

    @staticmethod
    def servo_angle(angle, neutral_offset=0.1):
        neutral_ms = 1.30 + neutral_offset
        left_ms = neutral_ms - 0.65
        right_ms = neutral_ms + 0.65
        pulse_ms = left_ms + ((angle - 35) / (145 - 35)) * (right_ms - left_ms)

        period_ms = 20.0
        high_time = pulse_ms / 1000.0
        low_time = (period_ms / 1000.0) - high_time

        GPIO.output(ServoPin, GPIO.HIGH)
        time.sleep(high_time)
        GPIO.output(ServoPin, GPIO.LOW)
        time.sleep(low_time)

# -----------------------------
# HuskyLens Helpers
# -----------------------------
def steering_angle(arrow, gain_orientation=0.15, gain_position=0.08, tolerance=5):
    dx = arrow.xHead - arrow.xTail
    mid_x = (arrow.xHead + arrow.xTail) / 2
    offset = mid_x - 160
    angle = 90 + gain_orientation * dx + gain_position * offset
    angle = max(35, min(145, angle))
    if 90 - tolerance <= angle <= 90 + tolerance:
        angle = 90
    return angle

hl = HuskyLensLibrary("SERIAL", "/dev/ttyS1", 9600)
print("Knock:", hl.knock())

try:
    print("Waiting for button press to start...")
    GPIO.wait_for_edge(ButtonPin, GPIO.RISING)
    print("Button pressed, Starting line follower.")

    Movement.start_motor(10, freq=200)  # slower continuous motor

    angle_buffer = []
    missing_frames = 0

    while True:
        results = hl.requestAll()
        if results:
            missing_frames = 0
            for r in results:
                if r.type == "ARROW":
                    target_angle = steering_angle(r)
                    angle_buffer.append(target_angle)

                    if len(angle_buffer) == 5:
                        avg_angle = sum(angle_buffer) / len(angle_buffer)
                        angle_buffer.clear()

                        print(f"Average Servo={avg_angle:.1f}°")
                        Movement.servo_angle(avg_angle, neutral_offset=0.1)

                        # Dynamic speed scaling
                        deviation = abs(avg_angle - 90)
                        speed = max(10, 10 - deviation * 0.2)  # slower on big turns
                        Movement.update_motor(speed)
        else:
            missing_frames += 1
            if missing_frames > 100:  # grace period
                print("Line lost, stopping motor")
                Movement.stop_motor()
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    Movement.stop_motor()
    GPIO.cleanup()
    print("GPIO cleanup complete.")
