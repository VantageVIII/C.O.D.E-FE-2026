import time
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor

# === GPIO pin definitions ===

XSHUT_RF = 22
XSHUT_LF = 23
XSHUT_F = 24

sensor_lb = DistanceSensor(echo=20, trigger=21)  # Left back
sensor_rb = DistanceSensor(echo=26, trigger=16)  # Right back

IN1 = 8
IN2 = 7
ENA = 25
SERVO_PIN = 16

laps_completed = 0

# === GPIO Initialization ===
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

for pin in [XSHUT_RF, XSHUT_LF, XSHUT_F]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
time.sleep(0.1)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm_motor = GPIO.PWM(ENA, 100)
pwm_servo = GPIO.PWM(SERVO_PIN, 50)
pwm_motor.start(0)
pwm_servo.start(7.5)

# === VL53L0X Setup ===
i2c = busio.I2C(board.SCL, board.SDA)
vl53_addresses = [0x30, 0x31, 0x32]
vl53_sensors = []

for i, pin in enumerate([XSHUT_RF, XSHUT_LF, XSHUT_F]):
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.1)
    sensor = adafruit_vl53l0x.VL53L0X(i2c)
    sensor.set_address(vl53_addresses[i])
    sensor.start_continuous()
    sensor.measurement_timing_budget = 200000 if i == 2 else 50000
    vl53_sensors.append(sensor)

sensor_rf = vl53_sensors[0]
sensor_lf = vl53_sensors[1]
sensor_f = vl53_sensors[2]

# === Motor and Steering Functions ===

def set_servo_angle(angle):
    if not 0 <= angle <= 180:
        print("Servo angle out of range")
        return
    duty = 2 + (angle / 18)
    pwm_servo.ChangeDutyCycle(duty)

def forward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm_motor.ChangeDutyCycle(speed)

def reverse(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(speed)

def stop():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH)
    pwm_motor.ChangeDutyCycle(0)

# === Sensor Helper Functions ===

def get_filtered_distance(sensor, samples=5):
    readings = [sensor.range for _ in range(samples)]
    time.sleep(0.03 * samples)
    return sum(readings) // len(readings)

# === Movement Logic ===

def main_loop():
    global laps_completed
    try:
        while laps_completed < 3:
            front_dist = get_filtered_distance(sensor_f, samples=6) / 10

            if front_dist < 40:
                print("Front trigger distance met. Activating side and ultrasonic sensors.")
                dist_lf = get_filtered_distance(sensor_lf, samples=3) / 10
                dist_rf = get_filtered_distance(sensor_rf, samples=3) / 10
                tolerance = 5

                if abs(dist_lf - dist_rf) > tolerance:
                    if dist_lf > dist_rf:
                        print(f"Turning left: Left = {dist_lf:.1f}, Right = {dist_rf:.1f}")
                        set_servo_angle(45)
                    else:
                        print(f"Turning right: Left = {dist_lf:.1f}, Right = {dist_rf:.1f}")
                        set_servo_angle(135)
                else:
                    set_servo_angle(90)

                forward(50)

            else:
                print("Entering wall-following mode.")
                dist_lb = sensor_lb.distance * 100
                dist_rb = sensor_rb.distance * 100
                tolerance = 5

                if abs(dist_lb - dist_rb) > tolerance:
                    if dist_lb > dist_rb:
                        print(f"Wall right, steer left: LB = {dist_lb:.1f}, RB = {dist_rb:.1f}")
                        set_servo_angle(45)
                    else:
                        print(f"Wall left, steer right: LB = {dist_lb:.1f}, RB = {dist_rb:.1f}")
                        set_servo_angle(135)
                else:
                    set_servo_angle(90)

                forward(50)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping robot...")

    finally:
        stop()
        pwm_servo.stop()
        pwm_motor.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main_loop()
