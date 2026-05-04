import time
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor
from picamera import PiCamera
import numpy as np
import cv2

#Gpio 

# === GPIO pin definitions ===

# VL53L0X XSHUT pins for assigning unique addresses
XSHUT_RF = 22  # Right front sensor XSHUT (GPIO22)
XSHUT_LF = 23  # Left front sensor XSHUT (GPIO23)
XSHUT_F = 24   # Center front sensor XSHUT (GPIO24)

# Ultrasonic sensors at the back
sensor_lb = DistanceSensor(echo=20, trigger=21)  # Left back
sensor_rb = DistanceSensor(echo=26, trigger=16)  # Right back

# Motor and steering pins
IN1 = 8       # Motor input 1
IN2 = 7       # Motor input 2
ENA = 25      # Motor PWM (speed)
SERVO_PIN = 16  # Rear steering servo

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
    if i == 2:
        sensor.measurement_timing_budget = 200000
    else:
        sensor.measurement_timing_budget = 50000
    vl53_sensors.append(sensor)

sensor_rf = vl53_sensors[0]
sensor_lf = vl53_sensors[1]
sensor_f = vl53_sensors[2]

# === PiCamera Setup ===
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 60
time.sleep(2)  # Camera warm-up time

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

# === Sensor Functions ===

def get_filtered_distance(sensor, samples=5):
    readings = []
    for _ in range(samples):
        readings.append(sensor.range)
        time.sleep(0.03)
    return sum(readings) // len(readings)

def detect_block():
    # Capture 640x480 frame
    frame = np.empty((480, 640, 3), dtype=np.uint8)
    camera.capture(frame, 'bgr', use_video_port=True)

    # Flip vertically if needed (depends on camera mount)
    frame = cv2.flip(frame, 0)

    # Crop center 320x240 ROI
    h, w, _ = frame.shape
    x_start = w//2 - 160
    y_start = h//2 - 120
    roi = frame[y_start:y_start+240, x_start:x_start+320]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Red mask (two ranges)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red, mask_red2)

    # Green mask
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    def largest_contour_center(contours):
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 300:  # threshold to filter noise
                x, y, w, h = cv2.boundingRect(largest)
                return x + w // 2
        return None

    red_x = largest_contour_center(contours_red)
    green_x = largest_contour_center(contours_green)

    class Block:
        def __init__(self, color, x):
            self.color = color
            self.x = x

    if red_x is not None:
        return Block("red", red_x)
    elif green_x is not None:
        return Block("green", green_x)
    else:
        return None

def decide_turn_side(block):
    # If green block, go left around it
    # If red block, go right around it
    if block.color == "green":
        return "left"
    elif block.color == "red":
        return "right"
    else:
        return "left"  # default fallback

def obstacle_in_front():
    dist_f = get_filtered_distance(sensor_f, samples=6) / 10
    dist_lf = get_filtered_distance(sensor_lf, samples=3) / 10
    dist_rf = get_filtered_distance(sensor_rf, samples=3) / 10
    return dist_f < 25 or dist_lf < 20 or dist_rf < 20

def obstacle_at_back_side():
    dist_lb = sensor_lb.distance * 100
    dist_rb = sensor_rb.distance * 100
    if dist_lb < 15:
        return "left"
    elif dist_rb < 15:
        return "right"
    return None

# === Movement Logic ===

def go_around_block(side):
    angle_left = 45
    angle_right = 135
    angle_straight = 90

    set_servo_angle(angle_left if side == "left" else angle_right)

    forward(30)
    timeout = time.time() + 5

    while time.time() < timeout:
        block = detect_block()
        if block is None:
            break

        if obstacle_in_front():
            stop()
            reverse(30)
            time.sleep(0.5)
            stop()
            break

        back_obs = obstacle_at_back_side()
        if back_obs == "left":
            set_servo_angle(angle_right)
        elif back_obs == "right":
            set_servo_angle(angle_left)

        time.sleep(0.1)

    set_servo_angle(angle_straight)

# === Main Loop ===

def main_loop():
    global laps_completed
    try:
        while laps_completed < 3:
            block = detect_block()
            if block:
                print(f"Block detected: color={block.color}, position={block.x}")
                side = decide_turn_side(block)
                print(f"Going around block on the {side} side")
                go_around_block(side)
            else:
                if obstacle_in_front():
                    print("Obstacle ahead! Stopping and reversing.")
                    stop()
                    reverse(30)
                    time.sleep(0.5)
                    stop()
                else:
                    set_servo_angle(90)
                    forward(40)

                back_obs = obstacle_at_back_side()
                if back_obs == "left":
                    print("Back obstacle left side, steering right.")
                    set_servo_angle(135)
                elif back_obs == "right":
                    print("Back obstacle right side, steering left.")
                    set_servo_angle(45)
                else:
                    set_servo_angle(90)

            time.sleep(0.05)  # small delay to reduce CPU load

    except KeyboardInterrupt:
        print("Stopping robot...")

    finally:
        stop()
        pwm_servo.stop()
        pwm_motor.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main_loop()
