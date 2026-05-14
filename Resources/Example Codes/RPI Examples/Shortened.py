from gpiozero import DistanceSensor
from time import sleep
import RPi.GPIO as GPIO

# Constants
SENSOR_PINS = {
    "rf": (21, 20),  # Right Front
    "rb": (26, 19),  # Right Back
    "f": (16, 13),   # Front
    "lf": (None, None),  # Left Front
    "lb": (None, None),  # Left Back
    "b": (None, None)    # Back
}

MOTOR_PINS = {"IN1": 17, "IN2": 27, "ENA": 18}
SERVO_PIN = 12
PWM_FREQ = {"servo": 50, "motor": 100}
ANGLE_LIMITS = {"min": 0, "max": 180}
DISTANCE_RANGE = {"max": 100, "min": 40}

# Initialize GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(list(MOTOR_PINS.values()) + [SERVO_PIN], GPIO.OUT)

# Initialize PWM
pwm_servo = GPIO.PWM(SERVO_PIN, PWM_FREQ["servo"])
pwm_motor = GPIO.PWM(MOTOR_PINS["ENA"], PWM_FREQ["motor"])
pwm_servo.start(90)  # Neutral position
pwm_motor.start(0)

# Initialize Sensors
sensors = {name: DistanceSensor(echo=echo, trigger=trigger) for name, (echo, trigger) in SENSOR_PINS.items() if echo and trigger}

def set_angle(angle):
    """Rotate servo to specified angle (0-180)."""
    if ANGLE_LIMITS["min"] <= angle <= ANGLE_LIMITS["max"]:
        duty = 2 + (angle / 16)
        pwm_servo.ChangeDutyCycle(duty)
    else:
        print("Angle out of range.")

def motor_control(direction, speed):
    """Control motor direction and speed."""
    GPIO.output(MOTOR_PINS["IN1"], direction == "reverse")
    GPIO.output(MOTOR_PINS["IN2"], direction == "forward")
    pwm_motor.ChangeDutyCycle(speed if 0 <= speed <= 100 else 0)

def stop_motor():
    GPIO.output(list(MOTOR_PINS.values())[:2], GPIO.HIGH)

def perform_sequence(orientation):
    """Perform movement based on sensor data."""
    distances = {name: sensor.distance * 100 for name, sensor in sensors.items()}
    print(f"Front Distance: {distances['f']:.1f} cm")

    if DISTANCE_RANGE["min"] <= distances["f"] <= DISTANCE_RANGE["max"]:
        angle = 90 + ((DISTANCE_RANGE["max"] - distances["f"]) / (DISTANCE_RANGE["max"] - DISTANCE_RANGE["min"]) * 25)
        set_angle(angle if orientation == "clockwise" else 90 - angle)
        motor_control("forward", 20)
    elif distances["f"] < DISTANCE_RANGE["min"]:
        set_angle(107.5 if orientation == "clockwise" else 64.5)
        motor_control("forward", 20)
    else:
        set_angle(90)
        motor_control("forward", 15)

    sleep(0.001)

try:
    colour_count, lap = 0, 0
    set_angle(90)

    while lap < 3:
        orientation = "clockwise" if colour_count % 2 == 0 else "counterclockwise"
        perform_sequence(orientation)
        if colour_count == 4:
            lap += 1
            colour_count = 0
        sleep(0.01)

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    pwm_servo.stop()
    pwm_motor.stop()
    GPIO.cleanup()
