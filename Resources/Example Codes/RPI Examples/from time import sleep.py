from time import sleep
import RPi.GPIO as GPIO

# Motor and steering control pins
IN1 = 17
IN2 = 27
ENA = 18
SERVO_PIN = 12

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Initialize PWM for motor and servo control
pwm_servo = GPIO.PWM(SERVO_PIN, 50)  # Servo: 50 Hz
pwm_motor = GPIO.PWM(ENA, 100)       # Motor: 100 Hz

# Start PWM outputs
pwm_servo.start(90)  # Neutral position
pwm_motor.start(0)

def set_angle(angle):
    """Rotate servo to specified angle (0-180)."""
    if not 0 <= angle <= 180:
        print("Angle out of range. Must be 0-180 degrees.")
        return
    duty = 2 + (angle / 16)  # Adjust duty calculation for faster response
    pwm_servo.ChangeDutyCycle(duty)

def reverse(speed):
    """Run motor in reverse direction at specified speed (0-100)."""
    if not 0 <= speed <= 100:
        print("Speed out of range. Must be 0-100%.")
        return
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(speed)

def forward(speed):
    """Run motor in forward direction at specified speed (0-100)."""
    if not 0 <= speed <= 100:
        print("Speed out of range. Must be 0-100%.")
        return
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm_motor.ChangeDutyCycle(speed)

def stop():
    """Stop the motor."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH)

def coast():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

try():
    forward()
    sleep(15)
    