import RPi.GPIO as GPIO
from RPLCD.i2c import CharLCD
from time import sleep

# Initialize I2C LCD
lcd = CharLCD('PCF8574', 0x27)  # Corrected I2C address format
IN1 = 17
IN2 = 27
ENA = 18
SERVO_PIN = 12

# Setup GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Create separate PWM objects
pwm_servo = GPIO.PWM(SERVO_PIN, 50)  # Servo motor PWM frequency set to 50 Hz
pwm_motor = GPIO.PWM(ENA, 100)       # Motor PWM frequency set to 100 Hz

# Start PWM outputs
pwm_servo.start(0)
pwm_motor.start(0)

lcd.clear()

def set_angle(angle):
    """Rotate servo to specified angle (0-180)."""
    if not 0 <= angle <= 180:
        print("Angle out of range. Must be 0-180 degrees.")
        return
    duty = 2 + (angle / 16)  # Calculate duty cycle for servo
    GPIO.output(SERVO_PIN, True)
    pwm_servo.ChangeDutyCycle(duty)
    sleep(0.2)
    GPIO.output(SERVO_PIN, False)
    pwm_servo.ChangeDutyCycle(0)

def forward(speed):
    """Run motor forward at specified speed (0-100)."""
    if not 0 <= speed <= 100:
        print("Speed out of range. Must be 0-100%.")
        return
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(speed)
    
def reverse(speed):
    """Run motor in reverse at specified speed (0-100)."""
    if not 0 <= speed <= 100:
        print("Speed out of range. Must be 0-100%.")
        return
    GPIO.output(IN1, GPIO.LOW)  # Corrected pin outputs for reverse motion
    GPIO.output(IN2, GPIO.HIGH)
    pwm_motor.ChangeDutyCycle(speed)

def power_pulse():
    """Give a short power pulse to the motor to start movement."""
    pwm_motor.ChangeDutyCycle(100)

def stop():
    """Stop the motor."""
    pwm_motor.ChangeDutyCycle(0)

def perform_sequence():
    """Perform the motor and servo sequence."""

    sleep(0.2)
    set_angle(90)
    reverse(speed=75)
    sleep(0.05)

    print("Left (35)")
    set_angle(67)
    sleep(0.2)
    
    print("Forward")
    reverse(speed=50)
    sleep(0.175)

try:
    for _ in range(12):  # Repeat sequence 8 times
        perform_sequence()

finally:
    # Cleanup resources
    pwm_servo.stop()
    pwm_motor.stop()
    GPIO.cleanup()
