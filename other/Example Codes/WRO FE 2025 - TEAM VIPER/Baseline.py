from gpiozero import DistanceSensor
from time import sleep
import RPi.GPIO as GPIO

# Initialize the ultrasonic sensors
sensorf = DistanceSensor(echo=21, trigger=20)   # Back sensor (Echo 8, Trigger 25)
sensor1 = DistanceSensor(echo=26, trigger=19)  # Front sensor (Echo 24, Trigger 23)

# Motor and steering control pins
IN1 = 17
IN2 = 27
ENA = 18
SERVO_PIN = 16

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Initialize PWM for motor and servo control
pwm_servo = GPIO.PWM(SERVO_PIN, 50)  # Servo PWM frequency set to 50 Hz
pwm_motor = GPIO.PWM(ENA, 100)       # Motor PWM frequency set to 100 Hz

# Start PWM outputs
pwm_servo.start(90)  # Start with servo at neutral position (straight)
pwm_motor.start(0)

# Function to set servo angle
def set_angle(angle):
    """Rotate servo to specified angle (0-180)."""
    if not 0 <= angle <= 180:
        print("Angle out of range. Must be 0-180 degrees.")
        return
    duty = 2 + (angle / 12)  # Calculate duty cycle for servo
    GPIO.output(SERVO_PIN, True)
    pwm_servo.ChangeDutyCycle(duty)
    sleep(0.1)
    GPIO.output(SERVO_PIN, False)
    pwm_servo.ChangeDutyCycle(0)

# Function to move the motor forward
def forward(speed):
    """Run motor forward at specified speed (0-100)."""
    if not 0 <= speed <= 100:
        print("Speed out of range. Must be 0-100%.")
        return
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(speed)
    
def reverse(speed):
    """Run motor forward at specified speed (0-100)."""
    if not 0 <= speed <= 100:
        print("Speed out of range. Must be 0-100%.")
        return
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm_motor.ChangeDutyCycle(speed)
    
def stop():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH)

# Function to stop the motor
def coast():
    """Stop the motor."""
    pwm_motor.ChangeDutyCycle(0)

# Function to give a 1 millisecond pulse of 90% power
def power_pulse():
    """Give a short power pulse to the motor to start movement."""
    pwm_motor.ChangeDutyCycle(100)



    try:
        while True:
            # Get distance from the back and front sensors
            distance1 = sensor1.distance * 100  # Back sensor distance in cm
            distance2 = sensorf.distance * 100  # Front sensor distance in cm

            # Condition to go straight if the front and back sensor distances are equal
            if distance2 <= 50:
                # If the front and back distances are equal (within tolerance), go straight
                set_angle(135)  # Go straight (neutral position)
                power_pulse()  # Give a power pulse
                forward(25)    # Move forward at a slow speed

            elif distance2 > 60:
                # If the front sensor sees a value greater than 20 cm, turn right
                set_angle(90)  # Turn right at a 45-degree angle
                power_pulse()  # Give a power pulse
                reverse(25)    # Move forward at a slow speed

            else:
                set_angle(90)
                power_pulse()
                reverse(25)

            sleep(0.001)  # Small delay to avoid overloading the system

    except KeyboardInterrupt:
        print("Measurement stopped.")
        stop()

    finally:
        # Cleanup resources
        pwm_servo.stop()
        pwm_motor.stop()
        GPIO.cleanup()


