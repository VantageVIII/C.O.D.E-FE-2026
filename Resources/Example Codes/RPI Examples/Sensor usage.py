from gpiozero import DistanceSensor
from time import sleep
import RPi.GPIO as GPIO

# Initialize the ultrasonic sensors
sensor_rf = DistanceSensor(echo=21, trigger=20)   # Front sensor
sensor_rb = DistanceSensor(echo=26, trigger=19)   # Back sensor
sensor_f = DistanceSensor(echo=16, trigger=13)	  # front facing sensor

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
    duty = 2 + (angle / 16)
    GPIO.output(SERVO_PIN, True)
    pwm_servo.ChangeDutyCycle(duty)
    sleep(0.2)
    GPIO.output(SERVO_PIN, False)
    pwm_servo.ChangeDutyCycle(0)

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
    pwm_motor.ChangeDutyCycle(0)

def perform_sequence():
    """Perform distance-based movement and steering."""
    # Get distances from sensors
    distance2 = sensor_rb.distance * 100  # Back sensor in cm
    distance1 = sensor_rf.distance * 100  # Front sensor in cm
    distance3 = sensor_f.distance * 100  # Front sensor in cm

    print(f"Right_Front: {distance2:.1f} cm, Right_Back: {distance1:.1f} cm, Front: {distance3:.1f}")
        
    if distance3 <= 20:
        print('side sensor initiate')

        if distance1 == distance2:
            set_angle(90)
            forward(15)
            
        elif distance1 > distance2:
            set_angle(60)
            forward(15)
            sleep(0.47)
        else:
            sleep(0.001)
    else:
        if distance2 > 20 and distance1 > 20:
            print("Clear in both directions – turning left and moving forward briefly")
            set_angle(107.5)       # Turn left
            forward(30)         # Move forward at moderate speed
            sleep(0.5)         # For 0.25 seconds
            return

        elif distance1 <= 5:
            print("wall avoidance front")
            set_angle(107.5)
            forward(15)
            
        elif distance2 <= 5:
            print('wall avoidance back')
            set_angle(60)
            forward(15)
        else:
            forward(15)


    sleep(0.0001)  # Allow some time before next check

try:
    while True:
        perform_sequence()

except KeyboardInterrupt:
    print("Measurement stopped by user.")
    stop()

finally:
    pwm_servo.stop()
    pwm_motor.stop()
    GPIO.cleanup()
