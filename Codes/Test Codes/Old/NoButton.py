import sys
import signal
from turtle import speed
import Hobot.GPIO as GPIO
import time
import scipy
import numpy as np

# -----------------------------
# General Setup
# -----------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# -----------------------------
# Pins
# -----------------------------
IN1 = 29
IN2 = 31
LEDPin = 37
ServoPin = 32
ENA = 33
#button pin

# -----------------------------
# GPIO Setup
# -----------------------------
outputPins = [IN1, IN2, LEDPin]
GPIO.setup(outputPins, GPIO.OUT)

# -----------------------------
# PWM Setup
# -----------------------------
ENA_PWM = GPIO.PWM(ENA, 48000)
ServoPWM = GPIO.PWM(ServoPin, 50)
ENA_PWM.start(0)                 
ServoPWM.start(7.5)                 

# -----------------------------
# Confirm channels
# -----------------------------
print("GPIO29: ", GPIO.gpio_function(29))
print("GPIO31: ", GPIO.gpio_function(31))
print("GPIO37: ", GPIO.gpio_function(37))
print("GPIO32: ", GPIO.gpio_function(32))
print("GPIO33: ", GPIO.gpio_function(33))
# -----------------------------
# Movement Class
# -----------------------------
class Movement:
# Forward
    @staticmethod
    def forward(speed):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        ENA_PWM.ChangeDutyCycle(speed)

# Backward
    @staticmethod
    def backward(speed):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        ENA_PWM.ChangeDutyCycle(speed)

# Coast
    @staticmethod
    def coast():
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        ENA_PWM.ChangeDutyCycle(0)

# Brake
    @staticmethod
    def brake():
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.HIGH)
        ENA_PWM.ChangeDutyCycle(0)

# Steering Angle
    @staticmethod
    def servo_angle(angle):
        duty = 2 + (angle / 18)
        ServoPWM.ChangeDutyCycle(duty)

# -----------------------------
# Main Program
# -----------------------------
try:
    while True:
        Movement.servo_angle(90)
        Movement.forward(50)
        time.sleep(2)
        Movement.backward(50)
        Movement.servo_angle(35)
        time.sleep(2)
        Movement.coast()
        time.sleep(1)
        Movement.servo_angle(90)
        Movement.forward(50)
        time.sleep(2)
        Movement.brake()
        time.sleep(1)


except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    # --- Cleanup ---
    ENA_PWM.stop()
    ServoPWM.stop()
    GPIO.cleanup()
    print("GPIO cleanup complete, PWM stopped.")
