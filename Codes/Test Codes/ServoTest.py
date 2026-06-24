#!/usr/bin/env python3
import Hobot.GPIO as GPIO
import time, signal, sys

SERVO_PIN = 32   # adjust to your wiring

GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN, GPIO.OUT)

running = True
def signal_handler(sig, frame):
    global running
    running = False
signal.signal(signal.SIGINT, signal_handler)

def angle_to_pulse(angle):
    """
    Map 0–300° to 500–2500 µs pulse width.
    """
    return 500 + (angle / 300.0) * 2000  # µs

def set_servo_angle(angle, duration=0.5):
    pulse_us = angle_to_pulse(angle)
    pulse_s = pulse_us / 1_000_000.0
    cycles = int(duration / 0.02)
    for i in range(cycles):
        GPIO.output(SERVO_PIN, GPIO.HIGH)
        time.sleep(pulse_s)
        GPIO.output(SERVO_PIN, GPIO.LOW)
        time.sleep(0.02 - pulse_s)

# Midpoint of 300° servo
CENTER = 150
LEFT   = CENTER - 45   # 105°
RIGHT  = CENTER + 45   # 195°

print("Sweeping ±45° around 150° center. CTRL+C to stop.")
try:
    while running:
        print("Left")
        set_servo_angle(LEFT, duration=1)
        time.sleep(0.5)
        
        print("Center")
        set_servo_angle(CENTER, duration=1)
        time.sleep(0.5)
        
        print("Right")
        set_servo_angle(RIGHT, duration=1)
        time.sleep(0.5)

        print("Center")
        set_servo_angle(CENTER, duration=1)
        time.sleep(0.5)
finally:
    GPIO.cleanup()
