#!/usr/bin/env python3
"""
Full runtime script for Route A and Route B starting from side 0 (bearing 0°).
- Starts on previous side at bearing 0°
- Medium manual turn to centre (exit on orange)
- Route A: sharp left into inner lane, then hold -90° until blue
- Route B: S-shaped manual turn (small right nudge then left sweep), then hold -90° until blue
Adjust ROUTE at the top to "A" or "B".
"""

import time
import threading
import serial
import struct
import Hobot.GPIO as GPIO
from huskylib import HuskyLensLibrary

# -----------------------------
# Configuration
# -----------------------------
ROUTE = "B"  # "A" or "B"
HUSKYLENS_SERIAL_PORT = "/dev/ttyS2"
HUSKYLENS_BAUD = 9600
HUSKYLENS_ORANGE_ID = 2
HUSKYLENS_BLUE_ID = 1

# Motion tuning (tweak these on the robot)
NORMAL_SPEED = 65
TURN_SPEED = 70
TURN_CRAWL_SPEED = 50
TURN_MAX_ANGLE = 45
TURN_SETTLE_FRAMES = 6
EXIT_BURST_FRAMES = 8
SERVO_PERIOD_S = 0.02  # 20 ms servo period

# GPIO pins (BOARD numbering)
IN1, IN2, ENA, ServoPin, ButtonPin = 29, 31, 33, 32, 18

# -----------------------------
# HuskyLens init
# -----------------------------
try:
    hl = HuskyLensLibrary("SERIAL", HUSKYLENS_SERIAL_PORT, HUSKYLENS_BAUD)
    hl.algorthim("ALGORITHM_COLOR_RECOGNITION")
    print("HuskyLens initialized")
except Exception as e:
    hl = None
    print("HuskyLens init failed:", e)

# -----------------------------
# GPIO setup
# -----------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup([IN1, IN2, ENA, ServoPin], GPIO.OUT)
GPIO.setup(ButtonPin, GPIO.IN)

# -----------------------------
# Movement class (servo + motor PWM threads)
# -----------------------------
class Movement:
    # pulse_ms controls servo pulse width in ms (1.0 - 2.0 typical)
    pulse_ms = 1.5
    _servo_thread_running = False
    _motor_thread_running = False
    _motor_power = 0

    @staticmethod
    def set_steering_angle(angle, max_angle=TURN_MAX_ANGLE, full_range=True):
        """
        angle: -max_angle .. +max_angle
        maps to pulse width around 1.5ms center +/- 0.5ms
        """
        if full_range:
            Movement.pulse_ms = 1.5 + (angle / float(max_angle)) * 0.5
        else:
            Movement.pulse_ms = 1.5

    @staticmethod
    def _servo_loop():
        while Movement._servo_thread_running:
            high_time = Movement.pulse_ms / 1000.0
            GPIO.output(ServoPin, GPIO.HIGH)
            time.sleep(high_time)
            GPIO.output(ServoPin, GPIO.LOW)
            # ensure non-negative sleep
            rest = SERVO_PERIOD_S - high_time
            if rest > 0:
                time.sleep(rest)

    @staticmethod
    def start_servo():
        if not Movement._servo_thread_running:
            Movement._servo_thread_running = True
            threading.Thread(target=Movement._servo_loop, daemon=True).start()

    @staticmethod
    def stop_servo():
        Movement._servo_thread_running = False
        GPIO.output(ServoPin, GPIO.LOW)

    @staticmethod
    def set_motor_forward(power=50):
        """
        power: 0..100 duty
        sets direction forward and stores duty for PWM thread
        """
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        Movement._motor_power = max(0, min(100, int(power)))

    @staticmethod
    def _motor_loop():
        freq = 200.0
        period = 1.0 / freq
        while Movement._motor_thread_running:
            pwr = Movement._motor_power
            if pwr > 0:
                high = (pwr / 100.0) * period
                low = period - high
                GPIO.output(ENA, GPIO.HIGH)
                time.sleep(high)
                GPIO.output(ENA, GPIO.LOW)
                time.sleep(low)
            else:
                GPIO.output(ENA, GPIO.LOW)
                time.sleep(period)

    @staticmethod
    def start_motor():
        if not Movement._motor_thread_running:
            Movement._motor_thread_running = True
            threading.Thread(target=Movement._motor_loop, daemon=True).start()

    @staticmethod
    def stop_motor():
        Movement._motor_thread_running = False
        Movement._motor_power = 0
        GPIO.output(ENA, GPIO.LOW)

# -----------------------------
# Gyro class (simple yaw reader)
# -----------------------------
class GyroSensor:
    def __init__(self, port='/dev/ttyS7', baud=9600):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
        except Exception:
            self.ser = None
        self.base_yaw = None
        self.yaw = 0.0

    def update(self):
        if not self.ser:
            return
        # read frames if available
        while self.ser.in_waiting >= 11:
            if self.ser.read(1) != b'\x55':
                continue
            typ = self.ser.read(1)
            data = self.ser.read(9)
            if len(data) != 9:
                continue
            if typ == b'\x53':
                raw = struct.unpack('<h', data[4:6])[0] / 32768.0 * 180.0
                if self.base_yaw is None:
                    self.base_yaw = raw
                self.yaw = raw - self.base_yaw
                if self.yaw > 180:
                    self.yaw -= 360
                elif self.yaw < -180:
                    self.yaw += 360

gyro = GyroSensor()

# -----------------------------
# Utility helpers
# -----------------------------
def normalize_angle_error(target, current):
    err = target - current
    if err > 180:
        err -= 360
    elif err < -180:
        err += 360
    return err

def request_huskylens():
    if not hl:
        return []
    try:
        return hl.requestAll()
    except Exception:
        return []

def wait_for_orange(timeout=6.0):
    start = time.time()
    while time.time() - start < timeout:
        dets = request_huskylens()
        if dets and any(d.ID == HUSKYLENS_ORANGE_ID for d in dets):
            return True
        time.sleep(0.03)
    return False

def wait_for_blue(timeout=10.0):
    start = time.time()
    while time.time() - start < timeout:
        dets = request_huskylens()
        if dets and any(d.ID == HUSKYLENS_BLUE_ID for d in dets):
            return True
        time.sleep(0.03)
    return False

# -----------------------------
# Route implementations
# -----------------------------
def medium_entry_until_orange():
    """
    Medium manual turn used to move from previous side into the lane centre.
    Uses a moderate left steering (-30) and the Round1 settle/accelerate/exit pattern.
    """
    Movement.set_steering_angle(-30, max_angle=TURN_MAX_ANGLE, full_range=True)
    # settle crawl
    for _ in range(TURN_SETTLE_FRAMES):
        Movement.set_motor_forward(TURN_CRAWL_SPEED)
        time.sleep(0.02)
    # accelerate into the manual turn
    Movement.set_motor_forward(TURN_SPEED)
    # wait for orange
    wait_for_orange()
    # exit burst to carry momentum out of the manual turn
    for _ in range(EXIT_BURST_FRAMES):
        Movement.set_motor_forward(TURN_SPEED)
        time.sleep(0.02)
    Movement.set_motor_forward(0)
    Movement.set_steering_angle(0)

def route_A_from_side0():
    """
    Route A:
    - medium entry until orange (centres on side 2)
    - sharp left into inner lane (manual turn pattern)
    - straighten and hold -90° until blue detected
    """
    print("[Route A] medium entry -> sharp left -> straight to -90 until blue")
    # medium entry to centre
    medium_entry_until_orange()

    # sharp left into inner lane
    Movement.set_steering_angle(-TURN_MAX_ANGLE, max_angle=TURN_MAX_ANGLE, full_range=True)
    for _ in range(TURN_SETTLE_FRAMES):
        Movement.set_motor_forward(TURN_CRAWL_SPEED)
        time.sleep(0.02)
    Movement.set_motor_forward(TURN_SPEED)
    wait_for_orange()
    for _ in range(EXIT_BURST_FRAMES):
        Movement.set_motor_forward(TURN_SPEED)
        time.sleep(0.02)
    Movement.set_motor_forward(0)
    Movement.set_steering_angle(0)

    # Straight at -90° until next blue
    Movement.set_motor_forward(NORMAL_SPEED)
    while True:
        gyro.update()
        err = normalize_angle_error(-90.0, gyro.yaw)
        # steering correction is negative of yaw error to reduce error
        Movement.set_steering_angle(max(-TURN_MAX_ANGLE, min(TURN_MAX_ANGLE, -err)))
        if wait_for_blue(timeout=0.1):
            print("[Route A] Blue detected — end of straight segment")
            break
    Movement.set_motor_forward(0)

def route_B_from_side0():
    """
    Route B:
    - medium entry until orange (centres on side 2)
    - S-shaped manual turn: small right nudge then left sweep until orange
    - straighten and hold -90° until blue detected
    """
    print("[Route B] medium entry -> S-shaped manual turn -> straight to -90 until blue")
    # medium entry to centre
    medium_entry_until_orange()

    # S-shaped manual turn: small right nudge
    Movement.set_steering_angle(+20, max_angle=TURN_MAX_ANGLE, full_range=True)
    for _ in range(TURN_SETTLE_FRAMES):
        Movement.set_motor_forward(TURN_CRAWL_SPEED)
        time.sleep(0.02)

    # then swing left and accelerate until orange
    Movement.set_steering_angle(-40, max_angle=TURN_MAX_ANGLE, full_range=True)
    Movement.set_motor_forward(TURN_SPEED)
    wait_for_orange()
    for _ in range(EXIT_BURST_FRAMES):
        Movement.set_motor_forward(TURN_SPEED)
        time.sleep(0.02)

    Movement.set_motor_forward(0)
    Movement.set_steering_angle(0)

    # Straight at -90° until next blue
    Movement.set_motor_forward(NORMAL_SPEED)
    while True:
        gyro.update()
        err = normalize_angle_error(-90.0, gyro.yaw)
        Movement.set_steering_angle(max(-TURN_MAX_ANGLE, min(TURN_MAX_ANGLE, -err)))
        if wait_for_blue(timeout=0.1):
            print("[Route B] Blue detected — end of straight segment")
            break
    Movement.set_motor_forward(0)

# -----------------------------
# Main execution
# -----------------------------
def main():
    try:
        print("Waiting for button press to start...")
        while GPIO.input(ButtonPin) == GPIO.LOW:
            time.sleep(0.05)
        print("Button pressed — starting sequence")

        Movement.set_steering_angle(0)
        Movement.start_servo()
        Movement.start_motor()

        print("Calibrating gyro...")
        for _ in range(50):
            gyro.update()
            time.sleep(0.01)
        print("Gyro calibrated, yaw baseline set to 0")

        # Start from side 0 (bearing 0). Choose route.
        if ROUTE == "A":
            route_A_from_side0()
        elif ROUTE == "B":
            route_B_from_side0()
        else:
            print("Unknown ROUTE value:", ROUTE)

        print("Route complete. Stopping motors and servo.")
    finally:
        Movement.stop_motor()
        Movement.stop_servo()
        try:
            GPIO.cleanup()
        except Exception:
            pass
        print("Cleaned up GPIO and exited.")

if __name__ == "__main__":
    main()
