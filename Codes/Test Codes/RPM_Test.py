#!/usr/bin/env python3
"""
gyro_hold_rpm_test.py

Standalone: includes Movement and GyroSensor, holds heading using gyro while
driving full power for 5 seconds, then asks for measured displacement and
computes acceleration and wheel RPM (65 mm wheels).

Run on the robot:
    sudo python3 gyro_hold_rpm_test.py
"""

import time
import math
import serial
import struct
import smbus2
import threading
import sys

import Hobot.GPIO as GPIO

# -----------------------------
# GPIO Setup and pins
# -----------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

IN1 = 29
IN2 = 31
LEDPin = 37
ServoPin = 32
ENA = 33
ButtonPin = 18

GPIO.setup([IN1, IN2, LEDPin, ENA, ServoPin], GPIO.OUT)
GPIO.output(LEDPin, GPIO.HIGH)
GPIO.setup(ButtonPin, GPIO.IN)

# -----------------------------
# Movement Class (complete)
# -----------------------------
class Movement:
    current_angle         = 0
    offset                = 0
    neutral_ms            = 1.4
    pulse_ms              = 1.4

    _servo_thread_running = False
    _motor_thread_running = False
    _motor_power          = 0

    @staticmethod
    def set_steering_angle(wheel_angle, max_angle=40, full_range=False):
        corrected = wheel_angle + Movement.offset
        Movement.current_angle = max(-max_angle, min(max_angle, corrected))

        if full_range:
            Movement.pulse_ms = 1.5 + (Movement.current_angle / max_angle) * 0.5
            Movement.pulse_ms = max(SERVO_TURN_MIN_MS, min(SERVO_TURN_MAX_MS, Movement.pulse_ms))
        else:
            Movement.pulse_ms = Movement.neutral_ms + (Movement.current_angle / max_angle) * 0.5
            Movement.pulse_ms = max(1.0, min(2.0, Movement.pulse_ms))

    @staticmethod
    def _servo_loop():
        while Movement._servo_thread_running:
            high_time  = Movement.pulse_ms / 1000.0
            frame_time = 0.02
            t0 = time.time()
            GPIO.output(ServoPin, GPIO.HIGH)
            time.sleep(high_time)
            GPIO.output(ServoPin, GPIO.LOW)
            elapsed = time.time() - t0
            remainder = frame_time - elapsed
            if remainder > 0:
                time.sleep(remainder)

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
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        Movement._motor_power = max(0, min(100, int(power)))

    @staticmethod
    def _motor_loop():
        freq   = 200
        period = 1.0 / freq
        while Movement._motor_thread_running:
            pwr = Movement._motor_power
            if pwr > 0:
                high_time = (pwr / 100.0) * period
                low_time  = period - high_time
                GPIO.output(ENA, GPIO.HIGH)
                time.sleep(high_time)
                GPIO.output(ENA, GPIO.LOW)
                if low_time > 0:
                    time.sleep(low_time)
            else:
                GPIO.output(ENA, GPIO.LOW)
                time.sleep(period)

    @staticmethod
    def start_motor():
        if not Movement._motor_thread_running:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            Movement._motor_thread_running = True
            threading.Thread(target=Movement._motor_loop, daemon=True).start()

    @staticmethod
    def stop_motor():
        Movement._motor_power = 0
        Movement._motor_thread_running = False
        time.sleep(0.015)
        GPIO.output(ENA, GPIO.LOW)

    @staticmethod
    def brake():
        Movement._motor_power = 0
        Movement._motor_thread_running = False
        time.sleep(0.015)
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(ENA, GPIO.LOW)

# -----------------------------
# Gyro Sensor Class (WT61PC-style)
# -----------------------------
class GyroSensor:
    def __init__(self, port='/dev/ttyS7', baudrate=9600, timeout=0.01):
        try:
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        except Exception:
            self.ser = None
        self.base_yaw = None
        self.yaw = 0.0

    def update(self):
        if not self.ser:
            return
        while self.ser.in_waiting >= 11:
            header = self.ser.read(1)
            if header != b'\x55':
                continue
            packet_type = self.ser.read(1)
            data = self.ser.read(9)
            if len(data) != 9:
                continue
            if packet_type == b'\x53':
                raw_yaw = struct.unpack('<h', data[4:6])[0] / 32768.0 * 180
                if self.base_yaw is None:
                    self.base_yaw = raw_yaw
                self.yaw = raw_yaw - self.base_yaw
                if self.yaw > 180:
                    self.yaw -= 360
                elif self.yaw < -180:
                    self.yaw += 360

# -----------------------------
# Servo range constants
# -----------------------------
SERVO_TURN_MIN_MS  = 0.9
SERVO_TURN_MAX_MS  = 2.1

# -----------------------------
# Helpers and physics
# -----------------------------
def normalize_angle_error(target, current):
    err = target - current
    if err > 180:
        err -= 360
    elif err < -180:
        err += 360
    return err

def compute_accel_and_rpm(displacement_m, run_time_s, wheel_diameter_mm=65.0):
    if run_time_s <= 0:
        raise ValueError("run_time_s must be > 0")
    d = displacement_m
    t = run_time_s
    a = 2.0 * d / (t * t)
    v_final = 2.0 * d / t
    v_avg = d / t
    radius_m = (wheel_diameter_mm / 1000.0) / 2.0
    circumference = 2.0 * math.pi * radius_m
    rpm_final = (v_final / circumference) * 60.0 if circumference > 0 else 0.0
    rpm_avg = (v_avg / circumference) * 60.0 if circumference > 0 else 0.0
    return {
        "acceleration_m_s2": a,
        "v_final_m_s": v_final,
        "v_avg_m_s": v_avg,
        "rpm_final": rpm_final,
        "rpm_avg": rpm_avg,
        "circumference_m": circumference
    }

# -----------------------------
# Main test routine with gyro heading hold
# -----------------------------
if __name__ == "__main__":
    RUN_TIME = 5.0
    POWER = 100
    WHEEL_DIAMETER_MM = 65.0

    # Steering controller gain (tune this)
    STEERING_KP = 0.8   # degrees of servo per degree of yaw error (signed). Reduce if oscillation.

    # Clamp servo angle limits (degrees)
    SERVO_MAX_ANGLE = 60

    # Centre servo at midpoint and start servo thread
    Movement.set_steering_angle(0)
    Movement.start_servo()

    # Start motor thread (idle until commanded)
    Movement.start_motor()

    try:
        print("\n=== Gyro-hold 5 s Full-Power Drive Test ===")
        print("Ensure the robot has room to roll and nothing in its path.")
        input("Press Enter to begin the 5 s run...")

        # Initialize gyro
        gyro = GyroSensor()
        if gyro.ser:
            # warm up / calibrate base yaw while robot is still
            print("Calibrating gyro (keep robot still)...")
            for _ in range(40):
                gyro.update()
                time.sleep(0.01)
            print(f"Gyro calibrated. base_yaw={gyro.base_yaw:.2f} current_yaw={gyro.yaw:.2f}")
        else:
            print("Warning: gyro serial port not available — heading hold disabled.")

        # Record target heading (use current yaw as zero reference)
        target_heading = 0.0  # we use gyro.base_yaw as zero; Movement.set_steering_angle(0) is midpoint

        # Start driving
        Movement.set_motor_forward(POWER)
        print(f"Driving forward at {POWER}% power for {RUN_TIME:.1f} s with gyro heading hold...")

        start_t = time.time()
        last_t = start_t

        # Loop at ~50 Hz to update steering from gyro
        while time.time() - start_t < RUN_TIME:
            now = time.time()
            dt = now - last_t
            last_t = now

            if gyro.ser:
                gyro.update()
                current_yaw = gyro.yaw
                # compute shortest signed error (deg)
                err = normalize_angle_error(target_heading, current_yaw)
                # proportional steering command (negative sign because servo angle sign may be inverted)
                steer = -STEERING_KP * err
                # clamp
                steer = max(-SERVO_MAX_ANGLE, min(SERVO_MAX_ANGLE, steer))
                Movement.set_steering_angle(steer, max_angle=SERVO_MAX_ANGLE)
                # small sleep to maintain loop rate
                time.sleep(0.02)
            else:
                # no gyro — keep servo centered
                Movement.set_steering_angle(0)
                time.sleep(0.02)

        # Stop motor
        Movement.stop_motor()
        print("Drive complete. Motor stopped.")

        # Prompt user for measured displacement
        while True:
            try:
                disp_input = input("\nEnter measured displacement during the run (meters), e.g. 1.23: ").strip()
                displacement = float(disp_input)
                if displacement < 0:
                    print("Displacement must be non-negative.")
                    continue
                break
            except ValueError:
                print("Invalid number — try again.")

        # Compute and print results
        results = compute_accel_and_rpm(displacement, RUN_TIME, wheel_diameter_mm=WHEEL_DIAMETER_MM)

        print("\n--- Results ---")
        print(f"Run time: {RUN_TIME:.2f} s")
        print(f"Displacement: {displacement:.4f} m")
        print("Assuming constant acceleration from rest:")
        print(f"  Average acceleration: {results['acceleration_m_s2']:.4f} m/s^2")
        print(f"  Average speed: {results['v_avg_m_s']:.4f} m/s")
        print(f"  Final speed: {results['v_final_m_s']:.4f} m/s")
        print(f"  Wheel circumference: {results['circumference_m']:.4f} m")
        print(f"  Estimated RPM (final speed): {results['rpm_final']:.2f} RPM")
        print(f"  Estimated RPM (average speed): {results['rpm_avg']:.2f} RPM")

        # Optional: show a few gyro samples if available
        if gyro and gyro.ser:
            print("\nFinal gyro yaw reading (deg): {:.2f}".format(gyro.yaw))
            print("Note: gyro yaw only measures wheel rotation if IMU rotates with the wheel; here it's used for chassis heading hold.")

    except KeyboardInterrupt:
        print("\nInterrupted — stopping motor.")
        try:
            Movement.stop_motor()
        except Exception:
            pass
    except Exception as e:
        print("Error:", e)
    finally:
        try:
            Movement.stop_motor()
        except Exception:
            pass
        try:
            Movement.stop_servo()
        except Exception:
            pass
        try:
            GPIO.output(LEDPin, GPIO.LOW)
        except Exception:
            pass
        try:
            GPIO.cleanup()
        except Exception:
            pass
        print("\nTest finished. GPIO cleaned up.")
        sys.exit(0)
