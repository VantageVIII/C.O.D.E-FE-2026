#!/usr/bin/env python3
"""
accel_hold_rpm_test_fixed.py

Accelerometer-based ramp detect with safer startup and gyro heading-hold fixes
to avoid an initial 180° spin. Run with sudo.
"""

import time
import math
import serial
import struct
import threading
import sys

import Hobot.GPIO as GPIO

# -----------------------------
# GPIO and pins
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
# Movement class (same as your code)
# -----------------------------
class Movement:
    current_angle = 0
    offset = 0
    neutral_ms = 1.4
    pulse_ms = 1.4

    _servo_thread_running = False
    _motor_thread_running = False
    _motor_power = 0

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
            high_time = Movement.pulse_ms / 1000.0
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
        freq = 200
        period = 1.0 / freq
        while Movement._motor_thread_running:
            pwr = Movement._motor_power
            if pwr > 0:
                high_time = (pwr / 100.0) * period
                low_time = period - high_time
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
# IMU reader (WT61PC-style assumed)
# -----------------------------
class IMU:
    def __init__(self, port='/dev/ttyS7', baudrate=9600, timeout=0.01):
        try:
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        except Exception:
            self.ser = None
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.yaw = 0.0
        self.base_yaw = None

    def read_packets(self):
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
            if packet_type == b'\x51':
                ax_raw = struct.unpack('<h', data[0:2])[0]
                ay_raw = struct.unpack('<h', data[2:4])[0]
                az_raw = struct.unpack('<h', data[4:6])[0]
                scale_g = 16.0 / 32768.0
                self.ax = ax_raw * scale_g
                self.ay = ay_raw * scale_g
                self.az = az_raw * scale_g
            elif packet_type == b'\x53':
                raw_yaw = struct.unpack('<h', data[4:6])[0] / 32768.0 * 180.0
                if self.base_yaw is None:
                    self.base_yaw = raw_yaw
                self.yaw = raw_yaw - self.base_yaw
                if self.yaw > 180:
                    self.yaw -= 360
                elif self.yaw < -180:
                    self.yaw += 360

# -----------------------------
# Constants and helpers
# -----------------------------
SERVO_TURN_MIN_MS = 0.9
SERVO_TURN_MAX_MS = 2.1

def lowpass(prev, new, alpha):
    return alpha * new + (1 - alpha) * prev

def accel_g_to_m_s2(g):
    return g * 9.80665

# Tuning
POWER = 100
RUN_TIME_MAX = 12.0
SAMPLE_INTERVAL = 0.02
WHEEL_DIAMETER_MM = 65.0

ACCEL_LP_ALPHA = 0.25
ACCEL_STABLE_WINDOW = 0.6
ACCEL_CHANGE_THRESHOLD = 0.03
MIN_FORWARD_ACCEL = 0.05

# Steering control
STEERING_KP = 0.6   # reduce if oscillation
SERVO_MAX_ANGLE = 60
STEERING_DEADBAND_DEG = 1.0  # if yaw error within this, don't steer

# Motor ramp
RAMP_STEPS = 5
RAMP_STEP_DELAY = 0.06

# -----------------------------
# Main
# -----------------------------
if __name__ == "__main__":
    # Start servo thread and centre servo BEFORE IMU calibration
    Movement.set_steering_angle(0)
    Movement.start_servo()
    time.sleep(0.25)  # allow servo to physically settle

    # Start motor thread (idle)
    Movement.start_motor()

    imu = IMU()
    if not imu.ser:
        print("IMU serial port not available. This script requires the IMU.")
        sys.exit(1)

    try:
        print("\n=== Accelerometer-based ramp detect (startup fixes) ===")
        print("Keep robot still and straight. Press Enter to calibrate IMU and begin.")
        input("Keep robot perfectly still and press Enter...")

        # Re-centre servo and wait again to ensure straight wheels
        Movement.set_steering_angle(0)
        time.sleep(0.25)

        # Calibrate IMU yaw and accel bias while robot is still
        calib_samples = 80
        gx = gy = gz = 0.0
        for i in range(calib_samples):
            imu.read_packets()
            gx += accel_g_to_m_s2(imu.ax)
            gy += accel_g_to_m_s2(imu.ay)
            gz += accel_g_to_m_s2(imu.az)
            time.sleep(0.01)
        gx /= calib_samples
        gy /= calib_samples
        gz /= calib_samples
        print(f"Calibration done. accel bias (m/s^2): gx={gx:.3f}, gy={gy:.3f}, gz={gz:.3f}")

        # Use current yaw as the target heading (so we hold whatever direction the robot is pointing now)
        # Ensure base_yaw is set and yaw is near zero after calibration
        for _ in range(10):
            imu.read_packets()
            time.sleep(0.005)
        # Reset base_yaw so imu.yaw reads zero from now on
        if imu.ser:
            imu.base_yaw = None
            # read a packet to set base_yaw
            for _ in range(10):
                imu.read_packets()
                time.sleep(0.005)
        print("IMU yaw zeroed to current heading. Servo is centered.")

        # Ramp motor power up gently to avoid sudden torque that might flip the robot
        print("Ramping motor power up...")
        for step in range(1, RAMP_STEPS + 1):
            p = int((POWER * step) / RAMP_STEPS)
            Movement.set_motor_forward(p)
            print(f"  motor power {p}%")
            time.sleep(RAMP_STEP_DELAY)

        print("Driving at target power with heading hold. Monitoring forward accel...")
        start_t = time.time()
        last_t = start_t
        filt_ax = 0.0
        velocity = 0.0
        recent_accels = []
        recent_times = []
        reached_steady = False
        steady_time = None
        steady_velocity = None

        while True:
            now = time.time()
            elapsed = now - start_t
            if elapsed > RUN_TIME_MAX:
                print("Safety timeout reached. Stopping motor.")
                break

            imu.read_packets()
            raw_ax_m = accel_g_to_m_s2(imu.ax) - gx
            filt_ax = lowpass(filt_ax, raw_ax_m, ACCEL_LP_ALPHA)

            dt = now - last_t if last_t is not None else SAMPLE_INTERVAL
            if dt <= 0:
                dt = SAMPLE_INTERVAL
            velocity += filt_ax * dt

            # Heading hold: compute yaw error and apply deadband
            yaw = imu.yaw
            err = -yaw  # target_heading = 0 (we zeroed base_yaw earlier)
            if abs(err) < STEERING_DEADBAND_DEG:
                steer = 0.0
            else:
                # If your robot steers the wrong way, invert the sign here (multiply by -1)
                steer = -STEERING_KP * err
            steer = max(-SERVO_MAX_ANGLE, min(SERVO_MAX_ANGLE, steer))
            Movement.set_steering_angle(steer, max_angle=SERVO_MAX_ANGLE)

            # record recent accel window
            recent_accels.append(filt_ax)
            recent_times.append(elapsed)
            while recent_times and (recent_times[-1] - recent_times[0]) > ACCEL_STABLE_WINDOW:
                recent_times.pop(0)
                recent_accels.pop(0)

            # stability check
            if len(recent_accels) >= 3:
                max_delta = max(abs(recent_accels[i] - recent_accels[i-1]) for i in range(1, len(recent_accels)))
                if abs(recent_accels[-1]) <= MIN_FORWARD_ACCEL and max_delta <= ACCEL_CHANGE_THRESHOLD and elapsed >= 0.4:
                    reached_steady = True
                    steady_time = elapsed
                    steady_velocity = velocity
                    print(f"Detected steady condition at t={steady_time:.3f}s, velocity≈{steady_velocity:.3f} m/s")
                    break

            # debug print (reduced rate)
            if int((elapsed) / 0.2) != int(((elapsed - dt) / 0.2) if dt>0 else -1):
                print(f"[t={elapsed:.2f}] yaw={yaw:.2f} err={err:.2f} steer={steer:.2f} filt_ax={filt_ax:.3f} vel={velocity:.3f}")

            last_t = now
            time.sleep(SAMPLE_INTERVAL)

        Movement.stop_motor()
        print("Motor stopped.")

        if not reached_steady:
            steady_time = elapsed
            steady_velocity = velocity
            print("Steady condition not detected; using last estimate.")

        final_speed = abs(steady_velocity)
        radius_m = (WHEEL_DIAMETER_MM / 1000.0) / 2.0
        circumference = 2.0 * math.pi * radius_m
        rpm_est = (final_speed / circumference) * 60.0 if circumference > 0 else 0.0
        est_accel = final_speed / steady_time if steady_time > 0 else 0.0
        est_distance = 0.5 * est_accel * (steady_time ** 2)

        print("\n--- Results ---")
        print(f"Time to steady: {steady_time:.3f} s")
        print(f"Estimated final speed: {final_speed:.3f} m/s")
        print(f"Estimated final RPM: {rpm_est:.1f} RPM")
        print(f"Estimated average acceleration: {est_accel:.3f} m/s^2")
        print(f"Estimated distance during ramp: {est_distance:.3f} m")

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
        print("\nFinished. GPIO cleaned up.")
        sys.exit(0)
