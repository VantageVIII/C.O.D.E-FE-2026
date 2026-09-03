#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# WRO 2026 Future Engineers — Obstacle Challenge
# ALL-IN-ONE MONOLITHIC SCRIPT
# =============================================================================
"""
===============================================================================
WRO FUTURE ENGINEERS - GITHUB DESCRIPTION TEMPLATE (5000+ characters required)
===============================================================================
[TEAM NAME] - WRO 2026 Future Engineers Obstacle Challenge

1. Introduction
(Write about your team, your country, and your overall approach to the 2026
Obstacle Challenge. Explain your hardware choices, specifically why you chose
the RDK X5 and HuskyLens sensors.)

2. Hardware Architecture
- Chassis: (Describe your physical car modifications)
- Processing: RDK X5 running Ubuntu/Python 3.10.12.
- Vision: 3x HuskyLens V1 sensors (Left, Right, Down) connected via Serial.
- IMU: WT901-style Serial Gyro on /dev/ttyS7 for heading hold and corner turns.
- Actuation: Bit-bang PWM control for the steering servo and main drive motor,
  controlled via custom Python daemon threads to prevent main-loop blocking.

3. Software Architecture
This script uses a modular state-machine approach flattened into a single file
for deployment simplicity. The architecture consists of:
- Hardware Abstraction: `Movement` (daemon threads for precise 50Hz/200Hz PWM),
  `GyroSensor` (UART parsing), and `HuskyLensWrapper` (safe serial wrappers).
- Perception: Sensor fusion combining Left and Right forward HuskyLens cameras
  to detect Green/Red pillars and Purple parking markers. The Down camera reads
  Blue/Orange lines to determine clockwise/counterclockwise track orientation.
- Navigation (State Machine): The `Navigator` class handles states:
  DETECTING_ORIENTATION -> STRAIGHT_DRIVING -> SIGN_AVOIDANCE ->
  MANUAL_TURN -> POST_SEQUENCE -> PARKING -> STOPPED.
- Parking Controller: A dedicated sub-state machine (`ParkingController`)
  that orchestrates the parallel parking sequence using time-based dead reckoning.

4. Obstacle Management & Avoidance
Our approach to the Obstacle Challenge heavily relies on early detection and
heading offsets. When the forward cameras detect a Green pillar (Keep Left),
the robot adds a +25 degree offset to its target gyro heading. For a Red pillar
(Keep Right), it adds -25 degrees. The state machine locks into `SIGN_AVOIDANCE`
mode until the pillar clears the camera's field of view for 8 consecutive frames,
preventing the robot from prematurely steering back into the obstacle.

5. Lap Counting & Cornering
We determine laps using the rotation array wrapping. The down camera detects the
entry gate color (Blue = Counter-clockwise, Orange = Clockwise). Each time the
floor line is crossed, the `current_index` advances. 4 crossings = 1 lap.
Cornering uses an open-loop "settle and pulse" method: the servo turns to full
lock while crawling to ensure mechanical limits are reached, then the motor
pulses at high speed to swing the rear end around, exiting with a burst of power
to stabilize the heading.

6. Parking Bonus Maneuver
After 3 completed laps (Rulebook §8), the vehicle transitions to a scanning mode
for the Purple parking marker. Once detected, the `ParkingController` executes:
- Approach: Drive toward the marker until it passes under the camera blind spot.
- Drive Past: Overshoot slightly to set up a reverse angle.
- Reverse Align: Back up straight alongside the parking zone.
- Turn-in: Full-lock reverse into the zone.
- Straighten: Counter-steer reverse to parallel the wall.
- Final Creep: Pull forward slightly to center in the zone.

(Expand these sections with your specific tuning, physical build struggles,
and team journey to reach the 5000 character limit required by WRO.)

===============================================================================
TUNING & TROUBLESHOOTING GUIDE
===============================================================================
1. Servo Twitching / Off-center: Adjust `SERVO_NEUTRAL_MS` (currently 1.4).
   If corners turn too tight one way, adjust `SERVO_OFFSET`.
2. Gyro Drift: Ensure robot is perfectly still during boot. Adjust
   `ARRAY_CORRECTION` to add/sub degrees per lap to fight mechanical drift.
3. False Pillar Detections: Increase `SIGN_CONFIRM_REQUIRED` (e.g., to 4 out of 5)
   or increase `SIGN_MIN_BOX_WIDTH` so small background objects are ignored.
4. Hitting Pillars: Increase `SIGN_STEER_OFFSET` (currently 25 deg).
5. Parking Overshoot: Lower `PARKING_TURN_IN_S` and `PARKING_STRAIGHTEN_S`.
6. No HuskyLens Data: Check wiring, ensure baud rate is 9600 in HuskyLens menu,
   and confirm the correct `/dev/ttySx` ports are assigned in the Config.
===============================================================================
"""

from __future__ import annotations
import sys
import time
import struct
import threading
import argparse
from enum import Enum, auto
from dataclasses import dataclass
from typing import Dict, List, Optional, Any

# =============================================================================
# 1. CONFIGURATION CONSTANTS
# =============================================================================
SIMULATE: bool = False

# ── GPIO Pins (RDK X5 BOARD numbering) ───
IN1        = 29
IN2        = 31
SERVO_PIN  = 32
ENA        = 33
BUTTON_PIN = 18

# ── Serial Ports ──────────────────────────────────────────────────────────────
GYRO_PORT       = "/dev/ttyS7"
HL_RIGHT_PORT   = "/dev/ttyS1"  # Right forward HuskyLens
HL_DOWN_PORT    = "/dev/ttyS2"  # Down-facing HuskyLens
HL_LEFT_PORT    = "/dev/ttyS3"  # Left forward HuskyLens
HL_BAUD         = 9600
GYRO_BAUD       = 9600

# ── Physical Dimensions ──────────────────────────────────────────────────────
WHEELSPAN_CM      = 11.5
WHEEL_DIAMETER_MM = 65
ROBOT_LENGTH_CM   = 20.0

# ── HuskyLens ID Mapping ─────────────────────────────────────────────────────
HL_DOWN_BLUE_ID   = 1           # Blue on the down camera
HL_DOWN_ORANGE_ID = 2           # Orange on the down camera

HL_FWD_GREEN_ID   = 1           # Green pillar — pass on the LEFT
HL_FWD_RED_ID     = 2           # Red pillar — pass on the RIGHT
HL_FWD_PURPLE_ID  = 3           # Purple — parking marker

# Legacy mapping (for Round 1 logic compatibility)
HUSKYLENS_BLUE_ID   = HL_DOWN_BLUE_ID
HUSKYLENS_ORANGE_ID = HL_DOWN_ORANGE_ID

# ── Servo Tuning ─────────────────────────────────────────────────────────────
SERVO_NEUTRAL_MS    = 1.4       # Calibrated neutral
SERVO_TURN_MIN_MS   = 0.9
SERVO_TURN_MAX_MS   = 2.1
SERVO_OFFSET        = 0

# ── Motor / Speed Tuning ─────────────────────────────────────────────────────
NORMAL_SPEED        = 65
CORRECTION_SPEED    = 60
TURN_SPEED          = 70
TURN_CRAWL_SPEED    = 65
EXIT_BURST_POWER    = 100
EXIT_BURST_FRAMES   = 10

MOTOR_PWM_FREQ      = 200
SERVO_PWM_FREQ      = 50

# ── Turn Geometry ────────────────────────────────────────────────────────────
TURN_MAX_ANGLE       = 45
TURN_SETTLE_FRAMES   = 6
TURN_ENTRY_DELAY     = 0.2
TURN_EXIT_DELAY      = 0.1

POST_SEQUENCE_REVERSE_RATIO  = 0.55
POST_SEQUENCE_NEUTRAL_ANGLE  = 0
ARRAY_OFFSET         = -10
ARRAY_CORRECTION     = 0

# ── Color Confirmation ───────────────────────────────────────────────────────
COLOR_READ_THRESHOLD  = 10
BLUE_CONFIRM_THRESHOLD = 5

FIRST_SIDE_MIN = 0.05
FIRST_SIDE_MAX = 10.0

# ── Competition Rules ────────────────────────────────────────────────────────
MAX_LAPS             = 3
MIN_LAPS_FOR_PARKING = 1

# ── Sign Avoidance Tuning ────────────────────────────────────────────────────
SIGN_STEER_OFFSET       = 25.0
SIGN_CONFIRM_SAMPLES    = 5
SIGN_CONFIRM_REQUIRED   = 3
SIGN_MIN_BOX_WIDTH      = 20
SIGN_APPROACH_SPEED     = 55
SIGN_CLEAR_FRAMES       = 8

# ── Parking Tuning ───────────────────────────────────────────────────────────
PARKING_APPROACH_SPEED    = 45
PARKING_REVERSE_SPEED     = 50
PARKING_TURN_IN_SPEED     = 50
PARKING_TURN_IN_ANGLE     = 40
PARKING_STRAIGHTEN_ANGLE  = -35

PARKING_DRIVE_PAST_S      = 0.6
PARKING_REVERSE_ALIGN_S   = 0.5
PARKING_TURN_IN_S         = 0.8
PARKING_STRAIGHTEN_S      = 0.4
PARKING_FINAL_CREEP_S     = 0.3
PARKING_MARKER_CENTER_X   = 160


# =============================================================================
# 2. MOCK CLASSES (For --simulate flag)
# =============================================================================
class MockGPIO:
    BOARD = "BOARD"
    OUT = "OUT"
    IN = "IN"
    HIGH = "HIGH"
    LOW = "LOW"
    @staticmethod
    def setmode(mode): pass
    @staticmethod
    def setup(pins, mode): pass
    @staticmethod
    def output(pin, state): pass
    @staticmethod
    def input(pin): return MockGPIO.HIGH  # Auto-press button in sim
    @staticmethod
    def setwarnings(flag): pass
    @staticmethod
    def cleanup(): pass

class MockHuskyLensLibrary:
    def __init__(self, interface, port, baud):
        self.port = port
    def algorthim(self, algo): pass
    def requestAll(self):
        return [] # Simulates empty frames by default

class MockSerial:
    def __init__(self, port, baudrate, timeout):
        self.in_waiting = 0
    def read(self, bytes): return b''
    def reset_input_buffer(self): pass

# =============================================================================
# 3. HARDWARE & GPIO SETUP
# =============================================================================
# Global GPIO reference
GPIO = None

def setup_gpio():
    global GPIO
    if SIMULATE:
        GPIO = MockGPIO
    else:
        try:
            import Hobot.GPIO as hobot_gpio  # type: ignore
            GPIO = hobot_gpio
        except ImportError:
            print("WARNING: Hobot.GPIO not found. Using MockGPIO.")
            GPIO = MockGPIO

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([IN1, IN2, ENA, SERVO_PIN], GPIO.OUT)
    GPIO.setup(BUTTON_PIN, GPIO.IN)

def wait_for_button():
    print("Waiting for button press to start...")
    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
        time.sleep(0.1)
    print("Button pressed — starting up...")


# =============================================================================
# 4. MOVEMENT CLASS (Bit-bang servo/motor)
# =============================================================================
class Movement:
    current_angle         = 0
    offset                = SERVO_OFFSET
    neutral_ms            = SERVO_NEUTRAL_MS
    pulse_ms              = SERVO_NEUTRAL_MS

    _servo_thread_running = False
    _motor_thread_running = False
    _motor_power          = 0

    @staticmethod
    def set_steering_angle(wheel_angle: float, max_angle: float = 40, full_range: bool = False):
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
            frame_time = 1.0 / SERVO_PWM_FREQ
            t0 = time.time()
            GPIO.output(SERVO_PIN, GPIO.HIGH)
            time.sleep(high_time)
            GPIO.output(SERVO_PIN, GPIO.LOW)
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
        GPIO.output(SERVO_PIN, GPIO.LOW)

    @staticmethod
    def set_motor_forward(power: int = 50):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        Movement._motor_power = max(0, min(100, int(power)))

    @staticmethod
    def set_motor_reverse(power: int = 50):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        Movement._motor_power = max(0, min(100, int(power)))

    @staticmethod
    def _motor_loop():
        period = 1.0 / MOTOR_PWM_FREQ
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

# =============================================================================
# 5. SENSORS (Gyro & HuskyLens)
# =============================================================================
class GyroSensor:
    def __init__(self, port="/dev/ttyS7", baudrate=9600):
        if SIMULATE:
            self.ser = MockSerial(port, baudrate, 0.01)
        else:
            try:
                import serial
                self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.01)
            except Exception as e:
                print(f"WARNING: Gyro init failed on {port}: {e}")
                self.ser = None

        self.base_yaw: Optional[float] = None
        self.yaw: float = 0.0

    def update(self) -> None:
        if self.ser is None: return
        try:
            while self.ser.in_waiting >= 11:
                header = self.ser.read(1)
                if header != b'\x55': continue
                packet_type = self.ser.read(1)
                data = self.ser.read(9)
                if len(data) != 9: continue
                if packet_type == b'\x53':
                    raw_yaw = struct.unpack('<h', data[4:6])[0] / 32768.0 * 180
                    # Only set base_yaw if it is None (this allows calibrate() to reset it)
                    if self.base_yaw is None:
                        self.base_yaw = raw_yaw
                        print(f"Gyro 0-point established at raw yaw: {self.base_yaw:.2f}")
                    
                    self.yaw = raw_yaw - self.base_yaw
                    if self.yaw > 180: self.yaw -= 360
                    elif self.yaw < -180: self.yaw += 360
        except Exception as e:
            pass # Suppressed read error logging to avoid terminal spam

    def calibrate(self, n_samples=50, delay=0.01):
        """Force the gyro to set its current heading as 0."""
        print("Calibrating gyro and establishing 0-point...")
        if self.ser is not None and not SIMULATE:
            self.ser.reset_input_buffer()
        
        # Setting base_yaw to None forces the very next update() to capture it!
        self.base_yaw = None 
        
        for _ in range(n_samples):
            self.update()
            time.sleep(delay)
            
        print(f"Gyro calibrated. Base yaw: {self.base_yaw}  Current: {self.yaw:.2f}°")


@dataclass
class Detection:
    ID: int
    x: int = 0
    y: int = 0
    width: int = 0
    height: int = 0

class HuskyLensWrapper:
    def __init__(self, name: str, port: str, baud: int = 9600):
        self.name = name
        self.port = port
        self.is_available = False
        self._hl = None

        if SIMULATE:
            self._hl = MockHuskyLensLibrary("SERIAL", port, baud)
            self.is_available = True
        else:
            try:
                from huskylib import HuskyLensLibrary
                self._hl = HuskyLensLibrary("SERIAL", port, baud)
                self._hl.algorthim("ALGORITHM_COLOR_RECOGNITION")
                self.is_available = True
                print(f"HuskyLens '{name}' initialized on {port}")
            except Exception as e:
                print(f"WARNING: HuskyLens '{name}' init failed on {port}: {e}")

    def request_all(self) -> List[Detection]:
        if not self.is_available or self._hl is None:
            return []
        try:
            results = self._hl.requestAll()
            if not results: return []
            dets = []
            for det in results:
                if hasattr(det, "ID"):
                    dets.append(Detection(
                        ID=det.ID,
                        x=getattr(det, "x", 0),
                        y=getattr(det, "y", 0),
                        width=getattr(det, "width", 0),
                        height=getattr(det, "height", 0),
                    ))
            return dets
        except Exception:
            return []

# =============================================================================
# 6. HELPERS
# =============================================================================
def normalize_angle_error(target: float, current: float) -> float:
    error = target - current
    if error > 180: error -= 360
    elif error < -180: error += 360
    return error

def build_rotation_array_cw(offset=ARRAY_OFFSET) -> list[float]:
    return [0 - offset, -90 - offset, 180 - offset, 90 - offset]

def build_rotation_array_ccw(offset=ARRAY_OFFSET) -> list[float]:
    return [0 + offset, 90 + offset, 180 + offset, -90 + offset]

def fuse_forward_detections(left_dets: List[Detection], right_dets: List[Detection]) -> Dict[str, Optional[Detection]]:
    all_dets = left_dets + right_dets
    def _largest(target_id: int) -> Optional[Detection]:
        matches = [d for d in all_dets if d.ID == target_id and d.width >= SIGN_MIN_BOX_WIDTH]
        if not matches: return None
        return max(matches, key=lambda d: d.width * d.height)

    return {
        "green":  _largest(HL_FWD_GREEN_ID),
        "red":    _largest(HL_FWD_RED_ID),
        "purple": _largest(HL_FWD_PURPLE_ID),
    }

# =============================================================================
# 7. STATE MACHINES (Parking & Navigation)
# =============================================================================
class ParkingState(Enum):
    IDLE = auto()
    APPROACH = auto()
    DRIVE_PAST = auto()
    REVERSE_ALIGN = auto()
    TURN_IN = auto()
    STRAIGHTEN = auto()
    FINAL_CREEP = auto()
    DONE = auto()

class ParkingCommand:
    __slots__ = ("angle", "speed", "direction")
    def __init__(self, angle: float, speed: int, direction: str):
        self.angle = angle
        self.speed = speed
        self.direction = direction

class ParkingController:
    def __init__(self):
        self.state = ParkingState.IDLE
        self._step_start = 0.0

    def start(self):
        self.state = ParkingState.APPROACH
        self._step_start = time.time()
        print("PARKING: sequence started — APPROACH")

    def _elapsed(self): return time.time() - self._step_start
    def _advance(self, next_state):
        self.state = next_state
        self._step_start = time.time()
        print(f"PARKING: → {next_state.name}")

    def step(self, yaw: float, purple_det: Optional[Detection]) -> ParkingCommand:
        if self.state == ParkingState.IDLE:
            return ParkingCommand(0, 0, "forward")

        if self.state == ParkingState.APPROACH:
            if purple_det is not None:
                error = purple_det.x - PARKING_MARKER_CENTER_X
                angle = max(-30, min(30, error * 0.2))
                return ParkingCommand(angle, PARKING_APPROACH_SPEED, "forward")
            else:
                self._advance(ParkingState.DRIVE_PAST)
                return ParkingCommand(0, PARKING_APPROACH_SPEED, "forward")

        if self.state == ParkingState.DRIVE_PAST:
            if self._elapsed() >= PARKING_DRIVE_PAST_S:
                self._advance(ParkingState.REVERSE_ALIGN)
            return ParkingCommand(0, PARKING_APPROACH_SPEED, "forward")

        if self.state == ParkingState.REVERSE_ALIGN:
            if self._elapsed() >= PARKING_REVERSE_ALIGN_S:
                self._advance(ParkingState.TURN_IN)
            return ParkingCommand(POST_SEQUENCE_NEUTRAL_ANGLE, PARKING_REVERSE_SPEED, "reverse")

        if self.state == ParkingState.TURN_IN:
            if self._elapsed() >= PARKING_TURN_IN_S:
                self._advance(ParkingState.STRAIGHTEN)
            return ParkingCommand(PARKING_TURN_IN_ANGLE, PARKING_TURN_IN_SPEED, "reverse")

        if self.state == ParkingState.STRAIGHTEN:
            if self._elapsed() >= PARKING_STRAIGHTEN_S:
                self._advance(ParkingState.FINAL_CREEP)
            return ParkingCommand(PARKING_STRAIGHTEN_ANGLE, PARKING_TURN_IN_SPEED, "reverse")

        if self.state == ParkingState.FINAL_CREEP:
            if self._elapsed() >= PARKING_FINAL_CREEP_S:
                self._advance(ParkingState.DONE)
            return ParkingCommand(0, PARKING_APPROACH_SPEED, "forward")

        return ParkingCommand(0, 0, "stop")


class NavState(Enum):
    DETECTING_ORIENTATION = auto()
    STRAIGHT_DRIVING = auto()
    SIGN_AVOIDANCE = auto()
    MANUAL_TURN_SETTLE = auto()
    MANUAL_TURN_PULSE = auto()
    EXIT_BURST = auto()
    POST_SEQUENCE = auto()
    PARKING = auto()
    STOPPED = auto()

class NavCommand:
    __slots__ = ("angle", "speed", "full_range", "direction", "brake")
    def __init__(self, angle=0, speed=NORMAL_SPEED, full_range=False, direction="forward", brake=False):
        self.angle = angle
        self.speed = speed
        self.full_range = full_range
        self.direction = direction
        self.brake = brake

class Navigator:
    def __init__(self):
        self.state = NavState.DETECTING_ORIENTATION
        self.orientation_colour = None
        self.rotation_array = [0]
        self.current_index = 0
        self.lap_count = 0

        self.manual_turn_target = 0
        self.manual_turn_start_angle = 0
        self.manual_turn_frames = 0
        self.manual_turn_pulse_frames = 0
        self.manual_turn_direction = None
        self.manual_turn_steer_target = 0
        self.manual_turn_cooldown_until = 0.0
        self.last_color_detected = None

        self.exit_burst_frames = 0
        self.orange_frames = 0
        self.blue_frames = 0
        self.line_cooldown = False

        self.first_side_measured = False
        self.first_side_start_time = None

        self._sign_type = None
        self._sign_clear_counter = 0
        self._sign_confirm_counter = 0

        self.parking_controller = ParkingController()
        self.frame_count = 0

    def step(self, yaw: float, down_dets: List[Detection], left_fwd_dets: List[Detection], right_fwd_dets: List[Detection]) -> NavCommand:
        self.frame_count += 1

        fused = fuse_forward_detections(left_fwd_dets, right_fwd_dets)
        has_down_blue = any(d.ID == HL_DOWN_BLUE_ID for d in down_dets)
        has_down_orange = any(d.ID == HL_DOWN_ORANGE_ID for d in down_dets)

        is_orientation_color = False
        is_opposite_color = False
        if self.orientation_colour == "orange":
            is_orientation_color = has_down_orange
            is_opposite_color = has_down_blue
        elif self.orientation_colour == "blue":
            is_orientation_color = has_down_blue
            is_opposite_color = has_down_orange

        # =========================================================================
        # 1. DETECTING ORIENTATION
        # =========================================================================
        if self.state == NavState.DETECTING_ORIENTATION:
            if has_down_orange:
                self.rotation_array = build_rotation_array_cw()
                self.orientation_colour = "orange"
                print("\nClockwise selected. Entering FIRST TURN immediately.")
                
                # TRIGGER TURN IMMEDIATELY
                self._enter_manual_turn(yaw)
                return NavCommand(0, TURN_CRAWL_SPEED, full_range=True)
                
            elif has_down_blue:
                self.rotation_array = build_rotation_array_ccw()
                self.orientation_colour = "blue"
                print("\nCounterclockwise selected. Entering FIRST TURN immediately.")
                
                # TRIGGER TURN IMMEDIATELY
                self._enter_manual_turn(yaw)
                return NavCommand(0, TURN_CRAWL_SPEED, full_range=True)
                
            return NavCommand(0, NORMAL_SPEED)

        # =========================================================================
        # 2. STRAIGHT DRIVING
        # =========================================================================
        if self.state == NavState.STRAIGHT_DRIVING:
            if fused["green"]:
                self._sign_confirm_counter += 1
                if self._sign_confirm_counter >= SIGN_CONFIRM_REQUIRED:
                    self.state = NavState.SIGN_AVOIDANCE
                    self._sign_type = "green"
                    self._sign_confirm_counter = 0
                    print("GREEN pillar confirmed — SIGN_AVOIDANCE (keep LEFT)")
                    return NavCommand(max(-60, min(60, -normalize_angle_error(self.rotation_array[self.current_index] + SIGN_STEER_OFFSET, yaw))), SIGN_APPROACH_SPEED)
            elif fused["red"]:
                self._sign_confirm_counter += 1
                if self._sign_confirm_counter >= SIGN_CONFIRM_REQUIRED:
                    self.state = NavState.SIGN_AVOIDANCE
                    self._sign_type = "red"
                    self._sign_confirm_counter = 0
                    print("RED pillar confirmed — SIGN_AVOIDANCE (keep RIGHT)")
                    return NavCommand(max(-60, min(60, -normalize_angle_error(self.rotation_array[self.current_index] - SIGN_STEER_OFFSET, yaw))), SIGN_APPROACH_SPEED)
            else:
                self._sign_confirm_counter = 0

            if not is_orientation_color and not is_opposite_color:
                self.last_color_detected = None

            # Check for regular corner turn
            if is_orientation_color and self.last_color_detected != "orientation":
                if time.time() >= self.manual_turn_cooldown_until:
                    if self.first_side_start_time is not None and not self.first_side_measured:
                        self.first_side_measured = True
                    print("\nCorner detected! Entering turn.")
                    self._enter_manual_turn(yaw)
                    return NavCommand(0, TURN_CRAWL_SPEED, full_range=True)

            target_angle = self.rotation_array[self.current_index]
            raw_angle = max(-60, min(60, -normalize_angle_error(target_angle, yaw)))

            if is_orientation_color:
                if self.orientation_colour == "orange":
                    self.orange_frames += 1; self.blue_frames = 0
                else:
                    self.blue_frames += 1; self.orange_frames = 0
            else:
                self.orange_frames = 0; self.blue_frames = 0

            if self.orange_frames < COLOR_READ_THRESHOLD and self.blue_frames < BLUE_CONFIRM_THRESHOLD:
                self.line_cooldown = False

            # Increment logic based on line confirm
            if (self.orientation_colour == "orange" and self.orange_frames >= COLOR_READ_THRESHOLD) or \
               (self.orientation_colour == "blue" and self.blue_frames >= BLUE_CONFIRM_THRESHOLD):
                if not self.line_cooldown and self.last_color_detected != self.orientation_colour:
                    self.line_cooldown = True
                    self.last_color_detected = self.orientation_colour
                    self._advance_rotation_index()

            if self.lap_count >= MAX_LAPS:
                self.state = NavState.POST_SEQUENCE
                print("\nSequence complete. Post-sequence mode...")
            
            return NavCommand(raw_angle, NORMAL_SPEED)

        # =========================================================================
        # 3. SIGN AVOIDANCE
        # =========================================================================
        if self.state == NavState.SIGN_AVOIDANCE:
            if (self._sign_type == "green" and fused["green"]) or (self._sign_type == "red" and fused["red"]):
                self._sign_clear_counter = 0
            else:
                self._sign_clear_counter += 1
            
            if self._sign_clear_counter >= SIGN_CLEAR_FRAMES:
                self.state = NavState.STRAIGHT_DRIVING
                return NavCommand(max(-60, min(60, -normalize_angle_error(self.rotation_array[self.current_index], yaw))), NORMAL_SPEED)
            
            target = self.rotation_array[self.current_index] + (-SIGN_STEER_OFFSET if self._sign_type == "red" else SIGN_STEER_OFFSET)
            return NavCommand(max(-60, min(60, -normalize_angle_error(target, yaw))), SIGN_APPROACH_SPEED)

        # =========================================================================
        # 4. MANUAL TURN SETTLE
        # =========================================================================
        if self.state == NavState.MANUAL_TURN_SETTLE:
            if is_opposite_color and self.last_color_detected != "opposite":
                self.state = NavState.MANUAL_TURN_PULSE
                self.manual_turn_pulse_frames = 0
                self.last_color_detected = "opposite"
            
            self.manual_turn_frames += 1
            raw_angle = self._compute_turn_angle(yaw)
            if self.manual_turn_frames <= TURN_SETTLE_FRAMES:
                return NavCommand(raw_angle, TURN_CRAWL_SPEED, full_range=True)
            return NavCommand(raw_angle, TURN_SPEED, full_range=True)

        # =========================================================================
        # 5. MANUAL TURN PULSE
        # =========================================================================
        if self.state == NavState.MANUAL_TURN_PULSE:
            self.manual_turn_pulse_frames += 1
            raw_angle = self._compute_turn_angle(yaw)
            if self.manual_turn_pulse_frames >= 8:
                self.state = NavState.EXIT_BURST
                self.exit_burst_frames = EXIT_BURST_FRAMES
                self.manual_turn_cooldown_until = time.time() + 1.5
                self._advance_rotation_index()
                if not self.first_side_measured and self.first_side_start_time is None and self.lap_count == 0:
                    self.first_side_start_time = time.time()
            return NavCommand(raw_angle, TURN_SPEED, full_range=True)

        # =========================================================================
        # 6. EXIT BURST
        # =========================================================================
        if self.state == NavState.EXIT_BURST:
            self.exit_burst_frames -= 1
            if self.exit_burst_frames <= 0: self.state = NavState.STRAIGHT_DRIVING
            return NavCommand(max(-60, min(60, -normalize_angle_error(self.rotation_array[self.current_index], yaw))), EXIT_BURST_POWER)

        # =========================================================================
        # 7. POST SEQUENCE
        # =========================================================================
        if self.state == NavState.POST_SEQUENCE:
            if fused.get("purple") or any(d.ID == HL_FWD_PURPLE_ID for d in down_dets):
                if self.lap_count >= MIN_LAPS_FOR_PARKING:
                    self.state = NavState.PARKING
                    self.parking_controller.start()
                    return NavCommand(0, 0)
            
            if (self.orientation_colour == "orange" and has_down_orange) or (self.orientation_colour == "blue" and has_down_blue):
                self.state = NavState.STOPPED
                return NavCommand(0, 0, brake=True)
            return NavCommand(max(-60, min(60, -normalize_angle_error(0, yaw))), NORMAL_SPEED)

        # =========================================================================
        # 8. PARKING
        # =========================================================================
        if self.state == NavState.PARKING:
            cmd = self.parking_controller.step(yaw, fused.get("purple"))
            if self.parking_controller.state == ParkingState.DONE:
                self.state = NavState.STOPPED
                return NavCommand(0, 0, brake=True)
            return NavCommand(cmd.angle, cmd.speed, direction=cmd.direction)

        return NavCommand(0, 0, brake=True)

    def _enter_manual_turn(self, yaw):
        self.state = NavState.MANUAL_TURN_SETTLE
        self.manual_turn_frames = 0
        self.manual_turn_pulse_frames = 0
        self.last_color_detected = "orientation"
        
        next_hdg = self.rotation_array[self.current_index + 1] if self.current_index + 1 < len(self.rotation_array) else self.rotation_array[0]
        offset = -50 if (self.orientation_colour == "blue" and next_hdg >= self.rotation_array[self.current_index]) else 50
        if self.orientation_colour == "orange":
            offset = 50 if next_hdg >= self.rotation_array[self.current_index] else -50
        
        self.manual_turn_target = self.rotation_array[self.current_index] + offset
        self.manual_turn_steer_target = self.manual_turn_target if self.current_index == 0 else next_hdg
        
        err = normalize_angle_error(self.manual_turn_steer_target, yaw)
        self.manual_turn_direction = "left" if self.orientation_colour == "blue" else ("left" if err > 0 else "right")

    def _compute_turn_angle(self, yaw):
        err = normalize_angle_error(self.manual_turn_steer_target, yaw)
        ang = -TURN_MAX_ANGLE if err > 0 else TURN_MAX_ANGLE
        return max(-TURN_MAX_ANGLE, min(0, ang)) if self.manual_turn_direction == "left" else max(0, min(TURN_MAX_ANGLE, ang))

    def _advance_rotation_index(self):
        self.current_index += 1
        if self.current_index >= len(self.rotation_array):
            self.current_index = 0
            self.lap_count += 1
            if self.lap_count >= 1:
                self.rotation_array = [h + (ARRAY_CORRECTION if self.orientation_colour == "blue" else -ARRAY_CORRECTION) for h in self.rotation_array]
        self.last_color_detected = None

# =============================================================================
# 8. MAIN ENTRY POINT
# =============================================================================
def main():
    global SIMULATE
    parser = argparse.ArgumentParser(description="WRO Obstacle Challenge Main Loop")
    parser.add_argument("--simulate", action="store_true", help="Run in simulation mode without hardware")
    args = parser.parse_args()
    
    if args.simulate:
        print("Starting in SIMULATION MODE...")
        SIMULATE = True

    setup_gpio()

    print("Initializing Gyro...")
    gyro = GyroSensor(port=GYRO_PORT, baudrate=GYRO_BAUD)
    
    print("Initializing HuskyLens...")
    hl_down = HuskyLensWrapper("down", HL_DOWN_PORT)
    hl_left = HuskyLensWrapper("left", HL_LEFT_PORT)
    hl_right = HuskyLensWrapper("right", HL_RIGHT_PORT)

    # 1. Wait for button press BEFORE doing anything that depends on movement
    wait_for_button()

    # 2. IMPORTANT: Calibrate gyro exactly when the button is pressed.
    # This flushes any weird rotations that happened while setting down the robot
    # and establishes this exact heading as 0 degrees.
    gyro.calibrate()

    # 3. Enable outputs
    Movement.set_steering_angle(0)
    Movement.start_servo()
    Movement.start_motor()

    nav = Navigator()

    try:
        while nav.state != NavState.STOPPED:
            gyro.update()
            down_dets = hl_down.request_all()
            left_dets = hl_left.request_all()
            right_dets = hl_right.request_all()

            cmd = nav.step(gyro.yaw, down_dets, left_dets, right_dets)

            Movement.set_steering_angle(cmd.angle, full_range=cmd.full_range)
            if cmd.brake:
                Movement.brake()
            elif cmd.direction == "reverse":
                Movement.set_motor_reverse(cmd.speed)
            else:
                Movement.set_motor_forward(cmd.speed)

            # Limit loop rate slightly so background threads aren't starved
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nManually stopped.")
    finally:
        print("Cleaning up hardware...")
        Movement.brake()
        Movement.stop_servo()
        Movement.stop_motor()
        GPIO.cleanup()
        print("Run Complete.")

if __name__ == "__main__":
    main()
