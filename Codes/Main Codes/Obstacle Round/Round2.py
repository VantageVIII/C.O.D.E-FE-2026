#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# WRO 2026 Future Engineers — Obstacle Challenge
# =============================================================================
"""
===============================================================================
TUNING & TROUBLESHOOTING GUIDE
===============================================================================
1. Servo Twitching / Off-center: Adjust `SERVO_NEUTRAL_MS` (currently 1.4).
   If corners turn too tight one way, adjust `SERVO_OFFSET`.4
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
import math
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
ROBOT_LENGTH_CM   = 30.0

# ── HuskyLens ID Mapping ─────────────────────────────────────────────────────
HL_DOWN_BLUE_ID   = 1           # Blue on the down camera
HL_DOWN_ORANGE_ID = 2           # Orange on the down camera

HL_FWD_GREEN_ID   = [1, 2, 3]   # Green pillar — pass on the LEFT
HL_FWD_RED_ID     = [4, 5, 6]   # Red pillar — pass on the RIGHT
HL_FWD_PURPLE_ID  = 7           # Purple — parking marker

# Legacy mapping (for Round 1 logic compatibility)
HUSKYLENS_BLUE_ID   = HL_DOWN_BLUE_ID
HUSKYLENS_ORANGE_ID = HL_DOWN_ORANGE_ID

# ── Servo Tuning ─────────────────────────────────────────────────────────────
SERVO_NEUTRAL_MS    = 1.4       # Calibrated neutral
SERVO_TURN_MIN_MS   = 0.9
SERVO_TURN_MAX_MS   = 2.1
SERVO_OFFSET        = 0

# ── Motor / Speed Tuning ─────────────────────────────────────────────────────
NORMAL_SPEED        = 15
CORRECTION_SPEED    = 13
TURN_SPEED          = 25
TURN_CRAWL_SPEED    = 25
EXIT_BURST_POWER    = 20
EXIT_BURST_FRAMES   = 10

MOTOR_PWM_FREQ      = 200
SERVO_PWM_FREQ      = 50

# ── Turn Geometry ────────────────────────────────────────────────────────────
TURN_MAX_ANGLE       = 45
TURN_SETTLE_FRAMES   = 5
TURN_EXIT_DELAY      = 0.0

POST_SEQUENCE_REVERSE_RATIO  = 0.55
POST_SEQUENCE_NEUTRAL_ANGLE  = 0
ARRAY_OFFSET         = 10
ARRAY_CORRECTION     = 0

# ── Color Confirmation ───────────────────────────────────────────────────────
COLOR_READ_THRESHOLD  = 10
BLUE_CONFIRM_THRESHOLD = 5

FIRST_SIDE_MIN = 0.05
FIRST_SIDE_MAX = 10.0

# ── Competition Rules ────────────────────────────────────────────────────────
MAX_LAPS             = 3
MIN_LAPS_FOR_PARKING = 3

# ── Sign Avoidance Tuning ────────────────────────────────────────────────────
SIGN_STEER_OFFSET       = -30.0
SIGN_CONFIRM_SAMPLES    = 5
SIGN_CONFIRM_REQUIRED   = 3
SIGN_MIN_BOX_WIDTH      = 20
SIGN_APPROACH_SPEED     = 55
SIGN_CLEAR_FRAMES       = 8

# ── Pillar Pipeline Tuning ───────────────────────────────────────────────────
MAX_COLOR_SLOTS_PER_CAMERA  = 2       # Max green/red detections kept per camera
CAMERA_FOV_DEG              = 60
CAMERA_OUTWARD_ANGLE_DEG    = 15
LENS_SPACING_MM             = 59
LENS_PIXEL_OFFSET           = 20      # positive means right-camera center is +20 px relative to left-camera center
PILLAR_STEER_MAX_ANGLE      = 30.0    # Max proportional steer from pillar offset
PILLAR_STEER_GAIN           = 0.8     # Pixels → degrees proportional gain
STRICT_CAMERA_SIDE_RULE     = True
INNER_ZONE_HALF_WIDTH       = 40
HALFWAY_RATIO               = 0.25    # fraction of half-frame from center
GREEN_HOLD_S                = 0.12    # seconds to hold straight while waiting
GREEN_SHARPEN               = 1.15
RED_SHARPEN                 = 1.10
PRIMARY_CAMERA_HOLD_S       = 0.10
IMMEDIATE_COLLISION_Y       = 160     # Y-coordinate (0-240) bottom edge emergency threshold
MAX_AVOID_ANGLE             = 45       # degrees max gyro offset for avoidance
SAFE_CLEARANCE_M            = 0.37     # 7cm camera overhang + 20cm wheel gap + 10cm safety margin
DETECTION_DEBOUNCE_S        = 0.15    # 150 ms debounce window
HUSKYLENS_CENTER_X          = 160     # Pixel center of the 320px-wide HuskyLens frame
CENTER_EPSILON_NORM         = 0.15    # normalized offset threshold for "near center"
SIZE_EPSILON                = 100     # Area tie-breaker threshold

# ── Parking Tuning ───────────────────────────────────────────────────────────
PARKING_APPROACH_SPEED    = 25
PARKING_REVERSE_SPEED     = 20
PARKING_TURN_IN_SPEED     = 20
PARKING_TURN_IN_ANGLE     = 40
PARKING_STRAIGHTEN_ANGLE  = -35

PARKING_DRIVE_PAST_S      = 0.6
PARKING_REVERSE_ALIGN_S   = 0.5
PARKING_TURN_IN_S         = 0.8
PARKING_STRAIGHTEN_S      = 0.4
PARKING_FINAL_CREEP_S     = 0.3
PARKING_MARKER_CENTER_X   = 160

def get_estimated_velocity(pwm_speed: int) -> float:
    """
    Maps motor PWM percentage to physical m/s.
    Based on calibration: 80 PWM = 0.92 m/s -> Ratio is 0.0115
    """
    return pwm_speed * 0.0115

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
    cam: str = ""

@dataclass
class PillarContext:
    """Result of the pillar detection pipeline for one control loop frame."""
    closest_pillar: Optional[Detection]   # Highest-priority green/red pillar
    all_color_dets: List[Detection]        # All kept green/red across cameras
    magenta_dets: List[Detection]          # All magenta (uncapped)
    primary_camera: Optional[str]          # "left" / "right" / None

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
                        cam=self.name
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

def _det_area(d: Detection) -> int:
    """Bounding-box area used as proximity metric (larger = closer)."""
    return d.width * d.height

def _det_center_dist(d: Detection) -> float:
    """Horizontal distance from HuskyLens frame center (lower = more centered)."""
    return abs(d.x - HUSKYLENS_CENTER_X)

def _det_sort_key(d: Detection):
    """Sort key: area descending (negate), then center-dist ascending."""
    return (-_det_area(d), _det_center_dist(d))

def filter_and_cap(raw_dets: List[Detection]) -> Dict[str, List[Detection]]:
    """
    Per-camera filtering:
    - Separates magenta (ID3) — no cap.
    - Keeps top MAX_COLOR_SLOTS_PER_CAMERA green/red (ID1/ID2) by area,
      filtered by SIGN_MIN_BOX_WIDTH minimum.
    """
    magenta = [d for d in raw_dets if d.ID == HL_FWD_PURPLE_ID]
    color   = [d for d in raw_dets
               if d.ID in (HL_FWD_GREEN_ID, HL_FWD_RED_ID) and d.width >= SIGN_MIN_BOX_WIDTH]
    color.sort(key=_det_sort_key)
    return {
        "color":   color[:MAX_COLOR_SLOTS_PER_CAMERA],
        "magenta": magenta,
    }

def build_pillar_context(
    left_raw: List[Detection],
    right_raw: List[Detection],
    is_manual_turn: bool,
    turn_direction: Optional[str],   # "left" or "right" or None
) -> PillarContext:
    """
    Full pillar detection pipeline for one control-loop frame.

    - Filters and caps each camera independently.
    - During manual turns, only the primary camera's color detections are used.
    - Outside manual turns, both cameras' detections are merged.
    - Returns a PillarContext with the closest pillar and all relevant detections.
    """
    left_f  = filter_and_cap(left_raw)
    right_f = filter_and_cap(right_raw)

    # Magenta is always merged from both cameras regardless of mode
    all_magenta = left_f["magenta"] + right_f["magenta"]

    primary_camera: Optional[str] = None

    def apply_zone_filter(dets: List[Detection], cam: str) -> List[Detection]:
        if not STRICT_CAMERA_SIDE_RULE:
            return dets
        allowed = []
        for d in dets:
            if cam == "left" and d.ID == HL_FWD_RED_ID:
                if d.x < HUSKYLENS_CENTER_X - INNER_ZONE_HALF_WIDTH:
                    continue  # Ignore red in outer-left band
            if cam == "right" and d.ID == HL_FWD_GREEN_ID:
                if d.x > HUSKYLENS_CENTER_X + INNER_ZONE_HALF_WIDTH:
                    continue  # Ignore green in outer-right band
            allowed.append(d)
        return allowed

    left_allowed = apply_zone_filter(left_f["color"], "left")
    right_allowed = apply_zone_filter(right_f["color"], "right")

    # ALWAYS merge both cameras' capped color detections (Primary camera isolation removed)
    color_dets = left_allowed + right_allowed

    # Sort merged list: closer (larger area) first, center-proximity as tiebreak
    color_dets.sort(key=_det_sort_key)

    closest = color_dets[0] if color_dets else None

    return PillarContext(
        closest_pillar=closest,
        all_color_dets=color_dets,
        magenta_dets=all_magenta,
        primary_camera=None,
    )

def compute_pillar_steer(
    pillar: Detection,
    base_heading: float,
    yaw: float,
) -> float:
    """
    Compute steering using a robot-centric horizontal offset.
    pillar.cam must be 'left' or 'right'.
    Positive robot_offset_px => pillar is to robot's right.
    """
    if pillar.cam == "left":
        # left camera is mounted left of robot center: shift its x to robot center
        # robot_offset_px positive = pillar to robot's right
        robot_offset_px = (pillar.x - HUSKYLENS_CENTER_X) + (LENS_PIXEL_OFFSET / 2)
    elif pillar.cam == "right":
        # right camera is mounted right of robot center: shift its x to robot center
        robot_offset_px = (pillar.x - HUSKYLENS_CENTER_X) - (LENS_PIXEL_OFFSET / 2)
    else:
        robot_offset_px = (pillar.x - HUSKYLENS_CENTER_X)

    if pillar.ID == HL_FWD_GREEN_ID:
        # Green (ID1): Steer left (negative angle).
        if robot_offset_px < 0:
            raw = -PILLAR_STEER_GAIN * (abs(robot_offset_px) + 20)  # extra push if already on our left
        else:
            raw = -PILLAR_STEER_GAIN * max(10, abs(robot_offset_px) - 10)
    elif pillar.ID == HL_FWD_RED_ID:
        # Red (ID2): Steer right (positive angle).
        if robot_offset_px > 0:
            raw = PILLAR_STEER_GAIN * (abs(robot_offset_px) + 20)   # extra push if already on our right
        else:
            raw = PILLAR_STEER_GAIN * max(10, abs(robot_offset_px) - 10)
    else:
        return 0.0

    steer = max(-PILLAR_STEER_MAX_ANGLE, min(PILLAR_STEER_MAX_ANGLE, raw))
    
    # Debug logging
    print(f"PILLAR: cam={pillar.cam} id={pillar.ID} x={pillar.x} area={pillar.width*pillar.height} robot_offset_px={robot_offset_px:.1f} steer={steer:.2f}")
    
    return steer

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
    DELAY_BEFORE_TURN = auto()
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

        # Pillar detection debounce state
        self._pillar_debounce_id: Optional[int] = None     # Last consistent pillar ID
        self._pillar_debounce_time: float = 0.0             # When that ID was first seen
        self._straight_until: float = 0.0                   # Go-straight override timestamp
        
        self.green_hold_start_time: float = 0.0
        self.red_hold_start_time: float = 0.0

        # Avoidance lifecycle state
        self._avoidance_target_angle: float = 0.0        # Gyro heading target during avoidance
        self._avoidance_entry_yaw: float = 0.0            # Yaw when avoidance started
        self._pillar_exit_time: float = 0.0               # When pillar exited corresponding camera
        self._pillar_exited: bool = False                  # Whether pillar has exited corresponding camera

        # Distance tracking for hybrid avoidance
        self._last_loop_time: float = 0.0                  # Last time step() ran
        self._clearance_distance_m: float = 0.0            # Physical clearance distance tracked
        self._lateral_offset_m: float = 0.0                # Cross-track error (negative = left, positive = right)

        # Advanced corner tracking
        self._turn_sharpness: float = 1.0                  # Multiplier for TURN_MAX_ANGLE
        self._turn_delay_start: float = 0.0                # Timer for delaying wide turns

    def _get_sharpened_steer(self, p: Detection, yaw: float, is_emergency: bool = False) -> float:
        """Compute sharpened steering: hold-then-turn for green, immediate for red.
        Returns a clamped steering angle in [-MAX_AVOID_ANGLE, +MAX_AVOID_ANGLE]."""
        now = time.time()
        offset_norm = (p.x - HUSKYLENS_CENTER_X) / (320.0 / 2)
        
        if p.ID == HL_FWD_GREEN_ID:
            halfway_x = HUSKYLENS_CENTER_X + (320.0 / 2) * HALFWAY_RATIO
            is_holding = p.x < halfway_x and abs(offset_norm) < CENTER_EPSILON_NORM
            
            if is_holding and not is_emergency:
                if self.green_hold_start_time == 0.0:
                    self.green_hold_start_time = now
                if now - self.green_hold_start_time > GREEN_HOLD_S:
                    # Timer expired! It's time to steer! 
                    # Do NOT reset the timer to 0.0 here, or it will infinitely loop!
                    steer = compute_pillar_steer(p, self.rotation_array[self.current_index], yaw) * GREEN_SHARPEN
                else:
                    steer = 0.0
            else:
                self.green_hold_start_time = 0.0
                steer = compute_pillar_steer(p, self.rotation_array[self.current_index], yaw) * GREEN_SHARPEN
                
            return max(-MAX_AVOID_ANGLE, min(MAX_AVOID_ANGLE, steer))
            
        elif p.ID == HL_FWD_RED_ID:
            # Red is immediate-sharpen without hold
            steer = compute_pillar_steer(p, self.rotation_array[self.current_index], yaw) * RED_SHARPEN
            return max(-MAX_AVOID_ANGLE, min(MAX_AVOID_ANGLE, steer))
            
        return 0.0

    def step(self, yaw: float, down_dets: List[Detection], left_fwd_dets: List[Detection], right_fwd_dets: List[Detection]) -> NavCommand:
        self.frame_count += 1

        # 1. Calculate time passed since last loop (dt)
        current_time = time.time()
        if self._last_loop_time == 0.0:
            self._last_loop_time = current_time
        dt = current_time - self._last_loop_time
        self._last_loop_time = current_time

        # 2. Calculate physical distance traveled in this loop
        current_velocity = get_estimated_velocity(SIGN_APPROACH_SPEED)
        distance_this_loop = current_velocity * dt

        # 3. Track lateral drift (cross-track error)
        # normalize_angle_error returns (target - yaw).
        # We invert it to get (yaw - target) so that pointing left (negative yaw) gives negative drift
        yaw_diff = -normalize_angle_error(self.rotation_array[self.current_index], yaw)
        lateral_velocity = current_velocity * math.sin(math.radians(yaw_diff))
        self._lateral_offset_m += lateral_velocity * dt

        # ── Pillar detection pipeline ──
        is_in_manual_turn = self.state in (NavState.MANUAL_TURN_SETTLE, NavState.MANUAL_TURN_PULSE, NavState.EXIT_BURST)
        turn_dir = self.manual_turn_direction  # "left" / "right" / None
        pillar_ctx = build_pillar_context(left_fwd_dets, right_fwd_dets, is_in_manual_turn, turn_dir)
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
                return self._trigger_smart_corner(yaw, left_fwd_dets, right_fwd_dets)
                
            elif has_down_blue:
                self.rotation_array = build_rotation_array_ccw()
                self.orientation_colour = "blue"
                print("\nCounterclockwise selected. Entering FIRST TURN immediately.")
                return self._trigger_smart_corner(yaw, left_fwd_dets, right_fwd_dets)
                
            return NavCommand(0, NORMAL_SPEED)

        # =========================================================================
        # 2. STRAIGHT DRIVING
        # =========================================================================
        if self.state == NavState.STRAIGHT_DRIVING:
            # ── Pillar detection with debounce ──
            p = pillar_ctx.closest_pillar
            if p is not None:
                now = time.time()
                area = _det_area(p)
                # Debounce: track consistent detections
                if p.ID == self._pillar_debounce_id:
                    elapsed_db = now - self._pillar_debounce_time
                else:
                    # New pillar ID — reset debounce timer
                    self._pillar_debounce_id = p.ID
                    self._pillar_debounce_time = now
                    elapsed_db = 0.0

                # Commit to avoidance if debounce passed OR immediate collision risk
                debounce_ok = elapsed_db >= DETECTION_DEBOUNCE_S
                collision_risk = area > IMMEDIATE_COLLISION_AREA
                if debounce_ok or collision_risk:
                    self._sign_confirm_counter += 1
                    if self._sign_confirm_counter >= SIGN_CONFIRM_REQUIRED:
                        if p.ID == HL_FWD_GREEN_ID:
                            self.state = NavState.SIGN_AVOIDANCE
                            self._sign_type = "green"
                            self._sign_confirm_counter = 0
                            self._pillar_debounce_id = None
                            self._avoidance_entry_yaw = yaw
                            self._pillar_exited = False
                            self._pillar_exit_time = 0.0
                            steer = self._get_sharpened_steer(p, yaw, is_emergency=collision_risk)
                            self._avoidance_target_angle = max(yaw - MAX_AVOID_ANGLE, min(yaw + MAX_AVOID_ANGLE, yaw + steer))
                            print(f"GREEN pillar confirmed — SIGN_AVOIDANCE (keep LEFT, steer={steer:.1f}°, target={self._avoidance_target_angle:.1f}°)")
                            return NavCommand(steer, SIGN_APPROACH_SPEED)
                        elif p.ID == HL_FWD_RED_ID:
                            self.state = NavState.SIGN_AVOIDANCE
                            self._sign_type = "red"
                            self._sign_confirm_counter = 0
                            self._pillar_debounce_id = None
                            self._avoidance_entry_yaw = yaw
                            self._pillar_exited = False
                            self._pillar_exit_time = 0.0
                            steer = self._get_sharpened_steer(p, yaw, is_emergency=collision_risk)
                            self._avoidance_target_angle = max(yaw - MAX_AVOID_ANGLE, min(yaw + MAX_AVOID_ANGLE, yaw + steer))
                            print(f"RED pillar confirmed — SIGN_AVOIDANCE (keep RIGHT, steer={steer:.1f}°, target={self._avoidance_target_angle:.1f}°)")
                            return NavCommand(steer, SIGN_APPROACH_SPEED)
                else:
                    pass  # Still debouncing — don't increment confirm counter yet
            else:
                self._sign_confirm_counter = 0
                self._pillar_debounce_id = None

            if not is_orientation_color and not is_opposite_color:
                self.last_color_detected = None

            # Check for regular corner turn
            if is_orientation_color and self.last_color_detected != "orientation":
                if time.time() >= self.manual_turn_cooldown_until:
                    if self.first_side_start_time is not None and not self.first_side_measured:
                        self.first_side_measured = True
                    return self._trigger_smart_corner(yaw, left_fwd_dets, right_fwd_dets)

            target_angle = self.rotation_array[self.current_index]
            
            # Cross-track error correction (Sharpened by 25% based on feedback)
            # If _lateral_offset_m is negative (drifted left), we want a POSITIVE (right) merge angle to get back.
            merge_angle = -self._lateral_offset_m * 190.0  
            merge_angle = max(-35.0, min(35.0, merge_angle))
            
            raw_angle = max(-60, min(60, -normalize_angle_error(target_angle + merge_angle, yaw)))

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
            p = pillar_ctx.closest_pillar

            # Determine which camera and color we are checking for clearance
            if self._sign_type == "green":
                target_id = HL_FWD_GREEN_ID
                cam_dets = left_fwd_dets
            else:
                target_id = HL_FWD_RED_ID
                cam_dets = right_fwd_dets

            # Check if ANY pillar of that color is still visible in that camera
            pillar_still_visible = len([d for d in cam_dets if d.ID == target_id]) > 0

            if pillar_still_visible:
                self._clearance_distance_m = 0.0

                if p is not None and p.ID == target_id:
                    is_emergency = (p.y + p.height / 2) >= IMMEDIATE_COLLISION_Y
                    # _get_sharpened_steer already returns the perfect raw steering angle (- for left, + for right)
                    steer_out = self._get_sharpened_steer(p, yaw, is_emergency=is_emergency)
                    return NavCommand(steer_out, SIGN_APPROACH_SPEED)
                
                # Fallback if no closest pillar is found but some are visible
                escape_steer = -35.0 if self._sign_type == "green" else 35.0
                return NavCommand(escape_steer, SIGN_APPROACH_SPEED)
            else:
                # The pillar has left the camera! Accumulate physical distance traveled.
                self._clearance_distance_m += distance_this_loop

                if self._clearance_distance_m >= SAFE_CLEARANCE_M:
                    print(f"  Pillar CLEARED. Traveled {self._clearance_distance_m:.2f}m. Gentle recovery.")
                    self.state = NavState.STRAIGHT_DRIVING
                    self._clearance_distance_m = 0.0
                    self._pillar_exited = False
                    self.green_hold_start_time = 0.0

                    gentle_steer = max(-20.0, min(20.0, -normalize_angle_error(self.rotation_array[self.current_index], yaw)))
                    return NavCommand(gentle_steer, NORMAL_SPEED)
                else:
                    # Pillar lost, keep steering diagonally to escape
                    # Green = steer left (-35), Red = steer right (+35)
                    escape_steer = -35.0 if self._sign_type == "green" else 35.0
                    return NavCommand(escape_steer, SIGN_APPROACH_SPEED)

        # =========================================================================
        # 3.5. DELAY BEFORE TURN
        # =========================================================================
        elif self.state == NavState.DELAY_BEFORE_TURN:
            # Wait ~350ms to drive slightly past the apex before starting the wide turn
            if time.time() - self._turn_delay_start >= 0.35:
                self._enter_manual_turn(yaw)
                return NavCommand(0, TURN_CRAWL_SPEED, full_range=True)
            else:
                # Keep tracking straight down the current lane
                target_angle = self.rotation_array[self.current_index]
                raw_angle = max(-60, min(60, -normalize_angle_error(target_angle, yaw)))
                return NavCommand(raw_angle, NORMAL_SPEED)

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

            # Blend pillar avoidance from primary camera during turn
            p = pillar_ctx.closest_pillar
            if p is not None:
                is_emergency = (p.y + p.height / 2) >= IMMEDIATE_COLLISION_Y
                pillar_blend = self._get_sharpened_steer(p, yaw, is_emergency=is_emergency)
                raw_angle = max(-60, min(60, raw_angle + pillar_blend * 0.5))  # 50% blend

            if self.manual_turn_frames <= TURN_SETTLE_FRAMES:
                return NavCommand(raw_angle, TURN_CRAWL_SPEED, full_range=True)
            return NavCommand(raw_angle, TURN_SPEED, full_range=True)

        # =========================================================================
        # 5. MANUAL TURN PULSE
        # =========================================================================
        if self.state == NavState.MANUAL_TURN_PULSE:
            self.manual_turn_pulse_frames += 1
            raw_angle = self._compute_turn_angle(yaw)

            # Blend pillar avoidance from primary camera during turn
            p = pillar_ctx.closest_pillar
            if p is not None:
                is_emergency = (p.y + p.height / 2) >= IMMEDIATE_COLLISION_Y
                pillar_blend = self._get_sharpened_steer(p, yaw, is_emergency=is_emergency)
                raw_angle = max(-60, min(60, raw_angle + pillar_blend * 0.5))  # 50% blend

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
            has_magenta = len(pillar_ctx.magenta_dets) > 0 or any(d.ID == HL_FWD_PURPLE_ID for d in down_dets)
            if has_magenta:
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
            closest_magenta = pillar_ctx.magenta_dets[0] if pillar_ctx.magenta_dets else None
            cmd = self.parking_controller.step(yaw, closest_magenta)
            if self.parking_controller.state == ParkingState.DONE:
                self.state = NavState.STOPPED
                return NavCommand(0, 0, brake=True)
            return NavCommand(cmd.angle, cmd.speed, direction=cmd.direction)

        return NavCommand(0, 0, brake=True)

    def _trigger_smart_corner(self, yaw: float, left_fwd_dets: List[Detection], right_fwd_dets: List[Detection]) -> NavCommand:
        print("\nCorner detected! Analyzing pillars for turn profile...")
        
        turn_dir = "left" if self.orientation_colour == "blue" else "right"
        sharp_id = HL_FWD_RED_ID if turn_dir == "right" else HL_FWD_GREEN_ID
        wide_id = HL_FWD_GREEN_ID if turn_dir == "right" else HL_FWD_RED_ID
        
        # Only consider pillars that are physically close to the corner (Y > 120 or area > 2000)
        # This prevents reacting to a pillar that is 1 meter down the NEXT straightaway!
        valid_sharp = [d for d in left_fwd_dets + right_fwd_dets if d.ID == sharp_id and (d.y + d.height/2 > 100 or d.width * d.height > 1500)]
        valid_wide = [d for d in left_fwd_dets + right_fwd_dets if d.ID == wide_id and (d.y + d.height/2 > 100 or d.width * d.height > 1500)]
        
        has_sharp = len(valid_sharp) > 0
        has_wide = len(valid_wide) > 0
        
        if has_wide:
            print(f"  -> Wide pillar detected at corner. Delaying {turn_dir} turn.")
            self._turn_sharpness = 0.65  # Gentler arc
            self.state = NavState.DELAY_BEFORE_TURN
            self._turn_delay_start = time.time()
            return NavCommand(0, NORMAL_SPEED) # Keep going straight for a moment
        else:
            if has_sharp:
                print(f"  -> Sharp pillar detected at corner. Tight {turn_dir} turn.")
                self._turn_sharpness = 1.0
            else:
                print(f"  -> No pillar at corner. Normal {turn_dir} turn.")
                self._turn_sharpness = 0.75
                
            self._enter_manual_turn(yaw)
            return NavCommand(0, TURN_CRAWL_SPEED, full_range=True)

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
        ang = (-TURN_MAX_ANGLE if err > 0 else TURN_MAX_ANGLE) * self._turn_sharpness
        return max(-TURN_MAX_ANGLE, min(0, ang)) if self.manual_turn_direction == "left" else max(0, min(TURN_MAX_ANGLE, ang))

    def _advance_rotation_index(self):
        self._lateral_offset_m = 0.0  # Reset cross-track drift accumulation on corners!
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

def test_steer_logic():
    print("--- Running steer logic tests ---")
    
    # Synthetic detection: Left camera green at x=120
    d1 = Detection(ID=HL_FWD_GREEN_ID, x=120, y=120, width=50, height=100)
    d1.cam = "left"
    steer1 = compute_pillar_steer(d1, 0.0, 0.0)
    print(f"Test 1 (Left Green): {steer1:.2f}")
    assert steer1 < 0, f"Expected negative steer for left-green, got {steer1}"
    
    # Synthetic detection: Right camera red at x=200
    d2 = Detection(ID=HL_FWD_RED_ID, x=200, y=120, width=50, height=100)
    d2.cam = "right"
    steer2 = compute_pillar_steer(d2, 0.0, 0.0)
    print(f"Test 2 (Right Red): {steer2:.2f}")
    assert steer2 > 0, f"Expected positive steer for right-red, got {steer2}"
    
    print("Steer tests passed!\n")

if __name__ == "__main__":
    test_steer_logic()
    main()
