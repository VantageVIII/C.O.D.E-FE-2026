#/usr/bin/python3

import Hobot.GPIO as GPIO
import time
import serial
import struct
import smbus2
import threading

# HuskyLens colour recognition (primary colour sensor replacement)
from huskylib import HuskyLensLibrary
# Primary HuskyLens on serial port 2
try:
    hl = HuskyLensLibrary("SERIAL", "/dev/ttyS2", 9600)
    # RESTORED: Deliberate typo because the HuskyLens library spells it this way!
    hl.algorthim("ALGORITHM_COLOR_RECOGNITION")
    print("HuskyLens initialized on /dev/ttyS2")
except Exception as _e:
    hl = None
    print("HuskyLens init failed or not present:", _e)

# -----------------------------
# GPIO Setup
# -----------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

IN1 = 29
IN2 = 31
ServoPin = 32
ENA = 33
ButtonPin = 18
GPIO.setup([IN1, IN2, ENA, ServoPin], GPIO.OUT)
GPIO.setup(ButtonPin, GPIO.IN)

# -----------------------------
# Movement Class
# -----------------------------
class Movement:
    current_angle         = 0
    offset                = 0      # trim ±2–3 if one direction turns tighter/wider than the other
    neutral_ms            = 1.4
    # Initialise pulse_ms to neutral, NOT 1.5 — prevents the servo from
    # snapping right the moment the servo thread starts.
    pulse_ms              = 1.4

    _servo_thread_running = False
    _motor_thread_running = False
    _motor_power          = 0      # shared duty-cycle value, written by main thread,
                                   # read by motor thread — range 0–100

    # ── Servo bit-bang ────────────────────────────────────────────────────────
    @staticmethod
    def set_steering_angle(wheel_angle, max_angle=40, full_range=False):
        """
        full_range=False  normal driving: centres on calibrated neutral_ms (1.4 ms).
        full_range=True   turn mode:      centres at 1.5 ms so ±max_angle maps
                          across the widest safe physical range (SERVO_TURN_MIN_MS
                          to SERVO_TURN_MAX_MS).
        """
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
        """Bit-bang a 50 Hz servo signal.  Runs in its own daemon thread."""
        while Movement._servo_thread_running:
            high_time  = Movement.pulse_ms / 1000.0
            frame_time = 0.02          # 50 Hz → 20 ms frame
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

    # ── Motor bit-bang ────────────────────────────────────────────────────────
    @staticmethod
    def set_motor_forward(power=50):
        """
        Set direction to forward and update the target duty cycle.
        The motor thread reads _motor_power and handles all PWM timing —
        this call returns immediately so it never blocks the main loop.
        """
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        Movement._motor_power = max(0, min(100, int(power)))

    @staticmethod
    def _motor_loop():
        """
        Bit-bang ~200 Hz PWM on ENA in its own daemon thread.
        Completely decoupled from the main loop so timing is consistent
        regardless of gyro reads, colour reads, or print statements.
        """
        freq   = 200
        period = 1.0 / freq    # 5 ms per PWM cycle
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
                # Power is zero — keep ENA low for the full period
                GPIO.output(ENA, GPIO.LOW)
                time.sleep(period)

    @staticmethod
    def start_motor():
        if not Movement._motor_thread_running:
            # Direction defaults to forward; caller sets it before the first
            # set_motor_forward() if needed.
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            Movement._motor_thread_running = True
            threading.Thread(target=Movement._motor_loop, daemon=True).start()

    @staticmethod
    def stop_motor():
        """Stop the motor thread and ensure ENA is left low."""
        Movement._motor_power = 0
        Movement._motor_thread_running = False
        time.sleep(0.015)          # allow thread to finish its current pulse
        GPIO.output(ENA, GPIO.LOW)

    @staticmethod
    def brake():
        """Hard brake: short IN1/IN2, kill ENA, stop motor thread."""
        Movement._motor_power = 0
        Movement._motor_thread_running = False
        time.sleep(0.015)          # allow thread to finish its current pulse
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(ENA, GPIO.LOW)

# -----------------------------
# Gyro Sensor Class
# -----------------------------
class GyroSensor:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyS7', baudrate=9600, timeout=0.01)
        self.base_yaw = None
        self.yaw = 0.0

    def update(self):
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
# Helpers
# -----------------------------
def within_tolerance(value, target, tol=0.05):
    return abs(value - target) <= target * tol

def normalize_angle_error(target, current):
    error = target - current
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

# ------------------------------------------------------------------
# Tuning constants — only touch these to adjust behaviour
# ------------------------------------------------------------------
NORMAL_SPEED       = 75    # motor duty cycle during straight driving
CORRECTION_SPEED   = 60    # motor duty cycle during heading correction
TURN_SPEED         = 70    # motor duty cycle during the main turn phase
TURN_CRAWL_SPEED   = 65    # motor duty cycle during settle phase (barely rolling)
TURN_MAX_ANGLE     = 45    # servo angle during turns, degrees (±75)
TURN_SETTLE_FRAMES = 6     # frames the servo 5is held at full lock before
                           # accelerating — gives wheels time to reach endpoint
EXIT_BURST_POWER   = 100    # brief high-power pulse after exiting turn mode
EXIT_BURST_FRAMES  = 10    # number of frames the burst lasts (doubled for longer momentum)
POST_SEQUENCE_REVERSE_RATIO = 0.55
POST_SEQUENCE_NEUTRAL_ANGLE = 0
TURN_ENTRY_DELAY = 0.15   # delay after a gate is seen before manual turn starts
TURN_EXIT_DELAY  = 0   # delay after the turn pulse ends before exiting turn mode

SERVO_TURN_MIN_MS  = 0.9
SERVO_TURN_MAX_MS  = 2.1

arrayOffset = -10         # degrees to add/subtract from each heading in rotation_array
arrayCorrection = 0   # degrees to add/subtract from each heading after each lap
# ------------------------------------------------------------------
def confirm_hl_burst(target_id, samples=5, interval_s=0.05, required_positives=5):
    """Confirm a HuskyLens ID across several frames by polling requestAll()."""
    if hl is None:
        return False
    positives = 0
    for _ in range(samples):
        try:
            results = hl.requestAll()
            found = False
            if results:
                for det in results:
                    if hasattr(det, "ID") and det.ID == target_id:
                        found = True
                        break
            if found:
                positives += 1
            time.sleep(interval_s)
        except Exception:
            time.sleep(interval_s)
    return positives >= required_positives

# -----------------------------
# Main
# -----------------------------
gyro  = GyroSensor()
# HuskyLens V1 serial colour detector replaces the old ColorSensor for all runtime colour detection.

rotation_array = [0]
current_index  = 0
lap_count      = 0
max_laps       = 3
orientation_colour = None

manual_turn_mode        = False
manual_turn_target      = 0
manual_turn_start_angle = 0
manual_turn_frames      = 0
manual_turn_pulse_mode  = False
manual_turn_pulse_frames = 0
manual_turn_direction   = None   # "left" or "right" — locked at turn entry
manual_turn_steer_target = 0     # heading used for servo error (index 0: overshoot target)
manual_turn_cooldown_until = 0.0
last_color_detected     = None
color_read_threshold    = 10 # Faster confirmation threshold for blue only
blue_confirm_threshold  = 5
correction_mode         = False
correction_target       = 0
correction_frames       = 0
exit_burst_frames       = 0
post_sequence_mode      = False
orange_frames           = 0
blue_frames             = 0
line_cooldown           = False
forward_start_time      = None
# First-side timing state (measured once)
FIRST_SIDE_MEASURED     = False
FIRST_SIDE_START_TIME   = None
FIRST_SIDE_DURATION     = None
FIRST_SIDE_HALF         = None
FIRST_SIDE_MIN          = 0.05
FIRST_SIDE_MAX          = 10.0

# HuskyLens colour IDs (adjust if you trained different IDs)
HUSKYLENS_ORANGE_ID = 2   # treat ID 0 as orange/orientation colour
HUSKYLENS_BLUE_ID   = 1   # treat ID 1 as blue/opposite colour


def advance_rotation_index():
    global current_index, lap_count, last_color_detected, rotation_array
    current_index += 1
    if current_index >= len(rotation_array):
        current_index = 0
        lap_count += 1

        # Apply drift correction after each lap completion
        if lap_count >= 1:
            if orientation_colour == "orange":   # clockwise sequence
                for i in range(len(rotation_array)):
                    rotation_array[i] -= arrayCorrection
            elif orientation_colour == "blue":   # anticlockwise sequence
                for i in range(len(rotation_array)):
                    rotation_array[i] += arrayCorrection
            print(f"Lap {lap_count} complete — applied ±10° correction for {orientation_colour} orientation. "
                  f"New rotation_array = {rotation_array}")

        last_color_detected = None
        print(f"\nLap {lap_count} complete")


print("Waiting for button press to start...")
while GPIO.input(ButtonPin) == GPIO.LOW:
    time.sleep(0.1)

print("Button pressed — starting up...")

Movement.set_steering_angle(0)
Movement.start_servo()
Movement.start_motor()

print("Calibrating gyro...")
for _ in range(50):
    gyro.update()
    time.sleep(0.01)
print(f"Gyro calibrated.  Base yaw: {gyro.base_yaw:.2f}°  Current: {gyro.yaw:.2f}°")

frame_count = 0

while True:
    gyro.update()

    husky_detections = []
    if hl is not None:
        try:
            results = hl.requestAll()
            if results:
                for det in results:
                    if hasattr(det, "ID"):
                        husky_detections.append(det)
        except Exception:
            husky_detections = []
    else:
        husky_detections = []

    # Default raw/EMA placeholders for logging compatibility
    raw_r = raw_g = raw_b = 0
    r = g = b = 0

    is_orientation_color = False
    is_opposite_color = False
    is_white_reading = False  # HuskyLens does not provide a white concept

    if husky_detections:
        if orientation_colour is None:
            if any(d.ID == HUSKYLENS_ORANGE_ID for d in husky_detections):
                rotation_array = [0 - arrayOffset, -90 - arrayOffset, 180 - arrayOffset, 90 - arrayOffset]
                orientation_colour = "orange"
                print("\nClockwise rotation sequence selected (HuskyLens)")
            elif any(d.ID == HUSKYLENS_BLUE_ID for d in husky_detections):
                rotation_array = [0 - arrayOffset, 90 - arrayOffset, 180 - arrayOffset, -90 - arrayOffset]
                orientation_colour = "blue"
                print("\nCounterclockwise rotation sequence selected (HuskyLens)")

        if orientation_colour == "orange":
            is_orientation_color = any(d.ID == HUSKYLENS_ORANGE_ID for d in husky_detections)
            is_opposite_color    = any(d.ID == HUSKYLENS_BLUE_ID for d in husky_detections)
        else:  # orientation_colour == "blue"
            is_orientation_color = any(d.ID == HUSKYLENS_BLUE_ID for d in husky_detections)
            is_opposite_color    = any(d.ID == HUSKYLENS_ORANGE_ID for d in husky_detections)
    else:
        is_orientation_color = False
        is_opposite_color = False
        last_color_detected = None

    frame_count += 1

    # ── Reset colour tracking on non-colour frames ───────────────────────────
    if is_white_reading or (not is_orientation_color and not is_opposite_color):
        last_color_detected = None

    # ── Manual turn mode ─────────────────────────────────────────────────────
    if manual_turn_mode:
        if is_opposite_color and last_color_detected != "opposite" and not manual_turn_pulse_mode:
            manual_turn_pulse_mode   = True
            manual_turn_pulse_frames = 0
            last_color_detected      = "opposite"
            print("\nOpposite color detected — entering pulse phase...")

        error = normalize_angle_error(manual_turn_steer_target, gyro.yaw)

        raw_angle = -TURN_MAX_ANGLE if error > 0 else TURN_MAX_ANGLE

        if manual_turn_direction == "left":
            raw_angle = max(-TURN_MAX_ANGLE, min(0, raw_angle))
        else:  # "right"
            raw_angle = max(0, min(TURN_MAX_ANGLE, raw_angle))
        Movement.set_steering_angle(raw_angle, max_angle=TURN_MAX_ANGLE, full_range=True)

        if manual_turn_pulse_mode:
            manual_turn_pulse_frames += 1
            Movement.set_motor_forward(TURN_SPEED)

            if manual_turn_pulse_frames >= 8:
                time.sleep(TURN_EXIT_DELAY)
                manual_turn_mode        = False
                manual_turn_pulse_mode  = False
                manual_turn_frames      = 0
                manual_turn_cooldown_until = time.time() + 1.5
                exit_burst_frames       = EXIT_BURST_FRAMES
                advance_rotation_index()
                print(f"\nPulse complete — advancing to index {current_index}.  "
                      f"Current angle: {gyro.yaw:.2f}°  "
                      f"Burst: {exit_burst_frames} frames at {EXIT_BURST_POWER}%")
                if (not FIRST_SIDE_MEASURED) and (FIRST_SIDE_START_TIME is None) and lap_count == 0:
                    FIRST_SIDE_START_TIME = time.time()
                    print(f"Started FIRST_SIDE timing at {FIRST_SIDE_START_TIME:.2f}")
        else:
            manual_turn_frames += 1

            if manual_turn_frames <= TURN_SETTLE_FRAMES:
                Movement.set_motor_forward(TURN_CRAWL_SPEED)
                print(f"TURN SETTLE  frame={manual_turn_frames}/{TURN_SETTLE_FRAMES} | "
                      f"Target={manual_turn_target}° | Yaw={gyro.yaw:.2f}° | "
                      f"Pulse={Movement.pulse_ms:.3f} ms | RGB={r,g,b}")
            else:
                Movement.set_motor_forward(TURN_SPEED)
                print(f"MANUAL TURN  Target={manual_turn_target}° | Yaw={gyro.yaw:.2f}° | "
                      f"Error={error:.2f}° | Angle={raw_angle:.0f}° | "
                      f"Pulse={Movement.pulse_ms:.3f} ms | RGB={r,g,b}")

    # ── Correction mode ───────────────────────────────────────────────────────
    elif correction_mode:
        if is_orientation_color and last_color_detected != "orientation":
            time.sleep(TURN_ENTRY_DELAY)
            manual_turn_mode         = True
            manual_turn_frames       = 0
            manual_turn_pulse_mode   = False
            manual_turn_pulse_frames = 0
            correction_mode          = False
            manual_turn_start_angle  = gyro.yaw
            
            # --- TURN TARGET MATH FIX (CORRECTION MODE) ---
            if current_index + 1 < len(rotation_array):
                _next_hdg = rotation_array[current_index + 1]
            else:
                _next_hdg = rotation_array[0]
                
            _turn_diff = normalize_angle_error(_next_hdg, rotation_array[current_index])
            if _turn_diff > 0:
                manual_turn_target = rotation_array[current_index] + 50
            else:
                manual_turn_target = rotation_array[current_index] - 50

            last_color_detected = "orientation"
            print(f"\nOrientation colour during correction — entering manual turn.  "
                  f"Target: {rotation_array[current_index]}° → {manual_turn_target:.2f}° "
                  f"(next heading = {_next_hdg}°)")
                  
            # Lock the direction now so it can't flip mid-turn
            if current_index == 0:
                manual_turn_steer_target = manual_turn_target
                _init_err = normalize_angle_error(manual_turn_target, gyro.yaw)
            else:
                manual_turn_steer_target = _next_hdg
                _init_err = normalize_angle_error(_next_hdg, gyro.yaw)
                
            manual_turn_direction = "left" if _init_err > 0 else "right"
            print(f"Turn direction locked: {manual_turn_direction}")
            # ----------------------------------------------
        else:
            error     = normalize_angle_error(correction_target, gyro.yaw)
            raw_angle = max(-55, min(55, -error))
            Movement.set_steering_angle(raw_angle)
            Movement.set_motor_forward(CORRECTION_SPEED)
            print(f"CORRECTION  Target={correction_target}° | Yaw={gyro.yaw:.2f}° | "
                  f"Error={error:.2f}° | Angle={raw_angle:.1f}° | RGB={r,g,b}")

            correction_frames -= 1
            if correction_frames <= 0:
                correction_mode = False
                print("\nCorrection complete — returning to normal mode")

    # ── Normal mode ───────────────────────────────────────────────────────────
    else:
        if is_orientation_color and last_color_detected != "orientation":
            if time.time() < manual_turn_cooldown_until:
                is_orientation_color = False
            else:
                # Stop FIRST_SIDE timing when the next manual turn begins (measured once)
                if (FIRST_SIDE_START_TIME is not None) and (not FIRST_SIDE_MEASURED):
                    measured = time.time() - FIRST_SIDE_START_TIME
                    FIRST_SIDE_DURATION = measured
                    FIRST_SIDE_HALF = max(FIRST_SIDE_MIN, min(FIRST_SIDE_MAX, FIRST_SIDE_DURATION / 2.0))
                    FIRST_SIDE_MEASURED = True
                    print(f"First side measured: {FIRST_SIDE_DURATION:.2f}s -> half={FIRST_SIDE_HALF:.2f}s")

                time.sleep(TURN_ENTRY_DELAY)
                manual_turn_mode         = True
                manual_turn_frames       = 0
                manual_turn_pulse_mode   = False
                manual_turn_pulse_frames = 0
                manual_turn_start_angle  = gyro.yaw
                
                # --- TURN TARGET MATH FIX (NORMAL MODE) ---
                if current_index + 1 < len(rotation_array):
                    _next_hdg = rotation_array[current_index + 1]
                else:
                    _next_hdg = rotation_array[0]
                    
                _turn_diff = normalize_angle_error(_next_hdg, rotation_array[current_index])
                if _turn_diff > 0:
                    manual_turn_target = rotation_array[current_index] + 50
                else:
                    manual_turn_target = rotation_array[current_index] - 50

                last_color_detected = "orientation"
                print(f"\nOrientation colour detected — starting manual 50° turn.  "
                      f"Target: {rotation_array[current_index]}° → {manual_turn_target:.2f}° "
                      f"(next heading = {_next_hdg}°)")
                      
                # Lock the direction now so it can't flip mid-turn
                if current_index == 0:
                    manual_turn_steer_target = manual_turn_target
                    _init_err = normalize_angle_error(manual_turn_target, gyro.yaw)
                else:
                    manual_turn_steer_target = _next_hdg
                    _init_err = normalize_angle_error(_next_hdg, gyro.yaw)
                    
                manual_turn_direction = "left" if _init_err > 0 else "right"
                print(f"Turn direction locked: {manual_turn_direction}")
                # ------------------------------------------
        else:
            # Straight driving with gyro correction
            target_angle = rotation_array[current_index]
            error        = normalize_angle_error(target_angle, gyro.yaw)
            raw_angle    = max(-60, min(60, -error))

            Movement.set_steering_angle(raw_angle)

            # ── Exit burst: brief high-power pulse after a turn ──────────────
            if exit_burst_frames > 0:
                Movement.set_motor_forward(EXIT_BURST_POWER)
                exit_burst_frames -= 1
                print(f"EXIT BURST  frame={EXIT_BURST_FRAMES - exit_burst_frames}/{EXIT_BURST_FRAMES} | "
                      f"Power={EXIT_BURST_POWER}% | "
                      f"Target={target_angle}° | Yaw={gyro.yaw:.2f}°")
            else:
                Movement.set_motor_forward(NORMAL_SPEED)
                print(f"Target={target_angle}° | Yaw={gyro.yaw:.2f}° | "
                      f"Error={error:.2f}° | Angle={raw_angle:.1f}° | "
                      f"RGB={r,g,b} | Lap={lap_count}")

            # ── Consecutive-frame colour counters ────────────────────────────────────
            if orientation_colour == "orange" and is_orientation_color:
                orange_frames += 1
                blue_frames    = 0
            elif orientation_colour == "blue" and is_orientation_color:
                blue_frames   += 1
                orange_frames  = 0
            else:
                orange_frames  = 0
                blue_frames    = 0

            orange_confirmed = orange_frames >= color_read_threshold
            blue_confirmed   = blue_frames   >= blue_confirm_threshold

            # ── Line cooldown reset ──────────────────────────────────────────────────
            if not orange_confirmed and not blue_confirmed:
                line_cooldown = False

            # Use the defensive advance helper and cooldown to avoid double increments
            if orientation_colour == "orange" and is_orientation_color and last_color_detected != "orange":
                if not line_cooldown and orange_confirmed:
                    line_cooldown = True
                    last_color_detected = "orange"
                    advance_rotation_index()
                    print(f"\nOrange detected — moving to index {current_index}")
            elif orientation_colour == "blue" and is_orientation_color and last_color_detected != "blue":
                if not line_cooldown and blue_confirmed:
                    line_cooldown = True
                    last_color_detected = "blue"
                    advance_rotation_index()
                    print(f"\nBlue detected — moving to index {current_index}")

        # --- 0.55 END SEQUENCE FIX ---
        if lap_count >= max_laps:
            print("\nSequence complete. Entering post-sequence forward mode...")

            if FIRST_SIDE_MEASURED and FIRST_SIDE_DURATION is not None:
                post_duration = FIRST_SIDE_DURATION * 0.55
                Movement.start_servo()
                _end = time.time() + post_duration
                while time.time() < _end:
                    gyro.update()
                    # Keep straight using heading 0°
                    error = normalize_angle_error(0, gyro.yaw)
                    raw_angle = max(-60, min(60, -error))
                    Movement.set_steering_angle(raw_angle)
                    Movement.set_motor_forward(NORMAL_SPEED)
                    time.sleep(0.01)
                
                Movement.brake()
                Movement.set_motor_forward(0)
                print(f"Post-laps forward for {post_duration:.2f}s complete. Stopping.")
            else:
                Movement.brake()
                Movement.set_motor_forward(0)
                print("Sequence complete, stopping.")
            
            # Break safely out of the loop and end the script
            break
        # -----------------------------

    time.sleep(0.01)