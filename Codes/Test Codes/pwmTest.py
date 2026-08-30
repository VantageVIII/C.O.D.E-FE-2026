#!/usr/bin/env python3
import os
import time
import glob
import errno
import Hobot.GPIO as GPIO

# -----------------------------
# Helper: unexport all exported PWM channels
# -----------------------------
def unexport_all_pwms():
    """
    For each /sys/class/pwm/pwmchipX:
      - list pwmN entries (exported channels)
      - write the channel number to unexport to free it
    Requires root privileges to write to sysfs.
    """
    pwmchip_paths = glob.glob("/sys/class/pwm/pwmchip*")
    for chip in pwmchip_paths:
        unexport_path = os.path.join(chip, "unexport")
        if not os.path.exists(unexport_path):
            continue
        # find exported pwmN directories inside this chip
        for entry in os.listdir(chip):
            if entry.startswith("pwm") and entry != "pwmchip":
                # entry is like 'pwm0', 'pwm1'
                try:
                    pwm_index = int(entry.replace("pwm", ""))
                except ValueError:
                    continue
                try:
                    # write the index to unexport
                    with open(unexport_path, "w") as f:
                        f.write(str(pwm_index))
                except PermissionError:
                    raise PermissionError(
                        "Permission denied writing to %s. Run script as root (sudo)." % unexport_path
                    )
                except OSError as e:
                    # ignore if already unexported by race condition
                    if e.errno != errno.EINVAL and e.errno != errno.ENOENT:
                        raise

# -----------------------------
# Main: cleanup then init servo
# -----------------------------
ServoPin = 32  # BOARD numbering
NEUTRAL = 150
LEFT = NEUTRAL - 45
RIGHT = NEUTRAL + 45

# Attempt to clear any leftover PWM exports first
try:
    unexport_all_pwms()
except PermissionError as e:
    print("ERROR:", e)
    print("Run this script with sudo/root to allow PWM unexport.")
    raise SystemExit(1)

# Now safe to initialize GPIO and create PWM objects
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ServoPin, GPIO.OUT)

# Create PWM after sysfs cleanup
servo = GPIO.PWM(ServoPin, 50)  # 50 Hz
servo.start(0)

def set_servo_angle(angle):
    # Map 0–300° to 0.5–2.5 ms pulse width
    min_ms, max_ms = 0.5, 2.5
    period_ms = 20.0
    pulse_ms = min_ms + (angle / 300.0) * (max_ms - min_ms)
    duty = (pulse_ms / period_ms) * 100.0
    servo.ChangeDutyCycle(duty)

try:
    print("Neutral (150°)")
    set_servo_angle(NEUTRAL)
    time.sleep(1.5)

    print("Left (105°)")
    set_servo_angle(LEFT)
    time.sleep(1.5)

    print("Right (195°)")
    set_servo_angle(RIGHT)
    time.sleep(1.5)

    print("Back to neutral")
    set_servo_angle(NEUTRAL)
    time.sleep(1.5)

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    servo.stop()
    GPIO.cleanup()
    print("Cleanup complete.")
