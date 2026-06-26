import Hobot.GPIO as GPIO
import time
import threading

ServoPin = 32

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ServoPin, GPIO.OUT)

# Simple bit-bang loop to hold servo at a fixed pulse width
def servo_loop(pulse_ms):
    while True:
        high_time = pulse_ms / 1000.0
        frame_time = 0.02  # 50 Hz → 20 ms frame
        t0 = time.time()
        GPIO.output(ServoPin, GPIO.HIGH)
        time.sleep(high_time)
        GPIO.output(ServoPin, GPIO.LOW)
        elapsed = time.time() - t0
        remainder = frame_time - elapsed
        if remainder > 0:
            time.sleep(remainder)

# For your servo calibration, 150° midpoint corresponds to ~1.5 ms pulse
midpoint_ms = 1.5

print("Setting servo to 150° midpoint for mechanical adjustment...")
threading.Thread(target=servo_loop, args=(midpoint_ms,), daemon=True).start()

# Keep running until you stop the script
while True:
    time.sleep(1)
