import Hobot.GPIO as GPIO
import time

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

# -----------------------------
# GPIO Setup
# -----------------------------
outputPins = [IN1, IN2, LEDPin, ENA]
GPIO.setup(outputPins, GPIO.OUT)
GPIO.setup(ServoPin, GPIO.OUT)

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
    # Helper method for motor PWM
    @staticmethod    
    def _motor_pwm(power, freq, duration=None):
        period = 1.0 / freq
        high_time = (power / 100.0) * period
        low_time = period - high_time
        
        if duration is None:
            try:
                while True:
                    GPIO.output(ENA, GPIO.HIGH)
                    time.sleep(high_time)
                    GPIO.output(ENA, GPIO.LOW)
                    time.sleep(low_time)
            except KeyboardInterrupt:
                Movement.coast()
        else:
            end_time = time.time() + duration
            while time.time() < end_time:
                GPIO.output(ENA, GPIO.HIGH)
                time.sleep(high_time)
                GPIO.output(ENA, GPIO.LOW)
                time.sleep(low_time)
            
    # Forward
    @staticmethod
    def forward(power, duration, freq=500):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        Movement._motor_pwm(power, freq, duration)

    # Backward
    @staticmethod
    def backward(power, duration, freq=500):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        Movement._motor_pwm(power, freq, duration)

    # Continuous forward
    @staticmethod
    def continuous_forward(power, freq=500):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        Movement._motor_pwm(power, freq)
    
    # Continuous backward
    @staticmethod
    def continuous_backward(power, freq=500):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        Movement._motor_pwm(power, freq)
            
    # Coast
    @staticmethod
    def coast():
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(ENA, GPIO.LOW)

    # Brake
    @staticmethod
    def brake():
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(ENA, GPIO.LOW)

    # Steering Angle (bit-banged)
    @staticmethod
    def servo_angle(angle):
        # Define the actual neutral pulse width (straight ahead)
        neutral_ms = 1.0

        # Define usable range around that neutral
        left_ms = neutral_ms - 0.65   # tweak until full left looks correct
        right_ms = neutral_ms + 0.65  # tweak until full right looks correct

        # Map 35–145° into left_ms–right_ms
        pulse_ms = left_ms + ((angle - 35) / (145 - 35)) * (right_ms - left_ms)

        period_ms = 20.0
        high_time = pulse_ms / 1000.0
        low_time = (period_ms / 1000.0) - high_time

        for i in range(50):
            GPIO.output(ServoPin, GPIO.HIGH)
            time.sleep(high_time)
            GPIO.output(ServoPin, GPIO.LOW)
            time.sleep(low_time)
    
    @staticmethod
    def drive(power, duration=None, angle=90, freq=500):
        """
        power: -100 to +100 (%)
            positive = forward
            negative = backward
        duration: seconds to run (None = continuous)
        angle: steering angle (default 90 = straight)
        freq: PWM frequency (Hz)
        """
        # Set steering first
        Movement.servo_angle(angle)

        # Determine direction from sign of power
        if power >= 0:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
        else:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
            power = abs(power)  # use magnitude for PWM

        # Run motor with PWM
        Movement._motor_pwm(power, freq, duration)

# -----------------------------
# Main Program
# -----------------------------
try:
    while True:
        print("Testing forward...")
        Movement.drive(20, 2, 90)
        
        print("Testing coast...")
        Movement.coast()
        time.sleep(1)
        
        print("Testing backward...")
        Movement.drive(-80, 2, 90)
        
        print("Braking...")
        Movement.brake()

        print("Testing forward...")
        Movement.drive(20, 2, 35)
        
        print("Testing coast...")
        Movement.coast()
        time.sleep(1)
        
        print("Testing backward...")
        Movement.drive(-80, 2, 145)
        
        print("Braking...")
        Movement.brake()
        
        print("Servo front")
        Movement.servo_angle(90)
                
        print("Testing left turn...")
        Movement.servo_angle(35)
        time.sleep(2)
        
        print("Testing right turn...")
        Movement.servo_angle(145)
        time.sleep(2)


except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    GPIO.cleanup()
    print("GPIO cleanup complete.")
