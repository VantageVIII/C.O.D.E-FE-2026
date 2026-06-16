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
ButtonPin = 18

# -----------------------------
# GPIO Setup
# -----------------------------
outputPins = [IN1, IN2, LEDPin, ENA, ServoPin]
GPIO.setup(outputPins, GPIO.OUT)
GPIO.output(LEDPin, GPIO.HIGH)
GPIO.setup(ButtonPin, GPIO.IN)

# -----------------------------
# Confirm channels
# -----------------------------
print("GPIO29: ", GPIO.gpio_function(29))
print("GPIO31: ", GPIO.gpio_function(31))
print("GPIO37: ", GPIO.gpio_function(37))
print("GPIO32: ", GPIO.gpio_function(32))
print("GPIO33: ", GPIO.gpio_function(33))
print("GPIO18: ", GPIO.gpio_function(18))

# -----------------------------
# Movement Class
# -----------------------------
class Movement:
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
            
    @staticmethod
    def forward(power, duration, freq=500):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        Movement._motor_pwm(power, freq, duration)

    @staticmethod
    def backward(power, duration, freq=500):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        Movement._motor_pwm(power, freq, duration)

    @staticmethod
    def continuous_forward(power, freq=500):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        Movement._motor_pwm(power, freq)
    
    @staticmethod
    def continuous_backward(power, freq=500):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        Movement._motor_pwm(power, freq)
            
    @staticmethod
    def coast():
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(ENA, GPIO.LOW)

    @staticmethod
    def brake():
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(ENA, GPIO.LOW)

    @staticmethod
    def servo_angle(angle):
        neutral_ms = 1.0
        left_ms = neutral_ms - 0.65   
        right_ms = neutral_ms + 0.65  
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
        Movement.servo_angle(angle)
        if power >= 0:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
        else:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
            power = abs(power)
        Movement._motor_pwm(power, freq, duration)

# -----------------------------
# Main Program
# -----------------------------
try:
    print("Waiting for button press to start...")
    GPIO.wait_for_edge(ButtonPin, GPIO.RISING)
    print("Button pressed, Starting code.")

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    GPIO.cleanup()
    print("GPIO cleanup complete.")
