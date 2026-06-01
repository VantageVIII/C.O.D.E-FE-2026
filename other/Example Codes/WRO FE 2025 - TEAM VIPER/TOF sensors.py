import time
import board
import busio
import digitalio
import adafruit_vl53l0x

# I2C setup
i2c = busio.I2C(board.SCL, board.SDA)

# Define XSHUT pins for the sensors
xshut_pins = [board.D22, board.D23, board.D24]
addresses = [0x30, 0x31, 0x32]  # New I2C addresses for each sensor
 
# Setup XSHUT GPIOs
xshuts = []
for pin in xshut_pins:
    xshut = digitalio.DigitalInOut(pin)
    xshut.direction = digitalio.Direction.OUTPUT
    xshuts.append(xshut)

# Disable all sensors first
for xshut in xshuts:
    xshut.value = False
time.sleep(0.5)

sensors = []

# Initialize each sensor one by one
for i, xshut in enumerate(xshuts):
    xshut.value = True  # Power on one sensor at a time
    time.sleep(0.5)
    try:
        sensor = adafruit_vl53l0x.VL53L0X(i2c)
        sensor.set_address(addresses[i])
        print(f"Sensor {i+1} initialized at I2C address 0x{addresses[i]:02X}")
        sensors.append(sensor)
    except Exception as e:
        print(f"Sensor {i+1} failed to initialize: {e}")
        sensors.append(None)

# Distance reading loop
while True:
    for i, sensor in enumerate(sensors):
        if sensor is not None:
            try:
                distance_mm = sensor.range
                distance_cm = distance_mm / 10
                print(f"Sensor {i+1} (0x{addresses[i]:02X}): {distance_mm} mm ({distance_cm:.1f} cm)")
            except Exception as e:
                print(f"Sensor {i+1} read error: {e}")
        else:
            print(f"Sensor {i+1} not initialized.")
    print("-" * 40)
    time.sleep(0.5)

