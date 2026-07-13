import smbus2
import time
import Hobot.GPIO as GPIO # GPIO library for RDK X5
from huskylib import HuskyLensLibrary #Huskylens Python Library

# -----------------------------
# Camera Setup
# -----------------------------
hl = HuskyLensLibrary("SERIAL", "/dev/ttyS1", 9600)
print("Knock:", hl.knock())




