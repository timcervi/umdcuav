import time
import RPi.GPIO as GPIO

# ============ SETUP GPIO =============
GPIO.setmode (GPIO.BCM)  # use GPIO port names, not pin numbers
GPIO.setup (22, GPIO.IN)


counter = 0
while True:
	counter += 1
	if GPIO.input(22):
		print counter
	time.sleep(0.1)
