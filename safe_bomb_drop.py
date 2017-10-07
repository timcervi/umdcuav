# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685


# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 160 # 150  # Min pulse length out of 4096
servo_max = 425 #600  # Max pulse length out of 4096
netlaunch_min = 160
netlaunch_max = 425

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(44)
servoval0 = 300
servoval1 = 300
servoval2 = 300
servoval3 = 300
servoval4 = 300

#servoval4 = int(input ("\nEnter value for Servo 4 (net launch servo) : "))
servoval4 = 200  # safe value

if (servoval0 <= servo_max) and (servoval0 >= servo_min):
	pwm.set_pwm(0, 0, servoval0)
else:
	print 'Value error for servo 0. Data not sent.'
if (servoval1 <= servo_max) and (servoval1 >= servo_min):
	pwm.set_pwm(1, 0, servoval1)
else:
	print 'Value error for servo 1. Data not sent.'
if (servoval2 <= servo_max) and (servoval2 >= servo_min):
	pwm.set_pwm(2, 0, servoval2)
else:
	print 'Value error for servo 2. Data not sent.'
if (servoval3 <= servo_max) and (servoval3 >= servo_min):
	pwm.set_pwm(3, 0, servoval3)
else:
	print 'Value error for servo 3. Data not sent.'
if (servoval4 <= netlaunch_max) and (servoval4 >= netlaunch_min):
	pwm.set_pwm(4, 0, servoval4)
else:
	print 'Value error for servo 4. Data not sent.'

print 'Valid Values Sent.'
print 'Current Values (%d, %d, %d, %d, %d)'%(servoval0,servoval1,servoval2,servoval3, servoval4)
