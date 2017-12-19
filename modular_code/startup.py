from builtins import input
import cuav_run as ce	#get the main file for universal variables
if not(ce.debug):
	import smbus #i2C
	import RPi.GPIO as GPIO
print("Sensor Pod Software - UMD CUAV Team - Fall 2017")
print('\nOct 10, 2017: version 51 - no sonar !')
print('\n rewriting vision tracking')
print('\nImage Recording starts automatically once a target is acquired.')
print('\n  - Hit Key "m" or "M" to toggle image recording')
print('To hunt down background processes, use:  "ps -u root" and look for "python"')
print('To kill background processes manually, use:  "sudo kill [XXXX]"')

print('Vehicular Yaw control and Pitch Gimbal always active !')
print('Vehicular Roll and Vehicular Pitch on LaserRanger Airborne Detection')
print('Be sure to update the trim values for the vehicle!')



def startupfunc():
	if not(ce.debug):#get debug from main file
		# ============ SETUP I2C ============
		bus = smbus.SMBus(1)

		# ============ SETUP GPIO =============
		GPIO.setmode (GPIO.BCM)  # use GPIO port names, not pin numbers
		GPIO.setup (22, GPIO.IN)   # laser rangefinder for testing if we are airborne
		GPIO.setup (23, GPIO.OUT, initial = 0)  # Beeper Control Bit
	else:
		bus=0

	# ============ USER INPUTS =============
	verbosequery = input ("\n\nVerbose Mode ? (y/n)  ")
	if verbosequery == "y":
		VerboseMode = True
		print ("OK, All available text will be printed.")
	else:
		VerboseMode = False
		print ("OK, limiting the text.")

	graphicsquery = input ("\nShow Lots of Graphics? (y/n)  ")
	if graphicsquery == "y":
		ShowGraphics = True
		print ("OK, showing graphics.")
	else:
		ShowGraphics = False
		print ("OK, showing only absolute minimal graphics.")

	decloudquery = input("\nUse a Tanh Nonlinearity? (Declouding) ?  (y/n)  ")
	if decloudquery == "y":
		DecloudP = True
		print ("OK, Using the tanh algorithm.")
	else:
		DecloudP = False
		print ("OK, No Tanh Declouding.")

	print ('This version tracks black against white...')
	return(bus,VerboseMode,ShowGraphics,DecloudP)
	#loggingquery = raw_input("\nAppend Log Data (or Images) to File?  (y/n)  ")
	#if loggingquery == "y":
	#    LoggingP = True
	#    print ("OK, will log flight and tracking data.")
	#else:
	#    LoggingP = False
	#    print ("OK, NOT logging data.")
