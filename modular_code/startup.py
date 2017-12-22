from builtins import input
import numpy as np
from multiprocessing import Queue
import time

versionstring = '\nSensorPod 52 SRPY Attack Version\n'
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

#=============VARIABLES===============
BeeperP = False  # beeps at the beginning and whenever the sonar detects something close
ServoP = False
NetEnable = True
UseSonar = False
LoggingP = True
MovieP = False   # always starts False - initially started by tracking, but then toggled by the m key
#MovieP = True  # debug
RunThreadsFlag = True  # always starts False - used internally to shutdown threads

servo_queue = Queue()
image_queue = Queue()
image_label_queue = Queue()
text_queue = Queue()
queue_from_cam = Queue(maxsize = 1)

sonar_reading = 765
sonar_reading_threshold = 200   # this is the threshold for firing the net in units = cm

resolution_x = 320
Xcenter = resolution_x / 2
resolution_y = 240
Ycenter = resolution_y / 2
dcx = 0
dcx_vel = 0
dcx_vel_avg = 0
dcy = 0
dcy_vel = 0
dcy_vel_avg = 0
framecounter = 0
dcx_acc_avg = 0
dcy_acc_avg =0
est_x=Xcenter
est_vx=0
est_y=Ycenter
est_vy=0

trackp = 0
restart_code = 0 #unused
gray = np.zeros((240,320))
ROIblob = np.zeros((50, 50))
trackingtimeout = 0

bk_scale = 0.04     # blob kernel scaling
cs_scale = -0.1
#cs_scale = -0.15   #   center-surround kernel scaling
blbmrg = 5      # border margin for blob detection
tracksize0=18 #base ROI size
tracksize=tracksize0
trackscale=1
pyrscale = 0.7  # image pyramid step scaling factor	

newtime=time.time()
originaltime=time.time()

centsurr = np.load('centsurr_kernel.npy')#these are only used in the main loop (declouding)
print 'Loaded filter kernel file: centsurr_kernel.npy'
centsurr2 = np.load('centsurr2_kernel.npy')
print 'Loaded filter kernel file: centsurr2_kernel.npy'

blobkernel = np.load('sharp_centsurr3.npy')#used in find a blob and declouding
print 'Loaded filter kernel file: sharp_centsurr3.npy' 

# ==================== end of variables ===========

def startupfunc(debug):
	if not(debug):#get debug from main file
		import smbus #i2C
		import RPi.GPIO as GPIO
		# ============ SETUP I2C ============
		bus = smbus.SMBus(1)

		# ============ SETUP GPIO =============
		GPIO.setmode (GPIO.BCM)  # use GPIO port names, not pin numbers
		GPIO.setup (22, GPIO.IN)   # laser rangefinder for testing if we are airborne
		GPIO.setup (23, GPIO.OUT, initial = 0)  # Beeper Control Bit
	else:
		bus=0

	# ============ USER INPUTS =============
	global VerboseMode, ShowGraphics, DecloudP
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
	#loggingquery = raw_input("\nAppend Log Data (or Images) to File?  (y/n)  ")
	#if loggingquery == "y":
	#    LoggingP = True
	#    print ("OK, will log flight and tracking data.")
	#else:
	#    LoggingP = False
	#    print ("OK, NOT logging data.")
