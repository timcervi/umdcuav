versionstring = '\nSensorPod 52 SRPY Attack Version\n'

#import cv2  # openCV

import math
import numpy as np
import time
from multiprocessing import Process, Queue
import threading
debug=1
if not(debug):
	import RPi.GPIO as GPIO
	from picamera import PiCamera
	from picamera.array import PiRGBArray
#setup i2c, gpio, and query for user inputs ALL ON IMPORT, returns the bus and mode flags
	from startup import bus,VerboseMode,ShowGraphics,DecloudP
if debug:
	from startup import VerboseMode,ShowGraphics,DecloudP
from servocontrolthread import AsyncServoControl

BeeperP = False  # beeps at the beginning and whenever the sonar detects something close
ServoP = True
NetEnable = True
UseSonar = False
LoggingP = True
MovieP = False   # always starts False - initially started by tracking, but then toggled by the m key
#MovieP = True  # debug
RunThreadsFlag = True  # always starts False - used internally to shutdown threads

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
est_x=0
est_y=0

#only used in camera process
bk_scale = 0.04     # blob kernel scaling
cs_scale = -0.1
#cs_scale = -0.15   #   center-surround kernel scaling
blbmrg = 5      # border margin for blob detection

pyrscale = 0.7  # image pyramid step scaling factor
    
trackp = 0
restart_code = 0 #unused
gray = np.zeros((240,320))
ROIblob = np.zeros((50, 50))
trackingtimeout = 0 

# ==================== beginning of the servo control thread ===========
servo_queue = Queue()
# ServoP is defined earlier above
if ServoP:
	servothread = AsyncServoControl()
	servothread.daemon = True
	servothread.start()
	print "Servo Thread Start..."
# ========== end of the servo control thread =========

