print('===================== start cuav_run ===================:)')
import cv2  # openCV
from multiprocessing import Process, Queue
import threading
debug=1
if not(debug):
	import RPi.GPIO as GPIO

#import the other used modules
import startup as vars
from servocontrolthread import AsyncServoControl
import datalogger
import cameraproc
import blobFuncs
import MainAnalysisLoop

#setup i2c, gpio, variables, and get user inputs, returns the bus and mode flags
vars.startupfunc(debug)

#==========Start the servo thread
if vars.ServoP:
	servothread = AsyncServoControl(debug)
	servothread.daemon = True
	servothread.start()
	print("Servo Thread Start...")

# ============ Start the datalogger process
if vars.LoggingP == True:
	datalogger_process = Process(target=datalogger.data_logger, args=(vars.image_queue, vars.image_label_queue, vars.text_queue, vars.servo_queue,vars.DecloudP,vars.versionstring))
	datalogger_process.start() 
	print('Datalogger Process start')

# ============ Start the Camera Process
cam_process = Process(target=camproc.cam_loop, args=(vars.queue_from_cam, vars.DecloudP))#decloudp unused
cam_process.start()

# ============ Start the Main Analysis Loop
MainAnalysisLoop.mainLoop()


# ============Cleanup/Exit Code
avgFPS = vars.framecounter / (vars.newtime - vars.originaltime)

print '\nAverage FPS was %f frames per second'%(avgFPS)
vars.RunThreadsFlag=False
if vars.LoggingP:
    filestring = 'Average FPS was %f frames per second'%(avgFPS)
    vars.text_queue.put (filestring)

    datalogger_process.terminate()
    datalogger_process.join()
    print 'Terminated.    (datalogger process)'

cv2.destroyAllWindows()
print 'closing windows'
cv2.waitKey(1)

cam_process.terminate()
print 'Waiting for camera process to terminate... '
cam_process.join()
print 'Terminated.    (camera process)'

if not(debug):
	GPIO.cleanup ()
	
print '\nEverything appears to have closed gracefully.'