versionstring = '\nSensorPod 51 SRPY Attack Version\n'

print "Sensor Pod Software - UMD CUAV Team - Fall 2017"
print 'Oct 5, 2017: version 51 - rewriting vision tracking'
print '\nImage Recording starts automatically once a target is acquired.'
print '\n  - Hit Key "m" or "M" to toggle image recording'
print 'To hunt down background processes, use:  "ps -u root" and look for "python"'
print 'To kill background processes manually, use:  "sudo kill [XXXX]"'

print 'Vehicular Yaw control and Pitch Gimbal always active !'
print 'Vehicular Roll and Vehicular Pitch on LaserRanger Airborne Detection'
print 'Be sure to update the trim values for the vehicle!'

import cv2  # openCV
from picamera import PiCamera
from picamera.array import PiRGBArray
import math
import numpy as np
import time
from multiprocessing import Process, Queue
import threading
import smbus   # i2c
import Adafruit_PCA9685  # servo pwm controller board
import random
import RPi.GPIO as GPIO

# ============ SETUP I2C ============
bus = smbus.SMBus(1)
sonar_reading = 765
sonar_reading_threshold = 200   # this is the threshold for firing the net in units = cm

# ============ SETUP GPIO =============
GPIO.setmode (GPIO.BCM)  # use GPIO port names, not pin numbers
GPIO.setup (22, GPIO.IN)   # laser rangefinder for testing if we are airborne
GPIO.setup (23, GPIO.OUT, initial = 0)  # Beeper Control Bit

BeeperP = False  # beeps at the beginning and whenever the sonar detects something close
ServoP = True
NetEnable = True

lastgoodframe = 0
lastgoodrange = 0
framecounter = 0
dcx_acc_avg = 0
dcy_acc_avg =0

verbosequery = raw_input ("\n\nVerbose Mode ? (y/n)  ")
if verbosequery == "y":
	VerboseMode = True
	print ("OK, All available text will be printed.")
else:
	VerboseMode = False
	print ("OK, limiting the text.")
    
graphicsquery = raw_input ("\nShow Lots of Graphics? (y/n)  ")
if graphicsquery == "y":
	ShowGraphics = True
	print ("OK, showing graphics.")
else:
	ShowGraphics = False
	print ("OK, showing only absolute minimal graphics.")

LoggingP = True
#loggingquery = raw_input("\nAppend Log Data (or Images) to File?  (y/n)  ")
#if loggingquery == "y":
#    LoggingP = True
#    print ("OK, will log flight and tracking data.")
#else:
#    LoggingP = False
#    print ("OK, NOT logging data.")

print ('This version tracks black against white...')
decloudquery = raw_input("\nUse a Tanh Nonlinearity? (Declouding) ?  (y/n)  ")
if decloudquery == "y":
    DecloudP = True
    print ("OK, Using the tanh algorithm.")
else:
    DecloudP = False
    print ("OK, No Tanh Declouding.")

MovieP = False   # always starts False - initially started by tracking, but then toggled by the m key
#MovieP = True  # debug

RunThreadsFlag = True  # always starts False - used internally to shutdown threads

resolution_x = 320
Xcenter = resolution_x / 2
resolution_y = 240
Ycenter = resolution_y / 2
dcx = 0
dcx_vel = 0
dcx_vel_avg = 0
olddcx_vel_avg = 0
dcy = 0
dcy_vel = 0
dcy_vel_avg = 0
olddcy_vel_avg = 0

bk_scale = 0.04     # blob kernel scaling
cs_scale = -0.1
#cs_scale = -0.15   #   center-surround kernel scaling
pyrscale = 0.7  # image pyramid step scaling factor
blbmrg = 5      # border margin for blob detection
    
trackp = 0
restart_code = 0
gray = np.zeros((240,320))
ROIblob = np.zeros((50, 50))
trackingtimeout = 0

# ==================== beginning of the servo control thread ===========

class AsyncServoControl (threading.Thread):
	def __init__(self):
		global pwm
		super(AsyncServoControl, self).__init__()
		# Initialise the PWM device using the default address
		pwm = Adafruit_PCA9685.PCA9685()
		pwm.set_pwm_freq(44)      # Set frequency to 44, was initially 60 Hz

		VehRollMiddle = 301
		VehPitchMiddle = 299
		VehYawMiddle = 301	# 300 (@ 44Hz pwm update rate) is roughly center stick 
		servo_pitch_middle = 300      # 300 points the camera mostly up
		netlaunch_safe = 200
		pwm.set_pwm_freq(44)      # Set frequency to 44, was initially 60 Hz
		pwm.set_pwm(0, 0, servo_pitch_middle)
		pwm.set_pwm(1, 0, VehRollMiddle)
		pwm.set_pwm(2, 0, VehPitchMiddle)
		pwm.set_pwm(3, 0, VehYawMiddle)
		pwm.set_pwm(4, 0, netlaunch_safe)

		print 'Initial values sent to the PWM board.'
		if BeeperP:
			for qq in range(3):
				GPIO.output (23, 1)
				time.sleep (0.05)
				GPIO.output (23, 0)
				time.sleep (0.1)
		
		self._stop = threading.Event()
		
	def run(self):
		global sonar_reading, servo_queue, pwm, est_x, est_y, dcx, dcy, dcx_vel_avg,dcx_acc_avg
		global dcy_vel_avg,dcy_acc_avg, trackp, trackingtimeout, RunThreadsFlag, VerboseMode
		global framecounter, sonar_reading_threshold, NetEnable
		servo_freeze = True   # don't try to predict for servo movement
		maxstep = 50
		airborne = False # once set, this will not be reset to False
		netfired = False # once set, this will not be reset to False
		lastupdate = time.time() * 1000  # time given in seconds, convert to milliseconds 
		Kp = 0.25 #-0.1   # proportional gain for the servos
		Kd = 0.2  #-0.1  # derivative gain
		Ka = 1.0  # acceleration gain
		K_servo_2_cam_pitch = 0.75   # converts servo units to pixel units
		yawKp_veh = 1.0  # vehicle yaw control gain
		yawKd_veh = 5.0  # derivative term for yaw control
		rollKp_veh = 0.75
		rollKd_veh = 0.75
		pitchKp_veh = 0.35  # increased from 0.15 (used 0.15 for outdoor demo)
		Cpitch = -0.4
			  
		VehRollMin = 270
		VehRollMiddle = 301    #   <====== ROLL TRIM
		VehRollMax = 330
		VehPitchMin = 270
		VehPitchMiddle = 299    #   <=====  PITCH TRIM
		VehPitchMax = 340
		VehPitchAttack = 215  # ATTACK PITCH  (emulates stick all the way down)
 		VehYawMin = 270
		VehYawMiddle = 301    #   <=====  YAW TRIM
		VehYawMax = 330
		servo_pitch_min = 250 # 185 is vertical and camera points front 
		servo_pitch_middle = 300      # 
		servo_pitch_max = 365 # 365 is horizontal and camera points up
		netlaunch_safe = 200  # make sure this value always matches the one in the init section above.
		netlaunch_fire = 425

		vehroll_f = VehRollMiddle   # floating pt calculations
		vehroll = int(round(vehroll_f))  # int() versions to send
		vehpitch_f = VehPitchMiddle
		vehpitch = int(round(vehpitch_f))
		vehyaw_f = VehYawMiddle
		vehyaw = int(round(vehyaw_f))
		servo_pitch_f = servo_pitch_middle
		servo_pitch = int(round(servo_pitch_middle))
		effcopy_pitch = 0.0
		
		servotimeoutval = 5   # of servo process cycles before flying neutral ~ 1 sec
		servotimeout = servotimeoutval
		sonarcountdown = 3    # sonar ping interval (in multiples of the servo loop)
		try:
			bus.write_byte (0x50, 81)   # trigger a sonar reading
		except:
			print "I2C bus error : Bad I2C Write Attempt"
		while RunThreadsFlag:
			if GPIO.input(22):
				airborne = True  # triggers the airborne flag for the servo loop
			nowtime = time.time() * 1000
			while abs(nowtime - lastupdate) < 40.0:  # minimum 40 ms between updates
				nowtime = time.time()*1000 # milliseconds
				time.sleep(0.001)
			# sample the sonar every other cycle of the servo loop
			sonarcountdown -= 1
			if sonarcountdown == 0:
				try:
					val = bus.read_i2c_block_data (0x50, 0, 2)   # read sonar value (highbyte, lowbyte)
				except:
					print 'I2C bus error : Bad I2C READ attempt.'
					val = (2, 253)
				cval = val[0]*256 + val[1]
				sonar_reading = cval
				if NetEnable and ((cval >= 20) and (cval < sonar_reading_threshold) and trackp and not netfired):  # need to add 'and airborne'
					# fixed action pattern : pitch up and fire net ======= < < ==========================
					print 'Rising to Attack'
					pwm.set_pwm (2, 0, VehPitchMiddle)
					logtime = int(round(time.time()*1000))
                                	filestring = '%d %d Rising to Attack - Sonar Value: %d\n'%(framecounter, logtime, cval)
                                	servo_queue.put (filestring)
					time.sleep (1)
					print 'Pitching to Fire...'
					pwm.set_pwm (0, 0, 185) # camera down as far as possible
					pwm.set_pwm (1, 0, VehRollMiddle) # set roll flat
					pwm.set_pwm (2, 0, VehPitchAttack) # ATTACK PITCH
					pwm.set_pwm (3, 0, VehYawMiddle) # stop and yaw
					logtime = int(round(time.time()*1000))
                                	filestring = '%d %d Net Launch Maneuver ! - Pitch used: %d\n'%(framecounter, logtime, VehPitchAttack)
                                	servo_queue.put (filestring)
					time.sleep (0.2) # 1 second
					print 'Firing Net!'
					pwm.set_pwm (4, 0, netlaunch_fire) # will take some time
					time.sleep (1.0)  # hold for 1 second
					netfired = True
					pwm.set_pwm (2, 0, VehPitchMiddle) # stop with flat pitch
					#pwm.set_pwm (4, 0, netlaunch_safe) # return the netlaunch servo to safe position
					logtime = int(round(time.time()*1000))
                                        filestring = '%d %d Finished Maneuver - Going to Hover Position.\n'%(framecounter, logtime)
                                        servo_queue.put (filestring)
					print 'Going to Flat Trim...  Switch to Manual and Fly Home!\n'
				try:
					bus.write_byte (0x50, 81)   # trigger new reading
				except:
					print "i2c bus error" 
				logtime = int(round(time.time()*1000))
                                filestring = '%d %d Sonar: %d\n'%(framecounter, logtime, cval)
                                servo_queue.put (filestring)
				sonarcountdown = 2  # reset the counter  (2 = ~100 ms)
			
			# OK, main loop
			if ((trackp == 1) or (trackingtimeout > 0)) and not netfired:
				servotimeout = servotimeoutval

				# pitch servo section
				dservo_pitch = dcy*Kp + dcy_vel*Kd  # was previously set to 3.9 for the model servos
                		servo_pitch_f += dservo_pitch
				effcopy_pitch += K_servo_2_cam_pitch * dservo_pitch
				# print effcopy_pitch (efference copy change in pitch servo)
				if (effcopy_pitch > 1.0) and not servo_freeze:  # compensation for servo movement
					est_y += np.floor(effcopy_pitch)
					effcopy_pitch = 0
				if (effcopy_pitch < -1.0) and not servo_freeze: # compensation for servo movement
					est_y -= np.ceil(effcopy_pitch)
					effcopy_pitch = 0
				servo_pitch = int(round(servo_pitch_f))
				if (servo_pitch > servo_pitch_max):  # check limits
					servo_pitch_f = servo_pitch_max
					servo_pitch = int(round(servo_pitch_f))
				if (servo_pitch < servo_pitch_min):
					servo_pitch_f = servo_pitch_min
					servo_pitch = int(round(servo_pitch_f))
				pwm.set_pwm(0,0,servo_pitch)   # is the Y microservo

				# Vehicular Yaw Section
        	                # servo angle = pi * (servo_pitch - 185) / 180 / 2
				servo_angle = 0.008726 * (servo_pitch - 185)  # 180 servo steps = 90 degrees
				roll_yaw_mix = servo_angle / 1.57	# pi/2			
				#print 'servo angle (radians) = %f'%servo_angle
				vehyaw_f = VehYawMiddle - roll_yaw_mix*(dcx*yawKp_veh + dcx_vel_avg*yawKd_veh)
				if vehyaw > VehYawMax:
					vehyaw_f = VehYawMax
				if vehyaw < VehYawMin:
					vehyaw_f = VehYawMin
				vehyaw = int(round(vehyaw_f))
                                pwm.set_pwm (3, 0, vehyaw)

				# Vehicular ROLL
				if airborne: # in flight, we roll
					vehroll_f = VehRollMiddle - (1 - roll_yaw_mix)*(dcx*rollKp_veh + dcx_vel_avg*rollKd_veh)
					if vehroll > VehRollMax:
						vehroll_f = VehRollMax
					if vehroll < VehRollMin:
						vehroll_f = VehRollMin
	                                vehroll = int(round(vehroll_f))
					pwm.set_pwm (1, 0, vehroll)
				else: # if on the ground, use zero roll
					vehroll_f = VehRollMiddle
					vehroll = int(round(vehroll_f))
					pwm.set_pwm (1, 0, vehroll)

				# Vehicular PITCH for Pursuit
				if airborne: # in flight, we pitch
					vehpitch_f = VehPitchMiddle - ((servo_pitch-300)+(dcy*Cpitch))*pitchKp_veh
					#print dcy*Cpitch	
					if vehpitch_f > VehPitchMax:
						vehpitch_f = VehPitchMax
					if vehpitch_f < VehPitchMin:
						vehpitch_f = VehPitchMin
					vehpitch = int(round(vehpitch_f))
					pwm.set_pwm (2, 0, vehpitch)
				else: # if on the ground, use zero pitch
					vehpitch_f = VehPitchMiddle
					vehpitch = int(round(vehpitch_f))
					pwm.set_pwm (2, 0, vehpitch)

				logtime = int(round(time.time()*1000))
				filestring = '%d %d Vehicle Control (SRPY) %d, %d, %d, %d\n'%(framecounter, logtime, servo_pitch, vehroll, vehpitch, vehyaw)
				servo_queue.put (filestring)

			else:   # if we're not tracking
				servotimeout -= 1
				if servotimeout < 1:
					if netfired:
						print 'Net was fired, now hovering ---------------'
					else:
						print 'Lost tracking., Servo Position to Middle'
					servo_pitch_f = servo_pitch_middle
					servo_pitch = int(round(servo_pitch_middle))
					pwm.set_pwm(0, 0, servo_pitch)
					vehroll_f = VehRollMiddle
					vehroll = int(round(vehroll_f))
					pwm.set_pwm (1, 0, vehroll)
					vehpitch_f = VehPitchMiddle
                                        vehpitch = int(round(vehpitch_f))
					pwm.set_pwm (2, 0, vehpitch)
					vehyaw_f = VehYawMiddle
					vehyaw = int(round(vehyaw_f))
					pwm.set_pwm (3, 0, vehyaw)
					servotimeout = servotimeoutval
                    			logtime = int(round(time.time()*1000))
					if netfired:
						filestring = '%d %d Net was fired. Now hovering (SRPY) %d, %d, %d, %d\n'%(framecounter, logtime, servo_pitch, vehroll, vehpitch, vehyaw)
					else:
                    				filestring = '%d %d Vehicle Control RESET (SRPY) %d, %d, %d, %d\n'%(framecounter, logtime, servo_pitch, vehroll, vehpitch, vehyaw)
                    			servo_queue.put (filestring)
                    			time.sleep (0.05)

			lastupdate = time.time() * 1000

	def stop(self):
		self._stop.set()

	def stopped(self):
		return self._stop.isSet()

# ServoP is defined earlier above
if ServoP:
	servothread = AsyncServoControl()
	servothread.daemon = True
	servothread.start()
	print "Servo Thread Start..."
    
# ========== end of the servo control thread =========

centsurr = np.load('centsurr_kernel.npy')
print 'Loaded filter kernel file: centsurr_kernel.npy'
centsurr2 = np.load('centsurr2_kernel.npy')
print 'Loaded filter kernel file: centsurr2_kernel.npy'

blobkernel = np.load('sharp_centsurr3.npy')
print 'Loaded filter kernel file: sharp_centsurr3.npy'

# ============== DATA LOGGING PROCESS =============
image_queue = Queue()
image_label_queue = Queue()
text_queue = Queue()
servo_queue = Queue()

def data_logger(image_queue, image_label_queue, text_queue, servo_queue):
	global CALIB, DecloudP, versionstring
	print versionstring
	print 'Initializing Data Logger Process: Data stored in datalog2.txt'
	f = open('datalog2.txt','a')
	tmp = time.time()

	f.write (versionstring)
	f.write(time.ctime(tmp)+'\n')

	f.write('Format: time, frame, dcx, dcy, dcx_vel_avg, dcy_vel_avg, dcx_acc_avg, dcy_acc_avg, trackp, autop\n')
	f.write('Timestamps for tracking reset notifications are at\n')
	f.write('the time of logging, not occurrence.\n')
	if DecloudP == 1:
		f.write ('De-clouding algorithm running.\n')
	f.write(str(tmp)+': Initiating Data Logging...\n')
	f.close()
	#frame = 0
	while True:
		while (not text_queue.empty()) or (not servo_queue.empty()):
			f = open('datalog2.txt','a')
			if not text_queue.empty():
				filestring = text_queue.get()
			else:
				filestring = servo_queue.get()
			f.write(filestring)
			#print 'saved some text'
			f.close()
		while not image_queue.empty():
			#frame += 1
			#print 'datalogger(%d): image saved.'%(frame)
			#imgtime = int(round(time.time()*1000))
			imgtime = image_label_queue.get()
			saveimg = image_queue.get()
			fname = 'tmp%03d.png'%(imgtime)
			cv2.imwrite (fname, saveimg)
# go ahead and start up the logger
if LoggingP == True:
    datalogger_process = Process(target=data_logger, args=(image_queue, image_label_queue, text_queue, servo_queue))
    datalogger_process.start()    
                                 
# ============= CAMERA PROCESS ===============
queue_from_cam = Queue(maxsize = 1)

def cam_loop(queue_from_cam, DeCloudP):
    print 'Initializing PiCamera'
    with PiCamera() as camera:
        camera.resolution = (resolution_x, resolution_y)
	camera.framerate = 90  # was 30
	time.sleep(1)
        rawCapture = PiRGBArray(camera, size = (resolution_x, resolution_y))
	for frame in camera.capture_continuous (rawCapture, format='bgr', use_video_port = True):
            #blu, grn, red = cv2.split(frame.array)
            bgr_image = frame.array
            #cv2.imshow ('test', bgr_image)
            #cv2.waitKey(1)
            queue_from_cam.put (bgr_image)
	    rawCapture.truncate(0)
	    # print 'another frame in the queue.'

cam_process = Process(target=cam_loop, args=(queue_from_cam, DecloudP))
cam_process.start()

# =========== END:   CAMERA PROCESS ============
def ROI_find_a_blob (img1, est_x, est_y, cent, blob, scale_of_interest):
	global trackp, tracksize, trackscale, pyrscale, bk_scale, cs_scale, blbmrg, tracksize0, resolution_x, resolution_y
	ax = est_x
	ay = est_y
	#limitsp = check_fix_limits() 
	#print 'scale of interest : %d'%scale_of_interest
	tmp_tracksize = tracksize0*np.power(1/pyrscale, scale_of_interest-1)
	if (ax - tmp_tracksize < 1):
		ax = tmp_tracksize
	if (ax + tmp_tracksize > resolution_x):
		ax = resolution_x - tmp_tracksize - 1
	if (ay - tmp_tracksize < 1):
		ay = tmp_tracksize
	if (ay + tmp_tracksize > resolution_y):
		ay = resolution_y - tmp_tracksize - 1
	ROIwidth = 2*tracksize0
	txa = int(round(ax - tmp_tracksize))  # setup the ROI corner coordinates
	txb = int(round(ax + tmp_tracksize))
	tya = int(round(ay - tmp_tracksize))
	tyb = int(round(ay + tmp_tracksize))
	tmp = gray[tya:tyb,txa:txb]
	dim = (ROIwidth, ROIwidth)  # all ROIs are made to this fixed size
	ROI_img = cv2.resize(tmp, dim, interpolation = cv2.INTER_AREA)
	ROIfilt = cv2.filter2D(ROI_img,-1,cs_scale*cent)       # contrast filter the ROI
	ROIfilt = np.maximum(0, ROIfilt-(0.1*np.amax(ROIfilt))) # remove low-level background clutter
	ROIblob = cv2.filter2D(ROIfilt,-1,bk_scale* blob)
	ROIwid = 2 * tracksize
	ROIblob [0:blbmrg, 0:ROIwid] = 0
	ROIblob [ROIwid-blbmrg:ROIwid, 0:ROIwid] = 0
	ROIblob [0:ROIwid,0:blbmrg] = 0
	ROIblob [0:ROIwid, ROIwid-blbmrg:ROIwid] = 0 
	ROIblob = np.maximum(0, ROIblob)  # clip out the negative values
	ROIblobsum = np.sum(ROIblob)
	return (ROIblob, ROIblobsum, ax, ay)

# ================== begin:    Find A Blob ======================
def find_a_blob (img1, cent, blob, scalelist, blbsumthrsh):
    global trackp, tracksize, pyrscale, bk_scale, cs_scale, blbmrg, tracksize0, trackscale
    #print 'Running:  Find a Blob.'
    max1 = 0
    max2 = 0
    max3 = 0
    max4 = 0
    max5 = 0
    max6 = 0
    height1, width1 = img1.shape
    scalefactor = 1
    
    if (1 in scalelist):
        txtr1 = cv2.filter2D(img1,-1,cs_scale*cent)
        txtr1 = np.maximum(0,txtr1-(0.1*np.amax(txtr1)))
        blob1 = cv2.filter2D(txtr1,-1,bk_scale* blobkernel)
        #cv2.imshow ('txtr1',txtr1)
        blob1 [0:blbmrg, 0:width1] = 0
        blob1 [height1-blbmrg:height1, 0:width1] = 0
        blob1 [0:height1,0:blbmrg] = 0
        blob1 [0:height1, width1-blbmrg:width1] = 0
        blob1 = np.maximum(0, blob1)
	sum1 = np.sum(blob1)
    
    if (2 in scalelist):
        scalefactor = pyrscale
        dim = (int(width1*scalefactor),int(height1*scalefactor))
	img2 = cv2.resize (img1, dim, interpolation = cv2.INTER_CUBIC)
        txtr2 = cv2.filter2D(img2,-1,cs_scale*cent)
        blob2 = cv2.filter2D(txtr2,-1,bk_scale* blobkernel)
        #cv2.imshow ('txtr2',txtr2)
        h2 = int(height1 * scalefactor)
        w2 = int(width1 * scalefactor)
        blob2 [0:blbmrg, 0:w2] = 0
        blob2 [h2-blbmrg:h2, 0:w2] = 0
        blob2 [0:h2,0:blbmrg] = 0
        blob2 [0:h2, w2-blbmrg:w2] = 0
	blob2 = np.maximum(0, blob2)
	sum2 = np.sum(blob2)
    
    if (3 in scalelist):
        scalefactor = pyrscale * pyrscale
        dim = (int(width1*scalefactor),int(height1*scalefactor))
	img3 = cv2.resize (img1, dim, interpolation = cv2.INTER_CUBIC)
        txtr3 = cv2.filter2D(img3,-1,cs_scale*cent)
        blob3 = cv2.filter2D(txtr3,-1,bk_scale* blobkernel)
        #cv2.imshow ('txtr3',txtr3)
        h3 = int(height1 * scalefactor)
        w3 = int(width1 * scalefactor)
        blob3 [0:blbmrg, 0:w3] = 0
        blob3 [h3-blbmrg:h3, 0:w3] = 0
        blob3 [0:h3,0:blbmrg] = 0
        blob3 [0:h3, w3-blbmrg:w3] = 0
	blob3 = np.maximum(0,blob3)
	sum3 = np.sum(blob3)
    
    if (4 in scalelist):
        scalefactor = pyrscale * pyrscale * pyrscale
        dim = (int(width1*scalefactor),int(height1*scalefactor))
	img4 = cv2.resize (img1, dim, interpolation = cv2.INTER_CUBIC)
        txtr4 = cv2.filter2D(img4,-1,cs_scale*cent)
        blob4 = cv2.filter2D(txtr4,-1,bk_scale* blobkernel)
        #cv2.imshow ('txtr4',txtr4)
        h4 = int(height1*scalefactor)
        w4 = int(width1 * scalefactor)
        blob4 [0:blbmrg, 0:w4] = 0
        blob4 [h4-blbmrg:h4, 0:w4] = 0
        blob4 [0:h4,0:blbmrg] = 0
        blob4 [0:h4, w4-blbmrg:w4] = 0
	blob4 = np.maximum(0,blob4)
	sum4 = np.sum(blob4)
    
    if (5 in scalelist):
        scalefactor = np.power(pyrscale, 4)
        dim = (int(width1*scalefactor),int(height1*scalefactor))
        img5 = cv2.resize (img1, dim, interpolation = cv2.INTER_CUBIC)
        txtr5 = cv2.filter2D(img5,-1,cs_scale*cent)
        blob5 = cv2.filter2D(txtr5,-1,bk_scale* blobkernel)
        #cv2.imshow ('img5',img5)
        h5 = int(height1*scalefactor)
        w5 = int(width1*scalefactor)
        blob5 [0:blbmrg, 0:w5] = 0
        blob5 [h5-blbmrg:h5, 0:w5] = 0
        blob5 [0:h5,0:blbmrg] = 0
        blob5 [0:h5, w5-blbmrg:w5] = 0
	blob5 = np.maximum(0,blob5)
	sum5 = np.sum(blob5)
	
    if (6 in scalelist):
        scalefactor = np.power(pyrscale, 5)
        dim = (int(width1*scalefactor),int(height1*scalefactor))
        img6 = cv2.resize (img1, dim, interpolation = cv2.INTER_CUBIC)
        txtr6 = cv2.filter2D(img6,-1,cs_scale*cent)
        blob6 = cv2.filter2D(txtr6,-1,bk_scale* blobkernel)
        #cv2.imshow ('img6',img6)
        h6 = int(height1*scalefactor)
        w6 = int(width1*scalefactor)
        blob6 [0:blbmrg, 0:w6] = 0
        blob6 [h6-blbmrg:h6, 0:w6] = 0
        blob6 [0:h6,0:blbmrg] = 0
        blob6 [0:h6, w6-blbmrg:w6] = 0
        #cv2.imshow ('blob6',blob6)
	blob6 = np.maximum(0,blob6)
	sum6 = np.sum(blob6)
    
    sumvec = (sum1, sum2, sum3, sum4, sum5, sum6)
    if max(sumvec) >= blbsumthrsh:
        max_scale = np.where(sumvec == max(sumvec))[-1][0]
        if max_scale == 0:
            pyrsc = 1.0
            trackscale = 1
            tracksize = tracksize0
	    M = cv2.moments(blob1 + 0.01)
            ntx = int (M['m10']/M['m00'])
            nty = int (M['m01']/M['m00'])
            cv2.rectangle(img1,(ntx-tracksize,nty-tracksize),(ntx+tracksize,nty+tracksize),(128,128,128),1)
        elif max_scale == 1:
            pyrsc = 1/pyrscale
            trackscale = 2
            tracksize = int(tracksize0 * pyrsc)
	    M = cv2.moments(blob2 + 0.01)
            ntx = int (pyrsc * M['m10']/M['m00'])
            nty = int (pyrsc * M['m01']/M['m00'])
            cv2.rectangle(img1,(ntx-tracksize,nty-tracksize),(ntx+tracksize,nty+tracksize),(128,128,128),1)
        elif max_scale == 2:
            pyrsc = 1/pyrscale/pyrscale
            trackscale = 3
            tracksize = int(tracksize0 * pyrsc)
	    M = cv2.moments(blob3 + 0.01)
            ntx = int (pyrsc * M['m10']/M['m00'])
            nty = int (pyrsc * M['m01']/M['m00'])
            cv2.rectangle(img1,(ntx-tracksize,nty-tracksize),(ntx+tracksize,nty+tracksize),(128,128,128),1)
        elif max_scale == 3:
            pyrsc = 1/pyrscale/pyrscale/pyrscale
            trackscale = 4
            tracksize = int(tracksize0 * pyrsc)
            M = cv2.moments(blob4 + 0.01)
            ntx = int (pyrsc * M['m10']/M['m00'])
            nty = int (pyrsc * M['m01']/M['m00'])
            cv2.rectangle(img1,(ntx-tracksize,nty-tracksize),(ntx+tracksize,nty+tracksize),(128,128,128),1)
        elif max_scale == 4:
            pyrsc = 1/pyrscale/pyrscale/pyrscale/pyrscale
            trackscale = 5
            tracksize = int(tracksize0 * pyrsc)
            M = cv2.moments(ROIblob + 0.01)
            ntx = int (pyrsc * M['m10']/M['m00'])
            nty = int (pyrsc * M['m01']/M['m00'])
            cv2.rectangle(img1,(ntx-tracksize,nty-tracksize),(ntx+tracksize,nty+tracksize),(128,128,128),1)
        elif max_scale == 5:
            pyrsc = 1/pyrscale/pyrscale/pyrscale/pyrscale/pyrscale
            trackscale = 6
            tracksize = int(tracksize0 * pyrsc)
	    M = cv2.moments(ROIblob + 0.01)
            ntx = int (pyrsc * M['m10']/M['m00'])
            nty = int (pyrsc * M['m01']/M['m00'])
            cv2.rectangle(img1,(ntx-tracksize,nty-tracksize),(ntx+tracksize,nty+tracksize),(128,128,128),1)
        else:
            print 'ACK!  Problems!'
     
        print 'ntx = %d, nty = %d, trackscale = %d'%(ntx,nty,trackscale)
        trackp = 1
    else:
        print 'No target detected.'
        ntx = width1 / 2  # jump to the center
        nty = height1 / 2
        pyrsc = 1.0
	trackp = 0
        
    #cv2.imshow('img1',img1)
    suggested_target = (ntx, nty, 0, 0)  # zero velocities
    return suggested_target
   
# =================== end: Find-a-Blob ==================
    
def move_trackbox (event, x, y, flags, param):
    global est_x, est_y, est_vx, est_vy
    if event == cv2.EVENT_LBUTTONDOWN:
        est_x = x
        est_y = y
        est_vx = 0
        est_vy = 0
        print 'Moving the box !'

def check_fix_limits ():
    global est_x, est_y, resolution_x, resolution_y, tracksize
    hitlimitsp = 0
    max_est_x = (resolution_x - tracksize) - 1
    min_est_x = tracksize + 1
    max_est_y = (resolution_y - tracksize) - 1
    min_est_y = tracksize + 1
    if (est_x >= max_est_x):  # check limits
        est_x = max_est_x
        hitlimitsp = 1    # means that we hit a limit
    if (est_x <= min_est_x):
        est_x = min_est_x
        hitlimitsp = 1
    if (est_y >= max_est_y):
        est_y = max_est_y
        hitlimitsp = 1
    if (est_y <= min_est_y):
        est_y = min_est_y
        hitlimitsp = 1  
    return hitlimitsp

def update_max_min(tracksize):
    global resolution_x, resolution_y, max_est_x, max_est_y, min_est_x, min_est_y
    max_est_x = (resolution_x - tracksize) - 1
    min_est_x = tracksize + 1
    max_est_y = (resolution_y - tracksize) - 1
    min_est_y = tracksize + 1

# ====================== Main Parameters ==================

# predicted position of tracked object
est_x = Xcenter   # center
est_y = Ycenter 
# update_max_min(tracksize)
est_vx = 0
est_vy = 0
new_x = 0
new_y = 0

originaltime = time.time()   # grab the starting time of the program
datime = originaltime

trackingtimeout = 0
tracksize0 = 18 # base ROI size
tracksize = tracksize0
trackscale = 1
ROIwidth = 2*tracksize0
checkROItimer = 10
checkROItimer_reset = 5
ROIsum_thresh = 600   # is this being used anywhere?
ROIfilt = np.zeros((2*tracksize,2*tracksize))
blobsumthresh = 50

check_fix_limits ()

# ======   Startup Services ======

airborne_main = False  # this is a triggered flag.  Once detected, airborne_main remains true.
trackingtimeout = 0  # this is the counter for how long to persist after we lose tracking (trackp)
trackingtimeout_reset = 20
trackp = 0
ROI_img = np.zeros((50,50))
ROIblob_up = np.zeros((50,50))
ROIblob_down = np.zeros((50,50))

cv2.imshow ('ROIfilt',ROIfilt)  # put up a display of ROIfilt early so that we can use the keyboard inputs.

# ================================== MAIN LOOP ===============      

while True:

        while queue_from_cam.empty():   # wait for a frame to arrive
                pass
        bgr_image = queue_from_cam.get()   # the frame buffer is only 1 frame deep

        #  Yeay! - a new image to work on.
        blu, grn, red = cv2.split(bgr_image)
        if DecloudP:
                invblu = blu
                #invblu = 255 - blu  # this inversion is used for tracking bright blue
                invblumax = np.amax(invblu)
                invblumin = np.amin(invblu)
                blutanhthresh = invblumin + 0.3*(invblumax - invblumin)
                tanhimg = 128 + 128*np.tanh(0.03*(invblu - blutanhthresh))
                gray = tanhimg.astype ('uint8')
        else:
                gray = blu   # this is used for tracking black against white
                #gray = 255 - blu   # this inversion is used for tracking bright blue
                
        lasttrackp = trackp  # hold the state of trackp from last cycle
        olddcx = dcx  # keeps state from last cycle
        olddcy = dcy
        olddcx_vel = dcx_vel
        olddcy_vel = dcy_vel
        framecounter = framecounter + 1
        newtime = time.time()
        if VerboseMode:
                print '\nFrame %d'%(framecounter)
        datime = newtime

        # if we start to lose tracking because (ROIblobsum < threshold),
        # we will begin checking at different scales using the existing ROI position.
        # if this fails, trackp ==> 0, but we start the trackingtimeout countdown and persist.
        # we keep looking at that ROI position, but at three different scales.
        
        # call find_a_target on startup, will set trackp as well as the est_x, etc.
        if (trackingtimeout < 1) and (trackp == 0):   # initiate broad search
                print '%d: calling Find_a_Blob (all scales) '%(framecounter)
                (est_x, est_y, est_vx, est_vy)=find_a_blob(gray, centsurr2, blobkernel, [1, 2, 3, 4, 5, 6], blobsumthresh)
	
        if (trackingtimeout > 0) or (trackp == 1):   # keep cycling if trackingtimeout > 0, otherwise that's it for the frame.
                if VerboseMode:
                        print 'Tracking Scale = %d'%trackscale
                # update where we think the target will be based on previous cycle
		est_x += est_vx  # est_vx in units of pixels
		est_y += est_vy
		# when tracksize is changed, we need to setup/change the edge limits
		#update_max_min(tracksize)
		limitsp = check_fix_limits() 

                (ROIblob, ROIblobsum, adjx, adjy) = ROI_find_a_blob(gray, est_x, est_y, centsurr2, blobkernel, trackscale)
		if ROIblobsum < blobsumthresh:    # Tracking Threshold
			print '%d: ROI blob too small (%f).'%(framecounter, ROIblobsum)
			trackingtimeout -= 1
			trackp = 0   # NOT tracking any more
			if trackingtimeout < 1: # time to give up
				print '%d: Giving up.  Time for a broad search.'%(framecounter)
				#(est_x, est_y, est_vx, est_vy) = find_a_blob(gray, centsurr2, blobkernel,[trackscale-1,trackscale,trackscale+1])
				#trackingtimeout = trackingtimeout_reset   # start the timeout counter
			else: # persisting... check +1 and -1 scales
				print trackingtimeout
                                #start hunting for it at different scales, but use the ROI, when we give up, we go full frame
				testsum_up = 0
				testsum_down = 0
				if trackscale < 6:
					(ROIblob_up, testsum_up, adjx_up, adjy_up) = ROI_find_a_blob (gray, est_x, est_y, centsurr2, blobkernel, trackscale + 1)
					#print 'testsum_up = %f'%testsum_up
				if trackscale > 1:
					(ROIblob_down, testsum_down, adjx_down, adjy_down) = ROI_find_a_blob (gray, est_x, est_y, centsurr2, blobkernel, trackscale - 1)
					#print 'testsum_down = %f'%testsum_down
				sumvec = (testsum_down, ROIblobsum, testsum_up)
				
				if max(sumvec) < blobsumthresh:  # if it's still too low, we have simply lost the target.
                                        print 'lost tracking, but persisting ...'
                                else:  # here, we have found the new scale to use
                                        trackp = 1  # trackp status restored
                                        max_scale = np.where(sumvec == max(sumvec))[-1][0]
                                        if max_scale == 0:
                                                trackscale -= 1
                                                tracksize = tracksize * pyrscale
                                                ROIblob = ROIblob_down
						est_x = adjx_down
						est_y = adjy_down
                                        if max_scale == 2:
                                                trackscale += 1
                                                tracksize = tracksize / pyrscale
                                                ROIblob = ROIblob_up
						est_x = adjx_up
						est_y = adjy_up
                			if (max_scale != 1):
                                                print 'Changing scale to : %d'%trackscale
		else: # if blobsum < blobsumthresh
			trackp = 1
				
		if trackp == 1:   # ROIblobsum IS big enough = WE'RE TRACKING
			#print 'ROIblobmax = %f'%(ROIblobmax)
			trackingtimeout = trackingtimeout_reset  # reset the timeout value
			M = cv2.moments(ROIblob)
			centroid_x = int (M['m10']/M['m00'])
			centroid_y = int (M['m01']/M['m00'])
			pyrsc = 1 / math.pow(pyrscale,trackscale-1)
			#print 'pyrsc = %f'%(pyrsc)
			ROIdx = pyrsc *(centroid_x - tracksize0)  # recall that the ROI is 2 tracksizes wide/high
			ROIdy = pyrsc * (centroid_y - tracksize0)  # delta_vx/y tell us how much the target has moved within the ROI 
			#est_vx += ROIdx    # updated the velocity estimate
			#est_vy += ROIdy 
			est_x += ROIdx    # corrected the position
			est_y += ROIdy
			limitsp = check_fix_limits()
			#print 'est_x = %d, est_y = %d'%(est_x,est_y)
			txa = int(round(est_x - tracksize))  # setup the ROI corner coordinates
			txb = int(round(est_x + tracksize))
			tya = int(round(est_y - tracksize))
			tyb = int(round(est_y + tracksize))

			if lasttrackp:  # if we were tracking before and the old dcx/dcy is accurate then use velocity
				dcx = est_x - Xcenter
				dcy = est_y - Ycenter
				dcx_vel = dcx - olddcx
				dcy_vel = dcy - olddcy
				dcx_vel_avg = 0.9*dcx_vel_avg + 0.1*dcx_vel
				dcy_vel_avg = 0.9*dcy_vel_avg + 0.1*dcy_vel
				dcx_acc = dcx_vel - olddcx_vel
				dcy_acc = dcy_vel - olddcy_vel
				dcx_acc_avg = 0.9*dcx_acc_avg + 0.1*dcx_acc
				dcy_acc_avg = 0.9*dcy_acc_avg + 0.1*dcy_acc
				# print 'dcx (vel) = %d (%d) and dcy (vel) = %d (%d)'%(dcx, dcx_vel, dcy, dcy_vel)	
			else:
				dcx = est_x - Xcenter    # for the servos
				dcy = est_y - Ycenter    # for the servos
				dcx_vel = 0
				dcy_vel = 0
				dcx_vel_avg = 0
				dcy_vel_avg = 0
				dcx_acc = 0
				dcy_acc = 0
				dcx_acc_avg = 0
				dcy_acc_avg = 0
				# dcx and dcy get shifted to olddcx and olddcy at the top of the loop
		            
		if VerboseMode:
			print 'trackp = %d'%trackp
			print "Distance to x center: ", (dcx)
			print "Distance to y center: ", (dcy)
            
	#saveimg = bgr_image
        #saveimg = gray
        if trackp == 1:
                cv2.rectangle(bgr_image, (Xcenter-5,Ycenter-5),(Xcenter+5,Ycenter+5),(128,128,128),1) # center box
                #cv2.line (saveimg,(Xcenter,Ycenter+5),(Xcenter,Ycenter-5),(0,0,0),1) # draw center crosshairs
                #cv2.line (saveimg,(Xcenter-5,Ycenter),(Xcenter+5,Ycenter),(0,0,0),1)
                
		#if lasttrackp:
                	#cv2.line (saveimg, (Xcenter, Ycenter),(Xcenter + int(5*dcx_vel_avg), Ycenter+int(5*dcy_vel_avg)),(255,255,255),2)
                    	#cv2.line (saveimg, (50, Ycenter),(50 + int(20*dcx_acc_avg), Ycenter+int(20*dcy_acc_avg)),(0,255,255),2)
                cv2.rectangle(bgr_image,(txa,tya),(txb,tyb),(255,255,255),2)
        else:
                txa = int(round(est_x - tracksize))  # setup the ROI corner coordinates
                txb = int(round(est_x + tracksize))
                tya = int(round(est_y - tracksize))
                tyb = int(round(est_y + tracksize))
                cv2.rectangle(bgr_image,(txa,tya),(txb,tyb),(128,128,128),1)   
        
	if trackp:   # one time trigger for MovieP
		MovieP = True
        if GPIO.input (22):  # one time trigger for airborne_main flag
                airborne_main = True
        if airborne_main:  # log file indicator for being airborne
                autop = 1
                cv2.line (bgr_image,(300,200),(300,220),(0, 0, 255),2)  # put a red line indicator in the stored image.
        else:
                autop = 0

        saveimg = bgr_image[::-1,::-1,:].copy()
        
        if (sonar_reading < 500):
		#print 'SonarPing Detected < 5 meters'
                textstr = 'sonar (%d)'%sonar_reading
                cv2.putText (saveimg,textstr,(100,230),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255))
                #cv2.line (saveimg,(310,200),(310,220),(255, 255, 0),2)  # put a line if we have a sonar reading
		sonar_reading = 765 # clear the stored sonar value
		
        imagetimelabel = int(round(time.time()*1000))        
        if MovieP:
                #if framecounter % 2 == 0:
                #imagetimelabel = int(round(time.time()*1000))
                image_label_queue.put (imagetimelabel)
                image_queue.put (saveimg)   # send image to be saved.

        if LoggingP:
                filestring = '%d %d Tracking (%d): %5.1f, %5.1f, %5.2f, %5.2f, %5.2f, %5.2f, %d, %d\n'%(framecounter, imagetimelabel, trackscale, dcx, dcy, dcx_vel_avg, dcy_vel_avg, dcx_acc_avg, dcy_acc_avg, trackp, autop)
                text_queue.put (filestring)
            
        if ShowGraphics:
                cv2.imshow('gray',saveimg2)
                #cv2.imshow('ROI_img',ROI_img)
                cv2.imshow('ROIfilt',ROIfilt)
                cv2.imshow('ROIblob',ROIblob)
        else:
                cv2.imshow ('ROIfilt',ROIfilt)
                cv2.imshow ('ROIblob',ROIblob)   # This is the only display in ShowGraphics = False

        # =====================================
        k = cv2.waitKey(1)
        if ((k == 27) or (k==113)):   # if key = ESC or q
                break
    
        if ((k == 109) or (k == 77)):   # m or M  to toggle 'movie' function
                if MovieP:
                        MovieP = False
                        print 'O O O O O O O O O O O O O O O O O O -- Image Logging OFF'
                else:
                        MovieP = True
                        print '1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1  -- Image Logging ON'

        #k = cv2.waitKey()

# ========= end of main loop ==========

avgFPS = framecounter / (newtime - originaltime)
print '\nAverage FPS was %f frames per second'%(avgFPS)
if LoggingP:
    filestring = 'Average FPS was %f frames per second'%(avgFPS)
    text_queue.put (filestring)


cv2.destroyAllWindows()
print 'closing windows'
cv2.waitKey(1)

# stop other processes
RunThreadsFlag = False
cam_process.terminate()
print 'Waiting for camera process to terminate... '
cam_process.join()
print 'Terminated.    (camera process)'

if LoggingP == True:
    datalogger_process.terminate()
    datalogger_process.join()
    print 'Terminated.    (datalogger process)'

GPIO.cleanup ()
print '\nEverything appears to have closed gracefully.'


