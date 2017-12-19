#DEBUG comments on lines 3, change file name on 6

#import Adafruit_PCA9685
import threading
import numpy as np
import time
import cuav_run as ce
debug=1
if not(debug):
	import RPi.GPIO as GPIO

class AsyncServoControl (threading.Thread):
    	def __init__(self):
		global pwm
		super(AsyncServoControl, self).__init__()
		# Initialise the PWM device using the default address
		if not(debug):
			pwm = Adafruit_PCA9685.PCA9685()
			pwm.set_pwm_freq(44)      # Set frequency to 44, was initially 60 Hz

		VehRollMiddle = 301
		VehPitchMiddle = 299
		VehYawMiddle = 301	# 300 (@ 44Hz pwm update rate) is roughly center stick 
		servo_pitch_middle = 300      # 300 points the camera mostly up
		netlaunch_safe = 200

		if not(debug):
			pwm.set_pwm_freq(44)      # Set frequency to 44, was initially 60 Hz
			pwm.set_pwm(0, 0, servo_pitch_middle)
			pwm.set_pwm(1, 0, VehRollMiddle)
			pwm.set_pwm(2, 0, VehPitchMiddle)
			pwm.set_pwm(3, 0, VehYawMiddle)
			pwm.set_pwm(4, 0, netlaunch_safe)

		print 'Initial values sent to the PWM board.'
		if ce.BeeperP and not(debug):
			for qq in range(3):
				GPIO.output (23, 1)
				time.sleep (0.05)
				GPIO.output (23, 0)
				time.sleep (0.1)
		
		self._stop = threading.Event()
		
	def run(self):
		#global sonar_reading, servo_queue, pwm, est_x, est_y, dcx, dcy, dcx_vel_avg,dcx_acc_avg
		#global dcy_vel_avg,dcy_acc_avg, trackp, trackingtimeout, RunThreadsFlag, VerboseMode
		#global framecounter, sonar_reading_threshold, NetEnable

		global pwm	#the rest of the variables are gotten in the loop for updating every loop
		RunThreadsFlag=ce.RunThreadsFlag#needed to start loop
		
		servo_freeze = True   # don't try to predict for servo movement
		maxstep = 50#unused
		sonar_reading = 765 # just initialize this to max    unused/already done   
		airborne = False # once set, this will not be reset to False
		netfired = False # once set, this will not be reset to False
		lastupdate = time.time() * 1000  # time given in seconds, convert to milliseconds 
		Kp = 0.25 #-0.1   # proportional gain for the servos
		Kd = 0.2  #-0.1  # derivative gain
		Ka = 1.0  # acceleration gain unused
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
		servo_pitch_min = 185 # 185 is vertical and camera points front 
		servo_pitch_middle = 300      # 
		servo_pitch_max = 365 # 365 is horizontal and camera points up
		netlaunch_safe = 200  # make sure this value always matches the one in the init section above. unused/already done
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
		UseSonar=ce.UseSonar	#get usesonar from main file
		if UseSonar and not(debug):
			try:
				bus.write_byte (0x50, 81)   # trigger a sonar reading
			except:
				print "I2C bus error : Bad I2C Write Attempt"
		while RunThreadsFlag:
    		#get the needed variables, ce is the main file where the variables scope is
			#unused:est_x, dcx_acc_avg,dcy_acc_avg, VerboseMode
			sonar_reading=ce.sonar_reading; servo_queue=ce.servo_queue; est_x=ce.est_x; est_y=ce.est_y
			dcx=ce.dcx; dcy=ce.dcy; dcx_vel_avg=ce.dcx_vel_avg;dcx_acc_avg=ce.dcx_acc_avg
			dcy_vel_avg=ce.dcy_vel_avg; dcy_acc_avg=ce.dcy_acc_avg; trackp=ce.trackp 
			trackingtimeout=ce.trackingtimeout; RunThreadsFlag=ce.RunThreadsFlag; VerboseMode=ce.VerboseMode
			framecounter=ce.framecounter; sonar_reading_threshold=ce.sonar_reading_threshold
			NetEnable=ce.NetEnable	

			if not(debug):
				if GPIO.input(22):
					airborne = True  # triggers the airborne flag for the servo loop
			nowtime = time.time() * 1000
			while abs(nowtime - lastupdate) < 40.0:  # minimum 40 ms between updates
				nowtime = time.time()*1000 # milliseconds
				time.sleep(0.001)
			# sample the sonar every other cycle of the servo loop
			sonarcountdown -= 1
			if (sonarcountdown == 0) and UseSonar and not(debug):
				try:
					val = bus.read_i2c_block_data (0x50, 0, 2)   # read sonar value (highbyte, lowbyte)
				except:
					print 'I2C bus error : Bad I2C READ attempt.'
					val = (2, 253)
				cval = val[0]*256 + val[1]
				sonar_reading = cval
				if NetEnable and airborne and ((cval >= 20) and (cval < sonar_reading_threshold) and trackp and not netfired) and not(debug):  # need to add 'and airborne'
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
				if not(debug):
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
				if not(debug):
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
				if not(debug):
					pwm.set_pwm (3, 0, vehyaw)

				# Vehicular ROLL
				if airborne: # in flight, we roll
					vehroll_f = VehRollMiddle - (1 - roll_yaw_mix)*(dcx*rollKp_veh + dcx_vel_avg*rollKd_veh)
					if vehroll > VehRollMax:
						vehroll_f = VehRollMax
					if vehroll < VehRollMin:
						vehroll_f = VehRollMin
	                                vehroll = int(round(vehroll_f))
					if not(debug):
						pwm.set_pwm (1, 0, vehroll)
				else: # if on the ground, use zero roll
					vehroll_f = VehRollMiddle
					vehroll = int(round(vehroll_f))
					if not(debug):
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
					if not(debug):
						pwm.set_pwm (2, 0, vehpitch)
				else: # if on the ground, use zero pitch
					vehpitch_f = VehPitchMiddle
					vehpitch = int(round(vehpitch_f))
					if not(debug):
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
					vehroll_f = VehRollMiddle
					vehroll = int(round(vehroll_f))
					vehpitch_f = VehPitchMiddle
					vehpitch = int(round(vehpitch_f))
					vehyaw_f = VehYawMiddle
					vehyaw = int(round(vehyaw_f))
					
					if not(debug):
						pwm.set_pwm(0, 0, servo_pitch)
						pwm.set_pwm (1, 0, vehroll)
						pwm.set_pwm (2, 0, vehpitch)
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
			#write any variables other processes need
			ce.sonar_reading=sonar_reading, ce.servo_queue=servo_queue, ce.est_y=est_y,
	def stop(self):
		self._stop.set()

	def stopped(self):
		return self._stop.isSet()


    
