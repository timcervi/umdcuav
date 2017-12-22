def mainLoop():
	# ====================== Main Parameters ==================
	import time
	import cv2
	import math
	import numpy as np
	import RPi.GPIO as GPIO
	from multiprocessing import Queue
	import blobFuncs    # our module with our functions
	import startup as vars
	
	originaltime = time.time()   # grab the starting time of the program
	datime = originaltime

        #get universal variables
	tracksize0 = vars.tracksize0; tracksize = vars.tracksize; trackscale = vars.trackscale
        est_x=vars.est_x; est_y=vars.est_y
        
	ROIwidth = 2*tracksize0
	checkROItimer = 10  #unused
	checkROItimer_reset = 5  #unused
	ROIsum_thresh = 600   # is this being used anywhere? unused
	ROIfilt = np.zeros((2*tracksize,2*tracksize))
	blobsumthresh = 50
        ROIblob=np.zeros((50,50))
        
	blobFuncs.check_fix_limits (est_x, est_y, tracksize)

	# ======   Startup Services ======

	airborne_main = False  # this is a triggered flag.  Once detected, airborne_main remains true.
	trackingtimeout = 0  # this is the counter for how long to persist after we lose tracking (trackp)
	trackingtimeout_reset = 20
	trackp = 0
	ROI_img = np.zeros((50,50))
	ROIblob_up = np.zeros((50,50))
	ROIblob_down = np.zeros((50,50))
	framecounter = 0
	centsurr=vars.centsurr
	centsurr2=vars.centsurr2
	blobkernel=vars.blobkernel

	cv2.imshow ('ROIfilt',ROIfilt)  # put up a display of ROIfilt early so that we can use the keyboard inputs.

	# ================================== MAIN LOOP ===============      

	while True:
		#get used variables from the main file
		tracksize=vars.tracksize; trackscale=vars.trackscale; trackp=vars.trackp; 
		VerboseMode=vars.VerboseMode; MovieP=vars.MovieP; LoggingP=vars.LoggingP; ShowGraphics=vars.ShowGraphics
		DecloudP=vars.DecloudP; sonar_reading=vars.sonar_reading
                dcx=vars.dcx; dcy=vars.dcy; dcx_vel=vars.dcx_vel; dcy_vel=vars.dcy_vel
                dcx_vel_avg=vars.dcx_vel_avg; dcy_vel_avg=vars.dcy_vel_avg;
                dcx_acc_avg=vars.dcx_acc_avg; dcy_acc_avg=vars.dcy_acc_avg
		while vars.queue_from_cam.empty():   # wait for a frame to arrive
			pass
		bgr_image = vars.queue_from_cam.get()   # the frame buffer is only 1 frame deep

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
			print('\nFrame %d'%(framecounter))
		datime = newtime

		# if we start to lose tracking because (ROIblobsum < threshold),
		# we will begin checking at different scales using the existing ROI position.
		# if this fails, trackp ==> 0, but we start the trackingtimeout countdown and persist.
		# we keep looking at that ROI position, but at three different scales.
			
		# call find_a_target on startup, will set trackp as well as the est_x, etc.
		if (trackingtimeout < 1) and (trackp == 0):   # initiate broad search
			print('%d: calling Find_a_Blob (all scales) '%(framecounter))
			(est_x, est_y, est_vx, est_vy)=blobFuncs.find_a_blob(gray, centsurr2, blobkernel, [1, 2, 3, 4, 5, 6], blobsumthresh)
		
		if (trackingtimeout > 0) or (trackp == 1):   # keep cycling if trackingtimeout > 0, otherwise that's it for the frame.
			if VerboseMode:
				print('Tracking Scale = %d'%trackscale)
			# update where we think the target will be based on previous cycle
			est_x += est_vx  # est_vx in units of pixels
			est_y += est_vy
			# when tracksize is changed, we need to setup/change the edge limits
			#update_max_min(tracksize)
			(est_x, est_y, limitsp) = blobFuncs.check_fix_limits(est_x, est_y, tracksize) 

			(ROIblob, ROIblobsum, adjx, adjy) = blobFuncs.ROI_find_a_blob(gray, est_x, est_y, centsurr2, blobkernel, trackscale)
			if ROIblobsum < blobsumthresh:    # Tracking Threshold
				print('%d: ROI blob too small (%f).'%(framecounter, ROIblobsum))
				trackingtimeout -= 1
				trackp = 0   # NOT tracking any more
				if trackingtimeout < 1: # time to give up
					print('%d: Giving up.  Time for a broad search.'%(framecounter))
					#(est_x, est_y, est_vx, est_vy) = find_a_blob(gray, centsurr2, blobkernel,[trackscale-1,trackscale,trackscale+1])
					#trackingtimeout = trackingtimeout_reset   # start the timeout counter
				else: # persisting... check +1 and -1 scales
					print(trackingtimeout)
					#start hunting for it at different scales, but use the ROI, when we give up, we go full frame
					testsum_up = 0
					testsum_down = 0
					if trackscale < 6:
						(ROIblob_up, testsum_up, adjx_up, adjy_up) = blobFuncs.ROI_find_a_blob (gray, est_x, est_y, centsurr2, blobkernel, trackscale + 1)
						#print 'testsum_up = %f'%testsum_up
					if trackscale > 1:
						(ROIblob_down, testsum_down, adjx_down, adjy_down) = blobFuncs.ROI_find_a_blob (gray, est_x, est_y, centsurr2, blobkernel, trackscale - 1)
					#print 'testsum_down = %f'%testsum_down
					sumvec = (testsum_down, ROIblobsum, testsum_up)
					
					if max(sumvec) < blobsumthresh:  # if it's still too low, we have simply lost the target.
						print('lost tracking, but persisting ...')
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
							print('Changing scale to : %d'%trackscale)
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
				(est_x, est_y, limitsp) = blobFuncs.check_fix_limits(est_x, est_y, tracksize)
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
			vars.image_label_queue.put (imagetimelabel)
			vars.image_queue.put (saveimg)   # send image to be saved.

		if LoggingP:
			filestring = '%d %d Tracking (%d): %5.1f, %5.1f, %5.2f, %5.2f, %5.2f, %5.2f, %d, %d\n'%(framecounter, imagetimelabel, trackscale, dcx, dcy, dcx_vel_avg, dcy_vel_avg, dcx_acc_avg, dcy_acc_avg, trackp, autop)
			vars.text_queue.put (filestring)
		
		if ShowGraphics:
			cv2.imshow('gray',saveimg)
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
		#set variables back to main file for use by other processes
		vars.trackp=trackp; vars.trackingtimeout=trackingtimeout; 
		vars.trackscale=trackscale; vars.tracksize=tracksize
		vars.est_x=est_x; vars.est_y=est_y; vars.dcy=dcy; vars.dcx=dcx; 
		vars.dcx_vel=dcx_vel; vars.dcy_vel=dcy_vel; vars.dcx_vel_avg=dcx_vel_avg; 
		vars.dcy_vel_avg=dcy_vel_avg; vars.dcx_acc_avg=dcx_acc_avg;
		vars.dcy_acc_avg=dcy_acc_avg;
		vars.sonar_reading=sonar_reading; vars.newtime=newtime; vars.originaltime=originaltime
		vars.framecounter=framecounter
		# ========= end of main loop ==========
