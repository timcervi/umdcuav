import startup as vars

def ROI_find_a_blob (img1, est_x, est_y, cent, blob, scale_of_interest):
#	global trackp, tracksize, trackscale, pyrscale, bk_scale, cs_scale, blbmrg, tracksize0, resolution_x, resolution_y
    trackp=vars.trackp; tracksize=vars.tracksize; trackscale=vars.trackscale; pyrscale=vars.pyrscale
    bk_scale=vars.bkscale; cs_scale=vars.cs_scale; blbmrg=vars.blbmrg; tracksize0=vars.tracksize0
    resolution_x=vars.resolution_x; resolution_y=vars.resolution_y

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
    #global trackp, tracksize, pyrscale, bk_scale, cs_scale, blbmrg, tracksize0, trackscale
    trackp=vars.trackp; tracksize=vars.tracksize; pyrscale=vars.pyrscale; bk_scale=vars.bk_scale
    cs_scale=vars.cs_scale; blbmrg=vars.blbmrg; tracksize0=vars.tracksize0; trackscale=vars.trackscale

    #print 'Running:  Find a Blob.'
    max1 = 0   #maxes are unused
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

    #set variables back to main file
    vars.trackscale=trackscale; vars.trackp=trackp; vars.tracksize=tracksize; vars.pyrscale=pyrscale;
    #cv2.imshow('img1',img1)
    suggested_target = (ntx, nty, 0, 0)  # zero velocities
    return suggested_target
   
# =================== end: Find-a-Blob ==================
    
def move_trackbox (event, x, y, flags, param):#unused
    #global est_x, est_y, est_vx, est_vy
    est_x=vars.est_x; est_y=vars.est_y; 
    
    if event == cv2.EVENT_LBUTTONDOWN:
        est_x = x
        est_y = y
        vars.est_vx = 0     #set these in main file (only time referenced in this fxn)
        vars.est_vy = 0
        print 'Moving the box !'

def check_fix_limits (est_x, est_y, tracksize):
   # global est_x, est_y, resolution_x, resolution_y, tracksize
    resolution_x=vars.resolution_x; resolution_y=vars.resolution_y; 

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

    return (est_x, est_y, hitlimitsp)

#def update_max_min(tracksize):#unused? all commented out
#    global resolution_x, resolution_y, max_est_x, max_est_y, min_est_x, min_est_y
#    max_est_x = (resolution_x - tracksize) - 1
#    min_est_x = tracksize + 1
#    max_est_y = (resolution_y - tracksize) - 1
#    min_est_y = tracksize + 1
