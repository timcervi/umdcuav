import numpy as np
import cv2
import math
import time

#options
LOGIMG = False #Log compression
GAUSSFILTERING = True #Whether to use the fourier transform of the gaussian to low pass filter the template
FILESOURCE = False

IMGWIDTH = 640
CNTRX = int(IMGWIDTH / 2)
IMGHEIGHT = 480
CNTRY = int(IMGHEIGHT / 2)
HALFBOX = 64
LFMASKWID = 2
HFMASKWID = 20

trackx = CNTRX
tracky = CNTRY

firstframe = True
#cv2.imwrite('testimg4.png',image)

# routine for handling the mouse click events
def click_trackbox (event, x, y, flags, param):
    global trackx, tracky, firstframe, HALFBOX
    if event == cv2.EVENT_LBUTTONDOWN:
        trackx = sorted((HALFBOX, x, IMGWIDTH - HALFBOX))[1]
        tracky = sorted((HALFBOX, y, IMGHEIGHT - HALFBOX - 1))[1]
        
        print('New trackx = %d, tracky = %d'%(trackx, tracky))
        firstframe = True
        print('moved the box.')

GAUSSMASK = np.zeros((2*HALFBOX,2*HALFBOX))
SIG = 2.5
for x in range(HALFBOX):
    for y in range(HALFBOX):
        g = math.exp(-(x*x+y*y)/SIG/SIG)
        GAUSSMASK[HALFBOX-y,HALFBOX-x] = g
        GAUSSMASK[HALFBOX+y,HALFBOX-x] = g
        GAUSSMASK[HALFBOX-y,HALFBOX+x] = g
        GAUSSMASK[HALFBOX+y,HALFBOX+x] = g

gauss_dft = cv2.dft(np.float32(GAUSSMASK),flags = cv2.DFT_COMPLEX_OUTPUT)


# Look into using a Gaussian (maybe squared?) to avoid contrast around edges of image
COSMASK = np.zeros((2*HALFBOX,2*HALFBOX))
for x in range(HALFBOX):
    for y in range(HALFBOX):
        r = math.sqrt(x*x+y*y)
        COSMASK[HALFBOX-y,HALFBOX-x] = math.cos(math.pi*r/HALFBOX/3)
        COSMASK[HALFBOX+y,HALFBOX-x] = math.cos(math.pi*r/HALFBOX/3)
        COSMASK[HALFBOX-y,HALFBOX+x] = math.cos(math.pi*r/HALFBOX/3)
        COSMASK[HALFBOX+y,HALFBOX+x] = math.cos(math.pi*r/HALFBOX/3)

# Grab the initial frame as the target
if FILESOURCE:
    vidcap = cv2.VideoCapture('IMG_6994_timcervi.mov')
    success, tmpimg = vidcap.read()
    img = cv2.resize (tmpimg, (IMGWIDTH,IMGHEIGHT), interpolation = cv2.INTER_CUBIC)
else:
    cam = cv2.VideoCapture(0)
    cam.set(3,IMGWIDTH)
    cam.set(4,IMGHEIGHT)
    for tmp in range(3):   # dump initial frames to allow autogain
        ret, img = cam.read()


blu, green, red = cv2.split(img)
if LOGIMG:
    logblu = np.log10(blu/2.0+1.0) # Dynamic range (0-128)
    image = (255*(logblu - np.amin(logblu))/(np.amax(logblu)-np.amin(logblu))).astype('uint8') # Normalization
else:
    image = blu
tya = tracky - HALFBOX
tyb = tracky + HALFBOX
txa = trackx - HALFBOX
txb = trackx + HALFBOX
trackbox = image[tya:tyb, txa:txb]

firstframe = True
mix_old = 0.9
# philter is the spatial domain version of the template
philter_offset_x = 0
philter_offset_y = 0

cv2.namedWindow('philter_abs_scaled')
cv2.moveWindow('philter_abs_scaled',20,10)

cv2.namedWindow('image')
cv2.moveWindow('image',20,170)

cv2.setMouseCallback ('image', click_trackbox)

cv2.namedWindow ('scanbox')
cv2.moveWindow('scanbox',170,10)

cv2.namedWindow('corrimg_scaled')
cv2.moveWindow('corrimg_scaled',470,10)

cv2.namedWindow('philter_scaled')
cv2.moveWindow('philter_scaled',320,10)

framecounter = 0

while True:
    framecounter += 1
    print('\nFrame = %d'%framecounter)
    #time.sleep(0.01)   # just to slow things down so that we can click on the target
    if not FILESOURCE:
        ret, img = cam.read()
    else:
        success, tmpimg = vidcap.read()  #cv2.imshow('tmp1.png',img)
        img = cv2.resize (tmpimg, (IMGWIDTH,IMGHEIGHT), interpolation = cv2.INTER_CUBIC)
    
    blu, green, red = cv2.split(img)
    if LOGIMG:
        logblu = np.log10(blu/2.0+1.0)
        image = (255*(logblu - np.amin(logblu))/(np.amax(logblu)-np.amin(logblu))).astype('uint8')
    else:
        image = blu

    trackx = sorted((HALFBOX, trackx, IMGWIDTH - HALFBOX))[1]
    tracky = sorted((HALFBOX, tracky, IMGHEIGHT - HALFBOX - 1))[1]
    
    tya = int(tracky - HALFBOX)
    tyb = int(tracky + HALFBOX)
    txa = int(trackx - HALFBOX)
    txb = int(trackx + HALFBOX)
    #print(tya,tyb,txa,txb)
    # Take the ROI of the image and elementwise multiply by the cosine mask
    scanbox = (cv2.multiply(image[tya:tyb, txa:txb].astype('double'),COSMASK)).astype('uint8')
    #print(scanbox.shape)
    
    cv2.imshow ('scanbox',scanbox)

    # Scan is the image we're going to compare to the template
    scan_dft = cv2.dft(np.float32(scanbox),flags = cv2.DFT_COMPLEX_OUTPUT)
    
    if firstframe:  # if this is the first frame, trackbox = scanbox
        print('new target image')
        trackbox = scanbox.copy()
        #cv2.imshow ('trackbox',trackbox)   # this is the new seed image
        #cv2.waitKey(1)
        track_dft = cv2.dft(np.float32(trackbox),flags = cv2.DFT_COMPLEX_OUTPUT)
        tmp = scan_dft.copy()
        # Complex Conjugate calculation
        tmp[:,:,0] = np.multiply(gauss_dft[:,:,0],track_dft[:,:,0])+np.multiply(gauss_dft[:,:,1],track_dft[:,:,1]) # Reals
        tmp[:,:,1] = np.multiply(gauss_dft[:,:,0],track_dft[:,:,1])-np.multiply(gauss_dft[:,:,1],track_dft[:,:,0]) # Imag
        track_dft_mag = cv2.magnitude(track_dft[:,:,0],track_dft[:,:,1]) + 0.01 # 0.01 to avoid divide by zero
        track_dft[:,:,0] = np.divide(tmp[:,:,0],track_dft_mag)  
        track_dft[:,:,1] = np.divide(tmp[:,:,1],track_dft_mag)
             
    dft_corr = np.zeros((2*HALFBOX,2*HALFBOX,2))
    # track_dft x scan_dft*    * means complex conjugate
    dft_corr [:,:,0] = np.multiply(track_dft[:,:,0],scan_dft[:,:,0]) + np.multiply(track_dft[:,:,1],scan_dft[:,:,1])
    dft_corr [:,:,1] = -np.multiply(track_dft[:,:,0],scan_dft[:,:,1]) + np.multiply(track_dft[:,:,1],scan_dft[:,:,0])
    #dft_corr = np.multiply (track_dft, scan_dft_conj)
    
    # create the correlation image
    corrimg_complex = cv2.idft(dft_corr)
    corrimg = corrimg_complex[:,:,0] 
    corrimg_min = np.amin(corrimg)
    corrimg_max = np.amax(corrimg)
    
    print('corrimg_max = %.4g'%(corrimg_max))
    # a flat correlation map would be a divide by zero
    if corrimg_max == corrimg_min:
        corrimg_scaled = corrimg * 0 + 128 # Might be a faster way to do this
    else:
        corrimg_scaled = (255 * (corrimg - corrimg_min)/(corrimg_max - corrimg_min)).astype('uint8')        
    #corrimg_shifted = np.fft.fftshift(corrimg_scaled)
    cv2.imshow('corrimg_scaled',corrimg_scaled)    
    
    # find the peak correlation
    wheremax = np.where(corrimg == corrimg_max) # [0]=y, [1]=x

    # -- NEW INTERPRETATION --
    shifty = wheremax[0][0] - HALFBOX
    shiftx = HALFBOX - wheremax[1][0]
    #print('shiftx = %d, shifty = %d'%(shiftx, shifty))
    PMW = 20  # Peak Mask Width (might not be used)
    #corrimg [HALFBOX+shifty-PMW:HALFBOX+shifty+PMW, HALFBOX+shiftx-PMW:HALFBOX+shiftx+PMW] = 0
    corrimg_average = np.average(corrimg)  # testing out average
    print('corr average = %.4g'%corrimg_average)
    corr_peak_to_average_ratio = corrimg_max / corrimg_average
    print('RATIO = %f'%corr_peak_to_average_ratio)
    #cv2.imshow('masked corrimg',corrimg/np.amax(corrimg))
    
    # OK, the last thing for the first frame
    if firstframe:
        firstframe = False
        corr_LPF = corrimg_max
        #cv2.waitKey(0)    
    
    # track the correlation peak
    if True:   #corrimg_max > 0.1*corr_LPF:
        # update
        trackx += int(shiftx)
        tracky -= int(shifty)
        trackx = sorted((IMGWIDTH - HALFBOX - 1 - philter_offset_x - 1, trackx, HALFBOX - philter_offset_x))[1]
        tracky = sorted((IMGHEIGHT - HALFBOX -1 - philter_offset_y, tracky, HALFBOX - philter_offset_y))[1]
    else:  # if correlation too low, draw a crossed out box
        cv2.line (image, (trackx-HALFBOX,tracky-HALFBOX),(trackx+HALFBOX,tracky+HALFBOX),(0, 255, 255),1)
        cv2.line (image, (trackx+HALFBOX,tracky-HALFBOX),(trackx-HALFBOX,tracky+HALFBOX),(0, 255, 255),1)
    
    # update and show the template
    # print('corr ratio = %f'%(corrimg_max / corr_LPF))
    if corr_peak_to_average_ratio > 2.4:  # adaptation only when we have a strong signal     
        print('updating philter.')
        
        tya = int(tracky - HALFBOX + philter_offset_y)  # grab the new frame with offsets
        tyb = int(tracky + HALFBOX + philter_offset_y)  # that will center the moment
        txa = int(trackx - HALFBOX + philter_offset_x)    # philter_offset_x and y are defined below
        txb = int(trackx + HALFBOX + philter_offset_x)
        #print(tya, tyb, txa, txb)
        scanbox = (cv2.multiply(image[tya:tyb, txa:txb].astype('double'),COSMASK)).astype('uint8')
        scan_dft = cv2.dft(np.float32(scanbox),flags = cv2.DFT_COMPLEX_OUTPUT)
        tmp = scan_dft.copy()  # create philter for the current scanbox
        tmp[:,:,0] = np.multiply(gauss_dft[:,:,0],scan_dft[:,:,0])+np.multiply(gauss_dft[:,:,1],scan_dft[:,:,1])
        tmp[:,:,1] = np.multiply(gauss_dft[:,:,0],scan_dft[:,:,1])-np.multiply(gauss_dft[:,:,1],scan_dft[:,:,0])
        tmp_dft_mag = cv2.magnitude(scan_dft[:,:,0],scan_dft[:,:,1]) + 0.01  # prevents divide by zero errors
        tmp[:,:,0] = np.divide(tmp[:,:,0],tmp_dft_mag)  
        tmp[:,:,1] = np.divide(tmp[:,:,1],tmp_dft_mag)
        
        if (philter_offset_x == 0) and (philter_offset_y == 0):
            mix_old = 0.9
        else:
            mix_old = 0.5
        track_dft = mix_old*track_dft + (1.0 - mix_old)*tmp # mix in the new philter image with the old

        # Visualization
        filter_complex = cv2.idft(track_dft)
        philter = filter_complex[:,:,0]   # real part of the philter image
        philter_min = np.amin(philter)
        philter_max = np.amax(philter)        
        philter_shifted = np.fft.fftshift(philter)
        philter_abs = np.absolute(philter_shifted)
        philter_abs_scaled = (255*(philter_abs-np.amin(philter_abs))/(np.amax(philter_abs)-np.amin(philter_abs))).astype('uint8')
        cv2.imshow('philter_abs_scaled',philter_abs_scaled)
        M = cv2.moments(philter_abs)
        centroid_x = int (M['m10']/M['m00'])
        centroid_y = int (M['m01']/M['m00'])
        #print(centroid_x - HALFBOX, centroid_y - HALFBOX)
        philter_offset_x = centroid_x - (HALFBOX - 2)   # use on the next cycle
        philter_offset_y = centroid_y - (HALFBOX -2)  # use on the next cycle
        philter_scaled = (255 * (philter_shifted - philter_min)/(philter_max - philter_min)).astype('uint8')        
        cv2.imshow('philter_scaled',philter_scaled)
        cv2.circle (image,(trackx,tracky),int(1.2*HALFBOX),(255,255,255),1)
        cv2.circle (image,(trackx,tracky),int(1.2*HALFBOX)+1,(0,255,255),1)
        #cv2.waitKey(0)
    
    update_mix_old = 0.95
    corr_LPF = update_mix_old * corr_LPF + (1.0 - update_mix_old)*corrimg_max
    # print('corr_LPF = %.4g'%corr_LPF)
    cv2.rectangle (image,(trackx-HALFBOX,tracky-HALFBOX),(trackx+HALFBOX,tracky+HALFBOX),(255,255,255),1)
    cv2.rectangle (image,(trackx-HALFBOX+1,tracky-HALFBOX+1),(trackx+HALFBOX-1,tracky+HALFBOX-1),(0,0,0),1)
    cv2.line (image, (CNTRX,CNTRY-3),(CNTRX,CNTRY+3),(128, 255, 255),1)
    cv2.line (image, (CNTRX-3,CNTRY),(CNTRX+3,CNTRY),(128, 255, 255),1)
    cv2.imshow('image',image)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cv2.waitKey(1)
