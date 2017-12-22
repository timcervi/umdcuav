from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import time

def cam_loop(queue_from_cam, DeCloudP):#decloudp unused
    print 'Initializing PiCamera'
    resolution_x=vars.resolution_x; resolution_y=vars.resolution_y
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
