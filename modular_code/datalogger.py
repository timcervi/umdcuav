import cv2
import time

def data_logger(image_queue, image_label_queue, text_queue, servo_queue,DecloudP,versionstring):
	#global CALIB, DecloudP, versionstring  #CALIB unused
	print(versionstring)
	print('Initializing Data Logger Process: Data stored in datalog2.txt')
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