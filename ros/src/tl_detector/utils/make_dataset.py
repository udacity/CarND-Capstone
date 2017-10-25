#!/usr/bin/env python
import numpy as np
import cv2
import glob
outcounter = 7910

def click_and_crop(event, x, y, flags, param):
	global img,dsp,outcounter

	p1x = x-32
	p1y = y-32

	if(p1x<0):
		return
	if(p1y<0):
		return

	p2x = x+32
	p2y = y+32

	if(p2x>400*4):
		print("x too big")
		return
	if(p2y>300*4):
		print("y too big")
		return
	if(event == 0):
		dsp = img.copy()
		cv2.rectangle(dsp, (p1x,p1y), (p2x,p2y), (255,0,0), 4)
		cv2.imshow('frame', dsp)
		
	if(event == 4):
		#cv2.imwrite("./traffic_lights/red/red%05d.jpg"%outcounter, img[p1y:p2y,p1x:p2x])
		cv2.imwrite("./traffic_lights/no-light/no-light%05d.jpg"%outcounter, img[p1y:p2y,p1x:p2x])

		outcounter = outcounter + 1
	#print(event)


#files = glob.glob("./SDC/object-dataset/*.jpg")
#files = glob.glob("./SDC/object-detection-crowdai/*.jpg")
#files = glob.glob("./SDC/Udacity-loop/*.jpg")
files = glob.glob("./SDC/sim_imgs/*.jpg")

fileindex = 0
print(files[fileindex])

img = cv2.imread(files[fileindex])
dsp = img.copy()

def run():
	cv2.imshow('frame', dsp)
	cv2.setMouseCallback("frame", click_and_crop)
run()

while True:
	
	k = cv2.waitKey(1)
	
	if k != 255:
		#print(k)
		pass
	if k == ord("q"):
		break
	
	if k == ord("n"):
		fileindex= fileindex + 1
		img = cv2.imread(files[fileindex])
		dsp = img.copy()
		cv2.imshow('frame', dsp)
		

