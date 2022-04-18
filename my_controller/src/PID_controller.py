#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from shapely.geometry import Polygon
import tensorflow as tf
from tensorflow.keras import models
from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model
from std_msgs.msg import String

import string
import os
import re
from numpy import asarray
import math
import time

def callbackLocation(data):
	global location
	location = int(filter(str.isdigit, str(data)))
	#print("location: "+str(location))

def callbackPlate(data):
	global plate
	plate = str(data.data)
	#print(plate)

def callbackPlateLocation(data):
	global plate
	global location
	locationStr,plate = str(data.data).split(",")
	location = int(locationStr)

def callbackObject(data):
	global ObjectList
	Object = str(data.data)
	ObjectList = re.split(', |[|\'|]', Object)

def check():
	global plate
	platechar = list(plate)
	if(plate[0] is '8'):
		platechar[0] = 'B'
	if(plate [1] is '8'):
		platechar[1] = 'B'
	if(plate[2] is 'B'):
		platechar[2] = '8'
	if(plate[3] is 'B'):
		platechar[3] = '8'
	if(plate[0] is '1'):
		platechar[0] = 'I'
	if(plate [1] is '1'):
		platechar[1] = 'I'
	if(plate[2] is 'I'):
		platechar[2] = '1'
	if(plate [3] is 'I'):
		platechar[3] = '1'
	if(plate[0] is '5'):
		platechar[0] = 'S'
	if(plate [1] is '5'):
		platechar[1] = 'S'
	if(plate[2] is 'S'):
		platechar[2] = '5'
	if(plate [3] is 'S'):
		platechar[3] = '5'
	if(plate[0] is '2'):
		platechar[0] = 'Z'
	if(plate [1] is '2'):
		platechar[1] = 'Z'
	if(plate[2] is 'Z'):
		platechar[2] = '2'
	if(plate [3] is 'Z'):
		platechar[3] = '2'
	if(plate[0] is '4'):
		platechar[0] = 'A'
	if(plate [1] is '4'):
		platechar[1] = 'A'
	if(plate[2] is 'A'):
		platechar[2] = '4'
	if(plate[3] is 'A'):
		platechar[3] = '4'
	plate = "".join(platechar)
	

def callback(data):
	global count
	global cumerr
	global err
	global preerr
	global location
	global flag
	global endFlag
	global humanFlag
	global redFlag
	global resetFlag
	global turnFlag


	#subLocation = rospy.Subscriber('LocationNumber', String, callbackLocation, queue_size=10)
	subPlateLocation = rospy.Subscriber('PlateLocationNumber', String, callbackPlateLocation, queue_size=10)
	subObject = rospy.Subscriber('ObjectDetection', String, callbackObject, queue_size=10)

	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
	cv_image = cv2.resize(cv_image,(320,240))
	shape = cv_image.shape
	gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	height = cv_image.shape[0]
	width = cv_image.shape[1]
	gray_image = cv2.GaussianBlur(gray_image,(5,5),0)
	#cv2.imshow("gray_image", gray_image)
	#cv2.waitKey(0)
	#denoised_image = cv2.fastNlMeansDenoising(gray_image,None,3,7,21)
	#edge_image = cv2.Canny(gray_image,100,200)
	ret,thresh1 = cv2.threshold(gray_image,92,255,cv2.THRESH_BINARY)
	ret,thresh2 = cv2.threshold(gray_image,80,255,cv2.THRESH_BINARY_INV)
	ret,threshred1 = cv2.threshold(gray_image,80,255,cv2.THRESH_BINARY)
	ret, threshred2 = cv2.threshold(gray_image,70,255,cv2.THRESH_BINARY_INV)
	thresh = cv2.bitwise_or(thresh1,thresh2)
	thresh = cv2.bitwise_not(thresh)
	threshRed = cv2.bitwise_or(threshred1,threshred2)
	threshRed = cv2.bitwise_not(threshRed)
	polygon = np.array([[(0,height),(width,height),
	    (width,170),(width/2,150),(0,170)]])
	if location in [1]:
		polygon = np.array([[(0,height),(width-15,height),
	    	(width-15,170),(width/2,150),(0,170)]])
	elif location in [3,6,9]:
		polygon = np.array([[(40,height),(width,height),
	    	(width,170),(width/2,150),(40,170)]])
	elif location in [4]:
		polygon = np.array([[(45,height),(width,height),
	    	(width,170),(width/2,150),(45,170)]])
	elif location in [7,8]:
		polygon = np.array([[(60,height),(width,height),
	    	(width,170),(width/2,150),(60,170)]])
	elif location in [0,2,5]:
		polygon = np.array([[(0,height),(width-30,height),
	    	(width-30,170),(width/2,150),(0,170)]])
	if turnFlag == 1:
		polygon = np.array([[(0,height),(width-30,height),
	    	(width-30,170),(width/2,150),(0,170)]])
	if location == 4 or location == 1:
		flag = 0
		turnFlag = 0
		#location = 9
	black_image = np.zeros_like(gray_image)
	mask = cv2.fillPoly(black_image,polygon,255)
	masked_image = cv2.bitwise_and(thresh,mask)
	# _, contours, _ = cv2.findContours(masked_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	# cv2.drawContours(cv_image,contours,-1,(255,0,0),3)
	# cv2.imshow("image", cv_image)
	# cv2.waitKey(1)
	#contours = sorted(contours,key=cv2.contourArea,reverse=True)[:3]
	# for c in contours:
	# 	area = cv2.contourArea(c)
	# 	if area<1000:
	# 		cv2.fillPoly(masked_image,pts=[c],color=0)
	_, contoursRed,_ = cv2.findContours(threshRed,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	_, contours, _ = cv2.findContours(masked_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	contours = sorted(contours,key=cv2.contourArea,reverse=True)[:3]
	contoursRed = sorted(contoursRed,key=cv2.contourArea,reverse=True)[:3]
	areaRed = 0
	for c in contoursRed:
		areaRed = cv2.contourArea(contoursRed[0])
		#print(areaRed)
	cv2.drawContours(cv_image,contours,-1,(255,0,0),3)
	# cv2.imshow("image", cv_image)
	# cv2.waitKey(1)
	cX = 0
	cY = 0
	move = Twist()
	for c in contours:
		M1 = cv2.moments(contours[0])
		area1 = cv2.contourArea(contours[0])
		M2 = 0
		area2 = 0
		
		# if(len(contours)>1):
		# 	M2 = cv2.moments(contours[1])
		# 	area2 = cv2.contourArea(contours[1])
		# 	print("area1: "+str(area1))
		# 	print("area2: "+str(area2))
		# 	if(M1["m00"]!=0 and M2["m00"]!=0):
		# 		cX1 = int(M1["m10"] / M1["m00"])
		# 		cY1 = int(M1["m01"] / M1["m00"])
		# 		cX2 = int(M2["m10"] / M2["m00"])
		# 		cY2 = int(M2["m01"] / M2["m00"])
		# 		cX = int((cX1*area1+cX2*area2)/(area1+area2))
		# 		cY = int((cY1*area1+cY2*area2)/(area1+area2))
			
		if(M1["m00"]!=0):
			cX = int(M1["m10"] / M1["m00"])
			cY = int(M1["m01"] / M1["m00"])
			
		break
	if(cX != 0 and cY != 0):
		cv2.circle(cv_image, (cX, cY), 20, (255, 255, 255), -1)
		cv2.imshow("image", cv_image)
		cv2.waitKey(1)
		err = (cX-shape[1]/2.0)/shape[1]*2.0

		kP = 5
		kI = 0.1*0
		kD = 2*0
		if(cumerr > 5):
			cumerr = 5
		D = err-preerr
		#print(preerr)
		ObjectLen = len(ObjectList)
		#print(ObjectList)
		if(location in [1, 7, 8] and ObjectLen>1 and ObjectList[1]=="Vehicle"):
			while(True):
				move.linear.x = 0
				move.angular.z = 0
				pub.publish(move)
				subObject = rospy.Subscriber('ObjectDetection', String, callbackObject, queue_size=1)
				#print(ObjectList)
				ObjectLen = len(ObjectList)

				if(ObjectLen<2):
					print("break")
					break
			time.sleep(6)

		#print(areaRed)
		if(areaRed>2100 and flag == 0 and location in [3,6]):
			redFlag = 1
			print(areaRed)
		if(ObjectLen>1 and ObjectList[1]=="Human" and flag == 0):
			humanFlag = 1

		if(humanFlag==1 and redFlag ==1):
			if (ObjectLen >1 and int(ObjectList[0])>150):
				print("larger than 150")
				while(True):
					move.linear.x = 0
					move.angular.z = 0
					pub.publish(move)
					subObject = rospy.Subscriber('ObjectDetection', String, callbackObject, queue_size=1)
					#print(ObjectList)
					ObjectLen = len(ObjectList)
					if(ObjectLen>1 and ObjectList[1]=="Human" and int(ObjectList[0])<140):
						print("break")
						flag = 1
						redFlag = 0
						humanFlag = 0
						turnFlag = 1
						#location = 10
						break
			elif(ObjectLen >1 and int(ObjectList[0])<150):
				print("less than 150")
				while(True):
					move.linear.x = 0
					move.angular.z = 0
					pub.publish(move)
					subObject = rospy.Subscriber('ObjectDetection', String, callbackObject, queue_size=1)
					ObjectLen = len(ObjectList)
					
					if(ObjectLen>1 and ObjectList[1]=="Human" and int(ObjectList[0])>160):
						print("break")
						flag = 1
						redFlag = 0
						humanFlag = 0
						turnFlag = 1
						#location = 10
						break
			else:
				move.linear.x = 0
				move.angular.z = 0
				pub.publish(move)
			
			print("end loop")

		if(cX<shape[1]/2-5):
			move.linear.x = 0.183
			if location in [1,7,8]:
				move.linear.x = 0.265
				# if location==4:
				# 	move.linear.x = 0.14
			move.angular.z = -(err*kP-D*kD-kI*cumerr)*1.81
			#print(-(err*kP-D*kD-kI*cumerr)*1.5)
			#print("err:" + str(err))
			pub.publish(move)
			

		elif(cX>shape[1]/2+5):
			move.linear.x = 0.183
			if location in [1,7,8]:
				move.linear.x = 0.265
			# if location==4:
			# 	move.linear.x = 0.14
			move.angular.z = -(err*kP-D*kD-kI*cumerr)*1.81
			pub.publish(move)
			#print(-(err*kP-D*kD-kI*cumerr)*1.5)
		else:
			move.linear.x = 0.19     #0.3
			move.angular.z = 0.0    #0.1
			if location in [1,7,8]:
				move.linear.x = 0.407
			# if location==4:
			# 	move.linear.x = 0.14
			pub.publish(move)

		check()

		preerr = err
		cumerr = cumerr + err
		platecheck = re.compile('[A-Z]{2}\d{2}')
		PublishString = "Ruixin,0707,"+str(location)+","+plate
		if location == 8:
			endFlag = 1
		


		if endFlag==1 and location ==7:
			pub_plate.publish("Ruixin,0707,-1,GDLK")
			endFlag = 2
			while(True):
				move.linear.x = 0
				move.angular.z = 0
				pub.publish(move)
		
		if(location<9 and location >0 and '%' not in plate and platecheck.match(plate) and endFlag!=2):
			pub_plate.publish(PublishString)
			

		
	# if(len(contours)==0):
	# 	move.linear.x = -0.1
	# 	#move.angular.z = 0
	# 	pub.publish(move)

flag = 0
resetFlag = 0
err = 0
preerr = 0
cumerr = 0
count = 0
location = 0
endFlag = 0
humanFlag = 0
redFlag = 0
turnFlag = 0
plate = "1111"
ObjectList = list()
rospy.init_node('topic_subscriber', anonymous=True)
pub = rospy.Publisher('/R1/cmd_vel',Twist,queue_size=1)
pub_plate = rospy.Publisher('/license_plate',String,queue_size=10)
time.sleep(1)
pub_plate.publish("Ruixin,0707,0,GDLK")
sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, callback, queue_size=1)
rospy.spin()
