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

def callbackLocation(data):
	global location
	location = int(filter(str.isdigit, str(data)))
	#print("location: "+str(location))

def callbackPlate(data):
	global plate
	plate = str(data.data)
	#print(plate)

def callbackObject(data):
	global ObjectList
	Object = str(data.data)
	ObjectList = re.split(', |[|\'|]', Object)
	

def callback(data):
	global count
	global cumerr
	global err
	global preerr
	global location
	global flag
	global endFlag
	subLocation = rospy.Subscriber('LocationNumber', String, callbackLocation, queue_size=10)
	subPlate = rospy.Subscriber('PlateNumber', String, callbackPlate, queue_size=10)
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
	if location in [0,1]:
		polygon = np.array([[(0,height),(width-40,height),
	    	(width-40,170),(width/2,150),(0,170)]])
	elif location in [7,8,2,3,4,5,6,9]:
		polygon = np.array([[(40,height),(width,height),
	    	(width,170),(width/2,150),(40,170)]])
	if location == 6:
		flag = 0
		location = 9
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
				print(ObjectList)
				ObjectLen = len(ObjectList)
				if(ObjectLen<2):
					print("break")
					break

		if(areaRed>900 and ObjectLen>1 and ObjectList[1]=="Human" and flag == 0):
			
			if int(ObjectList[0])>150:
				print("larger than 150")
				while(True):
					move.linear.x = 0
					move.angular.z = 0
					pub.publish(move)
					subObject = rospy.Subscriber('ObjectDetection', String, callbackObject, queue_size=1)
					print(ObjectList)
					ObjectLen = len(ObjectList)
					if(ObjectLen>1 and ObjectList[1]=="Human" and int(ObjectList[0])<140):
						print("break")
						flag = 1
						break
			else:
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
						break
			
			print("end loop")

		if(cX<shape[1]/2-5):
			move.linear.x = 0.2
			move.angular.z = -(err*kP-D*kD-kI*cumerr)*1.8
			#print(-(err*kP-D*kD-kI*cumerr)*1.5)
			#print("err:" + str(err))
			pub.publish(move)
			

		elif(cX>shape[1]/2+5):
			move.linear.x = 0.2
			move.angular.z = -(err*kP-D*kD-kI*cumerr)*1.8
			pub.publish(move)
			#print(-(err*kP-D*kD-kI*cumerr)*1.5)
		else:
			move.linear.x = 0.2     #0.3
			move.angular.z = 0.0    #0.1
			pub.publish(move)

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

		if(location!=9 and '%' not in plate and platecheck.match(plate) and endFlag!=2):
			pub_plate.publish(PublishString)
	# if(len(contours)==0):
	# 	move.linear.x = -0.1
	# 	#move.angular.z = 0
	# 	pub.publish(move)

flag = 0
err = 0
preerr = 0
cumerr = 0
count = 0
location = 0
endFlag = 0
plate = "1111"
ObjectList = list()
rospy.init_node('topic_subscriber', anonymous=True)
pub = rospy.Publisher('/R1/cmd_vel',Twist,queue_size=1)
pub_plate = rospy.Publisher('/license_plate',String,queue_size=10)
sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, callback, queue_size=1)
rospy.spin()
