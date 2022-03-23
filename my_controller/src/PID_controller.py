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

import string
import os
import re
from numpy import asarray
import math

def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

def get_num(x):
	return int(''.join(ele for ele in x if ele.isdigit()))

def callback(data):
	
	global count
	global sess1
	global graph1
	global last_plnum
	global last_last_plnum

	# global err
	# global preerr
	# global a
	# global sess1
	# global graph1

	# bridge = CvBridge()
	# cv_image = bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
	# shape = cv_image.shape
	# #cv_image = cv2.resize(cv_image,(320,240))
	# gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	# gray_image = cv2.GaussianBlur(gray_image,(3,3),0)
	# height = cv_image.shape[0]
	# width = cv_image.shape[1]
	# #gray_image = cv2.GaussianBlur(gray_image,(7,7),0)
	
	# #edge_image = cv2.Canny(gray_image,50,150)
	
	# ret,thresh1 = cv2.threshold(gray_image,87,255,cv2.THRESH_BINARY)
	# ret,thresh2 = cv2.threshold(gray_image,80,255,cv2.THRESH_BINARY_INV)

	
	# ret,threshPlate2_H = cv2.threshold(gray_image,103,255,cv2.THRESH_BINARY)
	# ret,threshPlate2_L = cv2.threshold(gray_image,101,255,cv2.THRESH_BINARY_INV)
	# ret,threshPlate3_H = cv2.threshold(gray_image,203,255,cv2.THRESH_BINARY)
	# ret,threshPlate3_L = cv2.threshold(gray_image,201,255,cv2.THRESH_BINARY_INV)
	# ret,threshPlate4_H = cv2.threshold(gray_image,123,255,cv2.THRESH_BINARY)
	# ret,threshPlate4_L = cv2.threshold(gray_image,120,255,cv2.THRESH_BINARY_INV)
	
	# thresh = cv2.bitwise_or(thresh1,thresh2)
	# thresh = cv2.bitwise_not(thresh)
	
	
	# threshPlate2 = cv2.bitwise_or(threshPlate2_H,threshPlate2_L)
	# threshPlate3 = cv2.bitwise_or(threshPlate3_H,threshPlate3_L)
	# threshPlate4 = cv2.bitwise_or(threshPlate4_H,threshPlate4_L)
	# threshPlate = cv2.bitwise_and(threshPlate3,threshPlate4)
	# threshPlate = cv2.bitwise_and(threshPlate,threshPlate2)
	# cv2.imshow("image1", threshPlate)
	# threshPlate = cv2.bitwise_not(threshPlate)
	# #threshPlate = cv2.fastNlMeansDenoising(threshPlate,None,3,7,21)
	# #Edge = cv2.Canny(threshPlate,150,200)

	# # cv2.imshow("frame",threshPlate)
	# # cv2.waitKey(1)
	
	# polygon = np.array([[(0,height),(width,height),
	#     (width,470),(width/2,height/2),(0,470)]])
	# rectangle = np.array([[(0,height),(width,height),
	#     (width,300),(0,300)]])
	# black_image = np.zeros_like(gray_image)
	# mask = cv2.fillPoly(black_image,polygon,255)
	# mask_rec = cv2.fillPoly(black_image,rectangle,255)
	# masked_image = cv2.bitwise_and(thresh,mask)
	# masked_rec_image = cv2.bitwise_and(threshPlate,mask_rec)
	
	# cv2.waitKey(1)
	# _, contours, _ = cv2.findContours(masked_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	# #cv2.drawContours(cv_image,contours,-1,(255,0,0),3)
	
	# contours = sorted(contours,key=cv2.contourArea,reverse=True)[:3]
	# for c in contours:
	# 	M = cv2.moments(contours[0])
	# 	area = cv2.contourArea(contours[0])
	# 	if(M["m00"]!=0):
	# 		cX = int(M["m10"] / M["m00"])
	# 		cY = int(M["m01"] / M["m00"])


	# plate = []
	# _, contoursPlate, _ = cv2.findContours(masked_rec_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	# cv2.drawContours(gray_image,contoursPlate,-1,0,5)
	# ret,masked_rec_image = cv2.threshold(gray_image,1,255,cv2.THRESH_BINARY_INV)
	# _, contoursPlate, _ = cv2.findContours(masked_rec_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	# #cv2.drawContours(cv_image,contoursPlate,-1,(0,255,255),2)
	# cv2.imshow("image", cv_image)
	# cv2.waitKey(1)
	# contoursPlate = sorted(contoursPlate,key=cv2.contourArea,reverse=True)[:3]
	
	# #print(contoursPlate)
	# cP = contoursPlate[0]
	# approx = cv2.approxPolyDP(cP,0.015*cv2.arcLength(cP,True),True)
	# # print(len(approx))
	# # print(cv2.contourArea(cP))
	# if((len(approx)==4) and cv2.contourArea(cP)>10000 and cv2.contourArea(cP)<45000):
	# 	cP = cP.reshape(-1,2)
	# 	plate.append(approx)
	# # 	print(plate)
	# # 	print(len(plate))
	# # 	print("-----------------------------")
	# # if (len(plate)>0):
	# 	#cv2.polylines(cv_image, plate, True, (255, 0, 0), 3)
	# 	#print(plate[0])
	# 	larW = max(plate[0][0][0][0],plate[0][1][0][0],plate[0][2][0][0],plate[0][3][0][0])
	# 	smlW = min(plate[0][0][0][0],plate[0][1][0][0],plate[0][2][0][0],plate[0][3][0][0])
	# 	larH = max(plate[0][0][0][1],plate[0][1][0][1],plate[0][2][0][1],plate[0][3][0][1])
	# 	smlH = min(plate[0][0][0][1],plate[0][1][0][1],plate[0][2][0][1],plate[0][3][0][1])
	# 	width = larW - smlW
	# 	height = larH - smlH
	# 	vertex = [smlW,smlH]
	# 	#print(height)
	# 	height = int(width*1.316)
	# 	#print(height)
	# 	plate_image = cv_image[vertex[1]:vertex[1]+height,vertex[0]:vertex[0]+width]
	# 	plate_image = cv2.resize(plate_image,(600,1800))
	# 	# cv2.imshow("plate", plate_image)
	# 	# cv2.waitKey(1)
	# 	image = []
	# 	height = 1548
	# 	image.append(plate_image[1250:height,45:145])
	# 	cv2.imshow("kkk", plate_image[1250:height,45:145])
	# 	image.append(plate_image[1250:height,145:245])
	# 	image.append(plate_image[1250:height,345:445])
	# 	image.append(plate_image[1250:height,445:545])
	# 	plnum = ""
	# 	for i in range(4):
	# 		with graph1.as_default():
	# 			set_session(sess1)
	# 			init = tf.global_variables_initializer()
	# 			sess1.run(init)

	# 			img_aug = np.expand_dims(image[i], axis=0)
	# 			y_predict = conv_model.predict(img_aug)[0]
	# 			max_index = np.where(y_predict == np.amax(y_predict))
	# 			caption = str(class_names[get_num(str(max_index))])
	# 			plnum = plnum+caption
	# 	print(plnum)
	


	# cv_img = cv2.imread("/home/fizzer/ros_ws/src/P1.png", cv2.IMREAD_GRAYSCALE)
	# frame = bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
	# sift = cv2.xfeatures2d.SIFT_create()
	# kp_image, desc_image = sift.detectAndCompute(cv_img, None)

	# index_params = dict(algorithm=0, trees=5)
	# search_params = dict()
	# flann = cv2.FlannBasedMatcher(index_params, search_params)

	# grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # trainimage
	# kp_grayframe, desc_grayframe = sift.detectAndCompute(grayframe, None)
	# matches = flann.knnMatch(desc_image, desc_grayframe, k=2)
	# good_points = []
	# for m, n in matches:
	# 	if m.distance < 0.6 * n.distance:
	# 		good_points.append(m)

	# if len(good_points) > 3:
	# 	query_pts = np.float32([kp_image[m.queryIdx].pt 
	# 		for m in good_points]).reshape(-1, 1, 2)
	# 	train_pts = np.float32([kp_grayframe[m.trainIdx].pt 
	# 		for m in good_points]).reshape(-1, 1, 2)
	# 	matrix, mask = cv2.findHomography(query_pts, 
	# 		train_pts, cv2.RANSAC, 5.0)
	# 	matches_mask = mask.ravel().tolist()
	# 	h,w = cv_img.shape
	# 	pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
	# 	dst = cv2.perspectiveTransform(pts, matrix) 
	# 	frame = cv2.polylines(frame, [np.int32(dst)], True, (255, 0, 0), 3)
		
	# 	adst = dst.astype(int)
	# 	P_area = cv2.contourArea(adst)
	# 	if (True and adst[0][0][0]>0 and adst[0][0][1]>0):
	# 		print(adst)
	# 		print(P_area)
	# 		print(adst[0][0][0])

	# 		Plate = gray_image[adst[0][0][1]-80:adst[0][0][1]+200,adst[0][0][0]-80:adst[0][0][0]+200]
	# 		cv2.imshow("frame", Plate)
	# 		cv2.waitKey(1)

	# cv2.imshow("image", frame)
	# cv2.waitKey(1)
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
	shape = cv_image.shape

	gray_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
	gray_image = cv2.GaussianBlur(gray_image,(3,3),0)
	
	thresh1_H = cv2.threshold(gray_image,105,255,cv2.THRESH_BINARY)
	thresh1_L = cv2.threshold(gray_image,100,255,cv2.THRESH_BINARY_INV)
	thresh1 = cv2.bitwise_or(thresh1_L[1],thresh1_H[1])

	thresh2_H = cv2.threshold(gray_image,125,255,cv2.THRESH_BINARY)
	thresh2_L = cv2.threshold(gray_image,120,255,cv2.THRESH_BINARY_INV)
	thresh2 = cv2.bitwise_or(thresh2_L[1],thresh2_H[1])

	thresh3_H = cv2.threshold(gray_image,205,255,cv2.THRESH_BINARY)
	thresh3_L = cv2.threshold(gray_image,200,255,cv2.THRESH_BINARY_INV)
	thresh3 = cv2.bitwise_or(thresh3_L[1],thresh3_H[1])

	thresh = cv2.bitwise_and(thresh1,thresh2)
	thresh = cv2.bitwise_and(thresh,thresh3)
	thresh = cv2.GaussianBlur(thresh,(3,3),1)
	thresh[0:300,:]=255*np.ones_like(thresh[0:300,:])

	thresh = cv2.threshold(thresh,128,255,cv2.THRESH_BINARY)[1]
	
	thresh = cv2.bitwise_not(thresh)
	contours = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]
	#print(contours)
	contours = sorted(contours,key=cv2.contourArea,reverse=True)
	#cv2.drawContours(cv_image, contours[0], -1, (0,255,0), 3)
	
	approx = []
	for c in contours:
		approx = cv2.approxPolyDP(c,0.01 * cv2.arcLength(c, True),True)
		# print(len(approx))
		# print(cv2.contourArea(approx))
		if(len(approx)==4 and cv2.contourArea(approx)>12000):
			# cv2.polylines(cv_image,[approx],1,(255,255,0),3)
			# cv2.imshow("plate",cv_image)
			# cv2.waitKey(1)
			#print(approx)
			plate_vertex = [approx[0][0],approx[3][0],approx[1][0],approx[2][0]]
			#print(plate_vertex)
			#print("______________________________")
			plate_vertex = sorted(plate_vertex , key=lambda k: k[1])
			up_vertex = sorted([plate_vertex[0],plate_vertex[1]], key = lambda k: k[0])
			down_vertex = sorted([plate_vertex[2],plate_vertex[3]],key = lambda k: k[0])
			#print(down_vertex)
			plate_vertex = [up_vertex[0],up_vertex[1],down_vertex[0],down_vertex[1]]
			#print(plate_vertex)
			#print("_______________________________")
			plate_vertex[2][1] = int(plate_vertex[0][1]+
				(plate_vertex[2][1]-plate_vertex[0][1])/1250.0*1800)
			plate_vertex[3][1] = int(plate_vertex[1][1]+
				(plate_vertex[3][1]-plate_vertex[1][1])/1250.0*1800)
			#print(plate_vertex)
			#print("................................")
			plate_sample = np.float32([[0,0],[600,0],[0,1800],[600,1800]])
			plate_vertex = np.float32(plate_vertex)
			# for val in plate_vertex:
			# 	cv2.circle(cv_image,(val[0],val[1]),5,(0,255,0),-1)
			
			Matrix = cv2.getPerspectiveTransform(plate_vertex,plate_sample)
			plate_image = cv2.warpPerspective(gray_image,Matrix,(600,1800))

			# cv2.imshow("image",plate_image)
			# cv2.waitKey(1)
			image = []
			image[:]=[]
			height = 1520
			image.append(plate_image[1320:height,45:145])
			cv2.imshow("1", plate_image[1320:height,45:145])
			cv2.waitKey(1)
			#cv2.imwrite("/home/fizzer/ros_ws/Data/"+str(count)+".png",plate_image[1320:height,45:145])
			count = count + 1
			image.append(plate_image[1320:height,145:245])
			cv2.imshow("2", plate_image[1320:height,145:245])
			cv2.waitKey(1)
			#cv2.imwrite("/home/fizzer/ros_ws/Data/"+str(count)+".png",plate_image[1320:height,145:245])
			count = count + 1
			image.append(plate_image[1320:height,345:445])
			cv2.imshow("3", plate_image[1320:height,345:445])
			cv2.waitKey(1)
			#cv2.imwrite("/home/fizzer/ros_ws/Data/"+str(count)+".png",plate_image[1320:height,345:445])
			count = count + 1
			image.append(plate_image[1320:height,445:545])
			cv2.imshow("4", plate_image[1320:height,445:545])
			cv2.waitKey(1)
			#cv2.imwrite("/home/fizzer/ros_ws/Data/"+str(count)+".png",plate_image[1320:height,445:545])
			count = count + 1
			plnum = ""
			y_predict = None
			LocNum_predict = None
			for i in range(4):
				image[i] = image[i].reshape(image[i].shape[0],image[i].shape[1],1)
				img_aug = np.expand_dims(image[i], axis=0)
				with graph1.as_default():
					set_session(sess1)
					y_predict = conv_model.predict(img_aug)[0]
				max_index = np.where(y_predict == np.amax(y_predict))
				caption = str(class_names[get_num(str(max_index))])
				plnum = plnum+caption
			if plnum == last_plnum and plnum == last_last_plnum:
				print(plnum)
			last_last_plnum = last_plnum
			last_plnum = plnum
			plnum = " "
			image[:] = [0]

			locationIm = plate_image[700:1100,300:600]
			locationIm = cv2.resize(locationIm,(60,80))
			cv2.imwrite("/home/fizzer/ros_ws/LocNum/h"+str(count)+".png",locationIm)
			locationIm = locationIm.reshape(locationIm.shape[0],locationIm.shape[1],1)
			img_loc = np.expand_dims(locationIm, axis=0)
			cv2.imshow("location", locationIm)
			cv2.waitKey(1)
			#cv2.imwrite("/home/fizzer/ros_ws/LocNum/h"+str(count)+".png",locationIm)
			with graph1.as_default():
				set_session(sess1)
				LocNum_predict = Loc_model.predict(img_loc)[0]
			max_index = np.where(LocNum_predict == np.amax(LocNum_predict))
			print(max_index)
			caption = str(loc_names[get_num(str(max_index))])
			print(caption)
			break


err = 0
preerr = 0
cumerr = 0
a = 0
count = 1
last_plnum = "INIT"
last_plnum = "INIT"
class_names = ['A', 'B', 'C', 'D', 'E',
               'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N',
              'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
               '0', '1', '2', '3', '4', '5', '6', '7', '8', '9','%']
loc_names = ['1','2','3','4','5','6','7','8']

sess1 = tf.Session()    
graph1 = tf.get_default_graph()
set_session(sess1)

#init = tf.global_variables_initializer()
#sess1.run(init)
conv_model = load_model("/home/fizzer/ros_ws/src/PlateRec_new.h5")
Loc_model = load_model("/home/fizzer/ros_ws/src/LocNum.h5")
rospy.init_node('topic_subscriber', anonymous=True)
pub = rospy.Publisher('/R1/cmd_vel',Twist,queue_size=1)
sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, callback, queue_size=1)
rospy.spin()





