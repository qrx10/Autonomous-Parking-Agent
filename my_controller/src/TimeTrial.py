#!/usr/bin/env python
import os
import re
from numpy import asarray
import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String



rospy.init_node('topic_subscriber', anonymous=True)
pub_vel = rospy.Publisher('/R1/cmd_vel',Twist,queue_size=1)
pub_plate = rospy.Publisher('/license_plate',String,queue_size=10)
#sub_time = rospy.Subscriber('/clock',Clock,callback)
count = 0


rate = rospy.Rate(2.5)

#pub_plate.publish(str('TeamRed,multi21,0,QW18'))
#sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, callback, queue_size=1)
while not rospy.is_shutdown():
	move = Twist()
	move.linear.x = 0.1
	move.angular.z = 0.25
	if(count==0):
		pub_plate.publish("TeamRed,multi21,0,EJ14")
		count = count+1
	elif (count==1):
		pub_plate.publish("TeamRed,multi21,0,EJ14")
		count = count +1
	elif(count==25):
		move.angular.z = 0.1
		pub_vel.publish(move)
		count = count+1
	elif(count==50):
		pub_plate.publish("TeamRed,multi21,-1,EJ14")
		move.linear.x = 0
		move.angular.z = 0
		pub_vel.publish(move)
		break
	else:
		pub_vel.publish(move)
		pub_plate.publish("TeamRed,multi21,1,EJ14")
		count = count+1
	rate.sleep()