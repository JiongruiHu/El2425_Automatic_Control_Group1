#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy as rp
from geometry_msgs.msg import Twist
from low_level_interface.msg import lli_ctrl_request

velocity = 0
steering = 0

def callback(data):
	#rp.loginfo(rp.get_caller_id() + "The data is %s", data.linear)
	#rp.loginfo(rp.get_caller_id() + "The data is %s", data.angular)
	velocity = int(data.linear.x)
	steering = int(data.angular.z)
	fun_str = "{steering: " + str(steering) + ", velocity: " + str(velocity) + ", transmission: 0, differential_front: 0, differential_rear: 0, ctrl_code: 0}"
	rp.loginfo(fun_str)
	#pub.publish(steering, velocity, 0, 0, 0, 0)
	

def listener():
	rp.init_node("translator", anonymous=True)
	rp.Subscriber("key_vel", Twist, callback)
	rp.spin()

if __name__=="__main__":
	pub = rp.Publisher("lli/ctrl_request", lli_ctrl_request, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10)

	pub.publish(0,10,0,0,0,0)
	#listener()




