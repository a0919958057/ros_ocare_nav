#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
 
def callback(data):
	twist = Twist()
	if  abs(data.axes[1]) > 0.05 :
		twist.linear.x = (data.axes[1]) * 100
		
	if abs(data.axes[2]) > 0.01:
		twist.angular.z = 3.1415926*(data.axes[2]) /2
		
	pub.publish(twist)
 
def joy_teleop():
	rospy.init_node('Joy_teleop')
	rospy.Subscriber("joy", Joy, callback)
	global pub
	pub = rospy.Publisher('/ocare/pose_fuzzy_controller/diff_cmd', Twist)
 
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		r.sleep()
 
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
 
if __name__ == '__main__':
	joy_teleop()
