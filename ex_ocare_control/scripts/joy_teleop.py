#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

linear_speed = 0.0
angular_speed = 0.0
angular_speed_micro = 0.0
angular_offset = 0.0
orient = 0.0

def callback(data):
    global angular_speed
    global angular_speed_micro
    global linear_speed
    global angular_offset

    linear_speed = data.axes[1] * 100
    angular_speed = data.axes[2]*0.01
    angular_speed_micro = data.axes[0]*0.003
    angular_offset = data.axes[3]*0.5

def joy_teleop():
    global angular_speed
    global angular_speed_micro
    global linear_speed
    global angular_offset
    global orient
    rospy.init_node('Joy_teleop')
    rospy.Subscriber("joy", Joy, callback)
    global pub
    pub = rospy.Publisher('/ocare/pose_fuzzy_controller/diff_cmd', Twist)
    r = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        orient = orient + angular_speed + angular_speed_micro
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = orient + angular_offset
        pub.publish(twist)
        r.sleep()
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    joy_teleop()
