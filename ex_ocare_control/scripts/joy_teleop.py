#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import math

pi = 3.1415926

linear_speed = 0.0
angular_speed = 0.0
angular_speed_micro = 0.0
# angular_offset = 0.0
orient = 0.0
# orient_90 = 0.0
# fg_orient_90 = 0
# last_orient_90_time = 0.0;

def callback(data):
    """

    :type data:Joy
    :return:
    """


    global angular_speed
    global angular_speed_micro
    global linear_speed
    global angular_offset
    # global fg_orient_90
    # global last_orient_90_time

    if(data.buttons[0] == 1):
        linear_speed = data.axes[1] * 255
    else:
        linear_speed = data.axes[1] * 100

    if(data.buttons[6] == 1):
        angular_speed = data.axes[2] * 0.005
    elif(data.buttons[7] == 1):
        angular_speed = data.axes[2] * 0.03
    else:
        angular_speed = data.axes[2]*0.01

    # if(data.buttons[2] == 1 and (rospy.get_time() - last_orient_90_time) > 0.2):
    #     last_orient_90_time = rospy.get_time()
    #     fg_orient_90 = pi/2
    # if(data.buttons[3] == 1 and (rospy.get_time() - last_orient_90_time) > 0.2):
    #     last_orient_90_time = rospy.get_time()
    #     fg_orient_90 = -pi/2

    angular_speed_micro = data.axes[0]*0.003
    # angular_offset = data.axes[3]*0.5

def joy_teleop():
    global angular_speed
    global angular_speed_micro
    global linear_speed
    # global angular_offset
    global orient

    # global orient_90
    # global fg_orient_90
    #
    # global last_orient_90_time

    rospy.init_node('Joy_teleop')

    last_orient_90_time = rospy.get_time()

    rospy.Subscriber("joy", Joy, callback)
    global pub
    pub = rospy.Publisher('/ocare/pose_fuzzy_controller/diff_cmd', Twist)
    r = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():

        # if(fg_orient_90 != 0):
        #     if(orient / (pi/2) > 0):
        #         times_90 = math.floor(orient / (pi/2))
        #     elif(orient / (pi/2) < 0):
        #         times_90 = math.ceil(orient / (pi / 2))
        #     else:
        #         times_90 = 0
        #
        #     orient_90 = times_90 * (pi/2) + fg_orient_90
        #     fg_orient_90 = 0
        #     orient = orient_90


        orient = orient + angular_speed + angular_speed_micro
        twist = Twist()
        twist.linear.x = linear_speed
        # twist.angular.z = orient + angular_offset
        twist.angular.z = orient
        pub.publish(twist)
        r.sleep()
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    joy_teleop()
