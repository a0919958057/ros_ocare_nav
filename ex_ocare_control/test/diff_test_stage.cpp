
#include<ros/ros.h>

// Use the enum of Topic CMD for Diff and Arm
#include"ocare_hw_node.h"

#include"std_msgs/MultiArrayDimension.h"
#include"std_msgs/MultiArrayLayout.h"
#include"std_msgs/UInt16MultiArray.h"
#include"sensor_msgs/LaserScan.h"
#include"sensor_msgs/Imu.h"
#include"geometry_msgs/Twist.h"
#include"tf/tf.h"

#include<cmath>

double front_length(10);
double orient(0);

void callback_laser(const sensor_msgs::LaserScanConstPtr &msg) {
    double length_sum(0);
    for(int i=250;i<260;i++)
     length_sum += msg.get()->ranges[i];
    front_length = length_sum / 10;
}

void callback_imu(const sensor_msgs::ImuConstPtr &msg) {
    tf::Quaternion m;
    tf::quaternionMsgToTF( msg.get()->orientation, m);
    orient = tf::getYaw(m);
}

int main(int argc, char** argv) {
    ros::init(argc,argv,"diff_test_stage_node");

    // Create a ros node
    ros::NodeHandle node;

    ros::Subscriber laser_sub =
            node.subscribe<sensor_msgs::LaserScan>("/scan",50,callback_laser);

    ros::Subscriber imu_sub =
            node.subscribe<sensor_msgs::Imu>("/imu",50,callback_imu);

    ros::Publisher diff_pub =
            node.advertise<std_msgs::UInt16MultiArray>("/diff_mode_cmd",50);

    ros::Publisher diff_twist_pub =
            node.advertise<geometry_msgs::Twist>("/ocare/pose_fuzzy_controller/diff_cmd",50);

    ros::Rate r(10);

    int laser_distence_overlimit_conter(0);

    while(ros::ok()) {

        std_msgs::UInt16MultiArray cmd_message;
        geometry_msgs::Twist cmd_twist;


        if(front_length < 0.2) {
            laser_distence_overlimit_conter ++;
        }

        // If there the distence less than 0.2 m too many times, we have confidence say that we need turning now
        if(laser_distence_overlimit_conter > 5) {
            cmd_message.data.clear();
            cmd_message.data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
            cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        } else {
            cmd_message.data.clear();
            cmd_message.data.push_back((uint16_t)DiffModbus::MODE_TRACK_LINE_CMD);
            cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
            cmd_twist.angular.z = -M_PI / 2;
        }

        if(fabs(orient+M_PI/2) < 0.1)
            cmd_twist.linear.x = 50;
        else
            cmd_twist.linear.x = 0;



        ROS_INFO("PUBLISH SUCCESS");

        diff_pub.publish(cmd_message);
        diff_twist_pub.publish(cmd_twist);

        // Sleep for a while
        r.sleep();
    }



    return 0;
}
