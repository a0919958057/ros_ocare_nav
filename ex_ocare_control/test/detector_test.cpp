// Include the main ROS header
#include<ros/ros.h>
#include<ros/console.h>

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

#ifdef __arm__
#include<wiringPi.h>
#endif


/**************** DEBUG Flags ******************/

#define DT_STAGE_CHANGE_DETECT "StageChangeDetecter"
#define DT_LOOP_TESK_DEBUG     "LoopTesk"

#define _STAGE_CHANGE_DETECT_DEBUG
#define _LOOP_TESK_DEBUG

/***********************************************/

double front_length(10);
double orient(0);
bool  fg_need_bulb(true);

bool stage_change_detect(int _stage);

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

    // Initial the ROS
    ros::init(argc,argv,"detector_test_node");

    // Create a ros node
    ros::NodeHandle node;

    // Subscribe the laser scan topic
    ros::Subscriber laser_sub =
            node.subscribe<sensor_msgs::LaserScan>("/scan",50,callback_laser);

    // Subscribe the IMU topic
    ros::Subscriber imu_sub =
            node.subscribe<sensor_msgs::Imu>("/imu",50,callback_imu);

/******************************** Publishers ********************************/

    /******Defination of the Chassis Mode
     * MODE_TRACK_LINE :     Tracking the path and follow the path
     * MODE_CONTROLLABLE:    The Arduino would use LEFT_WHEEL_TORQUE and RIGHT_WHEEL_TORQUE to effort the wheel
     * MODE_STOP:            All of the driver would stop until Mode become others.
     * **************************/

    // Create a publish that publish the differential wheel Mode
    ros::Publisher diff_pub =
            node.advertise<std_msgs::UInt16MultiArray>("/diff_mode_cmd",50);

    /******Defination of the Chassis Mode
     * MODE_TRACK_LINE :     Tracking the path and follow the path
     * MODE_CONTROLLABLE:    The Arduino would use LEFT_WHEEL_TORQUE and RIGHT_WHEEL_TORQUE to effort the wheel
     * MODE_STOP:            All of the driver would stop until Mode become others.
     * **************************/

    // Create a publish that publish the differential wheel Torque CMD
    ros::Publisher diff_twist_pub =
            node.advertise<geometry_msgs::Twist>("/ocare/pose_fuzzy_controller/diff_cmd",50);

/*****************************************************************************/

    // Using the 10 hz for while loop
    ros::Rate r(10);

    int stage(3);

    while(ros::ok()) {

        // Do unit stage change detect test
        stage_change_detect(stage);

        // Sleep for a while
        r.sleep();
    }



    return 0;
}


bool stage_change_detect(int _stage) {
    // Count the times of robot distence over minimum limit
    static int laser_distence_overlimit_conter(0);
    static bool fg_usetimer(false);
    static ros::Time last_time(ros::Time::now());


    switch(_stage) {
    case 0:
        // Detect the button stat, if Switch is on then change stage
        break;
    case 1:
        // Detect the Orient is stable at -45 degree
        break;
    case 2:
        // Detect the Orient is stable at -90 degree
        break;
    case 3:
        // Detect the Orient is cross at 0 degree
        last_time = ros::Time::now();
        if(fabs(orient - M_PI * 0.0/0.0) < 0.1) return true;
        break;
    case 4:
        // Detect the Orient is cross at 0 degree and Timing from last stage large than 1 sec
        if(fabs(last_time.toSec() - ros::Time::now().toSec()) > 1.0 && fabs(orient - M_PI * 0.0/0.0) < 0.1) {
            last_time = ros::Time::now();
            return true;
        }
        break;
    case 5:
        // Detect the Orient is cross at 0 degree and Timing from last stage large than 1 sec
        if(fabs(last_time.toSec() - ros::Time::now().toSec()) > 1.0 && fabs(orient - M_PI * 0.0/0.0) < 0.1) {
            last_time = ros::Time::now();
            return true;
        }
        break;
    case 6:
        // Detect the Orient is cross at 0 degree and Timing from last stage large than 1 sec
        if(fabs(last_time.toSec() - ros::Time::now().toSec()) > 1.0 && fabs(orient - M_PI * 0.0/0.0) < 0.1) {
            last_time = ros::Time::now();
            return true;
        }
        break;
    case 7:
        // Detect the Orient is stable at 90 degree and no line
        break;
    case 8:
        // Detect the Orient is stable at 90 degree and Distence is less than 0.3 m

        if(front_length < 0.3)
            laser_distence_overlimit_conter ++;

        // If there the distence less than 0.3 m too many times, we have confidence say that we need turning now
        if(laser_distence_overlimit_conter > 3)
            return true;

        break;

    case 9:
        // Detect the Orient is stable at 180 degree
        break;
    case 10:
        // Detect all white change to black
        break;
    case 11:
        if(fg_need_bulb) {
            fg_usetimer = true;
            last_time = ros::Time::now();
            return true;
        }
        break;
    case 110:
        if(fg_usetimer) {
            if(ros::Time::now().toSec() - last_time.toSec() > 1.0) {
                last_time = ros::Time::now();
                return true;
            }
        } else {
            ROS_ERROR("Stage ERROR %d",_stage);
        }
        break;
    case 111:
        if(fg_usetimer) {
            if(ros::Time::now().toSec() - last_time.toSec() > 1.0) {
                last_time = ros::Time::now();
                return true;
            }
        } else {
            ROS_ERROR("Stage ERROR %d",_stage);
        }
        break;
    case 112:
        if(fg_usetimer) {
            if(ros::Time::now().toSec() - last_time.toSec() > 1.0) {
                fg_usetimer = false;
                last_time = ros::Time::now();
                return true;
            }
        } else {
            ROS_ERROR("Stage ERROR %d",_stage);
        }
        break;
    case 113:
        // Detect all white change to black
        break;
    case 12:
        // Detect the Orient is stable at 0 degree and no line
        break;
    case 13:
        // Detect the Orient is stable at 0 degree and Distence is less than 0.5 m
        break;
    case 14:
        // Detect the Orient is stable at 90 degree
        break;
    case 15:
        // Detect any white
        break;
    case 16:
        // Detect the Orient is stable at 180 degree and Average turn to white
        break;
    case 17:
        // Detect the Orient is stable at 180 degree and Average turn to black
        break;
    case 18:
        // Detect the Orient is stable at -90 degree
        break;
    case 19:
        // Detect right sensor white
        break;
    case 20:
        fg_usetimer = true;
        last_time = ros::Time::now();
        return true;
    case 200:
        if(fg_usetimer) {
            if(ros::Time::now().toSec() - last_time.toSec() > 1.0) {
                last_time = ros::Time::now();
                return true;
            }
        } else {
            ROS_ERROR("Stage ERROR %d",_stage);
        }
        break;
    case 201:
        if(fg_usetimer) {
            if(ros::Time::now().toSec() - last_time.toSec() > 1.0) {
                last_time = ros::Time::now();
                return true;
            }
        } else {
            ROS_ERROR("Stage ERROR %d",_stage);
        }
        break;
    case 202:
        if(fg_usetimer) {
            if(ros::Time::now().toSec() - last_time.toSec() > 1.0) {
                last_time = ros::Time::now();
                return true;
            }
        } else {
            ROS_ERROR("Stage ERROR %d",_stage);
        }
        break;
    case 21:
        // Stay this stage
        break;

    }
#ifdef _STAGE_CHANGE_DETECT_DEBUG

    ROS_INFO_NAMED(DT_STAGE_CHANGE_DETECT,"Lopp T: %4d",_stage);

#endif // _STAGE_CHANGE_DETECT_DEBUG


    // Default False
    return false;
}
