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

//#define __arm__

#ifdef __arm__

extern "C" {
#include <wiringPi.h>
}

#define PIN_SW_NEED_BULB_TESK (21)
#define PIN_SW_START          (22)
bool fg_need_bulb(false);
bool fg_start(false);
#else
bool fg_need_bulb(true);
bool fg_start(true);

#endif


/****************    Config   *******************/

#define NO_LINE_THROSHOLD (SENSOR_REG_COUNT * 100 - 100)
#define SIZE_DATA_RECOARD (10)
#define CONVERG_THROSHOLD (M_PI * 5.0/180.0)

#define TASK_8_LENGTH_FRONT     (0.3)
#define TASK_10_LENGTH_RIGHT    (0.4)
#define TASK_13_LENGTH_FRONT    (0.5)

/************************************************/

/**************** DEBUG Flags ******************/

//#define _DEBUG_SENSOR

#define DT_STAGE_CHANGE_DETECT "StageChangeDetecter"
#define DT_LOOP_TESK_DEBUG     "LoopTesk"

#define _STAGE_CHANGE_DETECT_DEBUG
#define _LOOP_TESK_DEBUG

/***********************************************/

double front_length(10);
double right_length(10);
double left_length(10);

double orient(0);

bool sensor_ready(false);
bool imu_ready(false);
bool laser_ready(false);

int sensor_value[SENSOR_REG_COUNT] = { 0 };

void loop_tesk(int _stage, ros::Publisher* _diff_pub, ros::Publisher* _diff_twist_pub);
void do_gohome_tesk(int _stage, std_msgs::UInt16MultiArray* _mode_msg, geometry_msgs::Twist* _twist_msg);
void do_button_tesk(int _stage, std_msgs::UInt16MultiArray* _mode_msg, geometry_msgs::Twist* _twist_msg);
bool stage_change_detect(int _stage);
double get_sensor_average();
bool is_sensor_noline();
double cot_angle(double _degree);

void callback_laser(const sensor_msgs::LaserScanConstPtr &msg) {
    double length_sum(0);
    for(int i=0;i<10;i++)
     length_sum += msg.get()->ranges[i];
    right_length = length_sum / 10;

    length_sum = 0;
    for(int i=250;i<260;i++)
     length_sum += msg.get()->ranges[i];
    front_length = length_sum / 10;

    length_sum = 0;
    for(int i=500;i<510;i++)
     length_sum += msg.get()->ranges[i];
    left_length = length_sum / 10;

    laser_ready = true;
}

void callback_imu(const sensor_msgs::ImuConstPtr &msg) {
    tf::Matrix3x3 m(tf::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w));
    double roll, pitch, yaw;
    m.getRPY(roll,pitch,yaw);
    orient = (yaw);
    imu_ready = true;
}

void callback_sensor(const std_msgs::UInt16MultiArrayConstPtr &msg) {
    const short unsigned int* ptr = msg->data.data();
    for(int i=0; i<SENSOR_REG_COUNT; i++) {
        sensor_value[i] = 100-ptr[i];
    }
    sensor_ready = true;

    ROS_DEBUG_NAMED("SensorCallback","%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
            sensor_value[0],
            sensor_value[1],
            sensor_value[2],
            sensor_value[3],
            sensor_value[4],
            sensor_value[5],
            sensor_value[6],
            sensor_value[7],
            sensor_value[8],
            sensor_value[9],
            sensor_value[10],
            sensor_value[11],
            sensor_value[12]);
}

int main(int argc, char** argv) {

#ifdef __arm__
    if( wiringPiSetup() == -1)
        ROS_ERROR("WiringPi LIBRARY can't loading.");
#endif

    // Initial the ROS
    ros::init(argc,argv,"diff_test_stage_node");

    // Create a ros node
    ros::NodeHandle node;
    // Subscribe the laser scan topic
    ros::Subscriber laser_sub =
            node.subscribe<sensor_msgs::LaserScan>("/scan",50,callback_laser);

    // Subscribe the IMU topic
    ros::Subscriber imu_sub =
            node.subscribe<sensor_msgs::Imu>("/imu",50,callback_imu);

    ros::Subscriber sensor_sub =
            node.subscribe<std_msgs::UInt16MultiArray>("/track_line_sensor",50,callback_sensor);

/******************************** Publishers ********************************/
    /********** The Diff command Topic Struct for uint16_t array
    *  enum DiffTopicCMD {
    *     DIFF_MODE_CMD,
    *     TRACK_TORQUE_MODE_CMD,
    *     SENSOR_BW_MODE_CMD
    *  };
    ***************************************************************/


    /******Defination of the Chassis Mode for Control Mode
     * MODE_TRACK_LINE :     Tracking the path and follow the path
     * MODE_CONTROLLABLE:    The Arduino would use LEFT_WHEEL_TORQUE and RIGHT_WHEEL_TORQUE to effort the wheel
     * MODE_STOP:            All of the driver would stop until Mode become others.
     * **************************/

    /****** Defination of the Chassis Mode for Tracking line
     * HIGH :           Useing the higher torque Fuzzy rule
     * MED:             Useing the normal torque Fuzzy rule
     * LOW:             Useing the low torque Fuzzy rule
     * **************************/

    /******Defination of the Sensor BW Mode
     * BLACK :           Tracking the black line
     * WRITE:            Tracking the white line
     * **************************/

    // Create a publish that publish the differential wheel Mode
    ros::Publisher diff_pub =
            node.advertise<std_msgs::UInt16MultiArray>("/diff_mode_cmd",50);

    /******Defination of the Chassis Torque CMD
     * Linear.x :            The forward speed CMD
     * Angular.z:            The Orient CMD
     * **************************/

    // Create a publish that publish the differential wheel Torque CMD
    ros::Publisher diff_twist_pub =
            node.advertise<geometry_msgs::Twist>("/ocare/pose_fuzzy_controller/diff_cmd",50);


/*****************************************************************************/

    // Using the 10 hz for while loop
    ros::Rate r(20);

    int stage(0);

    printf("Which stage to start?(0~20) \n");
    scanf("%d", &stage);
    if(stage < 0 || stage > 21) {
        printf("Parameter error, Start at stage 0");
        stage = 0;
    }

    while(ros::ok()) {

#ifdef __arm__
        fg_need_bulb    =  digitalRead(PIN_SW_NEED_BULB_TESK);
        fg_start        =  digitalRead(PIN_SW_START);
#endif

#ifdef _DEBUG_SENSOR


        ROS_INFO("%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
                sensor_value[0],
                sensor_value[1],
                sensor_value[2],
                sensor_value[3],
                sensor_value[4],
                sensor_value[5],
                sensor_value[6],
                sensor_value[7],
                sensor_value[8],
                sensor_value[9],
                sensor_value[10],
                sensor_value[11],
                sensor_value[12]);

#else
        // Makesure every sensor topic is ready.
        if( sensor_ready && imu_ready && laser_ready ) {
            // Do the current stage tesk
            loop_tesk(stage, &diff_pub, &diff_twist_pub);

            // Detect the stage change
            if(stage == 11 && fg_need_bulb) {   // If Need Bulb tesk and stage is 11, then Change stage to Bulk tesk
                if(stage_change_detect(stage)) stage = 110;
            }
            else if( stage >= 110 && stage <=113) {
                if(stage_change_detect(stage)) stage++;
                if(stage == 114) stage = 12;
            }
            else if (stage == 20) {
                if(stage_change_detect(stage)) stage = 200;
            }
            else if( stage >= 200 && stage <=202) {
                if(stage_change_detect(stage)) stage++;
                if(stage == 203) stage = 21;
            }
            else {

                if(stage_change_detect(stage)) {
                    ROS_INFO("STAGE CHANGE");
                    stage++;
                }
            }
        }
#endif




        // Sleep for a while
        ros::spinOnce();
        r.sleep();
    }



    return 0;
}

void loop_tesk(int _stage, ros::Publisher* _diff_pub, ros::Publisher* _diff_twist_pub) {

    // The chassic's Mode CMD message
    std_msgs::UInt16MultiArray cmd_message;
    // The differential wheel Torque CMD message
    geometry_msgs::Twist cmd_twist;

    cmd_message.data.clear();

    switch(_stage) {
    case 0:
        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_STOP_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 0;
        cmd_twist.angular.z = 0;
        break;
    case 1:

        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 0;
        cmd_twist.angular.z = M_PI * -45.0/180.0;
        break;
    case 2:

        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_TRACK_LINE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_LOW_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 0;
        cmd_twist.angular.z = 0;
        break;
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:

        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_TRACK_LINE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 0;
        cmd_twist.angular.z = 0;
        break;
    case 8:

        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 50;
        cmd_twist.angular.z = M_PI * 90.0/180.0;
        break;
    case 9:

        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 0;
        cmd_twist.angular.z = M_PI * 180.0/180.0;
        break;
    case 10:

        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 50;
        cmd_twist.angular.z = M_PI * 180.0/180.0 - (right_length - TASK_10_LENGTH_RIGHT);
        break;
    case 11:
        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 40;
        cmd_twist.angular.z = M_PI * 180.0/180.0;
        break;
    case 110:
    case 111:
    case 112:
    case 113:

        do_button_tesk(_stage, &cmd_message, &cmd_twist);
        break;
    case 12:

        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_TRACK_LINE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 0;
        cmd_twist.angular.z = 0;
        break;
    case 13:
        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 50;
        cmd_twist.angular.z = 0;
        break;
    case 14:
        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 0;
        cmd_twist.angular.z = M_PI * 90.0/180.0;
        break;
    case 15:
        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 50;
        cmd_twist.angular.z = M_PI * 90.0/180.0;
        break;
    case 16:
        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_TRACK_LINE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 0;
        cmd_twist.angular.z = 0;
        break;
    case 17:
        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_TRACK_LINE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::BLACK_CMD);
        cmd_twist.linear.x = 0;
        cmd_twist.angular.z = 0;
        break;
    case 18:
    case 19:
        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_TRACK_LINE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 0;
        cmd_twist.angular.z = 0;
        break;
    case 20:
    case 200:
    case 201:
    case 202:

        do_gohome_tesk(_stage, &cmd_message, &cmd_twist);
        break;
    case 21:
        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_STOP_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        cmd_twist.linear.x = 0;
        cmd_twist.angular.z = 0;
        break;
    }

#ifdef _LOOP_TESK_DEBUG

    ROS_INFO_NAMED(DT_LOOP_TESK_DEBUG,"STAGE: %4d", _stage);

#endif // _LOOP_TESK_DEBUG

    // Publish the topic
    _diff_pub->publish(cmd_message);
    _diff_twist_pub->publish(cmd_twist);
}

class ConvergDetector {
public:
    ConvergDetector() :
        index_counter(0),
        fg_inited(false),
        fg_started(false),
        ref_data(0.0) {}

    ~ConvergDetector() {}

private:

    // Flag for Detector status
    bool fg_inited;
    bool fg_started;
    int index_counter;

    // Detector registers
    double ref_data;
    double data_recoard[SIZE_DATA_RECOARD];
public:
    // Need init first
    void init(double _ref) {
        fg_inited = true;
        ref_data = _ref;
        for(int i=0;i<SIZE_DATA_RECOARD;i++) {
            data_recoard[i] = 10.0;
        }
    }

    bool start() {
        if(fg_inited) {
            fg_started = true;
            return true;
        }
        else {
            return false;
        }
    }

    void stop() {
        fg_started = false;
        fg_inited = false;
    }

    void update() {
        if(fg_started) {
            data_recoard[index_counter] = cot_angle(fabs(ref_data - orient));
            index_counter++;
            if(index_counter == SIZE_DATA_RECOARD) index_counter = 0;
            ROS_INFO_NAMED("ConvergDetector", "Debug : Error= %f ,ref_data = %f,orient = %f",
                           fabs(ref_data - orient),ref_data,orient);
        }
        else {
            ROS_ERROR_NAMED("ConvergDetector", "Need start first!");
        }
    }

    bool isConverged() {

        if(fg_started) {
            double sum(0);
            double avg(0);
            for(int i=0;i<SIZE_DATA_RECOARD;i++) {
                sum += data_recoard[i];
            }
            avg = sum / SIZE_DATA_RECOARD;
            ROS_INFO_NAMED("ConvergDetector", "Debug : Avg = %f ",avg);
            return (CONVERG_THROSHOLD > avg);

        }

        else {
            ROS_ERROR_NAMED("ConvergDetector", "Need start first!");
            return false;
        }

    }

    bool isStarted() {
        return fg_started;
    }
};

class WBDetector {
public:
    WBDetector() :
        fg_inited(false),
        fg_started(false) {}

    ~WBDetector() {}

private:

    // Flag for Detector status
    bool fg_inited;
    bool fg_started;
    bool fg_w2b;

    int head;
    int w_detected;
    int b_detected;

public:
    // Need init first
    void init(bool _is_w2b) {
        fg_w2b = _is_w2b;
        head = 0;
        w_detected = -1;
        b_detected = -1;
        fg_inited = true;
    }

    bool start() {
        if(fg_inited) {
            fg_started = true;
            return true;
        }
        else {
            return false;
        }
    }

    void stop() {
        fg_started = false;
        fg_inited = false;
    }

    void update() {
        ROS_DEBUG_NAMED("WBDetector", "w_detected:%d, b_detected:%d, sensor AVG: %f",
                        w_detected, b_detected, get_sensor_average());
        if(fg_started) {
            if( get_sensor_average() > 70.0 && b_detected < 0)
                b_detected = head ++;
            if( get_sensor_average() < 30.0 && w_detected < 0)
                w_detected = head ++;
        }
        else {
            ROS_ERROR_NAMED("WBDetector", "Need start first!");
        }
    }

    bool isDetected() {
        if(fg_started) {
            if(b_detected < 0 || w_detected < 0) {
                return false;
            }
            else {
                if(fg_w2b)
                    return b_detected > w_detected;
                else
                    return w_detected > b_detected;
            }
        }
        else {
            ROS_ERROR_NAMED("WBDetector", "Need start first!");
            return false;
        }

    }

    bool isStarted() {
        return fg_started;
    }
};


bool stage_change_detect(int _stage) {
    // Count the times of robot distence over minimum limit
    static int laser_distence_overlimit_conter(0);
    static bool fg_usetimer(false);
    static ros::Time last_time = ros::Time::now();
    static ConvergDetector stable_detector;
    static WBDetector wb_detector;


    switch(_stage) {
    case 0:
        // Detect the button stat, if Switch is on then change stage
        if(fg_start) return true;
        break;
    case 1:
        // Detect the Orient is stable at -45 degree
        if(!stable_detector.isStarted()) {
            stable_detector.init(M_PI * -45.0/180);
            stable_detector.start();
        } else {
            stable_detector.update();
        }
        if(stable_detector.isConverged()) {
            stable_detector.stop();
            return true;
        }

        break;
    case 2:
        // Detect the Orient is stable at -90 degree
        if(!stable_detector.isStarted()) {
            stable_detector.init(M_PI * -90.0/180);
            stable_detector.start();
        } else {
            stable_detector.update();
        }
        if(stable_detector.isConverged()) {
            stable_detector.stop();
            return true;
        }

        break;
    case 3:
        // Detect the Orient is cross at 0 degree
        last_time = ros::Time::now();
        ROS_INFO("Orient %f",orient);
        if(fabs(orient - M_PI * 0.0/180.0) < 0.1) return true;

        break;
    case 4:
        // Detect the Orient is cross at 0 degree and Timing from last stage large than 1 sec
        if(fabs(last_time.toSec() - ros::Time::now().toSec()) > 1.0 && fabs(orient - M_PI * 0.0/180.0) < 0.1) {
            last_time = ros::Time::now();
            return true;
        }
        break;
    case 5:
        // Detect the Orient is cross at 0 degree and Timing from last stage large than 1 sec
        if(fabs(last_time.toSec() - ros::Time::now().toSec()) > 1.0 && fabs(orient - M_PI * 0.0/180.0) < 0.1) {
            last_time = ros::Time::now();
            return true;
        }
        break;
    case 6:
        // Detect the Orient is cross at 0 degree and Timing from last stage large than 1 sec
        if(fabs(last_time.toSec() - ros::Time::now().toSec()) > 1.0 && fabs(orient - M_PI * 0.0/180.0) < 0.1) {
            last_time = ros::Time::now();
            return true;
        }
        break;
    case 7:
        // Detect the Orient is stable at 90 degree and no line
        if(!stable_detector.isStarted()) {
            stable_detector.init(M_PI * 90.0/180);
            stable_detector.start();
        } else {
            stable_detector.update();
        }
        if(stable_detector.isConverged()  && is_sensor_noline() ) {
            stable_detector.stop();
            return true;
        }

        break;
    case 8:
        // Detect the Orient is stable at 90 degree and Distence is less than 0.3 m
        if(!stable_detector.isStarted()) {
            stable_detector.init(M_PI * 90.0/180);
            stable_detector.start();
        } else {
            stable_detector.update();
        }
        if(front_length < TASK_8_LENGTH_FRONT)
            laser_distence_overlimit_conter ++;

        // If there the distence less than 0.3 m too many times, we have confidence say that we need turning now
        if(stable_detector.isConverged()  && laser_distence_overlimit_conter > 3 ) {
            laser_distence_overlimit_conter = 0;
            stable_detector.stop();
            return true;
        }

        break;

    case 9:
        // Detect the Orient is stable at 180 degree
        if(!stable_detector.isStarted()) {
            stable_detector.init(M_PI * 180.0/180);
            stable_detector.start();
        } else {
            stable_detector.update();
        }
        if(stable_detector.isConverged()) {
            stable_detector.stop();
            return true;
        }
        break;
    case 10:
        // Detect all Black change to White
        if(!wb_detector.isStarted()) {
            wb_detector.init(false);
            wb_detector.start();
        } else {
            wb_detector.update();
        }
        if(wb_detector.isDetected()) {
            wb_detector.stop();
            return true;
        }
        break;
    case 11:
        if(!wb_detector.isStarted()) {
            wb_detector.init(true);
            wb_detector.start();
        } else {
            wb_detector.update();
        }
        if(wb_detector.isDetected()) {
            wb_detector.stop();
            if(fg_need_bulb) {
                fg_usetimer = true;
                last_time = ros::Time::now();
            }
            return true;
        }
        break;
    case 110:
        if(fg_usetimer) {
            if(ros::Time::now().toSec() - last_time.toSec() > 5.0) {
                last_time = ros::Time::now();
                return true;
            }
        } else {
            ROS_ERROR("Stage ERROR %d",_stage);
        }
        break;
    case 111:
        if(fg_usetimer) {
            if(ros::Time::now().toSec() - last_time.toSec() > 5.0) {
                last_time = ros::Time::now();
                return true;
            }
        } else {
            ROS_ERROR("Stage ERROR %d",_stage);
        }
        break;
    case 112:
        if(fg_usetimer) {
            if(ros::Time::now().toSec() - last_time.toSec() > 5.0) {
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
        if(!wb_detector.isStarted()) {
            wb_detector.init(true);
            wb_detector.start();
        } else {
            wb_detector.update();
        }
        if(wb_detector.isDetected()) {
            wb_detector.stop();
            return true;
        }

        break;
    case 12:
        // Detect the Orient is stable at 0 degree and no line
        if(!stable_detector.isStarted()) {
            stable_detector.init(M_PI * 0.0/180);
            stable_detector.start();
        } else {
            stable_detector.update();
        }
        if(stable_detector.isConverged() && is_sensor_noline()) {
            stable_detector.stop();
            return true;
        }

        break;
    case 13:
        // Detect the Orient is stable at 0 degree and Distence is less than 0.5 m
        if(!stable_detector.isStarted()) {
            stable_detector.init(M_PI * 0.0/180);
            stable_detector.start();
        } else {
            stable_detector.update();
        }
        if(front_length < TASK_13_LENGTH_FRONT)
            laser_distence_overlimit_conter ++;

        // If there the distence less than 0.3 m too many times, we have confidence say that we need turning now
        if(stable_detector.isConverged()  && laser_distence_overlimit_conter > 3 ) {
            laser_distence_overlimit_conter = 0;
            stable_detector.stop();
            return true;
        }
        break;
    case 14:
        // Detect the Orient is stable at 90 degree
        if(!stable_detector.isStarted()) {
            stable_detector.init(M_PI * 90.0/180);
            stable_detector.start();
        } else {
            stable_detector.update();
        }
        if(stable_detector.isConverged()) {
            stable_detector.stop();
            return true;
        }
        break;
    case 15:
        // Detect any white
        for(int i=0; i<SENSOR_REG_COUNT; i++) {
            if(sensor_value[i] < 30) return true;
        }
        break;
    case 16:
        // Detect the Orient is stable at 180 degree and Average turn to white
        if(!stable_detector.isStarted()) {
            stable_detector.init(M_PI * 180.0/180);
            stable_detector.start();
        } else {
            stable_detector.update();
        }
        if(stable_detector.isConverged() && get_sensor_average() < 30.0) {
            stable_detector.stop();
            return true;
        }

        break;
    case 17:
        // Detect the Orient is stable at 180 degree and Average turn to black
        if(!stable_detector.isStarted()) {
            stable_detector.init(M_PI * 180.0/180);
            stable_detector.start();
        } else {
            stable_detector.update();
        }
        if(stable_detector.isConverged() && get_sensor_average() > 70.0) {
            stable_detector.stop();
            return true;
        }

        break;
    case 18:
        // Detect the Orient is stable at -90 degree
        if(!stable_detector.isStarted()) {
            stable_detector.init(M_PI * -90.0/180);
            stable_detector.start();
        } else {
            stable_detector.update();
        }
        if(stable_detector.isConverged()) {
            stable_detector.stop();
            return true;
        }
        break;
    case 19:
        // Detect right sensor white
        if(sensor_value[12] < 50 && sensor_value[11] < 50) return true;
        break;
    case 20:
        fg_usetimer = true;
        last_time = ros::Time::now();
        return true;
    case 200:
        if(fg_usetimer) {
            if(ros::Time::now().toSec() - last_time.toSec() > 3.0) {
                last_time = ros::Time::now();
                return true;
            }
        } else {
            ROS_ERROR("Stage ERROR %d",_stage);
        }
        break;
    case 201:
        if(fg_usetimer) {
            if(ros::Time::now().toSec() - last_time.toSec() > 4.0) {
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

void do_button_tesk(int _stage, std_msgs::UInt16MultiArray* _mode_msg, geometry_msgs::Twist* _twist_msg) {
    switch(_stage) {
    case 110:
        _mode_msg->data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        _twist_msg->linear.x = 40;
        _twist_msg->angular.z = M_PI * 180.0/180.0;
        break;
    case 111:
        _mode_msg->data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        _twist_msg->linear.x = 0;
        _twist_msg->angular.z = M_PI * 150.0/180.0;
        break;
    case 112:
        _mode_msg->data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        _twist_msg->linear.x = 0;
        _twist_msg->angular.z = M_PI * -170/180.0;
        break;
    case 113:
        _mode_msg->data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        _twist_msg->linear.x = 40;
        _twist_msg->angular.z = M_PI * -170/180.0;
        break;
    }
    return;

}

void do_gohome_tesk(int _stage, std_msgs::UInt16MultiArray* _mode_msg, geometry_msgs::Twist* _twist_msg) {
    switch(_stage) {
    case 20:
    case 200:
        _mode_msg->data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        _twist_msg->linear.x = 40;
        _twist_msg->angular.z = M_PI * -90.0/180.0;
        break;
    case 201:
        _mode_msg->data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        _twist_msg->linear.x = 0;
        _twist_msg->angular.z = M_PI * 0.0/180.0;
        break;
    case 202:
        _mode_msg->data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);
        _mode_msg->data.push_back((uint16_t)DiffModbus::WHITE_CMD);
        _twist_msg->linear.x = -40;
        _twist_msg->angular.z = M_PI * 0.0/180.0;
        break;
    }
    return;
}

bool is_sensor_noline() {
    int sum(0);
    for(int i=0; i<SENSOR_REG_COUNT; i++) {
            sum += sensor_value[i];
    }
    ROS_INFO("SENSOR VALUE[0] = %d", sensor_value);
    ROS_INFO("NOLINE DEBUG sum = %d", sum);
    if(sum > NO_LINE_THROSHOLD)
        return true;
    else
        return false;
}

double get_sensor_average() {
    int sum(0);
    for(int i=0; i<SENSOR_REG_COUNT; i++) {
        sum += sensor_value[i];
    }
    return sum / SENSOR_REG_COUNT;
}

double cot_angle(double _degree) {

    if( _degree < 0.0) {
        double times_ = fabs(_degree) / (2*M_PI);
        return ( fmod( _degree + ( 2*M_PI * (floor(times_)+1) ) + M_PI  ,2*M_PI) - M_PI);
    } else if ( _degree > 0.0) {
        return ( fmod( _degree + M_PI ,2*M_PI) - M_PI);
    } else {
        return 0.0;
    }

}
