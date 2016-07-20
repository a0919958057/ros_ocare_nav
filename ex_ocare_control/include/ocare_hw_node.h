#ifndef OCARE_HW_NODE_H
#define OCARE_HW_NODE_H

#include <ros/ros.h>

// Inherent the RobotHW interface and implement it
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// Use modbus RTU protocal via Serial communication
#include <modbus/modbus-rtu.h>

// Subscribe the DiffMode, ArmMode
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int32.h>

#define _ROS


// include the robot HWModule modbus class
#include "armmodbus.h"
#include "diffmodbus.h"
#include "ardumodbus.h"

#define SLIDER_CLOSE_POSITION               (0)
#define SLIDER_OPENED_POSITION              (0.15)

/******************* ARM CONFIG ********************/

#define RIGHT_MOTOR1_INIT_VALUE				(520)
#define RIGHT_MOTOR2_INIT_VALUE				(815)

#define RIGHT_MOTOR1_MAX_VALUE				(844)
#define	RIGHT_MOTOR2_MAX_VALUE				(815)

#define RIGHT_MOTOR1_MIN_VALUE				(520)
#define RIGHT_MOTOR2_MIN_VALUE				(211)

#define RIGHT_MOTOR1_MAX_DEG                (90.0)
#define RIGHT_MOTOR1_MAX_DEG_VALUE          (RIGHT_MOTOR1_MAX_VALUE)
#define RIGHT_MOTOR1_MIN_DEG                (0.0)
#define RIGHT_MOTOR1_MIN_DEG_VALUE          (RIGHT_MOTOR1_MIN_VALUE)

#define RIGHT_MOTOR2_MAX_DEG                (90.0)
#define RIGHT_MOTOR2_MAX_DEG_VALUE          (RIGHT_MOTOR2_MIN_VALUE)
#define RIGHT_MOTOR2_MIN_DEG                (-90.0)
#define RIGHT_MOTOR2_MIN_DEG_VALUE          (RIGHT_MOTOR2_MAX_VALUE)


#define RIGHT_MOTOR1_BTN_POSE               (844)
#define RIGHT_MOTOR2_BTN_POSE           	(815)

/***************************************************/


// The Diff command Topic Struct for uint16_t array
enum DiffTopicCMD {
    DIFF_MODE_CMD,
    TRACK_TORQUE_MODE_CMD,
    SENSOR_BW_MODE_CMD
};

// The Arm command Topic Struct for uint16_t array
enum ArmTopicCMD {
    ARM_MODE_CMD,
    SLIDER_MODE_CMD,
    EFFORT_CATCH_LEVEL_CMD
};

class OcareRobot : public hardware_interface::RobotHW
{
public:
    // Initial the hardware interface :
    // Joint State interface, Joint Position interface
    OcareRobot();
    ~OcareRobot();

    // Must call this function to init the class
    void init(ros::NodeHandle* _node);

    ros::Time getTime() const { return ros::Time::now(); }
    ros::Duration getPeriod() const { return ros::Duration(0.01); }

    // Update Robot state using Packet by Serial interface
    void read(ros::Time time, ros::Duration period);

    // Write Robot state using Packet by Serial interface
    void write(ros::Time time, ros::Duration period);

    // Publish the sensor data
    void publish_sensor_data();

    // Subscribe callback for diffmode and armmode
    void diff_cmd_callback(const std_msgs::UInt16MultiArrayConstPtr _messages);
    void arm_cmd_callback(const std_msgs::UInt16MultiArrayConstPtr _messages);
    void command_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &referencePoint);



private:
    hardware_interface::JointStateInterface m_joint_state_interface;
    hardware_interface::PositionJointInterface m_joint_position_interfece;
    hardware_interface::EffortJointInterface m_joint_effort_interfece;

    double cmd[3];
    double pos[3];
    double vel[3];
    double eff[3];

    double wheel_cmd[2];
    double wheel_pos[2];
    double wheel_vel[2];
    double wheel_eff[2];

    // command of robot arm
    double m_arm1_pos;
    double m_arm2_pos;

    uint16_t m_sensor_datas[SENSOR_REG_COUNT];

    // The hardware Register define and implementation
    ArmModbus m_arm;
    DiffModbus m_diff;

    // The interface from ros to modbus
    ArduModbus m_modbus;

    // Subscriber for COMMAND
    ros::Subscriber m_diff_cmd_sub;
    ros::Subscriber m_arm_cmd_sub;
    // subscriber for arm position command
    ros::Subscriber m_sub_command;


    // Publisher who publish the Tracking line sensor
    ros::Publisher m_track_line_pub;

};

#endif // OCARE_HW_NODE_H
