#ifndef OCARE_HW_NODE_H
#define OCARE_HW_NODE_H

#include <ros/ros.h>

// Inherent the RobotHW interface and implement it
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// Use modbus RTU protocal via Serial communication
#include <modbus/modbus-rtu.h>

// Subscribe the DiffMode, ArmMode
#include <std_msgs/UInt16MultiArray.h>

#define _ROS


// include the robot HWModule modbus class
#include "armmodbus.h"
#include "diffmodbus.h"
#include "ardumodbus.h"

#define SLIDER_CLOSE_POSITION 0
#define SLIDER_OPENED_POSITION 0.15

// The Diff command Topic Struct for uint16_t array
enum DiffTopicCMD {
    DIFF_MODE_CMD,
    TRACK_TORQUE_MODE_CMD
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

    // Subscribe callback for diffmode and armmode
    void diff_cmd_callback(const std_msgs::UInt16MultiArrayConstPtr _messages);
    void arm_cmd_callback(const std_msgs::UInt16MultiArrayConstPtr _messages);



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

    // The hardware Register define and implementation
    ArmModbus m_arm;
    DiffModbus m_diff;

    // The interface from ros to modbus
    ArduModbus m_modbus;

    // Subscriber for COMMAND
    ros::Subscriber m_diff_cmd_sub;
    ros::Subscriber m_arm_cmd_sub;

};

#endif // OCARE_HW_NODE_H
