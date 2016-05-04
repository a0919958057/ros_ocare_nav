#ifndef OCARE_HW_NODE_H
#define OCARE_HW_NODE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// include the robot arm library
#include "robotarm.h"

class OcareRobot : public hardware_interface::RobotHW
{
public:
    // Initial the hardware interface :
    //      Joint State interface, Joint Position interface
    OcareRobot();

    ros::Time getTime() const { return ros::Time::now(); }
    ros::Duration getPeriod() const { return ros::Duration(0.01); }

    // Update Robot state using Packet by Serial interface
    void read(ros::Time time, ros::Duration period);

    // Write Robot state using Packet by Serial interface
    void write(ros::Time time, ros::Duration period);




private:
    hardware_interface::JointStateInterface m_joint_state_interface;
    hardware_interface::EffortJointInterface m_joint_position_interfece;
    double cmd[3];
    double pos[3];
    double vel[3];
    double eff[3];

    RobotArm m_arm;

};

#endif // OCARE_HW_NODE_H
