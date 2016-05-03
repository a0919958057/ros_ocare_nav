#ifndef OCARE_HW_NODE_H
#define OCARE_HW_NODE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class OcareRobot : public hardware_interface::RobotHW
{
public:
    // Initial the hardware interface :
    //      Joint State interface, Joint Position interface
    OcareRobot();

    // Update Robot state using Packet by Serial interface
    void update_state_from_robot();

    // Write Robot state using Packet by Serial interface
    void write_cmd_to_robot();




private:
    hardware_interface::JointStateInterface m_joint_state_interface;
    hardware_interface::PositionJointInterface m_joint_position_interfece;
    double cmd[3];
    double pos[3];
    double vel[3];
    double eff[3];
};

#endif // OCARE_HW_NODE_H
