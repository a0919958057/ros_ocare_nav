#include <ocare_hw_node.h>

OcareRobot::OcareRobot() : m_arm()
    {
    ROS_INFO("Create JointStateHandle...");

    hardware_interface::JointStateHandle state_handle_left_base_arm(
                "left_arm_base_link_joint", &pos[0], &vel[0], &eff[0]);
    m_joint_state_interface.registerHandle(state_handle_left_base_arm);

    hardware_interface::JointStateHandle state_handle_left_arm_1(
                "left_arm_1_link_joint", &pos[1], &vel[1], &eff[1]);
    m_joint_state_interface.registerHandle(state_handle_left_arm_1);

    hardware_interface::JointStateHandle state_handle_left_arm_2(
                "left_arm_2_link_joint", &pos[2], &vel[2], &eff[2]);
    m_joint_state_interface.registerHandle(state_handle_left_arm_2);

    /* Registe the Left wheel and Right wheel to hardware resource manager*/
    hardware_interface::JointStateHandle state_handle_left_wheel(
                "base_left_wheel_joint", &wheel_pos[0], &wheel_vel[0], &wheel_eff[0]);
    m_joint_state_interface.registerHandle(state_handle_left_wheel);

    hardware_interface::JointStateHandle state_handle_right_wheel(
                "base_right_wheel_joint", &wheel_pos[1], &wheel_vel[1], &wheel_eff[1]);
    m_joint_state_interface.registerHandle(state_handle_right_wheel);

    registerInterface(&m_joint_state_interface);

    ROS_INFO("Create JointHandle...");

    hardware_interface::JointHandle position_handle_left_base_arm(
                m_joint_state_interface.getHandle("left_arm_base_link_joint"), &cmd[0]);
    m_joint_position_interfece.registerHandle(position_handle_left_base_arm);

    hardware_interface::JointHandle position_handle_left_arm_1(
                m_joint_state_interface.getHandle("left_arm_1_link_joint"), &cmd[1]);
    m_joint_position_interfece.registerHandle(position_handle_left_arm_1);

    hardware_interface::JointHandle position_handle_left_arm_2(
                m_joint_state_interface.getHandle("left_arm_2_link_joint"), &cmd[2]);
    m_joint_position_interfece.registerHandle(position_handle_left_arm_2);

    registerInterface(&m_joint_position_interfece);

    /* registe wheel effort joint */
    hardware_interface::JointHandle position_handle_left_wheel(
                m_joint_state_interface.getHandle("base_left_wheel_joint"), &wheel_cmd[0]);
    m_joint_effort_interfece.registerHandle(position_handle_left_wheel);

    hardware_interface::JointHandle position_handle_right_wheel(
                m_joint_state_interface.getHandle("base_right_wheel_joint"), &wheel_cmd[1]);
    m_joint_effort_interfece.registerHandle(position_handle_right_wheel);

    registerInterface(&m_joint_effort_interfece);


}

void OcareRobot::read(ros::Time time, ros::Duration period) {
    // There is a test by giving a static Positon data
    // Current the ARM not ready yet!
    pos[0] = 0.05+0.05*sin(time.nsec / double(100000000.0));
    pos[1] = 0;
    pos[2] = 0;
    vel[0] = 0;
    vel[1] = 0;
    vel[2] = 0;
    eff[0] = 0;
    eff[1] = 0;
    eff[2] = 0;
    //ROS_INFO("Current Time %d", time.nsec);
    wheel_pos[0] = (double(time.nsec) / double(100000000.0));
    wheel_pos[1] = (time.nsec / double(100000000.0));

}

void OcareRobot::write(ros::Time time, ros::Duration period) {

    // TODO: add Robot Arm command list
    //byte aBuffer[8] = {0};

    //aBuffer[0];

   // m_serial.writeBytes();

}


