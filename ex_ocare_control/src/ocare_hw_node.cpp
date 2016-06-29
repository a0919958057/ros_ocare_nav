#include <ocare_hw_node.h>



OcareRobot::OcareRobot() :
    m_modbus(),
    m_arm("Robot left arm",sizeof("Robot left arm")),
    m_diff("Robot wheel",sizeof("Robot wheel"))
    {
    ROS_INFO("Create HWModuble Object.");

    ROS_INFO("Create JointStateHandle...");


    /********************* Register the Joint to Hardware Resource Manager
     *
     * For Arm Module:
     *
     *      left_arm_base_link_joint:    The slider platform
     *
     *      left_arm_1_link_joint:       The slider platform to first arm link
     *
     *      left_arm_2_link_joint:       The first arm link to second arm link
     *
     * For DiffWheel Module:
     *
     *      base_left_wheel_joint:       The base to Left Wheel
     *
     *      base_right_wheel_joint:      The base to Right Wheel
     *
     * ******************************************************************/

    /* Registe the arm to hardware resource manager*/
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

    /********************* Register the Joint Command from Controller
     *
     * For Arm Module:
     *
     *      left_arm_base_link_joint:    Joint Position Interface(Command type: Position)
     *
     *      left_arm_1_link_joint:       Joint Position Interface(Command type: Position)
     *
     *      left_arm_2_link_joint:       Joint Position Interface(Command type: Position)
     *
     * For DiffWheel Module:
     *
     *      base_left_wheel_joint:       Joint Effort Interface(Command type: Torque)
     *
     *      base_right_wheel_joint:      Joint Effort Interface(Command type: Torque)
     *
     * ******************************************************************/

    /* registe wheel position joint command register*/
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

    /* registe wheel effort joint command register */
    hardware_interface::JointHandle position_handle_left_wheel(
                m_joint_state_interface.getHandle("base_left_wheel_joint"), &wheel_cmd[0]);
    m_joint_effort_interfece.registerHandle(position_handle_left_wheel);

    hardware_interface::JointHandle position_handle_right_wheel(
                m_joint_state_interface.getHandle("base_right_wheel_joint"), &wheel_cmd[1]);
    m_joint_effort_interfece.registerHandle(position_handle_right_wheel);

    registerInterface(&m_joint_effort_interfece);

}

OcareRobot::~OcareRobot() {}

void OcareRobot::init(ros::NodeHandle* _node) {
    // subscribe the command from ros topic
    _node->subscribe("diff_mode_cmd", 50, &OcareRobot::diff_cmd_callback, this);
    _node->subscribe("arm_mode_cmd", 50, &OcareRobot::arm_cmd_callback, this);

    // Initial the modbus
    m_modbus.init("/dev/ttyUSB0",115200,2,'N');
    //m_modbus.registerHWModule(&m_arm);
    m_modbus.registerHWModule(&m_diff);
    m_modbus.connect_slave();
}



/****************** Subscribe callback for diffmode command *******************/

void OcareRobot::diff_cmd_callback(const std_msgs::UInt16MultiArrayConstPtr _messages) {

    // Read the Topic message by specific format, read : DiffTopicCMD
    uint16_t diff_mode      = _messages->data.at(DiffTopicCMD::DIFF_MODE_CMD);
    uint16_t torque_mode    = _messages->data.at(DiffTopicCMD::TRACK_TORQUE_MODE_CMD);

    // Pharse the command and write to DiffModbus class
    switch(DiffModbus::ChassisModeCMD(diff_mode)) {
    case DiffModbus::ChassisModeCMD::MODE_TRACK_LINE_CMD:
    case DiffModbus::ChassisModeCMD::MODE_CONTROLLABLE_CMD:
    case DiffModbus::ChassisModeCMD::MODE_STOP_CMD:

        // If the mode is exist then set mode
        m_diff.m_chassis_mode = DiffModbus::ChassisModeCMD(diff_mode);

        break;
    default:
        ROS_ERROR("Can't reslove the Diffmode Mode type !\n");
        break;
    }

    switch(DiffModbus::TrackingTorqueModeCMD(torque_mode)) {
    case DiffModbus::TrackingTorqueModeCMD::TORQUE_LOW_CMD:
    case DiffModbus::TrackingTorqueModeCMD::TORQUE_MED_CMD:
    case DiffModbus::TrackingTorqueModeCMD::TORQUE_HIGH_CMD:

        // If the mode is exist then set mode
        m_diff.m_track_torque_mode = DiffModbus::TrackingTorqueModeCMD(torque_mode);

        break;
    default:
        ROS_ERROR("Can't reslove the Diffmode Torque type !\n");
        break;
    }


}

/****************** Subscribe callback for arm mode command *******************/

void OcareRobot::arm_cmd_callback(const std_msgs::UInt16MultiArrayConstPtr _messages) {

    // Read the Topic message by specific format, read : ArmTopicCMD
    uint16_t arm_mode       = _messages->data.at(ArmTopicCMD::ARM_MODE_CMD);
    uint16_t slider_mode    = _messages->data.at(ArmTopicCMD::SLIDER_MODE_CMD);
    uint16_t catch_level    = _messages->data.at(ArmTopicCMD::EFFORT_CATCH_LEVEL_CMD);

    // Pharse the command and write to ArmModbus class
    switch(ArmModbus::ArmModeCMD(arm_mode)) {
    case ArmModbus::ArmModeCMD::ARM_HOME_CMD:
    case ArmModbus::ArmModeCMD::ARM_BUTTON_POSE_CMD:
    case ArmModbus::ArmModeCMD::ARM_FREE_CONTROLL_CMD:

        // If the mode is exist then set mode
        m_arm.m_l_mode = ArmModbus::ArmModeCMD(arm_mode);

        break;
    default:
        ROS_ERROR("Can't reslove the ArmMode type !\n");
        break;
    }

    switch(ArmModbus::SliderStateCMD(slider_mode)) {
    case ArmModbus::SliderStateCMD::SLIDER_OPEN_CMD:
    case ArmModbus::SliderStateCMD::SLIDER_CLOSE_CMD:

        // If the mode is exist then set mode
        m_arm.m_l_slider_mode = ArmModbus::SliderStateCMD(slider_mode);

        break;
    default:
        ROS_ERROR("Can't reslove the SliderMode type !\n");
        break;
    }

    if(catch_level >= 0 && catch_level <= 100) {
        m_arm.m_l_catch_level = catch_level;
    }

}

/********************* Variable mapping
 *
 * For Arm Module:
 *
 *      left_arm_base_link_joint:
 *          - pos[0]:   position(rad) double
 *          - cmd[0]:   position(rad) double
 *
 *      left_arm_1_link_joint:       Joint Position Interface(Command type: Position)
 *          - pos[1]:   position(rad) double
 *          - cmd[1]:   position(rad) double
 *
 *      left_arm_2_link_joint:       Joint Position Interface(Command type: Position)
 *          - pos[2]:   position(rad) double
 *          - cmd[2]:   position(rad) double
 *
 * For DiffWheel Module:
 *
 *      base_left_wheel_joint:       Joint Effort Interface(Command type: Torque)
 *         - wheel_pos[0]:  torque(N-m) double
 *         - wheel_cmd[0]:  torque(N-m) double
 *
 *      base_right_wheel_joint:      Joint Effort Interface(Command type: Torque)
 *         - wheel_pos[1]:  torque(N-m) double
 *         - wheel_cmd[0]:  torque(N-m) double
 *
 * ******************************************************************/

void OcareRobot::read(ros::Time time, ros::Duration period) {

    // Sync data from robot
    m_modbus.read();

    // Sync data from ArmModbus to ROS
    switch (m_arm.m_l_slider_mode) {
    case ArmModbus::SliderState::SLIDER_HOME:
        pos[0] = SLIDER_CLOSE_POSITION;
        break;
    case ArmModbus::SliderState::SLIDER_OPENING:
        pos[0] = (SLIDER_CLOSE_POSITION + SLIDER_OPENED_POSITION) / 2;
        break;
    case ArmModbus::SliderState::SLIDER_OPENED:
        pos[0] = SLIDER_OPENED_POSITION;
        break;
    case ArmModbus::SliderState::SLIDER_RETURNING:
        pos[0] = (SLIDER_CLOSE_POSITION + SLIDER_OPENED_POSITION) / 2;
        break;
    }
    pos[1] = m_arm.m_read_l_motor1_degree;
    pos[2] = m_arm.m_read_l_motor2_degree;

    // Sync data from DiffModbus to ROS
    // Current no data can read



}

void OcareRobot::write(ros::Time time, ros::Duration period) {

    // Sync data from ROS to ArmModbus
    m_arm.m_l_motor1_degree = cmd[1];
    m_arm.m_l_motor2_degree = cmd[2];

    // Sync data from ROS to DiffModbus
    m_diff.m_left_wheel_torque = wheel_cmd[0];
    m_diff.m_right_wheel_torque = wheel_cmd[1];

    // Sync data to robot
    m_modbus.write();

}


