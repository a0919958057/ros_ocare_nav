#include <ocare_hw_node.h>



OcareRobot::OcareRobot() :
    m_modbus(),
    m_arm("Robot left arm",sizeof("Robot left arm")),
    m_diff("Robot wheel",sizeof("Robot wheel")),
    m_arm1_pos(0.0),
    m_arm2_pos(0.0)
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
    m_diff_cmd_sub = _node->subscribe("/diff_mode_cmd", 50, &OcareRobot::diff_cmd_callback, this);
    m_arm_cmd_sub = _node->subscribe("/arm_mode_cmd", 50, &OcareRobot::arm_cmd_callback, this);
    // setup the subscribe for arm command
    m_sub_command = _node->subscribe("/arm_position_cmd", 1000, &OcareRobot::command_callback, this);
    m_track_line_pub = _node->advertise<std_msgs::UInt16MultiArray>("/track_line_sensor", 100);

    // Initial the modbus
    m_modbus.init("/dev/ttyUSB0",115200,2,'N');


    m_modbus.registerHWModule(&m_diff);
    m_modbus.registerHWModule(&m_arm);

    m_modbus.connect_slave();
}



/****************** Subscribe callback for diffmode command *******************/

void OcareRobot::diff_cmd_callback(const std_msgs::UInt16MultiArrayConstPtr _messages) {

    if(_messages->data.size() != 3) {
        ROS_ERROR("diff_cmd_callback Array size error");
        return;
    }

    // Read the Topic message by specific format, read : DiffTopicCMD
    uint16_t diff_mode      = _messages->data.at(DiffTopicCMD::DIFF_MODE_CMD);
    uint16_t torque_mode    = _messages->data.at(DiffTopicCMD::TRACK_TORQUE_MODE_CMD);
    uint16_t bw_mode        = _messages->data.at(DiffTopicCMD::SENSOR_BW_MODE_CMD);

    // Pharse the command and write to DiffModbus class
    switch(DiffModbus::ChassisModeCMD(diff_mode)) {
    case DiffModbus::ChassisModeCMD::MODE_TRACK_LINE_CMD:
    case DiffModbus::ChassisModeCMD::MODE_CONTROLLABLE_CMD:
    case DiffModbus::ChassisModeCMD::MODE_STOP_CMD:

        //ROS_INFO("Received DIFF MOD CMD topic : %d", diff_mode);
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

    switch(DiffModbus::SensorBWModeCMD(bw_mode)) {
    case DiffModbus::SensorBWModeCMD::BLACK_CMD:
    case DiffModbus::SensorBWModeCMD::WHITE_CMD:

        // If the mode is exist then set mode
        m_diff.m_sensor_bw_mode = DiffModbus::SensorBWModeCMD(bw_mode);

        break;
    default:
        ROS_ERROR("Can't reslove the Diffmode Torque type !\n");
        break;
    }



}

/****************** Subscribe callback for arm mode command *******************/

void OcareRobot::arm_cmd_callback(const std_msgs::UInt16MultiArrayConstPtr _messages) {

    if(_messages->data.size() != 3) {
        ROS_ERROR("arm_cmd_callback Array size error");
        return;
    }

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

void OcareRobot::command_callback(
        const trajectory_msgs::JointTrajectoryPoint::ConstPtr &referencePoint) {
    // TODO: get TrajectoryPoint and set the value to Class

    if(referencePoint->positions.size() != 2) {
        ROS_ERROR("FuzzyController::command_callback positions.size error");
        return;
    }

    const double *pos_ = referencePoint->positions.data();

    // The first position data is arm1, and second position data is arm2
    m_arm1_pos = pos_[0];
    m_arm2_pos = pos_[1];

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


    // Publish the sensor data
    publish_sensor_data();

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

    // Remapping the arm position cmd

    /************** Arm Mapping Information*******************
     *
     *  Arm1
     *      0   :   520     RIGHT_MOTOR1_MIN_VALUE
     *      90  :   844     RIGHT_MOTOR1_MAX_VALUE
     *
     *  Arm2
     *      -90 :   815     RIGHT_MOTOR2_MAX_VALUE
     *      0   :   513
     *      90  :   211     RIGHT_MOTOR2_MIN_VALUE
     *
     * *****************************************************/
    double new_arm1_pos(RIGHT_MOTOR1_INIT_VALUE);
    double new_arm2_pos(RIGHT_MOTOR2_INIT_VALUE);

    // Remapping the Arm1 pos
    if(m_arm1_pos > RIGHT_MOTOR1_MAX_DEG)

        new_arm1_pos = RIGHT_MOTOR1_MAX_DEG_VALUE;

    else if(m_arm1_pos < RIGHT_MOTOR1_MIN_DEG)

        new_arm1_pos = RIGHT_MOTOR1_MIN_DEG_VALUE;

    else {

        new_arm1_pos =
                RIGHT_MOTOR1_MIN_DEG_VALUE + (m_arm1_pos-RIGHT_MOTOR1_MIN_DEG) * (RIGHT_MOTOR1_MAX_DEG_VALUE - RIGHT_MOTOR1_MIN_DEG_VALUE);

    }

    // Remapping the Arm2 pos
    if(m_arm2_pos > RIGHT_MOTOR2_MAX_DEG)

        new_arm2_pos = RIGHT_MOTOR2_MAX_DEG_VALUE;

    else if(m_arm2_pos < RIGHT_MOTOR2_MIN_DEG)

        new_arm2_pos = RIGHT_MOTOR2_MIN_DEG_VALUE;

    else {

        new_arm2_pos =
                RIGHT_MOTOR2_MIN_DEG_VALUE + (m_arm2_pos-RIGHT_MOTOR2_MIN_DEG) * (RIGHT_MOTOR2_MAX_DEG_VALUE - RIGHT_MOTOR2_MIN_DEG_VALUE);

    }


    m_arm.m_l_motor1_degree = new_arm1_pos;
    m_arm.m_l_motor2_degree = new_arm2_pos;

    // Sync data from ROS to DiffModbus
    m_diff.m_left_wheel_torque = wheel_cmd[0];
    m_diff.m_right_wheel_torque = wheel_cmd[1];

    // Sync data to robot
    m_modbus.write();

}

void OcareRobot::publish_sensor_data() {
    // Create a Sensor data message
    std_msgs::UInt16MultiArray sensor_msg;
    for(int i=0; i<SENSOR_REG_COUNT; i++)
        sensor_msg.data.push_back(m_diff.m_read_sensor_datas[i]);

    // Sent Sensor data message
    m_track_line_pub.publish(sensor_msg);
//    ROS_INFO("%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
//            m_diff.m_read_sensor_datas[0],
//            m_diff.m_read_sensor_datas[1],
//            m_diff.m_read_sensor_datas[2],
//            m_diff.m_read_sensor_datas[3],
//            m_diff.m_read_sensor_datas[4],
//            m_diff.m_read_sensor_datas[5],
//            m_diff.m_read_sensor_datas[6],
//            m_diff.m_read_sensor_datas[7],
//            m_diff.m_read_sensor_datas[8],
//            m_diff.m_read_sensor_datas[9],
//            m_diff.m_read_sensor_datas[10],
//            m_diff.m_read_sensor_datas[11],
//            m_diff.m_read_sensor_datas[12]);
}


