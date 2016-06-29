#include "diffmodbus.h"

DiffModbus::DiffModbus(char* _name, size_t _length) :
    HWModule(_name, _length),
    m_chassis_mode(ChassisModeCMD::MODE_STOP_CMD),
    m_left_wheel_torque(0),
    m_right_wheel_torque(0),
    m_track_torque_mode(TrackingTorqueModeCMD::TORQUE_MED_CMD),
    m_read_chassis_mode(ChassisMode::MODE_STOP),
    m_read_left_wheel_torque(0),
    m_read_right_wheel_torque(0),
    m_read_track_torque_mode(TrackingTorqueMode::TORQUE_MED) {

    // Initial the Sensor data to read write <at value = 610
    m_read_sensor_datas[0] = 610;
    m_read_sensor_datas[1] = 610;

    // Set the slave ID
    m_slave_id = 6;
}

DiffModbus::~DiffModbus() {}

bool DiffModbus::write() {

    uint16_t reg[CMD_TRACKLINE_TORQUE_MODE-CMD_CHASSIS_MODE+1];

    /******************* The arduino mapping design
     *
     *   mb.addHreg(CHASSIS_MODE);              **Read Only
     *   mb.addHreg(LEFT_WHEEL_TORQUE);         **Read Only
     *   mb.addHreg(RIGHT_WHEEL_TORQUE);        **Read Only
     *   mb.addHreg(TRACKLINE_TORQUE_MODE);     **Read Only
     *   mb.addHreg(SENSOR_DATA1);              **Read Only
     *   mb.addHreg(SENSOR_DATA2);              **Read Only
     *   mb.addHreg(CMD_CHASSIS_MODE);          **Write Only
     *   mb.addHreg(CMD_LEFT_WHEEL_TORQUE);     **Write Only
     *   mb.addHreg(CMD_RIGHT_WHEEL_TORQUE);    **Write Only
     *   mb.addHreg(CMD_TRACKLINE_TORQUE_MODE); **Write Only
     *
     **********************************************/
    // Note there is a continue register

    reg[CMD_CHASSIS_MODE - CMD_CHASSIS_MODE]           = static_cast<uint16_t>(m_chassis_mode);
    reg[CMD_LEFT_WHEEL_TORQUE - CMD_CHASSIS_MODE]      = m_left_wheel_torque;
    reg[CMD_RIGHT_WHEEL_TORQUE - CMD_CHASSIS_MODE]     = m_right_wheel_torque;
    reg[CMD_TRACKLINE_TORQUE_MODE - CMD_CHASSIS_MODE]  = static_cast<uint16_t>(m_track_torque_mode);

    int num = modbus_write_registers(
                m_ctx, CMD_CHASSIS_MODE, CMD_TRACKLINE_TORQUE_MODE-CMD_CHASSIS_MODE+1, reg);

    if (num != CMD_TRACKLINE_TORQUE_MODE-CMD_CHASSIS_MODE+1) {// number of writed registers is not the one expected
#ifdef _ROS
        ROS_ERROR( "Failed to write: %s\n", modbus_strerror(errno))
#else
        fprintf(stderr,  "Failed to write: %s\n", modbus_strerror(errno));
#endif
        return false;
    }

    return true;
}

bool DiffModbus::read() {

    uint16_t reg[SENSOR_DATA2-CHASSIS_MODE+1];

    int num = modbus_read_registers(
                m_ctx, CHASSIS_MODE, SENSOR_DATA2-CHASSIS_MODE+1, reg);

    if (num != SENSOR_DATA2-CHASSIS_MODE+1) {// number of writed registers is not the one expected
#ifdef _ROS
        ROS_ERROR( "Failed to read: %s\n", modbus_strerror(errno))
#else
        fprintf(stderr,  "Failed to read: %s\n", modbus_strerror(errno));
#endif
        return false;
    }


    /******************* The arduino mapping design
     *
     *   mb.addHreg(CHASSIS_MODE);              **Read Only
     *   mb.addHreg(LEFT_WHEEL_TORQUE);         **Read Only
     *   mb.addHreg(RIGHT_WHEEL_TORQUE);        **Read Only
     *   mb.addHreg(TRACKLINE_TORQUE_MODE);     **Read Only
     *   mb.addHreg(SENSOR_DATA1);              **Read Only
     *   mb.addHreg(SENSOR_DATA2);              **Read Only
     *   mb.addHreg(CMD_CHASSIS_MODE);          **Write Only
     *   mb.addHreg(CMD_LEFT_WHEEL_TORQUE);     **Write Only
     *   mb.addHreg(CMD_RIGHT_WHEEL_TORQUE);    **Write Only
     *   mb.addHreg(CMD_TRACKLINE_TORQUE_MODE); **Write Only
     *
     **********************************************/

    m_read_chassis_mode         = ChassisMode(reg[CHASSIS_MODE - CHASSIS_MODE]);
    m_read_left_wheel_torque    = reg[LEFT_WHEEL_TORQUE - CHASSIS_MODE];
    m_read_right_wheel_torque   = reg[RIGHT_WHEEL_TORQUE - CHASSIS_MODE];
    m_read_track_torque_mode    = TrackingTorqueMode(reg[TRACKLINE_TORQUE_MODE - CHASSIS_MODE]);
    m_read_sensor_datas[0]      = reg[SENSOR_DATA1 - CHASSIS_MODE];
    m_read_sensor_datas[1]      = reg[SENSOR_DATA2 - CHASSIS_MODE];


    return true;
}
