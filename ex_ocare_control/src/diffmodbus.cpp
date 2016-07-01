#include "diffmodbus.h"

DiffModbus::DiffModbus(char* _name, size_t _length) :
    HWModule(_name, _length),
    m_chassis_mode(ChassisModeCMD::MODE_STOP_CMD),
    m_left_wheel_torque(0),
    m_right_wheel_torque(0),
    m_track_torque_mode(TrackingTorqueModeCMD::TORQUE_MED_CMD),
    m_sensor_bw_mode(SensorBWModeCMD::WHITE_CMD),
    m_read_chassis_mode(ChassisMode::MODE_STOP),
    m_read_left_wheel_torque(0),
    m_read_right_wheel_torque(0),
    m_read_track_torque_mode(TrackingTorqueMode::TORQUE_MED),
    m_read_sensor_bw_mode(SensorBWMode::WHITE){

    // Initial the Sensor data to read write <at value = 610
    m_read_sensor_datas[0] = 610;
    m_read_sensor_datas[1] = 610;

    // Set the slave ID
    m_slave_id = 6;
}

DiffModbus::~DiffModbus() {}

bool DiffModbus::write() {

    uint16_t reg[REG_CMD_END-REG_CMD_START+1];

    /******************* The arduino mapping design
     *
     *   mb.addHreg(CHASSIS_MODE);              **Read Only
     *   mb.addHreg(LEFT_WHEEL_TORQUE);         **Read Only
     *   mb.addHreg(RIGHT_WHEEL_TORQUE);        **Read Only
     *   mb.addHreg(TRACKLINE_TORQUE_MODE);     **Read Only
     *   mb.addHreg(SENSOR_BW_MODE);            **Read Only
     *   mb.addHreg(CMD_CHASSIS_MODE);          **Write Only
     *   mb.addHreg(CMD_LEFT_WHEEL_TORQUE);     **Write Only
     *   mb.addHreg(CMD_RIGHT_WHEEL_TORQUE);    **Write Only
     *   mb.addHreg(CMD_TRACKLINE_TORQUE_MODE); **Write Only
     *   mb.addHreg(CMD_SENSOR_BW_MODE);        **Write Only
     *
     **********************************************/
    // Note there is a continue register

    reg[CMD_CHASSIS_MODE - REG_CMD_START]           = static_cast<uint16_t>(m_chassis_mode);
    reg[CMD_LEFT_WHEEL_TORQUE - REG_CMD_START]      = m_left_wheel_torque;
    reg[CMD_RIGHT_WHEEL_TORQUE - REG_CMD_START]     = m_right_wheel_torque;
    reg[CMD_TRACKLINE_TORQUE_MODE - REG_CMD_START]  = static_cast<uint16_t>(m_track_torque_mode);
    reg[CMD_SENSOR_BW_MODE - REG_CMD_START]         = static_cast<uint16_t>(m_sensor_bw_mode);

    int num = modbus_write_registers(
                m_ctx, REG_CMD_START, REG_CMD_END-REG_CMD_START+1, reg);

    if (num != CMD_SENSOR_BW_MODE-CMD_CHASSIS_MODE+1) {// number of writed registers is not the one expected
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

    uint16_t reg[REG_READ_END-REG_READ_START+1];
    uint16_t sensor_reg[SENSOR_REG_COUNT];

    int num = modbus_read_registers(
                m_ctx, REG_READ_START, REG_READ_END-REG_READ_START+1, reg);

    if (num != REG_READ_END-REG_READ_START+1) {// number of writed registers is not the one expected
#ifdef _ROS
        ROS_ERROR( "Failed to read: %s\n", modbus_strerror(errno))
#else
        fprintf(stderr,  "Failed to read: %s\n", modbus_strerror(errno));
#endif
        return false;
    }

    int sensor_read_num = modbus_read_registers(
                m_ctx, SENSOR_REG_START, SENSOR_REG_COUNT, sensor_reg);


    if (sensor_read_num != SENSOR_REG_COUNT) {// number of writed registers is not the one expected
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
     *   mb.addHreg(SENSOR_BW_MODE);            **Read Only
     *   mb.addHreg(CMD_CHASSIS_MODE);          **Write Only
     *   mb.addHreg(CMD_LEFT_WHEEL_TORQUE);     **Write Only
     *   mb.addHreg(CMD_RIGHT_WHEEL_TORQUE);    **Write Only
     *   mb.addHreg(CMD_TRACKLINE_TORQUE_MODE); **Write Only
     *   mb.addHreg(CMD_SENSOR_BW_MODE);        **Write Only
     *
     **********************************************/
    // Note there is a continue register

    m_read_chassis_mode         = ChassisMode(reg[CHASSIS_MODE - REG_READ_START]);
    m_read_left_wheel_torque    = reg[LEFT_WHEEL_TORQUE - REG_READ_START];
    m_read_right_wheel_torque   = reg[RIGHT_WHEEL_TORQUE - REG_READ_START];
    m_read_track_torque_mode    = TrackingTorqueMode(reg[TRACKLINE_TORQUE_MODE - REG_READ_START]);
    m_read_sensor_bw_mode       = SensorBWMode(reg[SENSOR_BW_MODE - REG_READ_START]);

    for(int i=0;i<SENSOR_REG_COUNT;i++) {
        m_read_sensor_datas[i] = sensor_reg[i];
    }

    return true;
}
