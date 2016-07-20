#include "armmodbus.h"


ArmModbus::ArmModbus(char* _name, size_t _length) :
    HWModule(_name, _length),
    m_l_motor1_degree(MOTOR1_CMD_INIT_VALUE),
    m_l_motor2_degree(MOTOR2_CMD_INIT_VALUE),
    m_l_catch_level(CATCH_LEVEL_CMD_INIT_VALUE),
    m_l_mode(ArmModeCMD::ARM_HOME_CMD),
    m_l_slider_mode(SliderStateCMD::SLIDER_CLOSE_CMD),
    m_read_l_motor1_degree(MOTOR1_CMD_INIT_VALUE),
    m_read_l_motor2_degree(MOTOR2_CMD_INIT_VALUE),
    m_read_l_catch_level(CATCH_LEVEL_CMD_INIT_VALUE),
    m_read_l_mode(ArmMode::ARM_HOME),
    m_read_l_slider_mode(SliderState::SLIDER_HOME)
    {

    // Set the slave ID
    m_slave_id = 4;
}

ArmModbus::~ArmModbus() {}

bool ArmModbus::write() {

    uint16_t reg[REG_CMD_END-REG_CMD_START+1];

    /******************* The arduino mapping design
     *
     *   mb.addHreg(LEFT_MOTOR1_DEGREE);        **Read Only
     *   mb.addHreg(LEFT_MOTOR2_DEGREE);        **Read Only
     *   mb.addHreg(SLIDER_STATE);              **Read Only
     *   mb.addHreg(ARM_MODE);                  **Read Only
     *   mb.addHreg(EFFORT_CATCH_LEVEL);        **RW
     *   mb.addHreg(CMD_LEFT_MOTOR1_DEGREE);    **Write Only
     *   mb.addHreg(CMD_LEFT_MOTOR2_DEGREE);    **Write Only
     *   mb.addHreg(CMD_SLIDER_STATE);          **Write Only
     *   mb.addHreg(CMD_ARM_MODE);              **Write Only
     *
     **********************************************/
    // Note there is a continue register
    reg[EFFORT_CATCH_LEVEL - REG_CMD_START]     = m_l_catch_level;
    reg[CMD_LEFT_MOTOR1_DEGREE - REG_CMD_START] = m_l_motor1_degree;
    reg[CMD_LEFT_MOTOR2_DEGREE - REG_CMD_START] = m_l_motor2_degree;
    reg[CMD_SLIDER_STATE - REG_CMD_START]       = static_cast<uint16_t>(m_l_slider_mode);
    reg[CMD_ARM_MODE - REG_CMD_START]           = static_cast<uint16_t>(m_l_mode);

    int num = modbus_write_registers(
                m_ctx, REG_CMD_START, REG_CMD_END-REG_CMD_START+1, reg);

    if (num !=  REG_CMD_END-REG_CMD_START+1) {// number of writed registers is not the one expected

#ifdef _ROS
        ROS_ERROR( "Failed to write: %s\n", modbus_strerror(errno))
#else
        fprintf(stderr,  "Failed to write: %s\n", modbus_strerror(errno));
#endif
        return false;
    }

    return true;
}

bool ArmModbus::read() {

    uint16_t reg[REG_READ_END-REG_READ_START+1];

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


    /******************* The arduino mapping design
     *
     *   mb.addHreg(LEFT_MOTOR1_DEGREE);
     *   mb.addHreg(LEFT_MOTOR2_DEGREE);
     *   mb.addHreg(SLIDER_STATE);
     *   mb.addHreg(ARM_MODE);
     *   mb.addHreg(EFFORT_CATCH_LEVEL);
     *   mb.addHreg(CMD_LEFT_MOTOR1_DEGREE);
     *   mb.addHreg(CMD_LEFT_MOTOR2_DEGREE);
     *   mb.addHreg(CMD_SLIDER_STATE);
     *   mb.addHreg(CMD_ARM_MODE);
     *
     **********************************************/

    m_read_l_motor1_degree  = reg[LEFT_MOTOR1_DEGREE];
    m_read_l_motor2_degree  = reg[LEFT_MOTOR2_DEGREE];
    m_read_l_catch_level    = reg[EFFORT_CATCH_LEVEL];
    m_read_l_slider_mode    = SliderState(reg[SLIDER_STATE]);
    m_read_l_mode           = ArmMode(reg[ARM_MODE]);

    return true;
}
