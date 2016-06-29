#include "armmodbus.h"


ArmModbus::ArmModbus(char* _name, size_t _length) :
    HWModule(_name, _length),
    m_l_motor1_degree(0),
    m_l_motor2_degree(0),
    m_l_catch_level(0),
    m_l_mode(ArmModeCMD::ARM_HOME_CMD),
    m_l_slider_mode(SliderStateCMD::SLIDER_CLOSE_CMD),
    m_read_l_motor1_degree(0),
    m_read_l_motor2_degree(0),
    m_read_l_catch_level(0),
    m_read_l_mode(ArmMode::ARM_HOME),
    m_read_l_slider_mode(SliderState::SLIDER_HOME)
    {

    // Set the slave ID
    m_slave_id = 5;
}

ArmModbus::~ArmModbus() {}

bool ArmModbus::write() {

    uint16_t reg[CMD_ARM_MODE-EFFORT_CATCH_LEVEL+1];

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
    // Note there is a continue register
    reg[EFFORT_CATCH_LEVEL - EFFORT_CATCH_LEVEL]     = m_l_catch_level;
    reg[CMD_LEFT_MOTOR1_DEGREE - EFFORT_CATCH_LEVEL] = m_l_motor1_degree;
    reg[CMD_LEFT_MOTOR2_DEGREE - EFFORT_CATCH_LEVEL] = m_l_motor2_degree;
    reg[CMD_SLIDER_STATE - EFFORT_CATCH_LEVEL]       = static_cast<uint16_t>(m_l_slider_mode);
    reg[CMD_ARM_MODE - EFFORT_CATCH_LEVEL]           = static_cast<uint16_t>(m_l_mode);

    int num = modbus_write_registers(
                m_ctx, EFFORT_CATCH_LEVEL, CMD_ARM_MODE - EFFORT_CATCH_LEVEL +1, reg);

    if (num != CMD_ARM_MODE - EFFORT_CATCH_LEVEL +1) {// number of writed registers is not the one expected

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

    uint16_t reg[EFFORT_CATCH_LEVEL-LEFT_MOTOR1_DEGREE+1];

    int num = modbus_read_registers(
                m_ctx, LEFT_MOTOR1_DEGREE, EFFORT_CATCH_LEVEL-LEFT_MOTOR1_DEGREE+1, reg);

    if (num != EFFORT_CATCH_LEVEL-LEFT_MOTOR1_DEGREE+1) {// number of writed registers is not the one expected
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
