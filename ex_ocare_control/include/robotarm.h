#ifndef ROBOTARM_H
#define ROBOTARM_H


#include "arduserial.h"
#include "arm_packet_define.h"
#include <ros/ros.h>
#include <math.h>

class RobotArm
{
public:
    RobotArm();
    ~RobotArm();

    enum class CmdAction {
        ENABLE_SLIDER,
        ENABLE_RESET,
        ENABLE_ARM_PUSH,
        ENABLE_ARM_FREE,
        GET_MOTOR_DEGREE,
        GET_ARM_MODE,
    };

    enum class DataType {
        MOTOR_DEGREE,
        ARM_MODE
    };


    void send_cmd(CmdAction _action, double _cmd_1=0, double cmd_2=0, bool _catch = false);
    void get_data(DataType _type, void* _buffer);

public:
    double m_joint1_rad;
    double m_joint2_rad;
    bool m_catch;


private:
    static char get_sum(char* _input_packet);
    bool pendingWrite(char* _write_packet);

private:
    ArduSerial m_serial;
};

#endif // ROBOTARM_H
