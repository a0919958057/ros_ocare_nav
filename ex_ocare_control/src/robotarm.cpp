#include "robotarm.h"
#include <math.h>
RobotArm::RobotArm() :
    m_serial(), m_joint1_rad(0), m_joint2_rad(0), m_catch(false) {

    ROS_INFO("Setup Serial");
    m_serial.init("/dev/arduino/", 9600, ArduSerial::RAW);
    m_serial.set_Parity(8,1,'N');

}
RobotArm::~RobotArm() {}

void RobotArm::get_data(DataType _type, void *_buffer) {

    switch(_type) {
    case DataType::ARM_MODE:
        send_cmd(CmdAction::GET_ARM_MODE);
        break;
    case DataType::MOTOR_DEGREE:
        send_cmd(CmdAction::GET_MOTOR_DEGREE);
        break;
    }

}

void RobotArm::send_cmd(
        CmdAction _action, double _cmd_1, double _cmd_2, bool _catch) {

    char aBuffer[8] = { 0 };

    switch(_action) {
    case CmdAction::ENABLE_ARM_FREE: {

        aBuffer[0] = COMMAND_WRITE;
        aBuffer[1] = SET_ENABLE_ARM_FREE;

        // Note the aDeg is mapping 0~1023 to 0~360 degree
        int aDeg_1 = (_cmd_1 * 1024/(2*M_PI));
        // Set Motor 1 cmd High Byte
        aBuffer[2] = aDeg_1 >> 8;
        // Set Motor 1 cmd Low Byte
        aBuffer[3] = aDeg_1 & 0xff;

        // Note the aDeg is mapping 0~1023 to 0~360 degree
        int aDeg_2 = (_cmd_2 * 1024/(2*M_PI));
        // Set Motor 1 cmd High Byte
        aBuffer[4] = aDeg_2 >> 8;
        // Set Motor 1 cmd Low Byte
        aBuffer[5] = aDeg_2 & 0xff;

        // Set catch
        aBuffer[6] = _catch ? 0x00 : 0x01;

        break;
    }
    case CmdAction::ENABLE_ARM_PUSH:
        aBuffer[0] = COMMAND_WRITE;
        aBuffer[1] = SET_ENABLE_ARM_PUSH;
        break;
    case CmdAction::ENABLE_RESET:
        aBuffer[0] = COMMAND_WRITE;
        aBuffer[1] = SET_ENABLE_RESET;
        break;
    case CmdAction::ENABLE_SLIDER:
        aBuffer[0] = COMMAND_WRITE;
        aBuffer[1] = SET_ENABLE_SLIDER;
        break;
    case CmdAction::GET_ARM_MODE:
        aBuffer[0] = COMMAND_READ;
        aBuffer[1] = GET_ARM_MODE;
        break;
    case CmdAction::GET_MOTOR_DEGREE:
        aBuffer[0] = COMMAND_READ;
        aBuffer[1] = GET_MOTOR_DEGREE;
        break;
    }

    aBuffer[7] = RobotArm::get_sum(aBuffer);

    // give data until receive feed back
    while(!pendingWrite(aBuffer));

}



char RobotArm::get_sum(char* _input_packet)
{
    byte sum(0);

    for (int i = 0; i < 7; i++)
        sum += _input_packet[i];
    return ~(char)sum;
}

bool RobotArm::pendingWrite(char* _write_packet) {
    char buffer[200] = { 0 };
    bool status_send(false);


    // Ensure the count of byte that serial received more then 8
    int rec_sum(0);
    int rec_loop_count(0);
    int left_buffer_size(200);
    while(rec_sum>8 & left_buffer_size > 0) {
        // If there is no data then need to send data to arduino again
        if(rec_loop_count++ > 20) return false;

        int rec_size = m_serial.readBytes(buffer, left_buffer_size);
        rec_sum += rec_size;
        left_buffer_size -= rec_size;
    }

    for(int i=0; i<rec_sum-8; i++) {
        if(buffer[i] == COMMAND_RETURN)
            switch(buffer[i+1]) {
            case SET_ENABLE_SLIDER:
            case SET_ENABLE_ARM_FREE:
            case SET_ENABLE_RESET: {

                for(int j=0; j<5; i++) {
                    // assume is pass
                    status_send = true;
                    if(buffer[i+2+j] != 0x00) {
                        // assume is break
                        status_send = false;
                        break;
                    }
                }
                break;
            }

            case SET_ENABLE_ARM_PUSH: {
                if(buffer[i+7] == get_sum(&buffer[i])) {
                    return true;
                }
            }

            case GET_ARM_MODE: {
                if(buffer[i+7] == get_sum(&buffer[i])) {
                    // TODO: define ARM MODE and set and add member variable

                    return true;
                }
            }

            case GET_MOTOR_DEGREE: {
                if(buffer[i+7] == get_sum(&buffer[i])) {
                    int temp_degree(0);
                    temp_degree += int(buffer[i+2]) << 8;
                    temp_degree += int(buffer[i+3]) ;
                    m_joint1_rad = (temp_degree/1024.0) *2* M_PI;
                    temp_degree = 0;
                    temp_degree += int(buffer[i+4]) << 8;
                    temp_degree += int(buffer[i+5]) ;
                    m_joint2_rad = (temp_degree/1024.0) *2* M_PI;
                    m_catch = (buffer[i+6] == 0x00);
                    return true;
                }
            }


        }
        if(status_send == true) {
            if(char(buffer[i+7]) != get_sum(&buffer[i])) {
                status_send = false;
            } else {
                return true;
            }
        }

    }
    return false;
}
