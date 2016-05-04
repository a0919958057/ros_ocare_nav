#ifndef ARM_PACKET_DEFINE_H
#define ARM_PACKET_DEFINE_H

#endif // ARM_PACKET_DEFINE_H

typedef unsigned char byte;


const char COMMAND_WRITE= 0xC0;
const char COMMAND_READ= 0xC1;
const char COMMAND_CHECK_ERROR= 0xC2;
const char COMMAND_RETURN= 0xC3;

const char SET_ENABLE_SLIDER= 0x01;
const char SET_ENABLE_ARM_PUSH= 0x02;
const char SET_ENABLE_ARM_FREE= 0x03;
const char SET_ENABLE_RESET= 0x04;

const char GET_MOTOR_DEGREE= 0x05;
const char GET_ARM_MODE= 0x06;

const char RETURN_MOTOR_DEGREE= 0x01;
const char RETURN_ARM_MODE= 0x02;
