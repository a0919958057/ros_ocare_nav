#ifndef ARMMODBUS_H
#define ARMMODBUS_H

#include <iostream>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <modbus/modbus-rtu.h>

#include "hwmodule.h"

#define MOTOR1_CMD_INIT_VALUE       (844)
#define MOTOR2_CMD_INIT_VALUE       (520)
#define CATCH_LEVEL_CMD_INIT_VALUE  (0)

#define REG_CMD_START   EFFORT_CATCH_LEVEL
#define REG_CMD_END     CMD_ARM_MODE
#define REG_READ_START  LEFT_MOTOR1_DEGREE
#define REG_READ_END    EFFORT_CATCH_LEVEL

class ArmModbus : public HWModule
{
public:
    ArmModbus(char* _name, size_t _length);
    ~ArmModbus();

    // Define the Modbus HOLD_REGISTER Mapping
    enum StateHoldRegister {
      LEFT_MOTOR1_DEGREE,
      LEFT_MOTOR2_DEGREE,
      SLIDER_STATE,
      ARM_MODE,
      EFFORT_CATCH_LEVEL,
      CMD_LEFT_MOTOR1_DEGREE,
      CMD_LEFT_MOTOR2_DEGREE,
      CMD_SLIDER_STATE,
      CMD_ARM_MODE
    };

    /******Defination of the Slider state
     * SLIDER_HOME :       Slider at Home position
     * SLIDER_OPENING:     Slider Opening now
     * SLIDER_OPENED:      Slider is Opened, the arm can work around now
     * SLIDER_RETURNING:   Slider is Returning to HOME now
     * **************************/
    enum SliderState  {
        SLIDER_HOME,
        SLIDER_OPENING,
        SLIDER_OPENED,
        SLIDER_RETURNING
    };

    /******Defination of the Slider Command
     * SLIDER_OPEN_CMD :           Open the slider to the right position
     * SLIDER_CLOSE_CMD:           Close the slider to the home position
     *
     * **************************/
    enum SliderStateCMD  {
        SLIDER_OPEN_CMD,
        SLIDER_CLOSE_CMD
    };

    /******Defination of the Arm Mode
     * ARM_HOME :           Arm at Home position
     * ARM_BUTTON_POSE:     Arm is at push button pose
     * ARM_FREE_CONTROLL:   Arm is at motor controllable state,
     *                  the arm can be controll by ROS now
     * **************************/
    enum ArmMode  {
        ARM_HOME,
        ARM_BUTTON_POSE,
        ARM_FREE_CONTROLL
    };

    /******Defination of the Arm Mode Command
     * ARM_HOME_CMD :           Arm go to Home position
     * ARM_BUTTON_POSE_CMD:     Arm go to push button pose
     * ARM_FREE_CONTROLL_CMD:   Arm go to motor controllable state,
     *                          let arm can be controll by ROS.
     * **************************/
    enum ArmModeCMD  {
        ARM_HOME_CMD,
        ARM_BUTTON_POSE_CMD,
        ARM_FREE_CONTROLL_CMD
    };

    // Write DATA to Arduino via modbus
    bool write();

    // Read DATA from Arduino via modbus
    bool read();

public:

    // The robot state cmd
    uint16_t m_l_motor1_degree;
    uint16_t m_l_motor2_degree;
    uint16_t m_l_catch_level;
    ArmModeCMD m_l_mode;
    SliderStateCMD m_l_slider_mode;

    // The robot state current
    // Note :read only
    uint16_t m_read_l_motor1_degree;
    uint16_t m_read_l_motor2_degree;
    uint16_t m_read_l_catch_level;
    ArmMode m_read_l_mode;
    SliderState m_read_l_slider_mode;


};


#endif // ARMMODBUS_H
