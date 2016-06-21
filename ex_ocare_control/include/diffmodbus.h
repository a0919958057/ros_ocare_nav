#ifndef DIFFMODBUS_H
#define DIFFMODBUS_H

#include <iostream>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <modbus/modbus-rtu.h>

#include "hwmodule.h"

#define COUNT_SENSORS 2

#define SENSOR_REG_START 100
#define SENSOR_REG_COUNT 13

class DiffModbus : public HWModule
{
public:
    DiffModbus();
    ~DiffModbus();

    // Define the Modbus HOLD_REGISTER Mapping
    enum StateHoldRegister {
      CHASSIS_MODE,
      LEFT_WHEEL_TORQUE,
      RIGHT_WHEEL_TORQUE,
      TRACKLINE_TORQUE_MODE,
      SENSOR_DATA1,
      SENSOR_DATA2,
      CMD_CHASSIS_MODE,
      CMD_LEFT_WHEEL_TORQUE,
      CMD_RIGHT_WHEEL_TORQUE,
      CMD_TRACKLINE_TORQUE_MODE
    };

    /******Defination of the Chassis Mode
     * MODE_TRACK_LINE :     Tracking the path and follow the path
     * MODE_CONTROLLABLE:    The Arduino would use LEFT_WHEEL_TORQUE and RIGHT_WHEEL_TORQUE to effort the wheel
     * MODE_STOP:            All of the driver would stop until Mode become others.
     * **************************/
    enum ChassisMode {
        MODE_TRACK_LINE,
        MODE_CONTROLLABLE,
        MODE_STOP
    };

    /******Defination of the Chassis Mode command
     * MODE_TRACK_LINE_CMD :     Tracking the path and follow the path
     * MODE_CONTROLLABLE_CMD:    The Arduino would use LEFT_WHEEL_TORQUE and RIGHT_WHEEL_TORQUE to effort the wheel
     * MODE_STOP_CMD:            All of the driver would stop until Mode become others.
     * **************************/
    enum ChassisModeCMD {
        MODE_TRACK_LINE_CMD,
        MODE_CONTROLLABLE_CMD,
        MODE_STOP_CMD
    };

    /******Defination of the Chassis Mode
     * TORQUE_HIGH :           Useing the higher torque Fuzzy rule
     * TORQUE_MED:             Useing the normal torque Fuzzy rule
     * TORQUE_LOW:             Useing the low torque Fuzzy rule
     * **************************/
    enum TrackingTorqueMode {
        TORQUE_HIGH,
        TORQUE_MED,
        TORQUE_LOW
    };

    /******Defination of the Chassis Mode command
     * TORQUE_HIGH_CMD :           command arduino use the higher torque Fuzzy rule
     * TORQUE_MED_CMD:             command arduino use the normal torque Fuzzy rule
     * TORQUE_LOW_CMD:             command arduino use the low torque Fuzzy rule
     * **************************/
    enum TrackingTorqueModeCMD {
        TORQUE_HIGH_CMD,
        TORQUE_MED_CMD,
        TORQUE_LOW_CMD
    };


    // Write DATA to Arduino via modbus
    bool write();

    // Read DATA from Arduino via modbus
    bool read();

public:

    // The robot state cmd
    ChassisModeCMD m_chassis_mode;
    uint16_t m_left_wheel_torque;
    uint16_t m_right_wheel_torque;
    TrackingTorqueModeCMD m_track_torque_mode;

    // The robot state cueent
    // Note :read only
    ChassisMode m_read_chassis_mode;
    uint16_t m_read_left_wheel_torque;
    uint16_t m_read_right_wheel_torque;
    TrackingTorqueMode m_read_track_torque_mode;
    uint16_t m_read_sensor_datas[COUNT_SENSORS];


};

#endif // DIFFMODBUS_H
