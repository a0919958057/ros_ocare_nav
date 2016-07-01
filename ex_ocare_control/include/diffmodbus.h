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

#define SENSOR_REG_START 100
#define SENSOR_REG_COUNT 13

#define REG_CMD_START   CMD_CHASSIS_MODE
#define REG_CMD_END     CMD_SENSOR_BW_MODE
#define REG_READ_START  CHASSIS_MODE
#define REG_READ_END    SENSOR_BW_MODE

class DiffModbus : public HWModule
{
public:
    DiffModbus(char* _name, size_t _length);
    ~DiffModbus();

    /********************** Modbus Information ************************/

    // Define the Modbus HOLD_REGISTER Mapping
    enum StateHoldRegister {
        CHASSIS_MODE,
        LEFT_WHEEL_TORQUE,
        RIGHT_WHEEL_TORQUE,
        TRACKLINE_TORQUE_MODE,
        SENSOR_BW_MODE,
        CMD_CHASSIS_MODE,
        CMD_LEFT_WHEEL_TORQUE,
        CMD_RIGHT_WHEEL_TORQUE,
        CMD_TRACKLINE_TORQUE_MODE,
        CMD_SENSOR_BW_MODE
    };

    /******Defination of the Chassis Mode
    * TRACK_LINE :     Tracking the path and follow the path
    * CONTROLLABLE:    The Arduino would use LEFT_WHEEL_TORQUE and RIGHT_WHEEL_TORQUE to effort the wheel
    * STOP:            All of the driver would stop until Mode become others.
    * **************************/
    enum ChassisMode {
        MODE_TRACK_LINE,
        MODE_CONTROLLABLE,
        MODE_STOP
    };

    /******Defination of the Chassis Mode command
    * TRACK_LINE :     Tracking the path and follow the path
    * CONTROLLABLE:    The Arduino would use LEFT_WHEEL_TORQUE and RIGHT_WHEEL_TORQUE to effort the wheel
    * STOP:            All of the driver would stop until Mode become others.
    * **************************/
    enum ChassisModeCMD {
        MODE_TRACK_LINE_CMD,
        MODE_CONTROLLABLE_CMD,
        MODE_STOP_CMD
    };

    /******Defination of the Chassis Mode
    * HIGH :           Useing the higher torque Fuzzy rule
    * MED:             Useing the normal torque Fuzzy rule
    * LOW:             Useing the low torque Fuzzy rule
    * **************************/
    enum TrackingTorqueMode {
        TORQUE_HIGH,
        TORQUE_MED,
        TORQUE_LOW
    };

    /******Defination of the Chassis Mode command
    * HIGH :           command arduino use the higher torque Fuzzy rule
    * MED:             command arduino use the normal torque Fuzzy rule
    * LOW:             command arduino use the low torque Fuzzy rule
    * **************************/
    enum TrackingTorqueModeCMD {
        TORQUE_HIGH_CMD,
        TORQUE_MED_CMD,
        TORQUE_LOW_CMD
    };

    /******Defination of the Sensor BW Mode
    * BLACK :           Tracking the black line
    * WRITE:            Tracking the white line
    * **************************/
    enum SensorBWMode {
        BLACK,
        WHITE
    };

    /******Defination of the Sensor BW Mode command
    * BLACK_CMD :           Command arduino track the black line
    * WRITE_CMD:            Command arduino track white line
    * **************************/
    enum SensorBWModeCMD {
        BLACK_CMD,
        WHITE_CMD,
    };

    /******************************************************************/


    // Write DATA to Arduino via modbus
    bool write();

    // Read DATA from Arduino via modbus
    bool read();

public:

    // The robot state cmd
    ChassisModeCMD m_chassis_mode;
    int16_t m_left_wheel_torque;
    int16_t m_right_wheel_torque;
    TrackingTorqueModeCMD m_track_torque_mode;
    SensorBWModeCMD m_sensor_bw_mode;

    // The robot state cueent
    // Note :read only
    ChassisMode m_read_chassis_mode;
    int16_t m_read_left_wheel_torque;
    int16_t m_read_right_wheel_torque;
    TrackingTorqueMode m_read_track_torque_mode;
    SensorBWMode m_read_sensor_bw_mode;
    uint16_t m_read_sensor_datas[SENSOR_REG_COUNT];


};

#endif // DIFFMODBUS_H
