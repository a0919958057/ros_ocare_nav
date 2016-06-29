#ifndef ARDUMODBUS_H
#define ARDUMODBUS_H

#include <iostream>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <vector>

#include <modbus/modbus-rtu.h>

#include "hwmodule.h"


#define HOLD_REG_SIZE 9
#define HOLD_REG_READ_ONLY_SIZE 4

// If using ROS system then Define ROS
#define _ROS

#ifdef _ROS
#include "ros/ros.h"
#endif

class ArduModbus
{
public:
    ArduModbus();
    ~ArduModbus();

    // Initial the Modbus
    bool init(char* _device, int _baud, int _stopbit, int _parity);

    // Connect the slave by Modbus protocal
    bool connect_slave();

    // Register the HWModule to the class
    bool registerHWModule(HWModule* _p_module);

    // Write DATA to Arduino via modbus
    bool write();

    // Read DATA from Arduino via modbus
    bool read();



public:

    // The main modbus object
    modbus_t *m_ctx;
    bool is_serial_state_ready;
    bool is_modbus_state_ready;

    // The HWModues arroy
    std::vector<HWModule*> m_hw_modules;

};

#endif // ARDUMODBUS_H
