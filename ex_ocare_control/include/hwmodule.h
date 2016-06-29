#ifndef HWMODULE_H
#define HWMODULE_H



#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <modbus/modbus-rtu.h>

class HWModule
{
public:

    HWModule(char* _name, size_t _length);
    ~HWModule();

    // Read via Modbus
    virtual bool read() = 0;
    virtual bool write() = 0;

    // Return the current slave ID
    uint16_t getSlaveID();

    // regist the
    bool registerModbusInterface(modbus_t* _ctx);

public:

    // The main modbus object
    modbus_t *m_ctx;


    // The remote slave ID
    uint16_t m_slave_id;

    // The hardware name
    char m_name[20];

};

#endif // HWMODULE_H
