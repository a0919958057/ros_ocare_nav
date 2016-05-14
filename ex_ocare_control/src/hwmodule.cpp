#include "hwmodule.h"


HWModule::HWModule() :
    m_ctx(nullptr), m_slave_id(0) {

}

HWModule::~HWModule() {}


bool HWModule::registerModbusInterface(modbus_t* _ctx) {

    if(_ctx == nullptr) return false;

    // Register the modbus interface to this class
    m_ctx = _ctx;
    return true;
}

uint16_t HWModule::getSlaveID() {
    return m_slave_id;
}
