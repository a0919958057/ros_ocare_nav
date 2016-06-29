#include "hwmodule.h"


HWModule::HWModule(char* _name, size_t _length) :
    m_ctx(nullptr), m_slave_id(0) {

    if(sizeof(m_name) < _length) {
        fprintf(stderr,  "Failed to set HWModule name. ");
    } else {

        // If the length then copy the parameter name to member name variable
        for(int i=0 ;i<_length;i++) {
            m_name[i] = _name[i];
        }
    }

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
