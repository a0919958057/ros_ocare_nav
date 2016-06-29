#include "ardumodbus.h"

ArduModbus::ArduModbus() :
    m_ctx(nullptr),
    is_serial_state_ready(false),
    is_modbus_state_ready(false)
    {

}

ArduModbus::~ArduModbus() {
    if(is_serial_state_ready == true) {
        modbus_free(m_ctx);
    }
}

bool ArduModbus::init(char *_device, int _baud, int _stopbit, int _parity) {

    if(_device != nullptr)
        m_ctx = modbus_new_rtu(_device, _baud, _parity, 8, _stopbit);
    else {

#ifdef _ROS
        ROS_ERROR("Modbus Serial device init Parameter error! \n");
#else
        fprintf(stderr, "Modbus Serial device init Parameter error! \n");
#endif
        return false;
    }

    if (m_ctx == NULL) {
#ifdef _ROS
        ROS_ERROR("Unable to create the libmodbus context\n");
#else
        fprintf(stderr, "Unable to create the libmodbus context\n");
#endif
        return false;
    }

    // Setup the modbus serial mode to RS232 <USE FDTI232>
    modbus_rtu_set_serial_mode(m_ctx, MODBUS_RTU_RS232);

#ifdef _DEBUG
    modbus_set_debug(m_ctx, ON);
#else
    modbus_set_debug(m_ctx, ON);
#endif

    is_serial_state_ready = true;
    return true;
}

bool ArduModbus::connect_slave() {

    // Makesure there is serial connected
    if( is_serial_state_ready == false) return false;

    // Assume all slave will connect successful, find the connect error and set false
    bool result(true);

    for(HWModule* _hw : m_hw_modules) {
        if(_hw->getSlaveID() > 0 && _hw->getSlaveID() < 254) {

            // Setting Arduino slaver ID
            modbus_set_slave(m_ctx, _hw->getSlaveID());

            // Delay 5 ms for waiting switch slaver
            usleep(0.005 * 1000000);
            if (modbus_connect(m_ctx) == -1) {
#ifdef _ROS
                ROS_ERROR("Modbus Connection failed: %s\n", modbus_strerror(errno));
#else
                fprintf(stderr, "Modbus Connection failed: %s\n", modbus_strerror(errno));
#endif
                modbus_free(m_ctx);
                result = false;
            } else {
                modbus_flush(m_ctx);
            }
        }
        else
        {
#ifdef _ROS
            ROS_ERROR("Modbus : Slave id must 1~253 \n");
#else
            fprintf(stderr, "Modbus : Slave id must 1~253 \n");
#endif
        }
    }

    if(result == true) {
       is_modbus_state_ready = true;
       return true;
    }

    return false;
}

bool ArduModbus::registerHWModule(HWModule* _p_module) {

    if(_p_module == nullptr) return false;

    // Let HWModule know the ArduModbus, so that HWModule can use ArduModbus's modbus interface
    _p_module->registerModbusInterface(m_ctx);

    m_hw_modules.push_back(_p_module);
#ifdef _ROS
        ROS_INFO("ArduModbus : HWModule register successful! \n");
#else
        fprintf(stdout, "ArduModbus : HWModule register successful! \n");
#endif
    return true;
}

bool ArduModbus::write() {

    // Makesure that modbus is connected
    if(is_modbus_state_ready == false) {
#ifdef _ROS
        ROS_ERROR("Modbus Write: Modbus not connect yet! \n");
#else
        fprintf(stderr, "Modbus : Modbus not connect yet! \n");
#endif
        return false;
    }

    for(HWModule* _hw : m_hw_modules) {
        // Setting Arduino slaver ID
        modbus_set_slave(m_ctx, _hw->getSlaveID());

        // Delay 5 ms for waiting switch slaver
        usleep(0.005 * 1000000);

        if(!_hw->write()) {
#ifdef _ROS
        ROS_ERROR("HWModule Write: HWModule %s can not write! \n", _hw->m_name);
#else
        fprintf(stderr, "HWModule Write: HWModule can not write! \n", _hw->m_name);
#endif
        return false;
        }
    }

    return true;
}

bool ArduModbus::read() {

    // Makesure that modbus is connected
    if(is_modbus_state_ready == false) {
#ifdef _ROS
        ROS_ERROR("Modbus Read: Modbus not connect yet! \n");
#else
        fprintf(stderr, "Modbus Read: Modbus not connect yet! \n");
#endif
        return false;
    }

    for(HWModule* _hw : m_hw_modules) {
        // Setting Arduino slaver ID
        modbus_set_slave(m_ctx, _hw->getSlaveID());

        // Delay 5 ms for waiting switch slaver
        usleep(0.005 * 1000000);

        if(!_hw->read()) {
#ifdef _ROS
        ROS_ERROR("HWModule Read: HWModule %s can not read! \n", _hw->m_name);
#else
        fprintf(stderr, "HWModule Read: HWModule %s can not read! \n", _hw->m_name);
#endif
        return false;
        }
    }

    return true;
}
