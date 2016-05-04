#ifndef ARDUSERIAL_H
#define ARDUSERIAL_H

// Unix standard define Header
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// Posix terminal protocal define Header
#include <termios.h>

// Standard IO and Library define Header
#include <stdio.h>
#include <stdlib.h>

#include <memory.h>



class ArduSerial
{
public:
    ArduSerial();
    ~ArduSerial();

    enum class SerialEvent {
        CREATE_SUCCESSFUL,
        CREATE_FAIL,
        CONNECTED,
        DISCONNECTE,
        GET_ATTR_FAIL,
        GET_ATTR_SUCCESSFUL,
        SET_ATTR_SUCCESSFUL,
        SET_ATTR_FAIL,
        DATA_SIZE_ATTR_ERROR,
        PARITY_ATTR_ERROR,
        STOPBIT_ATTR_ERROR,
        INIT_SUCCESSFUL,
    };

    enum InputMode {
        RAW,
        CANON
    };

    SerialEvent init(char* _device, unsigned int _baud, InputMode _mode);

    SerialEvent set_Parity(int _databits, int _stopbits, int _parity);
    SerialEvent set_Speed(int _speed);

    int readBytes(char* _buffer, size_t _size);
    int writeBytes(char* _buffer, size_t _size);

private:
    SerialEvent update_Attr();
    SerialEvent set_Attr();

    SerialEvent m_state;

    int m_fd;
    struct termios m_oldtio;
    struct termios m_newtio;

public:

    static const int SPEED_ARR[];
    static const  int NAME_ARR[];
};

#endif // ARDUSERIAL_H
