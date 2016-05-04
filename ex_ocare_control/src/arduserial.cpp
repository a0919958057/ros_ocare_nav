#include "arduserial.h"

const int ArduSerial::SPEED_ARR[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
                B38400, B19200, B9600, B4800, B2400, B1200, B300 };
 const  int ArduSerial::NAME_ARR[] = {38400,  19200,  9600,  4800,  2400,  1200,  300,
            38400,  19200,  9600, 4800, 2400, 1200,  300 };
ArduSerial::ArduSerial()
                        : m_fd(0){
    m_state = SerialEvent::DISCONNECTE;
}

ArduSerial::~ArduSerial() {

    tcflush(m_fd,TCIOFLUSH); /* Update the options and do it NOW */
    tcsetattr(m_fd,TCSANOW,&m_oldtio);

    close(m_fd);
}

ArduSerial::SerialEvent ArduSerial::init(char* _device, unsigned int _baud, InputMode _mode) {
    if(_device != nullptr) {
        m_fd = open(_device, O_RDWR | O_NOCTTY);
        if (m_fd < 0) {
            return SerialEvent::CREATE_FAIL;
        }

        // Save current Device setting
        tcgetattr(m_fd, &m_oldtio);
        memset(&m_newtio, 0, sizeof(m_newtio));

        /*
          BAUDRATE: 設定 bps 的速度. 你也可以用 cfsetispeed 及 cfsetospeed 來設定.
          CRTSCTS : 輸出資料的硬體流量控制 (只能在具完整線路的纜線下工作
                    參考 Serial-HOWTO 第七節)
          CS8     : 8n1 (8 位元, 不做同位元檢查,1 個終止位元)
          CLOCAL  : 本地連線, 不具數據機控制功能
          CREAD   : 致能接收字元
        */
         m_newtio.c_cflag = SPEED_ARR[2] | CRTSCTS | CS8 | CLOCAL | CREAD;

        /*
          IGNPAR  : 忽略經同位元檢查後, 錯誤的位元組
          ICRNL   : 比 CR 對應成 NL (否則當輸入訊號有 CR 時不會終止輸入)
                    在不然把裝置設定成 raw 模式(沒有其它的輸入處理)
        */
         if(_mode == InputMode::RAW)
            m_newtio.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
         else if(_mode == InputMode::CANON){
             m_newtio.c_iflag |= IGNPAR | ICRNL;
         }

        /*
         Raw 模式輸出.
        */
         m_newtio.c_oflag = 0;

        /*
          ICANON  : 致能標準輸入, 使所有回應機能停用, 並不送出信號以叫用程式
        */
         if(_mode == InputMode::RAW)
            m_newtio.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
         else if(_mode == InputMode::CANON){
            m_newtio.c_lflag |= ECHO | ICANON | IEXTEN | ISIG;
         }


        /*
          初始化所有的控制特性
          預設值可以在 /usr/include/termios.h 找到, 在註解中也有,
          但我們在這不需要看它們
        */
         m_newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
         m_newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
         m_newtio.c_cc[VERASE]   = 0;     /* del */
         m_newtio.c_cc[VKILL]    = 0;     /* @ */
         m_newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
         m_newtio.c_cc[VTIME]    = 0;     /* 不使用分割字元組的計時器 */
         m_newtio.c_cc[VMIN]     = 1;     /* 在讀取到 1 個字元前先停止 */
         m_newtio.c_cc[VSWTC]    = 0;     /* '\0' */
         m_newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
         m_newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
         m_newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
         m_newtio.c_cc[VEOL]     = 0;     /* '\0' */
         m_newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
         m_newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
         m_newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
         m_newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
         m_newtio.c_cc[VEOL2]    = 0;     /* '\0' */

        /*
          現在清除數據機線並啟動序列埠的設定
        */
        if( set_Attr() == SerialEvent::SET_ATTR_SUCCESSFUL)  {
            m_state = SerialEvent::CONNECTED;
            return SerialEvent::INIT_SUCCESSFUL;
        }
        else
            return SerialEvent::SET_ATTR_FAIL;


    }





}

ArduSerial::SerialEvent ArduSerial::set_Speed(int _speed) {

    if ( update_Attr() == SerialEvent::GET_ATTR_FAIL )
        return SerialEvent::GET_ATTR_FAIL;

    for(int i = 0; i < sizeof(SPEED_ARR); i++ ) {

        if( _speed == NAME_ARR[i] ) {
            cfsetispeed(&m_newtio, SPEED_ARR[i]);
            cfsetospeed(&m_newtio, SPEED_ARR[i]);

            return set_Attr();

        }

    }

}

ArduSerial::SerialEvent ArduSerial::update_Attr() {
    if  ( tcgetattr( m_fd,&m_newtio) !=  0 )
        return SerialEvent::GET_ATTR_FAIL;

    return SerialEvent::GET_ATTR_SUCCESSFUL;
}

ArduSerial::SerialEvent ArduSerial::set_Attr() {

    tcflush(m_fd,TCIOFLUSH); /* Update the options and do it NOW */

    if (tcsetattr(m_fd,TCSANOW,&m_newtio) != 0)
        return SerialEvent::SET_ATTR_FAIL;
    else
        return SerialEvent::SET_ATTR_SUCCESSFUL;
}

ArduSerial::SerialEvent ArduSerial::set_Parity(int _databits, int _stopbits, int _parity) {

    if( update_Attr() == SerialEvent::GET_ATTR_FAIL)
        return SerialEvent::GET_ATTR_FAIL;


    m_newtio.c_cflag &= ~CSIZE;

    // 設置數據位元數
    switch (_databits) {

        case 7:

            m_newtio.c_cflag |= CS7;

            break;

        case 8:

            m_newtio.c_cflag |= CS8;

            break;

        default:

            return SerialEvent::DATA_SIZE_ATTR_ERROR;

    }

    switch (_parity) {

        case 'n':

        case 'N':

            m_newtio.c_cflag &= ~PARENB;   /* Clear parity enable */

            m_newtio.c_iflag &= ~INPCK;     /* Enable parity checking */

            break;

        case 'o':

        case 'O':

            m_newtio.c_cflag |= (PARODD | PARENB);  /* 設置為奇效驗*/

            m_newtio.c_iflag |= INPCK;             /* Disnable parity checking */

            break;

        case 'e':

        case 'E':

            m_newtio.c_cflag |= PARENB;     /* Enable parity */

            m_newtio.c_cflag &= ~PARODD;   /* 轉換為偶效驗*/

            m_newtio.c_iflag |= INPCK;       /* Disnable parity checking */

            break;

        case 'S':

        case 's':  /*as no parity*/

            m_newtio.c_cflag &= ~PARENB;

            m_newtio.c_cflag &= ~CSTOPB;

            break;

        default:

            return SerialEvent::PARITY_ATTR_ERROR;

    }

    /* 設置停止位*/

    switch (_stopbits) {

        case 1:

            m_newtio.c_cflag &= ~CSTOPB;

            break;

        case 2:

            m_newtio.c_cflag |= CSTOPB;

            break;

        default:

            return SerialEvent::STOPBIT_ATTR_ERROR;

    }

    /* Set input parity option */

    if (_parity != 'n')

    m_newtio.c_iflag |= INPCK;

    m_newtio.c_cc[VTIME] = 150; // 15 seconds

    m_newtio.c_cc[VMIN] = 0;


    return set_Attr();
}

int ArduSerial::readBytes(char* _buffer, size_t _size) {
    if( m_state == SerialEvent::CONNECTED)
        return read(m_fd, _buffer, _size);
    else
        return -1;
}

int ArduSerial::writeBytes(char* _buffer, size_t _size) {
    if( m_state == SerialEvent::CONNECTED)
        return write(m_fd, _buffer, _size);
    else
        return -1;
}
