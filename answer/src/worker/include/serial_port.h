#ifndef _SERIAL_PORT_H_
#define _SERIAL_PORT_H_

#include "tools.h"
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <stdexcept>

using namespace std;
using namespace cv;

class Serialport{
private:
    int fd_;
    inline void openPort(const string& device,int baud_rate){
        fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open serial device: " + device);
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(fd_, &tty) != 0) {
            close(fd_);
            throw std::runtime_error("tcgetattr failed");
        }

        cfsetospeed(&tty, baud_rate);
        cfsetispeed(&tty, baud_rate);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        tty.c_iflag &= ~IGNBRK;                         // disable break processing
        tty.c_lflag = 0;                                 // no signaling, echo, etc.
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 0;                             // read doesn't block
        tty.c_cc[VTIME] = 5;                             // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);          // shut off flow control
        tty.c_cflag |= (CLOCAL | CREAD);                  // ignore modem controls
        tty.c_cflag &= ~(PARENB | PARODD);                // no parity
        tty.c_cflag &= ~CSTOPB;                            // 1 stop bit
        tty.c_cflag &= ~CRTSCTS;                           // no hardware flow control

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            close(fd_);
            throw std::runtime_error("tcsetattr failed");
        }
    }
    inline void closePort(){
        std::cerr << "closePort called, fd = " << fd_ << std::endl;
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
public:
    Serialport(){
        fd_ = -1;
        closePort();
    }
    Serialport(const string& device,int baud_rate){
        fd_ = -1;
        openPort(device,baud_rate);
    }
    ~Serialport(){
        closePort();
    }
    void write(const uint8_t* data, size_t size) {
        if (fd_ < 0) return;
        ssize_t written = ::write(fd_, data, size);
        if (written != static_cast<ssize_t>(size)) {
            throw std::runtime_error("Serial write error");
        }
        usleep(size * 500);
    }
};

#endif