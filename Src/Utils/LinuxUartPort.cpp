#include "LinuxUartPort.hpp"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <cstring>

using namespace uav::hw;

LinuxUartPort::LinuxUartPort(const std::string& device) : _device(device) {}

LinuxUartPort::~LinuxUartPort() { close(); }

bool LinuxUartPort::open(const char* device, uint32_t baud) {
    if (_fd >= 0) return true;
    _fd = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd < 0) {
        std::cerr << "[LinuxUartPort] open failed: " << strerror(errno) << "\n";
        return false;
    }
    termios tty{};
    if (tcgetattr(_fd, &_orig) != 0) {
        std::cerr << "[LinuxUartPort] tcgetattr failed\n";
    }
    cfmakeraw(&tty);
    speed_t speed;
    switch (baud) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default: speed = B115200; break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(_fd, TCSANOW, &tty) != 0) {
        std::cerr << "[LinuxUartPort] tcsetattr failed\n";
        ::close(_fd);
        _fd = -1;
        return false;
    }
    return true;
}

void LinuxUartPort::close() {
    if (_fd >= 0) {
        tcsetattr(_fd, TCSANOW, &_orig);
        ::close(_fd);
        _fd = -1;
    }
}

ssize_t LinuxUartPort::read(uint8_t* buf, size_t len, uint32_t timeoutMs) {
    if (_fd < 0) return -1;
    fd_set set;
    FD_ZERO(&set);
    FD_SET(_fd, &set);
    timeval tv{};
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;
    int rv = select(_fd + 1, &set, nullptr, nullptr, &tv);
    if (rv > 0 && FD_ISSET(_fd, &set)) {
        ssize_t r = ::read(_fd, buf, len);
        return r;
    }
    return 0;
}

ssize_t LinuxUartPort::write(const uint8_t* buf, size_t len) {
    if (_fd < 0) return -1;
    ssize_t w = ::write(_fd, buf, len);
    return w;
}
