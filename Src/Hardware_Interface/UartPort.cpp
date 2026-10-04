#include "UartPort.hpp"

#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

using namespace HW::IF;

UartPort::UartPort(std::string device) : m_device(std::move(device)) {}

UartPort::~UartPort() {
    ::close(m_fd);
}

bool UartPort::Open(const char* device, uint32_t baud) {
    if (m_fd >= 0) {
        return true;
    }
    m_fd = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_fd < 0) {
        std::cerr << "[UartPort] open failed: " << ::strerror(errno) << "\n";
        return false;
    }
    termios tty{};
    if (tcgetattr(m_fd, &m_orig) != 0) {
        std::cerr << "[UartPort] tcgetattr failed\n";
    }
    cfmakeraw(&tty);
    speed_t speed = 0;
    switch (baud) {
    case 9600:
        speed = B9600;
        break;
    case 19200:
        speed = B19200;
        break;
    case 38400:
        speed = B38400;
        break;
    case 57600:
        speed = B57600;
        break;
    case 115200:
        speed = B115200;
        break;
    case 230400:
        speed = B230400;
        break;
    default:
        speed = B115200;
        break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
        std::cerr << "[UartPort] tcsetattr failed\n";
        ::close(m_fd);
        m_fd = -1;
        return false;
    }
    return true;
}

void UartPort::Close() {
    if (m_fd >= 0) {
        tcsetattr(m_fd, TCSANOW, &m_orig);
        ::close(m_fd);
        m_fd = -1;
    }
}

ssize_t UartPort::Read(uint8_t* buf, size_t len, uint32_t timeout_ms) {
    if (m_fd < 0) {
        return -1;
    }
    fd_set set;
    FD_ZERO(&set);
    FD_SET(m_fd, &set);
    timeval tv{};
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int rv     = select(m_fd + 1, &set, nullptr, nullptr, &tv);
    if (rv > 0 && FD_ISSET(m_fd, &set)) {
        ssize_t r = ::read(m_fd, buf, len);
        return r;
    }
    return 0;
}

ssize_t UartPort::Write(const uint8_t* buf, size_t len) {
    if (m_fd < 0) {
        return -1;
    }
    ssize_t w = ::write(m_fd, buf, len);
    return w;
}
