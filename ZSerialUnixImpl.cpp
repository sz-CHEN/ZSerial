#if defined(__linux__) || defined(__APPLE__)
// reference (http://www.cmrr.umn.edu/~strupp/serial.html)
#include <fcntl.h>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/termios.h>
#include <unistd.h>
#include <cstring>
#include "ZSerial.h"
namespace ZSerial {
SerialPort::SerialPort(std::string portName, int baudrate, Parity parity,
                       DataBits databits, StopBits stopbits)
    : portName(portName),
      baudrate(baudrate),
      parity(parity),
      databits(databits),
      stopbits(stopbits),
      handshake(Handshake::None),
      hcom(0),
      opened(false) {}
SerialPort::~SerialPort() { Close(); }
void SerialPort::Close() {
    close((intptr_t)hcom);
    hcom = 0;
    opened = false;
}
void SerialPort::DiscardInBuffer() { tcflush((intptr_t)hcom, TCIFLUSH); }
void SerialPort::DiscardOutBuffer() { tcflush((intptr_t)hcom, TCOFLUSH); }
std::vector<std::string> SerialPort::GetPortNames() {
    return std::vector<std::string>();
}

std::vector<std::pair<std::string, std::string>>
SerialPort::GetPortNamesAndDescriptions() {
    return std::vector<std::pair<std::string, std::string>>();
}

int SerialPort::Open() {
    hcom =
        (void*)(intptr_t)open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if ((intptr_t)hcom < 0) {
        // printf("open %s error\n",portName.c_str());
        return 1;
    }

    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr((intptr_t)hcom, &options) != 0) {
        // printf("tcgetattr %s error\n",portName.c_str());
        return 2;
    }
    fcntl((intptr_t)hcom, F_SETFL, 0);

    options.c_cflag |= (CLOCAL | CREAD);

    cfsetospeed(&options, (intptr_t)baudrate);
    cfsetispeed(&options, (intptr_t)baudrate);

    switch (parity) {
        case Parity ::None:
            options.c_cflag &= ~PARENB;
            break;
        case Parity ::Odd:
            options.c_cflag |= (PARENB | PARODD);
            break;
        case Parity ::Even:
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            break;
#ifdef CMSPAR
        case Parity ::Mark:
            options.c_cflag |= (PARENB | PARODD | CMSPAR);
            break;
        case Parity ::Space:
            options.c_cflag |= (PARENB | CMSPAR);
            options.c_cflag &= ~PARODD;
            break;
#endif
        default:
            // printf("parity error\n");
            return 3;
    }

    switch (databits) {
        case DataBits::DB_8:
            options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
            break;
        case DataBits::DB_7:
            options.c_cflag = (options.c_cflag & ~CSIZE) | CS7;
            break;
        case DataBits::DB_6:
            options.c_cflag = (options.c_cflag & ~CSIZE) | CS6;
            break;
        case DataBits::DB_5:
            options.c_cflag = (options.c_cflag & ~CSIZE) | CS5;
            break;
        default:
            // printf("\nbytesize error\n");
            return 4;
    }

    switch (stopbits) {
        case StopBits::One:
            options.c_cflag &= ~CSTOPB;
            break;
        case StopBits::Two:
            options.c_cflag |= CSTOPB;
            break;
        case StopBits::OnePointFive:
        case StopBits::None:
        default:
            // printf("stopbit error\n");
            return 5;
    }

    switch (handshake) {
        case Handshake::None:
            options.c_iflag &= ~(IXON | IXOFF | IXANY);
            options.c_cflag &= ~CRTSCTS;
            break;
        case Handshake::RequestToSend:
            options.c_iflag &= ~(IXON | IXOFF | IXANY);
            options.c_cflag |= CRTSCTS;
            break;
        case Handshake::RequestToSendXOnXOff:
            options.c_iflag |= (IXON | IXOFF | IXANY);
            options.c_cflag |= CRTSCTS;
            break;
        case Handshake::XOnXOff:
            options.c_iflag |= (IXON | IXOFF | IXANY);
            options.c_cflag &= ~CRTSCTS;
            break;
        default:
            return 6;
    }
    options.c_iflag &= ~IGNBRK;
    options.c_lflag = 0;
    if (tcsetattr((intptr_t)hcom, TCSANOW, &options) != 0) {
        // printf("error %d from tcsetattr", errno);
        return 7;
    }
    opened = true;
    return 0;
}

int SerialPort::Read(char* buffer, int offset, int count) {
    return read((intptr_t)hcom, buffer + offset, count);
}
char SerialPort::ReadByte() {
    char c;
    read((intptr_t)hcom, &c, 1);
    return c;
}
std::string SerialPort::ReadExisting() {
    fcntl((intptr_t)hcom, F_SETFL, FNDELAY);
    int bytes_available;
    ioctl((intptr_t)hcom, FIONREAD, &bytes_available);
    std::string ret(bytes_available, 0);
    auto sz = read((intptr_t)hcom, &ret[0], bytes_available);
    ret.resize(sz);
    fcntl((intptr_t)hcom, F_SETFL, 0);
    return ret;
}

std::string SerialPort::ReadLine() {
    std::string str;
    char c;
    while (true) {
        auto s = read((intptr_t)hcom, &c, 1);
        if (s == 1) {
            if (c == '\n') {
                if (str.back() == '\r') {
                    str.pop_back();
                }
                return str;
            }
            str.push_back(c);
        }
    }
}

void SerialPort::Write(char* buffer, int offset, int count) {
    write((intptr_t)hcom, buffer + offset, count);
}
void SerialPort::Write(std::string text) {
    write((intptr_t)hcom, text.data(), text.size());
}
void SerialPort::WriteLine(std::string text) {
    text += "\r\n";
    write((intptr_t)hcom, text.data(), text.size());
}
bool SerialPort::IsOpen() { return opened; }
}  // namespace ZSerial
#endif