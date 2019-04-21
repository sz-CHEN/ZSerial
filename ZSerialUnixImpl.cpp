#if defined(__linux__) || defined(__APPLE__)
// reference (http://www.cmrr.umn.edu/~strupp/serial.html)
#include <dirent.h>
#include <fcntl.h>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/termios.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#ifdef __linux__
#include <linux/serial.h>
#endif
#ifdef __APPLE__
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOBSD.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/serial/ioss.h>
#endif
#include "ZSerial.h"
#ifndef TIOCINQ
#ifdef FIONREAD
#define TIOCINQ FIONREAD
#else
#define TIOCINQ 0x541B
#endif
#endif
namespace ZSerial {
#ifdef __APPLE__
static kern_return_t MyFindModems(io_iterator_t* matchingServices) {
    kern_return_t kernResult;
    mach_port_t masterPort;
    CFMutableDictionaryRef classesToMatch;

    kernResult = IOMasterPort(MACH_PORT_NULL, &masterPort);
    if (KERN_SUCCESS != kernResult) {
        printf("IOMasterPort returned %d\n", kernResult);
        goto exit;
    }

    // Serial devices are instances of class IOSerialBSDClient.
    classesToMatch = IOServiceMatching(kIOSerialBSDServiceValue);
    if (classesToMatch == NULL) {
        printf("IOServiceMatching returned a NULL dictionary.\n");
    } else {
        CFDictionarySetValue(classesToMatch, CFSTR(kIOSerialBSDTypeKey),
                             CFSTR(kIOSerialBSDAllTypes));

        // Each serial device object has a property with key
        // kIOSerialBSDTypeKey and a value that is one of
        // kIOSerialBSDAllTypes, kIOSerialBSDModemType,
        // or kIOSerialBSDRS232Type. You can change the
        // matching dictionary to find other types of serial
        // devices by changing the last parameter in the above call
        // to CFDictionarySetValue.
    }

    kernResult = IOServiceGetMatchingServices(masterPort, classesToMatch,
                                              matchingServices);
    if (KERN_SUCCESS != kernResult) {
        printf("IOServiceGetMatchingServices returned %d\n", kernResult);
        goto exit;
    }

exit:
    return kernResult;
}
static kern_return_t MyGetModemPath(io_iterator_t serialPortIterator,
                                    std::vector<std::string>& deviceFilePaths) {
    io_object_t modemService = serialPortIterator;
    kern_return_t kernResult = KERN_FAILURE;
    // Boolean modemFound = false;

    // Initialize the returned path
    char deviceFilePath[PATH_MAX] = "\0";

    // Iterate across all modems found. In this example, we exit after
    // finding the first modem.

    while (/*(!modemFound) &&*/
           (modemService = IOIteratorNext(serialPortIterator))) {
        CFTypeRef deviceFilePathAsCFString;

        // Get the callout device's path (/dev/cu.xxxxx).
        // The callout device should almost always be
        // used. You would use the dialin device (/dev/tty.xxxxx) when
        // monitoring a serial port for
        // incoming calls, for example, a fax listener.

        deviceFilePathAsCFString = IORegistryEntryCreateCFProperty(
            modemService, CFSTR(kIOCalloutDeviceKey), kCFAllocatorDefault, 0);
        if (deviceFilePathAsCFString) {
            Boolean result;

            // Convert the path from a CFString to a NULL-terminated C string
            // for use with the POSIX open() call.

            result = CFStringGetCString(
                CFCopyDescription(deviceFilePathAsCFString), deviceFilePath,
                PATH_MAX, kCFStringEncodingASCII);
            CFRelease(deviceFilePathAsCFString);

            if (result) {
                deviceFilePaths.push_back(deviceFilePath);
                // printf("BSD path: %s", deviceFilePath);
                // modemFound = true;
                kernResult = KERN_SUCCESS;
            }
        }

        // printf("\n");

        // Release the io_service_t now that we are done with it.
        (void)IOObjectRelease(modemService);
    }

    return kernResult;
}

#endif
SerialPort::SerialPort(std::string portName, int baudrate, Parity parity,
                       DataBits databits, StopBits stopbits,
                       Handshake handshake)
    : portName(portName),
      baudrate(baudrate),
      parity(parity),
      databits(databits),
      stopbits(stopbits),
      handshake(handshake),
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
    std::vector<std::string> rets;
#ifdef __APPLE__
    // int fileDescriptor;
    kern_return_t kernResult;

    io_iterator_t serialPortIterator;
    // char deviceFilePath[PATH_MAX];

    kernResult = MyFindModems(&serialPortIterator);
    // while (serialPortIterator != 0) {
    kernResult = MyGetModemPath(serialPortIterator, rets);
    // serialPortIterator = IOIteratorNext(serialPortIterator);
    // std::cout << deviceFilePath << std::endl;
    // }
    IOObjectRelease(serialPortIterator);  // Release the iterator.
    for (auto& name : rets) {
        name.erase(name.begin(), name.begin() + name.find_first_of("\"") + 1);
        name.erase(name.begin() + name.find_first_of("\""), name.end());
    }
#else
    struct dirent** namelist;
    int n;

    n = scandir("/dev/serial/by-id", &namelist, NULL, alphasort);
    if (n == -1) {
    } else {
        while (n--) {
            char name[PATH_MAX];
            auto* pname = realpath(
                ("/dev/serial/by-id/" + std::string(namelist[n]->d_name))
                    .c_str(),
                name);
            if (pname == name) {
                struct stat path_stat;
                stat(name, &path_stat);
                if (S_ISCHR(path_stat.st_mode)) {
                    rets.push_back(std::string(name));
                }
            }
            free(namelist[n]);
        }
        free(namelist);
    }
    std::ifstream f;
    f.open("/proc/tty/driver/serial");
    if (!f.good()) {
        int err = errno;
        if (err == EACCES) {
            std::cout << "Check local port need root permission" << std::endl;
        } else {
            std::cout << strerror(errno) << std::endl;
        }
    } else {
        std::string line;
        while (getline(f, line)) {
            if (line.find("tx") != line.npos) {
                std::string name = "/dev/ttyS";
                line.erase(line.begin() + line.find_first_of(':'), line.end());
                name += line;
                struct stat path_stat;
                stat(name.c_str(), &path_stat);
                if (S_ISCHR(path_stat.st_mode)) {
                    rets.push_back(std::string(name));
                } else {
                    std::transform(name.begin(), name.end(), name.begin(),
                                   ::tolower);
                    if (S_ISCHR(path_stat.st_mode)) {
                        rets.push_back(std::string(name));
                    }
                }
            }
        }
    }
    f.close();
#endif
    return rets;
}

std::vector<std::pair<std::string, std::string>>
SerialPort::GetPortNamesAndDescriptions() {
    std::vector<std::pair<std::string, std::string>> rets;
#ifdef __APPLE__

    std::vector<std::string> ret;
    kern_return_t kernResult;

    io_iterator_t serialPortIterator;
    // char deviceFilePath[PATH_MAX];

    kernResult = MyFindModems(&serialPortIterator);
    // while (serialPortIterator != 0) {
    kernResult = MyGetModemPath(serialPortIterator, ret);
    // serialPortIterator = IOIteratorNext(serialPortIterator);
    // std::cout << deviceFilePath << std::endl;
    // }
    IOObjectRelease(serialPortIterator);  // Release the iterator.
    for (auto& name : ret) {
        name.erase(name.begin(), name.begin() + name.find_first_of("\"") + 1);
        name.erase(name.begin() + name.find_first_of("\""), name.end());
        if (name.find_first_of('.') != name.npos) {
            rets.push_back(
                {name, std::string(name.begin() + name.find_first_of('.') + 1,
                                   name.end())});
        } else {
            rets.push_back({name, "Serial"});
        }
    }
#else
    struct dirent** namelist;
    int n;

    n = scandir("/dev/serial/by-id", &namelist, NULL, alphasort);
    if (n == -1) {
    } else {
        while (n--) {
            char name[PATH_MAX];
            auto* pname = realpath(
                ("/dev/serial/by-id/" + std::string(namelist[n]->d_name))
                    .c_str(),
                name);
            if (pname == name) {
                struct stat path_stat;
                stat(name, &path_stat);
                if (S_ISCHR(path_stat.st_mode)) {
                    rets.push_back(
                        {std::string(name), std::string(namelist[n]->d_name)});
                }
            }
            free(namelist[n]);
        }
        free(namelist);
    }
    std::ifstream f;
    f.open("/proc/tty/driver/serial");
    if (!f.good()) {
        int err = errno;
        if (err == EACCES) {
            std::cout << "Check local port need root permission" << std::endl;
        } else {
            std::cout << strerror(errno) << std::endl;
        }
    } else {
        std::string line;
        while (getline(f, line)) {
            if (line.find("tx") != line.npos) {
                std::string name = "/dev/ttyS";
                line.erase(line.begin() + line.find_first_of(':'), line.end());
                name += line;
                struct stat path_stat;
                stat(name.c_str(), &path_stat);
                if (S_ISCHR(path_stat.st_mode)) {
                    rets.push_back({std::string(name), "serial"});
                } else {
                    std::transform(name.begin(), name.end(), name.begin(),
                                   ::tolower);
                    if (S_ISCHR(path_stat.st_mode)) {
                        rets.push_back({std::string(name), "serial"});
                    }
                }
            }
        }
    }
    f.close();
#endif
    return rets;
}

int SerialPort::Open() {
    hcom = (void*)(intptr_t)open(portName.c_str(), O_RDWR | O_NOCTTY);
    if ((intptr_t)hcom < 0) {
        // printf("open %s error\n",portName.c_str());
        return 1;
    }

    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr((intptr_t)hcom, &options) != 0) {
        Close();
        // printf("tcgetattr %s error\n",portName.c_str());
        return 2;
    }
    fcntl((intptr_t)hcom, F_SETFL, 0);

    bool custom_baud = false;
    speed_t baudrate_ = B0;
    switch (baudrate) {
#ifdef B50
        case 50:
            baudrate_ = B50;
            break;
#endif
#ifdef B75
        case 75:
            baudrate_ = B75;
            break;
#endif
#ifdef B110
        case 110:
            baudrate_ = B110;
            break;
#endif
#ifdef B134
        case 134:
            baudrate_ = B134;
            break;
#endif
#ifdef B150
        case 150:
            baudrate_ = B150;
            break;
#endif
#ifdef B200
        case 200:
            baudrate_ = B200;
            break;
#endif
#ifdef B300
        case 300:
            baudrate_ = B300;
            break;
#endif
#ifdef B600
        case 600:
            baudrate_ = B600;
            break;
#endif
#ifdef B1200
        case 1200:
            baudrate_ = B1200;
            break;
#endif
#ifdef B1800
        case 1800:
            baudrate_ = B1800;
            break;
#endif
#ifdef B2400
        case 2400:
            baudrate_ = B2400;
            break;
#endif
#ifdef B4800
        case 4800:
            baudrate_ = B4800;
            break;
#endif
#ifdef B9600
        case 9600:
            baudrate_ = B9600;
            break;
#endif
#ifdef B19200
        case 19200:
            baudrate_ = B19200;
            break;
#endif
#ifdef B38400
        case 38400:
            baudrate_ = B38400;
            break;
#endif
#ifdef B57600
        case 57600:
            baudrate_ = B57600;
            break;
#endif
#ifdef B115200
        case 115200:
            baudrate_ = B115200;
            break;
#endif
#ifdef B230400
        case 230400:
            baudrate_ = B230400;
            break;
#endif
#ifdef B460800
        case 460800:
            baudrate_ = B460800;
            break;
#endif
#ifdef B500000
        case 500000:
            baudrate_ = B500000;
            break;
#endif
#ifdef B576000
        case 576000:
            baudrate_ = B576000;
            break;
#endif
#ifdef B921600
        case 921600:
            baudrate_ = B921600;
            break;
#endif
#ifdef B1000000
        case 1000000:
            baudrate_ = B1000000;
            break;
#endif
#ifdef B1152000
        case 1152000:
            baudrate_ = B1152000;
            break;
#endif
#ifdef B1500000
        case 1500000:
            baudrate_ = B1500000;
            break;
#endif
#ifdef B2000000
        case 2000000:
            baudrate_ = B2000000;
            break;
#endif
#ifdef B2500000
        case 2500000:
            baudrate_ = B2500000;
            break;
#endif
#ifdef B3000000
        case 3000000:
            baudrate_ = B3000000;
            break;
#endif
#ifdef B3500000
        case 3500000:
            baudrate_ = B3500000;
            break;
#endif
#ifdef B4000000
        case 4000000:
            baudrate_ = B4000000;
            break;
#endif
        default:
            custom_baud = true;
            break;
    }
    if (!custom_baud) {
        cfsetospeed(&options, (intptr_t)baudrate_);
        cfsetispeed(&options, (intptr_t)baudrate_);
    } else {
#ifdef __linux__
        struct serial_struct ser;
        if (-1 == ioctl((intptr_t)hcom, TIOCGSERIAL, &ser)) {
            return 8;
        }
        ser.custom_divisor = ser.baud_base / static_cast<int>(baudrate);
        ser.flags &= ~ASYNC_SPD_MASK;
        ser.flags |= ASYNC_SPD_CUST;
        if (-1 == ioctl((intptr_t)hcom, TIOCSSERIAL, &ser)) {
            return 8;
        }
#else
        speed_t new_baud = baudrate;
        if (-1 == ioctl((intptr_t)hcom, IOSSIOSPEED, &new_baud, 1)) {
            return 8;
        }
#endif
    }

    options.c_cflag |= (CLOCAL | CREAD);

    switch (parity) {
        case Parity ::None:
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            options.c_iflag &= ~ISTRIP;
            break;
        case Parity ::Odd:
            options.c_cflag |= (PARENB | PARODD);
            options.c_iflag |= INPCK | ISTRIP;
            break;
        case Parity ::Even:
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK | ISTRIP;
            break;
#ifdef CMSPAR
        case Parity ::Mark:
            options.c_cflag |= (PARENB | PARODD | CMSPAR);
            options.c_iflag |= INPCK | ISTRIP;
            break;
        case Parity ::Space:
            options.c_cflag |= (PARENB | CMSPAR);
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK | ISTRIP;
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
    // options.c_iflag &= ~IGNBRK;
    // options.c_lflag = 0;
    options.c_lflag &=
        (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);
#ifdef IUCLC
    options.c_iflag &= (tcflag_t)~IUCLC;
#endif
#ifdef PARMRK
    options.c_iflag &= (tcflag_t)~PARMRK;
#endif
    if (tcsetattr((intptr_t)hcom, TCSANOW, &options) != 0) {
        Close();
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
    int bytes_available = 0;
    if (ioctl((intptr_t)hcom, TIOCINQ, &bytes_available) < 0) {
        printf("%s", strerror(errno));
        return std::string();
    }
    if (bytes_available <= 0) {
        return std::string();
    }
    std::string ret(bytes_available, 0);
    auto sz = read((intptr_t)hcom, &ret[0], bytes_available);
    ret.resize(sz);
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

std::future<std::string> SerialPort::ReadAsync(uint64_t time) {
    return std::async(
        [this](int hcom, uint64_t time) -> std::string {
            fd_set fdread, fdexcept;
            struct timeval timeout;
            timeout.tv_sec = time / 1000;
            timeout.tv_usec = (time % 1000) * 1000;

            /* Initialize the input set */
            FD_ZERO(&fdread);
            FD_SET((intptr_t)hcom, &fdread);
            if (select((intptr_t)hcom + 1, &fdread, nullptr, nullptr,
                       &timeout) > 0 &&
                FD_ISSET((intptr_t)hcom, &fdread)) {
                return this->ReadExisting();
            }
            return std::string();
        },
        (int)(intptr_t)hcom, time);
}

void SerialPort::Write(char* buffer, int offset, int count) {
    write((intptr_t)hcom, buffer + offset, count);
}
void SerialPort::Write(std::string text) {
    write((intptr_t)hcom, text.data(), text.size());
}
void SerialPort::WriteLine(std::string text, bool hasCR) {
    if (hasCR) text += '\r';
    text += '\n';
    write((intptr_t)hcom, text.data(), text.size());
}
bool SerialPort::IsOpen() { return opened; }

int SerialPort::SetPortName(std::string port) {
    if (IsOpen()) {
        return -1;
    } else {
        this->portName = port;
        return 0;
    }
}

int SerialPort::SetBaudRate(int baudrate) {
    if (!IsOpen()) {
        this->baudrate = baudrate;
        return -1;
    }
    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr((intptr_t)hcom, &options) != 0) {
        // printf("tcgetattr %s error\n",portName.c_str());
        return 2;
    }
    bool custom_baud = false;
    speed_t baudrate_ = B0;
    switch (baudrate) {
#ifdef B50
        case 50:
            baudrate_ = B50;
            break;
#endif
#ifdef B75
        case 75:
            baudrate_ = B75;
            break;
#endif
#ifdef B110
        case 110:
            baudrate_ = B110;
            break;
#endif
#ifdef B134
        case 134:
            baudrate_ = B134;
            break;
#endif
#ifdef B150
        case 150:
            baudrate_ = B150;
            break;
#endif
#ifdef B200
        case 200:
            baudrate_ = B200;
            break;
#endif
#ifdef B300
        case 300:
            baudrate_ = B300;
            break;
#endif
#ifdef B600
        case 600:
            baudrate_ = B600;
            break;
#endif
#ifdef B1200
        case 1200:
            baudrate_ = B1200;
            break;
#endif
#ifdef B1800
        case 1800:
            baudrate_ = B1800;
            break;
#endif
#ifdef B2400
        case 2400:
            baudrate_ = B2400;
            break;
#endif
#ifdef B4800
        case 4800:
            baudrate_ = B4800;
            break;
#endif
#ifdef B9600
        case 9600:
            baudrate_ = B9600;
            break;
#endif
#ifdef B19200
        case 19200:
            baudrate_ = B19200;
            break;
#endif
#ifdef B38400
        case 38400:
            baudrate_ = B38400;
            break;
#endif
#ifdef B57600
        case 57600:
            baudrate_ = B57600;
            break;
#endif
#ifdef B115200
        case 115200:
            baudrate_ = B115200;
            break;
#endif
#ifdef B230400
        case 230400:
            baudrate_ = B230400;
            break;
#endif
#ifdef B460800
        case 460800:
            baudrate_ = B460800;
            break;
#endif
#ifdef B500000
        case 500000:
            baudrate_ = B500000;
            break;
#endif
#ifdef B576000
        case 576000:
            baudrate_ = B576000;
            break;
#endif
#ifdef B921600
        case 921600:
            baudrate_ = B921600;
            break;
#endif
#ifdef B1000000
        case 1000000:
            baudrate_ = B1000000;
            break;
#endif
#ifdef B1152000
        case 1152000:
            baudrate_ = B1152000;
            break;
#endif
#ifdef B1500000
        case 1500000:
            baudrate_ = B1500000;
            break;
#endif
#ifdef B2000000
        case 2000000:
            baudrate_ = B2000000;
            break;
#endif
#ifdef B2500000
        case 2500000:
            baudrate_ = B2500000;
            break;
#endif
#ifdef B3000000
        case 3000000:
            baudrate_ = B3000000;
            break;
#endif
#ifdef B3500000
        case 3500000:
            baudrate_ = B3500000;
            break;
#endif
#ifdef B4000000
        case 4000000:
            baudrate_ = B4000000;
            break;
#endif
        default:
            custom_baud = true;
            break;
    }
    if (!custom_baud) {
        cfsetospeed(&options, (intptr_t)baudrate_);
        cfsetispeed(&options, (intptr_t)baudrate_);
    } else {
#ifdef __linux__
        struct serial_struct ser;
        if (-1 == ioctl((intptr_t)hcom, TIOCGSERIAL, &ser)) {
            return 8;
        }
        ser.custom_divisor = ser.baud_base / static_cast<int>(baudrate);
        ser.flags &= ~ASYNC_SPD_MASK;
        ser.flags |= ASYNC_SPD_CUST;
        if (-1 == ioctl((intptr_t)hcom, TIOCSSERIAL, &ser)) {
            return 8;
        }
#else
        speed_t new_baud = baudrate;
        if (-1 == ioctl((intptr_t)hcom, IOSSIOSPEED, &new_baud, 1)) {
            return 8;
        }
#endif
    }
    if (tcsetattr((intptr_t)hcom, TCSANOW, &options) != 0) {
        // printf("error %d from tcsetattr", errno);
        return 7;
    }
    this->baudrate = baudrate;
    return 0;
}
int SerialPort::SetParity(Parity parity) {
    if (!IsOpen()) {
        this->parity = parity;
        return -1;
    }
    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr((intptr_t)hcom, &options) != 0) {
        // printf("tcgetattr %s error\n",portName.c_str());
        return 2;
    }
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
    if (tcsetattr((intptr_t)hcom, TCSANOW, &options) != 0) {
        // printf("error %d from tcsetattr", errno);
        return 7;
    }
    this->parity = parity;
    return 0;
}
int SerialPort::SetDataBits(DataBits databits) {
    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr((intptr_t)hcom, &options) != 0) {
        // printf("tcgetattr %s error\n",portName.c_str());
        return 2;
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
    if (tcsetattr((intptr_t)hcom, TCSANOW, &options) != 0) {
        // printf("error %d from tcsetattr", errno);
        return 7;
    }
    this->databits = databits;
    return 0;
}
int SerialPort::SetStopBits(StopBits stopbits) {
    if (!IsOpen()) {
        this->stopbits = stopbits;
        return -1;
    }
    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr((intptr_t)hcom, &options) != 0) {
        // printf("tcgetattr %s error\n",portName.c_str());
        return 2;
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
    if (tcsetattr((intptr_t)hcom, TCSANOW, &options) != 0) {
        // printf("error %d from tcsetattr", errno);
        return 7;
    }
    this->stopbits = stopbits;
    return 0;
}

int SerialPort::SetHandshake(Handshake handshake) {
    if (!IsOpen()) {
        this->handshake = handshake;
        return -1;
    }
    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr((intptr_t)hcom, &options) != 0) {
        // printf("tcgetattr %s error\n",portName.c_str());
        return 2;
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
    if (tcsetattr((intptr_t)hcom, TCSANOW, &options) != 0) {
        // printf("error %d from tcsetattr", errno);
        return 7;
    }
    this->handshake = handshake;
    return 0;
}

std::string SerialPort::GetPortName() { return this->portName; }

int SerialPort::GetBaudRate() {
    if (!IsOpen()) {
        return baudrate;
    }
    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr((intptr_t)hcom, &options) != 0) {
        // printf("tcgetattr %s error\n",portName.c_str());
        return -2;
    }
    baudrate = cfgetospeed(&options);
    return baudrate;
}
Parity SerialPort::GetParity() {
    if (!IsOpen()) {
        return parity;
    }
    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr((intptr_t)hcom, &options) != 0) {
        // printf("tcgetattr %s error\n",portName.c_str());
        return (Parity)-2;
    }
    if ((options.c_cflag & PARENB) == 0) {
        parity = Parity::None;
    } else if ((options.c_cflag & PARODD) == 0) {
#ifdef CMSPAR
        if (options.c_cflag & CMSPAR == 0) {
            parity = Parity::Even;
        } else {
            parity = Parity::Space;
        }
#else
        parity = Parity::Even;
#endif
    } else {
#ifdef CMSPAR
        if (options.c_cflag & CMSPAR == 0) {
            parity = Parity::Odd;
        } else {
            parity = Parity::Mark;
        }
#else
        parity = Parity::Odd;
#endif
    }
    return parity;
}
DataBits SerialPort::GetDataBits() {
    if (!IsOpen()) {
        return databits;
    }
    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr((intptr_t)hcom, &options) != 0) {
        // printf("tcgetattr %s error\n",portName.c_str());
        return (DataBits)-2;
    }
    switch (options.c_cflag & CSIZE) {
        case CS8:
            databits = DataBits::DB_8;
            break;
        case CS7:
            databits = DataBits::DB_7;
            break;
        case CS6:
            databits = DataBits::DB_6;
            break;
        case CS5:
            databits = DataBits::DB_5;
            break;
        default:
            return (DataBits)-4;
    }
    return databits;
}
StopBits SerialPort::GetStopBits() {
    if (!IsOpen()) {
        return stopbits;
    }
    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr((intptr_t)hcom, &options) != 0) {
        // printf("tcgetattr %s error\n",portName.c_str());
        return (StopBits)-2;
    }
    if ((options.c_cflag & CSTOPB) == 0) {
        stopbits = StopBits::One;
    } else {
        stopbits = StopBits::Two;
    }
    return stopbits;
}
Handshake SerialPort::GetHandshake() {
    if (!IsOpen()) {
        return handshake;
    }
    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr((intptr_t)hcom, &options) != 0) {
        // printf("tcgetattr %s error\n",portName.c_str());
        return (Handshake)-2;
    }
    int32_t crtscts;
    int32_t ix;
    if ((crtscts = options.c_cflag & CRTSCTS) == CRTSCTS) {
        if ((ix = (options.c_iflag & (IXON | IXOFF | IXANY)) ==
                  (IXOFF | IXON | IXANY))) {
            handshake = Handshake::RequestToSendXOnXOff;
        } else if (ix == 0) {
            handshake = Handshake::RequestToSend;
        } else {
            return (Handshake)-6;
        }
    } else if (crtscts == 0) {
        if ((ix = (options.c_iflag & (IXON | IXOFF | IXANY)) ==
                  (IXOFF | IXON | IXANY))) {
            handshake = Handshake::XOnXOff;
        } else if (ix == 0) {
            handshake = Handshake::None;
        } else {
            return (Handshake)-6;
        }
    } else {
        return (Handshake)-6;
    }
    return handshake;
}
}  // namespace ZSerial
#endif