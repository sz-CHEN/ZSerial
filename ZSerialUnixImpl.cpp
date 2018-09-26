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
#ifdef __APPLE__
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOBSD.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#endif
#include "ZSerial.h"
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
    hcom =
        (void*)(intptr_t)open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
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
    // #ifndef __APPLE__
    int bytes_available = 0;
    if (ioctl((intptr_t)hcom, FIONREAD, &bytes_available) < 0) {
        printf("%s", strerror(errno));
    }
    std::string ret(bytes_available, 0);
    auto sz = read((intptr_t)hcom, &ret[0], bytes_available);
    ret.resize(sz);
    return ret;
    // #else
    //     fd_set rfds;
    //     struct timeval tv;
    //     int retval;
    //     FD_ZERO(&rfds);
    //     FD_SET((intptr_t)hcom, &rfds);
    //     tv.tv_sec = 0;
    //     tv.tv_usec = 0;
    //     retval = select((intptr_t)hcom + 1, &rfds, NULL, NULL, &tv);
    //     if (retval > 0) {
    //         if (FD_ISSET((intptr_t)hcom, &rfds)) {
    //             std::string ret;
    //             char c;
    //             fcntl((intptr_t)hcom, F_SETFL, FNDELAY);
    //             while (read((intptr_t)hcom, &c, 1) > 0) {
    //                 ret.push_back(c);
    //             }
    //             // int bytes_available = 0;
    //             // if (ioctl((intptr_t)hcom, TIOCOUTQ, &bytes_available) < 0)
    //             {
    //             //     printf("%s", strerror(errno));
    //             // }
    //             // std::string ret(bytes_available, 0);
    //             // auto sz = read((intptr_t)hcom, &ret[0], bytes_available);
    //             // ret.resize(sz);
    //             fcntl((intptr_t)hcom, F_SETFL, 0);
    //             return ret;
    //         }
    //     }
    //     return "";
    // #endif
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
    cfsetospeed(&options, baudrate);
    cfsetispeed(&options, baudrate);
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