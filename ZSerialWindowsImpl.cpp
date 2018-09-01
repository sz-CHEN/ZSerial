#ifdef _WIN32
#include <Windows.h>
#include <process.h>
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
    CloseHandle(hcom);
    hcom = 0;
    opened = false;
}
void SerialPort::DiscardInBuffer() {
    if (!PurgeComm(hcom, PURGE_RXABORT | PURGE_RXCLEAR)) {
        printf_s("PurgeComm failed with error %d.\n", GetLastError());
    }
}
void SerialPort::DiscardOutBuffer() {
    if (!PurgeComm(hcom, PURGE_TXABORT | PURGE_TXCLEAR)) {
        printf_s("PurgeComm failed with error %d.\n", GetLastError());
    }
}
std::vector<std::string> SerialPort::GetPortNames() {
    std::vector<std::string> rets;
    HKEY hKey;
    RegOpenKeyA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM\\", &hKey);
    unsigned char buf[256];
    char buff[256];
    DWORD n = 256;
    DWORD nn = 256;
    int i = 0;
    while (ERROR_SUCCESS ==
           RegEnumValueA(hKey, i++, buff, &nn, NULL, NULL, buf, &n)) {
        std::string str(reinterpret_cast<char const*>(buf));
        rets.push_back(str);
        // printf_s("%s\t%s\t\n", buf, buff);
        n = 256;
        nn = 256;
    }
    // printf_s("\n");
    return rets;
}
int SerialPort::Open() {
    hcom = CreateFile(TEXT(("\\\\.\\" + portName).c_str()),
                      GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0,
                      NULL);
    if (hcom == INVALID_HANDLE_VALUE) {
        // printf_s("CreateFile failed with error %d.\n", GetLastError());
        return 1;
    }
    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hcom, &dcb)) {
        return 2;
    }
    dcb.BaudRate = (int)baudrate;
    dcb.ByteSize = (int)databits;
    dcb.Parity = (int)parity;
    dcb.StopBits = (int)stopbits;
    switch (handshake) {
        case Handshake::None:
            dcb.fRtsControl = RTS_CONTROL_DISABLE;
            dcb.fOutX = false;
            dcb.fInX = false;
            break;
        case Handshake::RequestToSend:
            dcb.fRtsControl = RTS_CONTROL_ENABLE;
            dcb.fOutX = false;
            dcb.fInX = false;
            break;
        case Handshake::RequestToSendXOnXOff:
            dcb.fRtsControl = RTS_CONTROL_ENABLE;
            dcb.fOutX = true;
            dcb.fInX = true;
            break;
        case Handshake::XOnXOff:
            dcb.fRtsControl = RTS_CONTROL_DISABLE;
            dcb.fOutX = true;
            dcb.fInX = true;
            break;
        default:
            return 6;
    }
    if (!SetCommState(hcom, &dcb)) {
        return 7;
    }
    opened = true;
    return 0;
}
int SerialPort::Read(char* buffer, int offset, int count) {
    unsigned long read = 0;
    ReadFile(hcom, buffer + offset, count, &read, NULL);
    return read;
}
char SerialPort::ReadByte() {
    char c;
    DWORD read;
    ReadFile(hcom, &c, 1, &read, NULL);
    return c;
}

std::string SerialPort::ReadExisting() {
    DWORD dwError;
    COMSTAT cs;
    ClearCommError(hcom, &dwError, &cs);
    std::string ret(cs.cbInQue, 0);
    DWORD read = 0;
    ReadFile(hcom, &ret[0], cs.cbInQue, &read, NULL);
    ret.resize(read);
    return ret;
}

std::string SerialPort::ReadLine() {
    std::string str;
    char c;
    DWORD read;
    while (true) {
        ReadFile(hcom, &c, 1, &read, NULL);
        if (read == 1) {
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
    DWORD write = 0;
    WriteFile(hcom, buffer + offset, count, &write, NULL);
}
void SerialPort::Write(std::string text) {
    DWORD write = 0;
    WriteFile(hcom, &text[0], text.size(), &write, NULL);
}
void SerialPort::WriteLine(std::string text) {
    text += "\r\n";
    unsigned long write = 0;
    WriteFile(hcom, &text[0], text.size(), &write, NULL);
}
bool SerialPort::IsOpen() { return opened; }
}  // namespace ZSerial
#endif