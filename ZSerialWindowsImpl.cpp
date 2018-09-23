#ifdef _WIN32
#ifndef _WINDOWS_
#include <Windows.h>
#endif
#include <SetupAPI.h>
#include <process.h>
#include "ZSerial.h"
#pragma comment(lib, "setupapi.lib")
namespace ZSerial {
SerialPort::SerialPort(std::string portName, int baudrate, Parity parity,
                       DataBits databits, StopBits stopbits,Handshake handshake)
    : portName(portName),
      baudrate(baudrate),
      parity(parity),
      databits(databits),
      stopbits(stopbits),
      handshake(handshake),
      hcom(0),
      opened(false){
          
      }
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
        std::string str(reinterpret_cast<char const *>(buf));
        rets.push_back(str);
        // printf_s("%s\t%s\t\n", buf, buff);
        n = 256;
        nn = 256;
    }
    // printf_s("\n");
    return rets;
}

std::vector<std::pair<std::string, std::string>>
SerialPort::GetPortNamesAndDescriptions() {
    {
        std::vector<std::pair<std::string, std::string>> rets;
        std::string strErr;
        GUID *guidDev = (GUID *)&GUID_CLASS_COMPORT;
        HDEVINFO hDevInfo = INVALID_HANDLE_VALUE;
        SP_DEVICE_INTERFACE_DETAIL_DATA_A *pDetData = NULL;
        hDevInfo = SetupDiGetClassDevsA(guidDev, NULL, NULL,
                                        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
        if (hDevInfo == INVALID_HANDLE_VALUE) {
            return rets;
        }
        BOOL bOk = TRUE;
        SP_DEVICE_INTERFACE_DATA ifcData;
        DWORD dwDetDataSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA) + 256;
        pDetData = (SP_DEVICE_INTERFACE_DETAIL_DATA_A *)new char[dwDetDataSize];
        ifcData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
        pDetData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
        for (DWORD ii = 0; bOk; ii++) {
            bOk = SetupDiEnumDeviceInterfaces(hDevInfo, NULL, guidDev, ii,
                                              &ifcData);
            if (bOk) {
                SP_DEVINFO_DATA devdata = {sizeof(SP_DEVINFO_DATA)};
                bOk = SetupDiGetDeviceInterfaceDetailA(hDevInfo, &ifcData,
                                                       pDetData, dwDetDataSize,
                                                       NULL, &devdata);
                if (bOk) {
                    char fname[256];
                    auto bSuccess = SetupDiGetDeviceRegistryPropertyA(
                        hDevInfo, &devdata, SPDRP_FRIENDLYNAME, NULL,
                        (PBYTE)fname, sizeof(fname), NULL);
                    if (bSuccess) {
                        std::string name(fname);
                        auto iterS = name.find("(") + name.begin();
                        auto iterE = name.find(")") + name.begin();
                        std::string desc(name.begin(), iterS);
                        name = std::string(iterS + 1, iterE);
                        rets.push_back({name, desc});
                    }
                } else {
                }
            } else {
                return rets;
            }
        }
        if (pDetData != NULL) delete[](char *) pDetData;
        if (hDevInfo != INVALID_HANDLE_VALUE)
            SetupDiDestroyDeviceInfoList(hDevInfo);

        return rets;
    }
    std::vector<std::pair<std::string, std::string>> rets;
    HKEY hKey;
    RegOpenKeyA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM\\", &hKey);
    unsigned char buf[256];
    char buff[256];
    DWORD n = 256;
    DWORD nn = 256;
    int i = 0;
    while (ERROR_SUCCESS ==
           RegEnumValueA(hKey, i++, buff, &nn, NULL, NULL, buf, &n)) {
        std::string com(reinterpret_cast<char const *>(buf));
        std::string des(reinterpret_cast<char const *>(buff));
        rets.push_back({com, des});
        // printf_s("%s\t%s\t\n", buf, buff);
        n = 256;
        nn = 256;
    }
    RegCloseKey(hKey);
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
int SerialPort::Read(char *buffer, int offset, int count) {
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

void SerialPort::Write(char *buffer, int offset, int count) {
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
int SerialPort::SetBaudRate(int baudrate) {
    if (!IsOpen()) {
        this->baudrate = baudrate;
        return -1;
    }
    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hcom, &dcb)) {
        return 2;
    }
    dcb.BaudRate = baudrate;
    if (!SetCommState(hcom, &dcb)) {
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
    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hcom, &dcb)) {
        return 2;
    }
    dcb.Parity = (int)parity;
    if (!SetCommState(hcom, &dcb)) {
        return 7;
    }
    this->parity = parity;
    return 0;
}
int SerialPort::SetDataBits(DataBits databits) {
    if (!IsOpen()) {
        this->databits = databits;
        return -1;
    }
    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hcom, &dcb)) {
        return 2;
    }
    dcb.ByteSize = (BYTE)databits;
    if (!SetCommState(hcom, &dcb)) {
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
    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hcom, &dcb)) {
        return 2;
    }
    dcb.StopBits = (BYTE)stopbits;
    if (!SetCommState(hcom, &dcb)) {
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
    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hcom, &dcb)) {
        return 2;
    }
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
    this->handshake = handshake;
    return 0;
}
int SerialPort::GetBaudRate() {
    if (!IsOpen()) {
        return baudrate;
    }
    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hcom, &dcb)) {
        return -2;
    }
    baudrate = dcb.BaudRate;
    return baudrate;
}
Parity SerialPort::GetParity() {
    if (!IsOpen()) {
        return parity;
    }
    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hcom, &dcb)) {
        return (Parity)-2;
    }
    parity = (Parity)dcb.Parity;
    return parity;
}
DataBits SerialPort::GetDataBits() {
    if (!IsOpen()) {
        return databits;
    }
    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hcom, &dcb)) {
        return (DataBits)-2;
    }
    databits = (DataBits)dcb.ByteSize;
    return databits;
}
StopBits SerialPort::GetStopBits() {
    if (!IsOpen()) {
        return stopbits;
    }
    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hcom, &dcb)) {
        return (StopBits)-2;
    }
    stopbits = (StopBits)dcb.StopBits;
    return stopbits;
}
Handshake SerialPort::GetHandshake() {
    if (!IsOpen()) {
        return handshake;
    }
    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hcom, &dcb)) {
        return (Handshake)-2;
    }
    if (dcb.fRtsControl == RTS_CONTROL_DISABLE) {
        if (dcb.fOutX && dcb.fInX) {
            handshake = Handshake::XOnXOff;
        } else {
            if (dcb.fOutX || dcb.fInX) {
                return (Handshake)-6;
            } else {
                handshake = Handshake::None;
            }
        }
    } else if (dcb.fRtsControl == RTS_CONTROL_ENABLE) {
        if (dcb.fOutX && dcb.fInX) {
            handshake = Handshake::RequestToSendXOnXOff;
        } else {
            if (dcb.fOutX || dcb.fInX) {
                return (Handshake)-6;
            } else {
                handshake = Handshake::RequestToSend;
            }
        }
    } else {
        return (Handshake)-6;
    }
    return handshake;
}
}  // namespace ZSerial
#endif