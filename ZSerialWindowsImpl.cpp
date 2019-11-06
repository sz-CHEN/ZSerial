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
    std::string strErr;
    GUID *guidDev = (GUID *)&GUID_CLASS_COMPORT;
    HDEVINFO hDevInfoSet = INVALID_HANDLE_VALUE;
    hDevInfoSet = SetupDiGetClassDevsA(guidDev, NULL, NULL,
                                       DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (hDevInfoSet == INVALID_HANDLE_VALUE) {
        return rets;
    }
    bool bMoreItems = true;
    int nIndex = 0;
    SP_DEVINFO_DATA devInfo = {0};
    while (bMoreItems) {
        // Enumerate the current device
        devInfo.cbSize = sizeof(SP_DEVINFO_DATA);
        bMoreItems = SetupDiEnumDeviceInfo(hDevInfoSet, nIndex, &devInfo);
        if (bMoreItems) {
            // Did we find a serial port for this device
            bool bAdded = false;

            HKEY deviceKey =
                SetupDiOpenDevRegKey(hDevInfoSet, &devInfo, DICS_FLAG_GLOBAL, 0,
                                     DIREG_DEV, KEY_QUERY_VALUE);
            if (deviceKey != INVALID_HANDLE_VALUE) {
                int nPort = 0;
                DWORD dwType, dwSize;
                dwSize = MAX_PATH;
                char pszValue[MAX_PATH];
                char sFriendlyName[MAX_PATH];
                ULONG nBytes;
                auto lRes =
                    RegQueryValueExA(deviceKey, "PortName", nullptr, &dwType,
                                     (LPBYTE)pszValue, &nBytes);

                if (lRes == ERROR_SUCCESS &&
                    !(dwType != REG_SZ && dwType != REG_EXPAND_SZ)) {
                    if (nBytes != 0) {
                        std::string name(pszValue);
                        rets.push_back(name);
                    }
                }
            }
            RegCloseKey(deviceKey);
        }
        ++nIndex;
    }
    SetupDiDestroyDeviceInfoList(hDevInfoSet);
    return rets;
    // HKEY hKey;
    // RegOpenKeyA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM\\",
    // &hKey); unsigned char buf[256]; char buff[256]; DWORD n = 256; DWORD nn =
    // 256; int i = 0; while (ERROR_SUCCESS ==
    //        RegEnumValueA(hKey, i++, buff, &nn, NULL, NULL, buf, &n)) {
    //     std::string str(reinterpret_cast<char const *>(buf));
    //     rets.push_back(str);
    //     // printf_s("%s\t%s\t\n", buf, buff);
    //     n = 256;
    //     nn = 256;
    // }
    // RegCloseKey(hKey);
    // // printf_s("\n");
    // return rets;
}

std::vector<std::pair<std::string, std::string>>
SerialPort::GetPortNamesAndDescriptions() {
    std::vector<std::pair<std::string, std::string>> rets;
    std::string strErr;
    GUID *guidDev = (GUID *)&GUID_CLASS_COMPORT;
    HDEVINFO hDevInfoSet = INVALID_HANDLE_VALUE;
    hDevInfoSet = SetupDiGetClassDevsA(guidDev, NULL, NULL,
                                       DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (hDevInfoSet == INVALID_HANDLE_VALUE) {
        return rets;
    }
    bool bMoreItems = true;
    int nIndex = 0;
    SP_DEVINFO_DATA devInfo = {0};
    while (bMoreItems) {
        // Enumerate the current device
        devInfo.cbSize = sizeof(SP_DEVINFO_DATA);
        bMoreItems = SetupDiEnumDeviceInfo(hDevInfoSet, nIndex, &devInfo);
        if (bMoreItems) {
            // Did we find a serial port for this device
            bool bAdded = false;

            HKEY deviceKey =
                SetupDiOpenDevRegKey(hDevInfoSet, &devInfo, DICS_FLAG_GLOBAL, 0,
                                     DIREG_DEV, KEY_QUERY_VALUE);
            if (deviceKey != INVALID_HANDLE_VALUE) {
                int nPort = 0;
                DWORD dwType, dwSize;
                dwSize = MAX_PATH;
                char pszValue[MAX_PATH];
                char sFriendlyName[MAX_PATH];
                ULONG nBytes;
                auto lRes =
                    RegQueryValueExA(deviceKey, "PortName", nullptr, &dwType,
                                     (LPBYTE)pszValue, &nBytes);

                if (lRes == ERROR_SUCCESS &&
                    !(dwType != REG_SZ && dwType != REG_EXPAND_SZ)) {
                    if (nBytes != 0) {
                        std::string name(pszValue);
                        if (SetupDiGetDeviceRegistryPropertyA(
                                hDevInfoSet, &devInfo, SPDRP_DEVICEDESC,
                                &dwType,
                                reinterpret_cast<PBYTE>(&(sFriendlyName[0])),
                                dwSize, &dwSize)) {
                            if (dwType == REG_SZ || dwType == REG_EXPAND_SZ) {
                                rets.push_back(
                                    {name, std::string(sFriendlyName)});
                            }
                        }
                    }
                }
            }
            RegCloseKey(deviceKey);
        }
        ++nIndex;
    }
    SetupDiDestroyDeviceInfoList(hDevInfoSet);
    return rets;
}

int SerialPort::Open() {
    hcom = CreateFile(TEXT(("\\\\.\\" + portName).c_str()),
                      GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
                      FILE_FLAG_OVERLAPPED, NULL);
    if (hcom == INVALID_HANDLE_VALUE) {
        // printf_s("CreateFile failed with error %d.\n", GetLastError());
        return 1;
    }
    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hcom, &dcb)) {
        Close();
        return 2;
    }
    dcb.BaudRate = (int)baudrate;
    dcb.ByteSize = (int)databits;
    dcb.Parity = (int)parity;
    dcb.StopBits = (int)stopbits;
    switch (handshake) {
        case Handshake::None:
            dcb.fRtsControl = RTS_CONTROL_DISABLE;
            dcb.fOutxCtsFlow = FALSE;
            dcb.fOutX = false;
            dcb.fInX = false;
            break;
        case Handshake::RequestToSend:
            dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
            dcb.fOutxCtsFlow = TRUE;
            dcb.fOutX = false;
            dcb.fInX = false;
            break;
        case Handshake::RequestToSendXOnXOff:
            dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
            dcb.fOutxCtsFlow = TRUE;
            dcb.fOutX = true;
            dcb.fInX = true;
            break;
        case Handshake::XOnXOff:
            dcb.fRtsControl = RTS_CONTROL_DISABLE;
            dcb.fOutxCtsFlow = FALSE;
            dcb.fOutX = true;
            dcb.fInX = true;
            break;
        default:
            return 6;
    }
    if (!SetCommState(hcom, &dcb)) {
        Close();
        return 7;
    }
    opened = true;
    return 0;
}
int SerialPort::Read(char *buffer, int offset, int count) {
    OVERLAPPED overlap = {0};
    overlap.hEvent = CreateEvent(nullptr, true, false, nullptr);
    DWORD read = 0;
    ReadFile(hcom, buffer + offset, count, &read, &overlap);
    GetOverlappedResult(hcom, &overlap, &read, true);
    CloseHandle(overlap.hEvent);
    return read;
}
char SerialPort::ReadByte() {
    char c = 0;
    Read(&c, 0, 1);
    return c;
}

std::string SerialPort::ReadExisting() {
    DWORD dwError;
    COMSTAT cs;
    memset(&cs, 0, sizeof(COMSTAT));
    if (!ClearCommError(hcom, &dwError, &cs)) {
        return std::string();
    }
    if (cs.cbInQue <= 0) {
        return std::string();
    }
    std::string ret(cs.cbInQue, 0);
    DWORD read = Read(&ret[0], 0, cs.cbInQue);
    ret.resize(read);
    return ret;
}

std::string SerialPort::ReadLine() {
    std::string str;
    char c;
    DWORD read;
    OVERLAPPED overlap = {0};
    overlap.hEvent = CreateEvent(nullptr, true, false, nullptr);
    while (true) {
        ReadFile(hcom, &c, 1, &read, &overlap);
        GetOverlappedResult(hcom, &overlap, &read, true);
        if (read == 1) {
            if (c == '\n') {
                if (str.back() == '\r') {
                    str.pop_back();
                }
                CloseHandle(overlap.hEvent);
                return str;
            }
            str.push_back(c);
        }
    }
}

std::future<std::string> SerialPort::ReadAsync(uint64_t timeout) {
    return std::async(
        [this](void *hcom, uint64_t time) -> std::string {
            SetCommMask(hcom, EV_RXCHAR);
            OVERLAPPED overlap = {0};
            overlap.hEvent = CreateEvent(nullptr, true, false, nullptr);
            DWORD mask = 0;
            WaitCommEvent(hcom, &mask, &overlap);
            if (WAIT_OBJECT_0 == WaitForSingleObject(overlap.hEvent, time)) {
                DWORD dwError;
                COMSTAT cs;
                memset(&cs, 0, sizeof(COMSTAT));
                if (!ClearCommError(hcom, &dwError, &cs)) {
                    CloseHandle(overlap.hEvent);
                    return std::string();
                }
                if (cs.cbInQue <= 0) {
                    CloseHandle(overlap.hEvent);
                    return std::string();
                }
                std::string ret(cs.cbInQue, 0);
                DWORD read = 0;
                ReadFile(hcom, &ret[0], cs.cbInQue, &read, &overlap);
                GetOverlappedResult(hcom, &overlap, &read, true);
                ret.resize(read);
                CloseHandle(overlap.hEvent);
                return ret;
            }
            CloseHandle(overlap.hEvent);
            return std::string();
        },
        hcom, timeout);
}

void SerialPort::Write(char *buffer, int offset, int count) {
    OVERLAPPED overlap = {0};
    overlap.hEvent = CreateEvent(nullptr, true, false, nullptr);
    DWORD write = 0;
    WriteFile(hcom, buffer + offset, count, &write, &overlap);
    GetOverlappedResult(hcom, &overlap, &write, true);
    CloseHandle(overlap.hEvent);
}

void SerialPort::Write(std::string text) {
    Write((char *)text.data(), 0, text.size());
}
void SerialPort::WriteLine(std::string text, bool hasCR) {
    if (hasCR) text += '\r';
    text += '\n';
    Write(text);
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
            dcb.fOutxCtsFlow = FALSE;
            dcb.fOutX = false;
            dcb.fInX = false;
            break;
        case Handshake::RequestToSend:
            dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
            dcb.fOutxCtsFlow = TRUE;
            dcb.fOutX = false;
            dcb.fInX = false;
            break;
        case Handshake::RequestToSendXOnXOff:
            dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
            dcb.fOutxCtsFlow = TRUE;
            dcb.fOutX = true;
            dcb.fInX = true;
            break;
        case Handshake::XOnXOff:
            dcb.fRtsControl = RTS_CONTROL_DISABLE;
            dcb.fOutxCtsFlow = FALSE;
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

std::string SerialPort::GetPortName() { return this->portName; }
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
        if (dcb.fOutX && dcb.fInX && !dcb.fOutxCtsFlow) {
            handshake = Handshake::XOnXOff;
        } else {
            if (dcb.fOutX || dcb.fInX || dcb.fOutxCtsFlow) {
                return (Handshake)-6;
            } else {
                handshake = Handshake::None;
            }
        }
    } else if (dcb.fRtsControl == RTS_CONTROL_HANDSHAKE) {
        if (dcb.fOutX && dcb.fInX && dcb.fOutxCtsFlow) {
            handshake = Handshake::RequestToSendXOnXOff;
        } else {
            if (dcb.fOutX || dcb.fInX || !dcb.fOutxCtsFlow) {
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