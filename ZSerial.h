#ifndef __BASE_SERIAL_H
#define __BASE_SERIAL_H
#include <string>
#include <vector>
#ifdef _WIN32
#define DLL_EXPORT _declspec(dllexport)
#else
#define DLL_EXPORT
#endif
namespace ZSerial {
enum class BaudRate : int {
    BR_110 = 110,
    BR_300 = 300,
    BR_600 = 600,
    BR_1200 = 1200,
    BR_2400 = 2400,
    BR_4800 = 4800,
    BR_9600 = 9600,
    BR_14400 = 14400,
    BR_19200 = 19200,
    BR_38400 = 38400,
    BR_56000 = 56000,
    BR_57600 = 57600,
    BR_115200 = 115200,
    BR_128000 = 128000,
    BR_256000 = 256000
};

enum class Parity : int { None = 0, Odd, Even, Mark, Space };

enum class StopBits : int { None = -1, One, OnePointFive, Two };

enum class DataBits : int { DB_5 = 5, DB_6, DB_7, DB_8 };

enum class Handshake : int {
    None = 0,
    RequestToSend,
    RequestToSendXOnXOff,
    XOnXOff
};
class DLL_EXPORT SerialPort {
   private:
    bool opened;
    void* hcom;
    std::string portName;
    int baudrate;
    Parity parity;
    StopBits stopbits;
    DataBits databits;
    Handshake handshake;

   public:
    SerialPort(std::string portName, int baudrate, Parity parity = Parity::None,
               DataBits databits = DataBits::DB_8,
               StopBits stopbits = StopBits::One,
               Handshake handshake = Handshake::None);
    ~SerialPort();
    void Close();
    bool IsOpen();
    void DiscardInBuffer();
    void DiscardOutBuffer();
    static std::vector<std::string> GetPortNames();
    static std::vector<std::pair<std::string, std::string>>
    GetPortNamesAndDescriptions();
    int Open();
    int Read(char* buffer, int offset, int count);
    char ReadByte();
    std::string ReadExisting();
    std::string ReadLine();
    void Write(char* buffer, int offset, int count);
    void Write(std::string text);
#ifdef _WIN32
    void WriteLine(std::string text, bool hasCR = true);
#else
    void WriteLine(std::string text, bool hasCR = false);
#endif
    int SetPortName(std::string port);
    int SetBaudRate(int baudrate);
    int SetParity(Parity parity);
    int SetDataBits(DataBits databits);
    int SetStopBits(StopBits stopbits);
    int SetHandshake(Handshake handshake);
    std::string GetPortName();
    int GetBaudRate();
    Parity GetParity();
    DataBits GetDataBits();
    StopBits GetStopBits();
    Handshake GetHandshake();
};
}  // nameSpace ZSerial
#endif