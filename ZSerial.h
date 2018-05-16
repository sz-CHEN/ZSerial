#ifndef __BASE_SERIAL_H
#define __BASE_SERIAL_H
#include <string>
#include <vector>
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
class SerialPort {
   public:
    void* hcom;
    std::string portName;
    BaudRate baudrate;
    Parity parity;
    StopBits stopbits;
    DataBits databits;
    Handshake handshake;
    SerialPort(std::string portName, BaudRate baudrate,
               Parity parity = Parity::None, DataBits databits = DataBits::DB_8,
               StopBits stopbits = StopBits::One);
    ~SerialPort();
    void Close();
    void DiscardInBuffer();
    void DiscardOutBuffer();
    static std::vector<std::string> GetPortNames();
    int Open();
    int Read(char* buffer, int offset, int count);
    char ReadByte();
    std::string ReadExisting();
    // std::string ReadLine();
    void Write(char* buffer, int offset, int count);
    void Write(std::string text);
    void WriteLine(std::string text);
};
}  // nameSpace ZSerial
#endif