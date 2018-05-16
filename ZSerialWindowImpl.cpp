#ifdef __WIN32__
namespace ZSerial {
SerialPort::SerialPort(std::string portName, BaudRate baudrate, Parity parity,
                       DataBits databits, StopBits stopbits)
    : portName(portName),
      baudrate(baudrate),
      parity(parity),
      databits(databits),
      stopbits(stopbits),
      handshake(Handshake::None),
      hcom(0) {}
void SerialPort::Close() {}
void SerialPort::DiscardInBuffer() {}
void SerialPort::DiscardOutBuffer() {}
std::vector<std::string> SerialPort::GetPortNames() {}
int SerialPort::Open() {}
int SerialPort::Read(char* buffer, int offset, int count) {}
char SerialPort::ReadByte() {}
std::string SerialPort::ReadExisting() {}
void SerialPort::Write(char* buffer, int offset, int count) {}
void SerialPort::Write(std::string text) {}
void SerialPort::WriteLine(std::string text) {}
}  // namespace ZSerial
#endif