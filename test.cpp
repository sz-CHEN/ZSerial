#include <cstdio>
#include <iostream>
#include "ZSerial.h"
using namespace ZSerial;
int main(int argc, char const* argv[]) {
    auto a = SerialPort::GetPortNamesAndDescriptions();
    for (auto& r : a) {
        std::cout << r.first <<"\t"<< std::endl;
    }
    // return 0;
    SerialPort serial("/dev/ttyp1", (int)BaudRate::BR_115200);
    if (serial.Open() != 0) {
        printf("err\n");
        printf(strerror(errno));
        getchar();
        return -1;
    }
    std::cout << "aaa" << std::endl;
    serial.DiscardInBuffer();
    serial.DiscardOutBuffer();
    // {
    //         auto br = serial.GetBaudRate();
    // int p = serial.SetBaudRate((int)BaudRate::BR_9600);
    // p = serial.SetStopBits(StopBits::Two);
    // p = serial.SetDataBits(DataBits::DB_5);
    // p = serial.SetParity(Parity::Even);
    // p=serial.SetHandshake(Handshake::RequestToSend);
    // br = serial.GetBaudRate();
    // auto parity=serial.GetParity();
    // auto stopb=serial.GetStopBits();
    // auto databit=serial.GetDataBits();
    // auto hans=serial.GetHandshake();
    // int aa=0;
    // }
    // SerialPort serial1("/dev/ptyp0", (int)BaudRate::BR_115200);
    // if (serial1.Open() != 0) {
    //     printf("err\n");
    //     printf(strerror(errno));
    //     getchar();
    //     return -1;
    // }
    // serial1.DiscardInBuffer();
    // serial1.DiscardOutBuffer();
    // auto br = serial1.GetBaudRate();
    // int p = serial.SetBaudRate((int)BaudRate::BR_9600);
    // p = serial.SetStopBits(StopBits::Two);
    // p = serial.SetDataBits(DataBits::DB_5);
    // p = serial.SetParity(Parity::Even);
    // p=serial.SetHandshake(Handshake::RequestToSend);
    // br = serial1.GetBaudRate();
    // auto parity=serial1.GetParity();
    // auto stopb=serial1.GetStopBits();
    // auto databit=serial1.GetDataBits();
    // auto hans=serial1.GetHandshake();
    while (true) {
        std::string str;
        std::cin >> str;
        serial.Write(str);
        // std::cout << serial.ReadExisting();// << std::endl;
    }
    serial.Close();
    // serial1.Close();
    return 0;
}
