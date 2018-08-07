#include <iostream>
#include "ZSerial.h"
#include <cstdio>
using namespace ZSerial;
int main(int argc, char const *argv[]) {
    // auto a=SerialPort::GetPortNames();
    SerialPort serial("/dev/ttys000", BaudRate::BR_115200);
    if (serial.Open() != 0) {
        printf("err\n");
        getchar();
        return -1;
    }
    SerialPort serial1("/dev/ttys003", BaudRate::BR_115200);
    if (serial1.Open() != 0) {
        printf("err\n");
        getchar();
        return -1;
    }
    while (true) {
        std::string str;
        std::cin>>str;
        serial1.Write(str);
        std::cout<<serial.ReadExisting()<<std::endl;
    }
    serial.Close();
    serial1.Close();
    return 0;
}
