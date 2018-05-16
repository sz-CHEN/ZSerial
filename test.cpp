#include <iostream>
#include "ZSerial.h"
#include <Windows.h>
#include <cstdio>
using namespace ZSerial;
int main(int argc, char const *argv[]) {
    auto a=SerialPort::GetPortNames();
    SerialPort serial(a[2], BaudRate::BR_115200);
    if (serial.Open() != 0) {
        printf_s("err\n");
        getchar();
        return -1;
    }
    SerialPort serial1(a[1], BaudRate::BR_115200);
    if (serial1.Open() != 0) {
        printf_s("err\n");
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
