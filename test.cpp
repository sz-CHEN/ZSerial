#include <iostream>
#include "ZSerial.h"
#include <cstdio>
using namespace ZSerial;
int main(int argc, char const *argv[]) {
    auto a=SerialPort::GetPortNamesAndDescriptions();
    for(auto& r:a){
        std::cout<<r.first<<std::endl;
    }
    // return 0;
    SerialPort serial(a[0].first, (int)BaudRate::BR_115200);
    if (serial.Open() != 0) {
        printf("err\n");
        printf(strerror(errno));
        getchar();
        return -1;
    }
    std::cout<<"aaa"<<std::endl;
    serial.DiscardInBuffer();
    serial.DiscardOutBuffer();
    SerialPort serial1("/dev/ptyp0", (int)BaudRate::BR_115200);
    if (serial1.Open() != 0) {
        printf("err\n");
        printf(strerror(errno));
        getchar();
        return -1;
    }
    serial1.DiscardInBuffer();
    serial1.DiscardOutBuffer();
    while (true) {
        std::string str;
        std::cin>>str;
        serial1.WriteLine(str);
        std::cout<<serial.ReadLine()<<std::endl;
    }
    serial.Close();
    serial1.Close();
    return 0;
}
