#include <iostream>
#include "ZSerial.h"
using namespace ZSerial;
int main(int argc, char const *argv[]) {
    SerialPort serial("/dev/ttys000", BaudRate::BR_115200);
    if (serial.Open() != 0) {
        printf("err\n");
        getchar();
        return -1;
    }
    while (true) {
        std::string c;
        std::cin>>c;
        if(c.empty()){
            continue;
        }
        serial.WriteLine("sadfe ae  e ea  fae ");
        break;
    }
    serial.Close();
    return 0;
}
