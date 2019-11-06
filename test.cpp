#include <iostream>
#include "ZSerial.h"

using namespace ZSerial;
int main() {
    SerialPort ser("COM1", 115200);
    if (0 == ser.Open()) {
        while (true) {
            auto str = ser.ReadAsync(1000).get();
            if (str.empty()) {
                ser.WriteLine("empty");
            } else {
                ser.WriteLine(str);
            }
        }
    }
    return 0;
}