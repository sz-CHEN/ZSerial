// Created by 陈世增 on 2017/5/24.
//
#if defined(__APPLE__)||defined(__linux__)||defined(__CYGWIN__)
#ifndef GPSRECEIVER_LINUXSERIAL_H
#define GPSRECEIVER_LINUXSERIAL_H
#include "BaseSerial.h"
namespace ZSerial {
    class LinuxSerial : public ZSerial::BaseSerial {
typedef int HCOM;
    private:
        bool reading;
        HCOM hcom;
        static void ReadThread(void*);
        ReadProcess read;
    public:
        LinuxSerial();
        ~LinuxSerial();
        int InitSerial(std::string port,
			ZSerial::BaudRate baudrate,bool async=true,
                       unsigned int bytesize = 8,
                       ZSerial::Parity parity = Parity::NOPARITY,
                       ZSerial::StopBits stopbits = StopBits::ONESTOPBIT);
        void DestroySerial();
        void ClearInputBuffer();
        void ClearOutputBuffer();
        void ClearBuffer();
        void Write(char* buf, unsigned int length);
        void StartRead(ReadProcess);
		void EndRead();
		unsigned long Read(char* buf, unsigned long len);
    };
}

#endif //GPSRECEIVER_LINUXSERIAL_H
#endif