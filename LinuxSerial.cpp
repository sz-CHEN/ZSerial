#if defined(__APPLE__) || defined(__linux__)||defined(__CYGWIN__)
#include <string>
#include <fcntl.h>
#include <sys/termios.h>
#include <sys/errno.h>
//#include <zconf.h>
#include "LinuxSerial.h"
#include <thread>
#include <unistd.h>
#include <cstring>

ZSerial::LinuxSerial::LinuxSerial() {

}

ZSerial::LinuxSerial::~LinuxSerial() {

}

int ZSerial::LinuxSerial::InitSerial(std::string port, ZSerial::BaudRate baudrate,bool async, unsigned int bytesize,
                                     ZSerial::Parity parity, ZSerial::StopBits stopbits) {
   // std::string strport="/dev/ttys002";//+std::to_string(port);
    hcom=open(("/dev/"+port).c_str(), O_RDWR|O_NOCTTY|(async?O_NDELAY|O_SYNC:0));
    if(hcom<0){
        printf("open %s error\n",port.c_str());
        return 1;
    }
    struct termios tio;
    memset (&tio, 0, sizeof(tio));
    if(tcgetattr(hcom,&tio)!=0){
        printf("tcgetattr %s error\n",port.c_str());
        return 2;
    }
    //printf("\nBaudRate = %d, ByteSize = %d, Parity = %d, StopBits = %d\n", oldtio., oldtio.c_cflag, oldtio.c_iflag&INPCK, dcb.StopBits);
    //memset(&tio,0, sizeof(tio));
    cfsetospeed (&tio, baudrate);
    cfsetispeed (&tio, baudrate);
    switch (bytesize) {
        case 8:
            tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
            break;
        case 7:
            tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS7;
            break;
        case 6:
            tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS6;
            break;
        case 5:
            tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS5;
            break;
        default:
            printf("\nbytesize error\n");
            return 3;
    }

    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tio.c_iflag &= ~IGNBRK;         // disable break processing
    tio.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tio.c_oflag = 0;                // no remapping, no delays
    tio.c_cc[VMIN]  = async?0:1;            // read doesn't block
    tio.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

    tio.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tio.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tio.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    switch (parity){
        case Parity ::NOPARITY:
            tio.c_cflag|=0;
            break;
        case Parity ::ODDPARITY:
            tio.c_cflag|=(PARENB|PARODD);
            break;
        case Parity ::EVENPARITY:
            tio.c_cflag|=PARENB;
            break;
#ifdef CMSPAR
        case Parity ::MARKPARITY:
            tio.c_cflag|=(PARENB|PARODD|CMSPAR);
            break;
        case Parity ::SPACEPARITY:
            tio.c_cflag|=(PARENB|CMSPAR);
            break;
#endif
        default:
            printf("parity error\n");
            return 4;
    }
    switch (stopbits){
        case StopBits ::ONESTOPBIT:
            tio.c_cflag&=~CSTOPB;
            break;
        case StopBits ::TWOSTOPBITS:
            tio.c_cflag|=CSTOPB;
            break;
        default:
            printf("stopbit error\n");
            return 5;
    }
    tio.c_cflag &= ~CRTSCTS;

    if (tcsetattr (hcom, TCSANOW, &tio) != 0)
    {
        printf ("error %d from tcsetattr", errno);
        return 6;
    }
    return 0;
}

void ZSerial::LinuxSerial::DestroySerial() {
    reading = false;
    read = nullptr;
    ClearBuffer();
    close(hcom);
    hcom=0;
}

void ZSerial::LinuxSerial::ClearInputBuffer() {
    tcflush(hcom,TCIFLUSH);
}

void ZSerial::LinuxSerial::ClearOutputBuffer() {
    tcflush(hcom,TCOFLUSH);
}

void ZSerial::LinuxSerial::ClearBuffer() {
    tcflush(hcom,TCIOFLUSH);
}

void ZSerial::LinuxSerial::Write(char *buf, unsigned int length) {
    write(hcom, buf, length);
}

void ZSerial::LinuxSerial::StartRead(ZSerial::ReadProcess func) {
    reading=true;
    read=func;
    std::thread t(ReadThread,this);
    t.detach();
}

void ZSerial::LinuxSerial::EndRead() {
    reading = false;
    read = nullptr;
}

void ZSerial::LinuxSerial::ReadThread(void * param) {

    LinuxSerial* p = (LinuxSerial*)param;
    while (p->hcom&&p->reading) {
        char buf[255];
        memset(buf,0,sizeof(buf));
        ssize_t n = ::read(p->hcom, buf, sizeof(buf));
        p->read(buf, n);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return;
}

unsigned long ZSerial::LinuxSerial::Read(char* buf, unsigned long len){
   return ::read(hcom, buf, len);
}

#endif // ˜