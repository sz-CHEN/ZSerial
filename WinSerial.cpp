
#ifdef _WIN32
#include "WinSerial.h"
#include <Windows.h>
#include <cstdio>
#include <process.h>
#include "GPSParse.h"
#include <fstream>
using namespace ZSerial;
void WinSerial::ReadThread(void * param)
{
	WinSerial* p = (WinSerial*)param; OVERLAPPED overlapped;
	overlapped.hEvent = CreateEventA(
		NULL,
		TRUE,
		FALSE,
		NULL
	);
	overlapped.Internal = 0;
	overlapped.InternalHigh = 0;
	overlapped.Offset = 0;
	overlapped.OffsetHigh = 0;
	while (p->hCom&&p->reading) {
		DWORD mask = 0;
		WaitCommEvent(p->hCom, &mask, &overlapped);
		DWORD num;
		GetOverlappedResult(p->hCom, &overlapped, &num, true);
		ResetEvent(overlapped.hEvent);
		if (mask&EV_RXCHAR) {
			p->ReceiveChar();
		}
	}
	CloseHandle(overlapped.hEvent);
	return;
}

//GPSParse gpsParse;
void ZSerial::WinSerial::ReceiveChar()
{
	DWORD dwError;
	COMSTAT cs;
	while (true) {
		if (!ClearCommError(hCom, &dwError, &cs))
		{
			printf_s("ClearCommError failed with error %d.\n",GetLastError());
			Sleep(10);
			DestroySerial();
			Sleep(10);
			//InitSerial(7, (BaudRate)CBR_19200);
			return;
		}
		if (cs.cbInQue == 0) {
			break;
		}
		char* buf = new char[cs.cbInQue];
		DWORD num = cs.cbInQue;
		OVERLAPPED osRead;
		memset(&osRead, 0, sizeof(OVERLAPPED));
		osRead.hEvent = CreateEventA(NULL, TRUE, FALSE, NULL);
		ReadFile(hCom, buf, cs.cbInQue, NULL, &osRead);
		//WaitForSingleObject(osRead.hEvent, INFINITE);
		GetOverlappedResult(hCom, &osRead, &num, true);
		CloseHandle(osRead.hEvent);
		if (read != nullptr) {
			//gpsParse.pushBinary(buf, num);
			read(buf, num);
		}
		delete[] buf;
	}
}

WinSerial::WinSerial():reading(false),hCom(nullptr),read(nullptr)
{
}


WinSerial::~WinSerial()
{
	DestroySerial();
}

int WinSerial::InitSerial(std::string port, BaudRate baudrate, bool async, unsigned int bytesize, Parity parity, StopBits stopbits)
{
	this->async = async;
	DCB dcb;
	//char portname[256] = "\\\\.\\COM";
	//char portnum[249];
	//_itoa_s(port, portnum, 10);
	//strcat_s(portname, sizeof(portname), portnum);
	hCom = CreateFileA(("\\\\.\\COM"+port).c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, async?FILE_FLAG_OVERLAPPED:NULL, NULL);
	if (hCom == INVALID_HANDLE_VALUE) {
		printf_s("CreateFile failed with error %d.\n", GetLastError());
		DestroySerial();
		return 1;
	}
	COMMTIMEOUTS timeouts;
	GetCommTimeouts(hCom, &timeouts);
	timeouts.ReadIntervalTimeout = 0;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 1000;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;
	SetCommTimeouts(hCom, &timeouts);
	static const int buffermax = 32768;
	if (!SetupComm(hCom, buffermax, buffermax))
	{
		printf_s("SetupComm %s failed", port.c_str());
		DestroySerial();
		return 5;
	}
	SecureZeroMemory(&dcb, sizeof(DCB));
	dcb.DCBlength = sizeof(DCB);
	if (!GetCommState(hCom, &dcb)) {
		printf_s("GetCommState failed with error %d.\n", GetLastError());
		DestroySerial();
		return 2;
	}
	printf_s("\nBaudRate = %d, ByteSize = %d, Parity = %d, StopBits = %d\n", dcb.BaudRate, dcb.ByteSize, dcb.Parity, dcb.StopBits);
	dcb.BaudRate = baudrate;
	dcb.ByteSize = bytesize;
	dcb.Parity = parity;
	dcb.StopBits = stopbits;
	if (!SetCommState(hCom, &dcb)) {
		printf_s("SetCommState failed with error %d.\n", GetLastError());
		DestroySerial();
		return 3;
	}
	if (!GetCommState(hCom, &dcb)) {
		printf_s("GetCommState failed with error %d.\n", GetLastError());
		DestroySerial();
		return 2;
	}
	printf_s("\nBaudRate = %d, ByteSize = %d, Parity = %d, StopBits = %d\n", dcb.BaudRate, dcb.ByteSize, dcb.Parity, dcb.StopBits);
	printf_s("Serial port %s successfully configured.\n", port.c_str());
	return 0;
}

void ZSerial::WinSerial::DestroySerial()
{
	reading = false;
	SetCommMask(hCom, 0);
	read = nullptr;
	EscapeCommFunction(hCom, CLRDTR);
	ClearBuffer();
	CloseHandle(hCom);
	hCom = NULL;
}

void ZSerial::WinSerial::ClearInputBuffer()
{
	if (!PurgeComm(hCom, PURGE_RXABORT | PURGE_RXCLEAR)) {
		printf_s("PurgeComm failed with error %d.\n", GetLastError());
	}
}

void ZSerial::WinSerial::ClearOutputBuffer()
{
	if (!PurgeComm(hCom, PURGE_TXABORT | PURGE_TXCLEAR)) {
		printf_s("PurgeComm failed with error %d.\n", GetLastError());
	}
}

void ZSerial::WinSerial::ClearBuffer()
{
	if (!PurgeComm(hCom, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR)) {
		printf_s("PurgeComm failed with error %d.\n", GetLastError());
	}
}

void ZSerial::WinSerial::Write(char * buf, unsigned int length)
{
	if (async){
		DWORD num;
		OVERLAPPED os;
		memset(&os, 0, sizeof(OVERLAPPED));
		os.hEvent = CreateEventA(NULL, TRUE, FALSE, NULL);
		WriteFile(hCom, buf, length, NULL, &os);
		GetOverlappedResult(hCom, &os, &num, true);
		CloseHandle(os.hEvent);
	}
	else{
		unsigned long write = 0;
		WriteFile(hCom, buf, length, &write, NULL);
	}
}

void ZSerial::WinSerial::StartRead(ReadProcess func)
{
	ClearBuffer();
	if (!SetCommMask(hCom, EV_RXCHAR)) {
		printf_s("SetCommMask failed with error %d.\n", GetLastError());
		DestroySerial();
		return;
	}
	reading = true;
	read = func;
	_beginthread(ReadThread, 0, this);
}

void ZSerial::WinSerial::EndRead()
{
	reading = false;
	SetCommMask(hCom, 0);
	read = nullptr;
}

unsigned long ZSerial::WinSerial::Read(char* buf, unsigned long len){
	unsigned long read = 0;
	ReadFile(hCom, buf, len, &read,NULL);
	return read;
}
#endif