#pragma once
#ifdef _WIN32
#ifndef __WINSERIAL_H
#define __WINSERIAL_H
#include "BaseSerial.h"

namespace ZSerial {
	class WinSerial :
		public BaseSerial
	{
	private:
		bool reading;
		void* hCom;
		bool async;
		static void ReadThread(void*);
		void ReceiveChar();
		ReadProcess read;
	public:
		WinSerial();
		~WinSerial();
		int InitSerial(std::string port,
			BaudRate baudrate, bool async = true,
			unsigned int bytesize = 8,
			Parity parity = Parity::NOPARITY,
			StopBits stopbits = StopBits::ONESTOPBIT); 
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
#endif
#endif