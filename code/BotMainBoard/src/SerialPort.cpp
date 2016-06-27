/*
 * SerialPort.cpp
 *
 *  Created on: 27.06.2016
 *      Author: SuperJochenAlt
 */

#include "SerialPort.h"
#include "Windows.h"
#include "string.h"


using namespace std;

SerialPort::SerialPort() {
	serialPortHandle = INVALID_HANDLE_VALUE;
}

SerialPort::~SerialPort() {
	if (serialPortHandle!=INVALID_HANDLE_VALUE)
	CloseHandle(serialPortHandle);

	serialPortHandle = INVALID_HANDLE_VALUE;
}

int SerialPort::connect() {
	return connect("COM1", 115200);
}


int SerialPort::connect( string device, int baudRate) {
	int error=0;
	DCB dcb;

	memset(&dcb,0,sizeof(dcb));

	dcb.DCBlength = sizeof(dcb);

	dcb.BaudRate = baudRate;
	dcb.Parity = NOPARITY;
	dcb.fParity = 0;
	dcb.StopBits = ONESTOPBIT;
	dcb.ByteSize = 8;

	LPCTSTR fn = (const char*)device.c_str();
	serialPortHandle =
			CreateFile(fn,
				GENERIC_READ | GENERIC_WRITE,
				0, //(share) 0:cannot share the COM port
				0, //security  (None)
				OPEN_EXISTING, // creation : open_existing
				FILE_FLAG_OVERLAPPED,// we want overlapped operation,
				0 // no templates file for COM port...
				);

	if (serialPortHandle != INVALID_HANDLE_VALUE) {
		if(!SetCommState(serialPortHandle,&dcb))
			error=2;
	}
	else {
		error=1;
	}

	if (error!=0) {
		disconnect();
	}
	else {
		clear();
	}

	return error;
}

void SerialPort::disconnect(void) {
	CloseHandle(serialPortHandle);
	serialPortHandle = INVALID_HANDLE_VALUE;

//printf("Port 1 has been CLOSED and %d is the file descriptionn", fileDescriptor);
}

int SerialPort::sendArray(unsigned char *buffer, int len) {
	unsigned long result;

	if (serialPortHandle!=INVALID_HANDLE_VALUE)
		WriteFile(serialPortHandle, buffer, len, &result, NULL);

	return result;
}

int SerialPort::getArray (unsigned char *buffer, int len) {
	unsigned long read_nbr;

	read_nbr = 0;
	if (serialPortHandle!=INVALID_HANDLE_VALUE)
	{
		ReadFile(serialPortHandle, buffer, len, &read_nbr, NULL);
	}

	return((int) read_nbr);
}

int SerialPort::sendString(string str) {
	int written = sendArray((unsigned char*)str.c_str(), str.length());
	return written;
}

int SerialPort::receive(string& str) {
	char buffer[1024];
	int bytesRead= getArray((unsigned char*)buffer, 1024);
	buffer[bytesRead] = 0;
	str = buffer;
	return bytesRead;

}

void SerialPort::clear() {
	PurgeComm (serialPortHandle, PURGE_RXCLEAR | PURGE_TXCLEAR);
}
