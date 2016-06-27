/*
 * SerialPort.h
 *
 *  Created on: 27.06.2016
 *      Author: SuperJochenAlt
 */

#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include "Windows.h"
#include <string>

using namespace std;

class SerialPort {
private:
	HANDLE serialPortHandle;

public:
	SerialPort();
	~SerialPort();

	int connect ();
	int connect (string device, int baudRate);
	void disconnect(void);

	int sendArray(unsigned char *buffer, int len);
	int getArray (unsigned char *buffer, int len);

	int sendString(string str);
	int receive(string& str);

	void clear();
	};

#endif /* SERIALPORT_H_ */
