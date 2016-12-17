/*
 * SerialPort.h
 *
 * Class for communicating with the uC board via serial connection
 *
 *  Created on: 27.06.2016
 *      Author: JochenAlt
 */

#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include "Windows.h"
#include <string>

using namespace std;

const string newlineStr = "\r";

class SerialPort {
private:
	HANDLE serialPortHandle;

public:
	SerialPort();
	~SerialPort();

	int connect (string device, int baudRate);
	void disconnect(void);

	int sendArray(unsigned char *buffer, int len);
	int getArray (unsigned char *buffer, int len);

	int sendString(string str);
	int receive(string& str);

	void clear();
};

#endif /* SERIALPORT_H_ */
