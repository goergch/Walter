
#ifndef UTIL_H_
#define UTIL_H_

#include <stdio.h>
#include "math.h"
#include <string>
#include <iostream>
#include <sstream>
#include "easylogging++.h"

using namespace std;
class Logger {
public:
	Logger(std::string pName) {
		methodName = pName;
		resultIsSet = false;
	    LOG(DEBUG) << "entering " << pName;
	}
	~Logger() {
	    LOG(DEBUG) << "leaving " << methodName << (resultIsSet?("->" + result):"");
	}

	void setResult(string s) {
		result = s;
		resultIsSet = true;
	}
	void setResult(bool s) {
		if (s)
			result = "true";
		else
			result = "false";
		resultIsSet = true;
	}

private:
	string result;
	bool resultIsSet;
	string methodName;
};

int randomInt(int min,int max);
float randomFloat (float a, float b);
bool randomBool();
int randomPosNeg();


long millis();
void delay(long);

#define ITOS( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

#endif
