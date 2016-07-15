
#ifndef UTIL_H_
#define UTIL_H_

#include <stdio.h>
#include "math.h"
#include <string>
#include <iostream>
#include <sstream>
#include "easylogging++.h"

using namespace std;


int randomInt(int min,int max);
float randomFloat (float a, float b);
bool randomBool();
int randomPosNeg();
long millis();
void delay(long);
string to_string(float number, int precision);

/* MINGW does not support std::to_string
#define ITOS( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()
*/

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            std::not1(std::ptr_fun<int, int>(std::isspace))));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
            std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}

static inline std::string replaceWhiteSpace(std::string s) {
	replace (s.begin(), s.end(), '\r' , 'R');
	replace (s.begin(), s.end(), '\n' , 'N');
	return s;
}

struct ActuatorStateType {
	float currentAngle;
	float minAngle;
	float maxAngle;
	float nullAngle;
};

#endif
