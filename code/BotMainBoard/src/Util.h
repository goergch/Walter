
#ifndef UTIL_H_
#define UTIL_H_

#include <stdio.h>
#include "math.h"
#include <string>
#include <iostream>
#include <sstream>
#include "easylogging++.h"

using namespace std;


typedef  double rational;

int randomInt(int min,int max);
rational randomFloat (rational a, rational b);
bool randomBool();
int randomPosNeg();
unsigned long millis();
void delay(long);
string to_string(rational number, int precision);

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


#define PI 3.141592653589793238462643383279502884
#define HALF_PI (PI/2.0)

// true, if both values differ by  given percentage only
static inline bool almostEqual(rational a, rational b, rational precision) {
	if (a==b)
		return true;
	if (a == 0)
		return (abs(b)<precision);
	if (b == 0)
		return (abs(a)<precision);

	if (b<a)
		return (abs((b/a)-1.0) < precision);
	else
		return (abs((a/b)-1.0) < precision);

}

// true if difference is smaller than 0.01 %
static inline bool almostEqual(rational a, rational b) {
	return almostEqual(a,b,0.0001);
}

static inline rational radians(rational degrees) {
	return degrees * (PI/ 180.0) ;
}


static inline rational  degrees(rational radians) {
	return radians * (180.0 / PI) ;
}


struct ActuatorStateType {
	float currentAngle;
	float minAngle;
	float maxAngle;
	float nullAngle;
};

#endif
