
#ifndef UTIL_H_
#define UTIL_H_

#include <stdio.h>
#include "math.h"
#include <string>
#include <iostream>
#include <sstream>

#include "setup.h"
#define _ELPP_THREAD_SAFE
#define ELPP_THREAD_SAFE
#define ELPP_DEFAULT_LOG_FILE "logs/Snorre.log"
#include "easylogging++.h"

using namespace std;


float roundValue(float x);

template<class T>
const T& constrain(const T& x, const T& a, const T& b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

template<class T>
T sgn(const T& a) {
    if(a < 0.0) {
        return -1.0;
    }
    else if(a > 0.0) {
        return +1.0;
    }
    else
        return 0;
}

template<class T>
T sqr(const T& x) {
    return x*x;
}


std::string string_format(const std::string &fmt, ...);
int randomInt(int min,int max);
rational randomFloat (rational a, rational b);
bool randomBool();
int randomPosNeg();
unsigned long millis();
void delay(long);
string to_string(rational number, int precision);

// pythagoras
static inline rational hypothenuseLength(rational a, rational b) {
    return sqrt(a*a+b*b);
}

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
#define HALF_PI (PI/2.0d)

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


static inline rational radians(rational degrees) {
	const rational fac = (PI/ 180.0);
	return degrees * fac;
}

static inline rational  degrees(rational radians) {
	const rational fac = (180.0/ PI);
	return radians * fac;
}

// cosine sentence
static inline rational triangleAlpha(rational a, rational b, rational c) {
	rational x = acos((a*a-b*b-c*c)/(-2*b*c));
    return x;
}


// cosine sentence
static inline rational triangleGamma(rational a, rational b, rational c) {
	return triangleAlpha(c,b,a);
}

vector<std::string> readDirectory(const string & dir, const string& ext);

#endif
