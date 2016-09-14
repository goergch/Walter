/*
 * util.h
 *
 * Various helper functions
 *
 *  Created on: 13.08.2016
 *      Author: JochenAlt
 */

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

#define LFCR '\n'

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

// random helper functions
int randomInt(int min,int max);
rational randomFloat (rational a, rational b);
bool randomBool();
int randomPosNeg();

// time helpers
milliseconds millis();
void delay(long);

// string helper functions
std::string string_format(const std::string &fmt, ...);
string to_string(rational number, int precision);
string int_to_string(int x);
bool string_starts_with(string s, string start);					// true, if s starts with start
void ltrim(std::string &s);											// trim from start (in place)
void rtrim(std::string &s);											// trim from end (in place)
void trim(std::string &s);											// trim from both ends (in place)
std::string replaceWhiteSpace(std::string s);
std::string string_to_hex(const std::string& input);
std::string hex_to_string(const std::string& input);

// math helper functions
#define PI 3.141592653589793238462643383279502884
#define HALF_PI (PI/2.0)

rational hypothenuseLength(rational a, rational b); 				// pythagoras
bool almostEqual(rational a, rational b, rational precision);		// true, if both values differ by  given percentage only
float roundValue(float x);											// round passed value to next floatPrecision number

rational radians(rational degrees);
rational degrees(rational radians);
rational triangleAlpha(rational a, rational b, rational c);			// cosine sentence
rational triangleGamma(rational a, rational b, rational c);			// cosine sentence
bool polynomRoot2ndOrder(rational a, rational b, rational c, rational& root0, rational& root1);

// file helper functions
bool fileExists(const string& filename);
vector<std::string> readDirectory(const string & dir, const string& ext);

// serializing functions
string floatToString(const string& tag, double x);
bool floatFromString (const string& tag, const string& str, double &x, int& idx);
string intToString(const string& tag, int x);
bool intFromString (const string& tag, const string& str, int &x, int& idx);
string uint32ToString(const string& tag, uint32_t x);
bool uint32FromString (const string& tag, const string& str, uint32_t &x, int& idx);

string stringToString(const string& tag, const string& x);
bool stringFromString (const string& tag, const string& str, string &x, int& idx);
string listStartToString(const string& tag, int x);
bool listStartFromString (const string& tag, const string& str, int &x, int& idx);
string listEndToString();
bool listEndFromString (const string& str, int& idx);

#endif
