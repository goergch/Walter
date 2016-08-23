/*
 * util.h
 *
 * Various general helper functions
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

unsigned long millis();
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

// math helper functions
#define PI 3.141592653589793238462643383279502884
#define HALF_PI (PI/2.0d)

rational hypothenuseLength(rational a, rational b); 				// pythagoras
bool almostEqual(rational a, rational b, rational precision);		// true, if both values differ by  given percentage only
float roundValue(float x);											// round passed value to next floatPrecision number

rational radians(rational degrees);
rational  degrees(rational radians);
rational triangleAlpha(rational a, rational b, rational c);			// cosine sentence
rational triangleGamma(rational a, rational b, rational c);			// cosine sentence

// file helper functions
bool fileExists(const string& filename);
vector<std::string> readDirectory(const string & dir, const string& ext);

#endif
