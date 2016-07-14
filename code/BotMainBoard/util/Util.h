
#ifndef UTIL_H_
#define UTIL_H_

#include <stdio.h>
#include "math.h"
#include <string>
#include <iostream>
#include <sstream>

int randomInt(int min,int max);
float randomFloat (float a, float b);
bool randomBool();
int randomPosNeg();


long millis();
void delay(long);

#define ITOS( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

#endif
