#include "Util.h"
#include <stdlib.h>

long mapLong(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int randomInt(int min,int max) {
	int r = rand()>>16;
	int result = mapLong(r,0,(RAND_MAX>>16)-1,min,max+1);
	return result;
}

float randomFloat (float a, float b) {
	return randomInt(a*1000, b*1000)/1000.;
}

bool randomBool() {
	return randomInt(0,100)>50;
}

int randomPosNeg() {
	return (randomInt(0,100)>50)?+1:-1;
}
