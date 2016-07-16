
#include "Util.h"
#include <stdlib.h>
#include <chrono>
#include <unistd.h>
#include <thread>
#include <iomanip>



long mapLong(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int randomInt(int min,int max) {
	int r = rand() % (max-min) + min;
	return r;
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

unsigned long millis() {
    auto epoch = std::chrono::high_resolution_clock::from_time_t(0);
    auto now   = std::chrono::high_resolution_clock::now();
    auto mseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - epoch).count();
    return mseconds;
}


void delay(long ms) {
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

string to_string(float number, int precision) {
	return
			static_cast< std::ostringstream & >(
					(std::ostringstream() << std::setprecision(precision) <<  number)
			).str();
}
