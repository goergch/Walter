/*
 * Printer.h
 *
 * Interface to access Adafruits Nano printer
 *
 * Author: JochenAlt
 */

#ifndef PRINTER_H_
#define PRINTER_H_

#include <Arduino.h>

using namespace std;

class Printer {
public:
	Printer();
	void setup();
	void print(const char* s);
	void println(const char* s);
	void println();
	void printWalterLogo();


private:
	bool boldMode = false;
	bool doubleHeight = false;
	bool doubleWidth= false;
	bool underline = false;
};

extern Printer printer;
#endif /* PRINTER_H_ */
