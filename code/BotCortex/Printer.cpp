/*
 * Printer.cpp
 *
 * Author: SuperJochenAlt
 */

#include <Config.h>
#include <Printer.h>
#include "Adafruit_Thermal.h"
#include "pins.h"
#include "walterlogo.h" // contains bitmap "walterlogo" as created by LCD Assistant

Printer printer;
Adafruit_Thermal printerImpl(printerComm);

Printer::Printer() {
}

void Printer::resetFormat() {
	boldMode = false;
	doubleHeight = false;
	doubleWidth= false;
	underline = false;
}

void Printer::setup() {
	printerComm->begin(PRINTER_BAUD_RATE);
	printerImpl.begin();
	printerImpl.upsideDownOn();

}

void Printer::boldOn() {
	printerImpl.boldOn();
}
void Printer::doubleHeightOn() {
	printerImpl.doubleHeightOn();
}
void Printer::doubleWidthOn() {
	printerImpl.doubleWidthOn();
}

void Printer::println(const char* s) {
	printerImpl.print(s);
	printerImpl.println();
}

void Printer::println() {
	printerImpl.println();
}

void Printer::printWalterLogo() {
	printerImpl.printBitmap(walterLogoWidth,walterLogoHeight, walterlogo, true /* from progmem */);
}

void Printer::print(const char* s) {

	logger->print("print \"");

	int len = strlen(s);
	bool escpapeMode = false;
	char ch, nextCh;

	for (int i = 0;i<len;i++) {
		ch = s[i];
		nextCh = s[i+1];
		bool nextCharValid = (i+1) < len;
		if (!escpapeMode && (ch == '\\') && nextCharValid) {
			switch (nextCh) {
			case 'l':{
				printerImpl.printBitmap(250, 343, walterlogo);
				boldMode = !boldMode;
				i++;
				}
				break;

			case 'b':{
				if (boldMode) {
					logger->print("</b>");
					printerImpl.boldOff();
				}
				else {
					logger->print("<b>");
					printerImpl.boldOn();
				}
				boldMode = !boldMode;
				i++;
				}
				break;
			case 'h':{
				if (doubleHeight) {
					logger->print("</2>");
					printerImpl.doubleHeightOff();
				}
				else {
					logger->print("<2>");
					printerImpl.doubleHeightOn();
				}
				doubleHeight = !doubleHeight;
				i++;
				}
				break;
			case 'w':{
				if (doubleWidth) {
					logger->print("</w>");
					printerImpl.doubleWidthOff();
				}
				else {
					logger->print("<w>");
					printerImpl.doubleWidthOn();
				}
				doubleWidth= !doubleWidth;
				i++;
				}
				break;
			case 'u':{
				if (underline) {
					logger->print("</u>");
					printerImpl.underlineOff();
				}
				else {
					logger->print("<u>");
					printerImpl.underlineOn();
				}
				underline= !underline;
				i++;
				}
				break;

			default:
				break;
			}
		} else {
			logger->print(ch);
			printerImpl.print(ch);
		}
	}
	logger->println("\"");
}

