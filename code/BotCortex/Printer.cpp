/*
 * Printer.cpp
 *
 * Author: SuperJochenAlt
 */

#include <Printer.h>
#include "Adafruit_Thermal.h"
#include "pins.h"
#include "config.h"

#include "walterlogo.h"

Printer printer;
Adafruit_Thermal printerImpl(printerComm);

Printer::Printer() {
}

void Printer::setup() {
	printerComm->begin(PRINTER_BAUD_RATE);
	printerImpl.begin();
	printerImpl.upsideDownOn();
}

void Printer::println(const char* s) {
	printerImpl.print(s);
	printerImpl.println();
}

void Printer::print(const char* s) {

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
				if (boldMode)
					printerImpl.boldOff();
				else
					printerImpl.boldOn();
				boldMode = !boldMode;
				i++;
				}
				break;

			case 'b':{
				if (boldMode)
					printerImpl.boldOff();
				else
					printerImpl.boldOn();
				boldMode = !boldMode;
				i++;
				}
				break;
			case 'h':{
				if (doubleHeight)
					printerImpl.doubleHeightOff();
				else
					printerImpl.doubleHeightOn();
				doubleHeight = !doubleHeight;
				i++;
				}
				break;
			case 'w':{
				if (doubleWidth)
					printerImpl.doubleWidthOff();
				else
					printerImpl.doubleWidthOn();
				doubleWidth= !doubleWidth;
				i++;
				}
				break;
			case 'u':{
				if (underline)
					printerImpl.underlineOn();
				else
					printerImpl.underlineOff();
				underline= !underline;
				i++;
				}
				break;

			default:
				break;
			}
		} else
			printerImpl.print(ch);
	}
}

