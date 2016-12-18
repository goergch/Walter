
#include <Arduino.h>
void watchdogReset() {
	noInterrupts();
	WDOG_REFRESH = 0xA602;
	WDOG_REFRESH = 0xB480;
	interrupts();
}

/* comment out original ResetHandler in C:\Program Files\Arduino\hardware\teensy\cores\teensy3\mk20dx128.c */

void ResetHandler() {
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
	WDOG_TOVALL = (30000); // The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to.
	WDOG_TOVALH = 0;
	WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN |
	WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG

	// WDOG_PRESC = 0; // This sets prescale clock so that the watchdog timer ticks at 1kHZ instead of the default 1kHZ/4 = 200 HZ
	startup_early_hook();
}

void setWatchdogTimeout(int milliseconds) {
	WDOG_TOVALL = (milliseconds/5); // watchdog checks with 200Hz
	WDOG_TOVALH = 0;
}
