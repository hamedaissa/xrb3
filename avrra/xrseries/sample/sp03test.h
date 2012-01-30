/* I2C speech demo */

#include "avrra/sp03.h"

int speechInit(){
	sp03Init();
    sysMsg("SP03 Ver:");
    sysReading(sp03GetVer());
    // say hello
    sp03Speak(1);
	return 0;
}

/* Do nothing */
int speechArb(){
	delayms(100);
	return 0;
}
