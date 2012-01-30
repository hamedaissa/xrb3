/* Sample AVRRA file to turn on an LED when a switch is pushed */

#include "../avrra/library/mini.h"

#define PIN_SWITCH      0x10    // Connect a switch from PC0 to ground
#define PIN_LED         0x11    // Connect your LED from PC1 to ground,
                                //    make sure the flat side of the LED is 
                                //    connected to ground

int main(){
    /* setup our LED as an output, AVR defaults to low to start (off) */
    digitalSetDirection(PIN_LED, AVRRA_OUTPUT);
    /* setup our switch, as an input,
         but also turn on pull up resistor */
    digitalSetDirection(PIN_SWITCH, AVRRA_INPUT);
    digitalSetData(PIN_SWITCH,AVRRA_PULLUP);
    
    /* main loop, do this forever and ever */
    while(1){
        if(digitalGetData(PIN_SWITCH) == AVRRA_LOW){
            /* when the switch is pressed, we will turn on the led*/
            digitalSetData(PIN_LED, AVRRA_HIGH);
        }else{
            digitalSetData(PIN_LED, AVRRA_LOW);
        }
        // delay a few milliseconds
        delayms(15);
    }
}
