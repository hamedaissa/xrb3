/*
 * XRB3 - Powered by AVRRA - Main
 * Copyright 2004-2008 Michael E. Ferguson
 * 
 * Known Issues:
 *       halt should be interrupt driven!
 *       Using go switch puts us out of date with console!
 *
 * todo: xrseries2.h - set register command
 *                     extend the get register command to other values...
 *       xr1state.h  - init from eeprom, save to eeprom 
 *                     user update hook...
 *       bootloader needs mods to disable watchdog, have 5 sec timeout
 *
 * done: avrcam.h - ET,DT seem to work, pass thru is functional
 *       behavior.h - behavior switch is completely finished. 
 *       clock.h - system clock, wait functions done
 *		 servo.h - all but a finer center adjust is done
 *       default.h - behavior is done (includes leftBump, rightBump)
 *       xr1.h - all hardware is ready, but could using cleaning up....
 */

// USER: add your hardware pin selections here!

#include "../avrra/xrseries/default.h"
// USER: add your behavior files here!
//#include "topo.h"
//#include "cam.h"
//#include "wander.h"
#include "mapbuilder.h"
//#include "topometric.h"

void projectInit(){
    // USER: add your behaviors here!
    addBeh("default",&defaultArbitrate,&defaultPlan,&defaultInit);
    //addBeh("topo",&topoArb,&topoPlan,&topoInit);
    //addBeh("cam",&camArb,&defaultPlan,&camInit);  
	//addBeh("wander",&wanderArb,&defaultPlan,&defaultInit);
    addBeh("mapbuilder",&mapbuilderArb,&defaultPlan,&mapbuilderInit); 
	//addBeh("topmetric",&topometricArb,&defaultPlan,&topometricInit);
}
