/*
 * XR Series Control 
 * Copyright 2004-2008 Michael E Ferguson
 */

#define XR_SERIES

// Build Options
//#define 	SERIAL_RATE	19200
#define 	SERIAL_RATE	115200
#define 	TGT_HAS_HEAD
#define 	TGT_HAS_CAMERA
#define 	TGT_HAS_EYES
//#define	NO_ACK				// uncomment to suppress ACK, NCK

// project file stuff
#define GO_BEH	2				// index of the behavior that GO switch starts
void projectInit();
	
// Device files
#include "../avrra/library/dev/pro.h"		
//#include "../avrra/library/dev/a4.h"

// System files		
#include "../avrra/xrseries/xrseries2.h"	

#include "project.h"		

int main()
{		
	xrinit();
	xrloop();
	return 0;
}
