/******************************************************************************
 * AVRRA: The AVR Robotics API
 * behavior.h - implements the behavior switch for XR-Series Control
 * 
 * Copyright (c) 2008, Michael E. Ferguson
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *   this list of conditions and the following disclaimer in the documentation 
 *   and/or other materials provided with the distribution.
 * - Neither the name of AVRRA nor the names of its contributors 
 *   may be used to endorse or promote products derived from this software 
 *   without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
 
#include <stdlib.h>

/** Various essentials for behavior switch. */
typedef struct{
	char * name;				// string name (we put this to console)
	int (*arbitrate)(void);	// arbitrate function
	int (*plan)(void);			// plan function
	int (*init)(void);			// init function
} _behavior;

_behavior * behlist;			// the behavior array
int behcount;					// the number of behaviors
int curbeh;						// current behavior

/** Function to add behaviors to the behavior list */
void addBeh(char * bname, int (*barbitrate)(void), int (*bplan)(void),
            int (*binit)(void)){
    _behavior * blisttmp = behlist;
    int i;
    behcount = behcount + 1;
    behlist = (_behavior *) malloc (sizeof(_behavior) * behcount);
    // copy over old stuff
    for(i = 0; i + 1 < behcount; i++){
        behlist[i].name=        blisttmp[i].name;
        behlist[i].arbitrate=   blisttmp[i].arbitrate;
        behlist[i].plan=        blisttmp[i].plan;
        behlist[i].init=        blisttmp[i].init;
    }
    behlist[behcount-1].name = bname;
    behlist[behcount-1].arbitrate = barbitrate;
    behlist[behcount-1].plan = bplan;
    behlist[behcount-1].init = binit;
    free(blisttmp);
}

/** Function that sets up the behavior array. */
void switchInit(){
	curbeh = 0;
    behcount = 0;
	projectInit();   
}

void sendBehList(){
	int i;
	delayms(200);
	Print("?Uploading\n");
	delayms(200);
	for(i=0; i < behcount; i++){
		Print("#def");
		if(i == curbeh){
			PrintNumber(100+i);
		}else{
			PrintNumber(i);
		}
		Print(behlist[i].name);
		serialWrite('\n');
		delayms(200);
	}
}

void behSwitch(int beh){
	if(beh < behcount){
		if(beh > -1){
			curbeh = beh;
			behlist[curbeh].init();
		}else{
			curbeh = -1;
		}
		motorStop();
		//return ACK();
	}
	//sysReading(beh);
	//return NCK();
}
