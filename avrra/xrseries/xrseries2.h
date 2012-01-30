/******************************************************************************
 * AVRRA: The AVR Robotics API
 * xrseries2.h- 2nd generation XR Series Parser, Init, and Loop.
 * 
 * Copyright (c) 2004-2008, Michael E. Ferguson
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

// behavior switch methods and data
#include "behavior.h"
#include <stdlib.h>
#include <string.h>
  
#define MAX_VALUE_LENGTH    4       // sign plus 3 digits
#define MAX_VALUE_COUNT     2

// Suppresses an ACK on successful command (used by GET)
#define COMMAND_NO_ACK		2

typedef enum{
    getRegisterCmd,
	setRegisterCmd,
	writeCmd,
    beginBehCmd,
    goCmd,
    haltCmd,
    queryCmd,
    readClockCmd,
    //resetCmd,
    // AVRCam passthru
	camDumpRawCmd,
	camDumpSegCmd,
	camSetColorMapCmd,
    camPingCmd,
    camEnableTrackingCmd,
    camDisableTrackingCmd,
    // default
    invalidCmd
} xrCmd_t;

// All commands are in the form CM VAL VAL VAL VAL VAL ... 
char p_buffer[MAX_VALUE_LENGTH + 1];
unsigned char bufferIndex = 0;      // index for buffer

int p_values[MAX_VALUE_COUNT];
unsigned char valCount = 0;         // index for p_values
unsigned char procCount = 0;        // number of times buffer was processed.

xrCmd_t receivedCmd = invalidCmd;
unsigned char receivedReg = 0;

// Clear buffers on startup...
void parseInit(){
    memset(p_buffer,0x00,MAX_VALUE_LENGTH + 1);
    memset(p_values,0x00,MAX_VALUE_COUNT);
}

// Converts the current buffer to usable data
static void convertToValue(void)
{
	int newValue;
	newValue = atoi(p_buffer);
	p_values[valCount] = newValue;
	memset(p_buffer,0x00,MAX_VALUE_LENGTH);
    bufferIndex = 0;
}

// Converts the current buffer to a register index
static void convertToReg(void)
{
	if ( (p_buffer[0] == 'S') &&
		 (p_buffer[1] == 'C') &&
         (p_buffer[2] == 'P') ) receivedReg = PANC;
    else if ( (p_buffer[0] == 'S') &&
		      (p_buffer[1] == 'C') &&
              (p_buffer[2] == 'T') ) receivedReg = TLTC;
    else if ( (p_buffer[0] == 'S') &&
		      (p_buffer[1] == 'P') ) receivedReg = PAN;
    else if ( (p_buffer[0] == 'S') &&
              (p_buffer[1] == 'T') ) receivedReg = TILT;
    else if ( (p_buffer[0] == 'I') &&
		      (p_buffer[1] == 'L') ) receivedReg = IR_LE;
    else if ( (p_buffer[0] == 'I') &&
		      (p_buffer[1] == 'R') ) receivedReg = IR_RI;
    else if ( (p_buffer[0] == 'I') &&
		      (p_buffer[1] == 'H') ) receivedReg = IR_HD;
    else if ( (p_buffer[0] == 'R') &&
		      (p_buffer[1] == 'H') ) receivedReg = SNR_HD;
	else
		/* don't recognize the cmd */
		receivedCmd = invalidCmd;
        
	memset(p_buffer,0x00,MAX_VALUE_LENGTH);
	bufferIndex = 0;
}

// Converts the current buffer to a command
void convertToCmd(void){
    if ( (p_buffer[0] == 'G') &&
		 (p_buffer[1] == 'T') ) receivedCmd = getRegisterCmd;
    else if ( (p_buffer[0] == 'G') &&
              (p_buffer[1] == 'E') &&
		      (p_buffer[2] == 'T') ) receivedCmd = getRegisterCmd;
    else if ( (p_buffer[0] == 'S') &&
		      (p_buffer[1] == 'T') ) receivedCmd = setRegisterCmd;
    else if ( (p_buffer[0] == 'S') &&
              (p_buffer[1] == 'E') &&
		      (p_buffer[2] == 'T') ) receivedCmd = setRegisterCmd;
    else if ( (p_buffer[0] == 'W') &&
		      (p_buffer[1] == 'R') ) receivedCmd = writeCmd;
    else if ( (p_buffer[0] == 'B') &&
		      (p_buffer[1] == 'B') ) receivedCmd = beginBehCmd;
    else if ( (p_buffer[0] == 'G') &&
		      (p_buffer[1] == 'O') ) receivedCmd = goCmd;
    else if ( (p_buffer[0] == 'H') &&
		      (p_buffer[1] == 'T') ) receivedCmd = haltCmd;
    else if ( (p_buffer[0] == 'Q') &&
		      (p_buffer[1] == 'U') ) receivedCmd = queryCmd;
    else if ( (p_buffer[0] == 'C') &&
		      (p_buffer[1] == 'L') ) receivedCmd = readClockCmd;
    else if ( (p_buffer[0] == 'D') &&
		      (p_buffer[1] == 'R') ) receivedCmd = camDumpRawCmd;
    else if ( (p_buffer[0] == 'D') &&
		      (p_buffer[1] == 'S') ) receivedCmd = camDumpSegCmd;
    else if ( (p_buffer[0] == 'S') &&
		      (p_buffer[1] == 'M') ){
		receivedCmd = camSetColorMapCmd;
	#ifdef TGT_HAS_CAMERA
		// Sort of a hack, but we need to handle this NOW!
		setColorMap();
	#endif
	}
    else if ( (p_buffer[0] == 'P') &&
		      (p_buffer[1] == 'G') ) receivedCmd = camPingCmd;
    else if ( (p_buffer[0] == 'E') &&
		      (p_buffer[1] == 'T') ) receivedCmd = camEnableTrackingCmd;
    else if ( (p_buffer[0] == 'D') &&
		      (p_buffer[1] == 'T') ) receivedCmd = camDisableTrackingCmd;
	else
		/* don't recognize the cmd */
		receivedCmd = invalidCmd;

    memset(p_buffer,0x00,MAX_VALUE_LENGTH);
	bufferIndex = 0;
}

// This will run the command in the buffers
byte runCommand(){
    if(receivedCmd == getRegisterCmd){
        // Send response - THIS NEEDS CLEANING UP
        if(receivedReg < SNR_HD){
            //sysReading(state[receivedReg]); -- changed 12/24/08 MEF
            serialWrite('#');
	        PrintNumber(state[receivedReg]);
	        serialWrite('\n');
        }else{
            //sysReading(SONAR);
            serialWrite('#');
	        PrintNumber(SONAR);
	        serialWrite('\n');
        }
		return COMMAND_NO_ACK;
    }else if(receivedCmd == setRegisterCmd){
        state[receivedReg] = p_values[0];
		// update servos!
	  #ifdef TGT_HAS_HEAD
		servoSetPosition(PIN_PAN_SERVO, state[PAN]);
		servoSetPosition(PIN_TILT_SERVO, state[TILT]);
   	  #endif
    }else if(receivedCmd == writeCmd){
        saveState();
    }else if(receivedCmd == beginBehCmd){
        behSwitch(p_values[0]);
    }else if(receivedCmd == goCmd){
        RSTATE = GO;
       #ifdef TGT_HAS_CLOCK
		unPulseEyes();
	   #endif
    }else if(receivedCmd == haltCmd){
        RSTATE = HALT;
	   #ifdef TGT_HAS_CAMERA
		if(camStatus==CAM_TRACKING){
			disableTracking();
		}
	   #endif
    }else if(receivedCmd == queryCmd){
        sendBehList();
    }else if(receivedCmd == readClockCmd){
       #ifdef TGT_HAS_CLOCK
		// return clock uptime (seconds)
		sysReading(systime/60);
       #else 
        sysMsg("No clock");
       #endif
    // AVRCam pass thru commands
   #ifdef TGT_HAS_CAMERA
    }else if(receivedCmd == camDumpRawCmd){
        passRawCam();
    }else if(receivedCmd == camDumpSegCmd){
    
    /*}else if(receivedCmd == camSetColorMapCmd){
        // not implemented...
        //setColorMap();*/
    }else if(receivedCmd == camPingCmd){
        pingCam();
    }else if(receivedCmd == camEnableTrackingCmd){
        enableTracking();
    }else if(receivedCmd == camDisableTrackingCmd){
        disableTracking();
   #endif
    }else{
        // unrecognized command
        return 0;
    }
    return 1;
}

// parse out any recieved data
void parse(){
	while(serialAvailable() > 0){
		byte data = serialRead();
        if (data == '\n' || data == '\r'){
			// process the last value sent
			if (procCount == 0){
				convertToCmd();
			}else if(procCount == 1 && (p_buffer[0] > '9')){
                // this is a register
                convertToReg();
            }else{
                convertToValue();
			}
			// process command
			data = runCommand();			
			if(data > 0){
				if(data != COMMAND_NO_ACK){
                	ACK();
				}
            }else{
                NCK();
			}			
			// reset all data
			procCount = 0;
            valCount = 0;
			memset(p_values,0x00,MAX_VALUE_COUNT);
            receivedCmd = invalidCmd;
		}else if (data == ' '){
			/* the end of a value has been reached */
			if (procCount == 0){
				convertToCmd();
				procCount++;   
			}else if(procCount == 1 && (p_buffer[0] > '9')){
                // this is a register
                convertToReg();
                procCount++;
            }else{
                if (valCount+1 >= MAX_VALUE_COUNT){
                    receivedCmd = invalidCmd;
                }else{
                    convertToValue();
                    procCount++;
                    valCount++;
                }
			}
		}else if ( (data >= 'A' && data <= 'Z') ||
				    (data >= '0' && data <= '9') || 
                    (data == '-')){
			/* a valid range of token was received */
			p_buffer[bufferIndex] = data;
			bufferIndex++;
		}else{
			/* an invalid character was received */
			receivedCmd = invalidCmd;
		}
	}
}						

/** This function starts the parser and sets up behaviors. */
void xrinit(){
    parseInit();
	// Initialize robot - usually in device file
	devInit();
	// Start motor driver - usually in device file
	motorInit();
	// Start sensors and state
	stateInit(); 
	// Start behavior switch
	switchInit();
	// Start serial output
	serialInit(SERIAL_RATE);
	// Start the vision system
  #ifdef TGT_HAS_CAMERA	
	visionInit();
	#endif
	// Setup system clock - usually in device file
  #ifdef TGT_HAS_CLOCK
	clockInit();
	#endif
	// Enable Interrupts
	sei();
	delayms(50);
	//sysMsg("MEGA324 CHIPSET");
	sendBehList();
}

/** The main loop, we parse, arbitrate and plan infinetly. */
void xrloop(){
	while(1){
		parse();
		// update sensors and state here
		update();
		// now do behaviors
		if(RSTATE>HALT){
			if(curbeh > -1){
				if(behlist[curbeh].arbitrate() > 0){
					behlist[curbeh].plan();
				}
			}			 
		}else{
			motorStop();
		  #ifdef TGT_HAS_CLOCK
			pulseEyes();
		    #endif
			// check go switch
          #ifdef PIN_GO
			if(digitalGetData(PIN_GO) == 0){
				// button pressed
				behSwitch(GO_BEH);
				RSTATE= GO;
			  #ifdef TGT_HAS_CLOCK
				unPulseEyes();
				#endif
			}
          #endif
			delayms(5);
		}
	}	
}


