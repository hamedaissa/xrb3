/******************************************************************************
 * AVRRA: The AVR Robotics API
 * xrseries.h - XR Series parser, init and loop
 *   THIS VERSION IS DEPRECIATED - USE THE NEWER xrseries2.h
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

// we need to define the parse mode
static int mode;			// our current mode of parsing
#define     PARSE_WAIT	0
#define   	PARSE_BEHB	1	// begin a behavior
#define     PARSE_BEHS	2
#define 	PARSE_WRITE	3	
#define		PARSE_REQ	4
#define		PARSE_SET	5
#define 	PARSE_GET	6

// for catching names of variables
char vname[3] = "   ";
int vrecieved = 0;
// for catching data
char dname[4] = "    ";
int drecieved = 0;

// parse out any recieved data
int parse(){
	if(mode == PARSE_WAIT){
		if(serialAvailable() > 0){
			byte data = serialRead();
			switch(data){
				case '!':
					// set or get request
					mode = PARSE_REQ;
					break;
				case 'B':
					mode = PARSE_BEHB;
					break;
				#ifdef TGT_HAS_CLOCK
				case 'C':
				  	// return clock uptime (seconds)
					sysReading(systime/60);
					break;
					#endif
				#ifdef TGT_HAS_CAMERA
				case 'D': // avr cam pass thru
				  while(serialAvailable() == 0)
                    ;
					if(serialRead() == 'T'){
						disableTracking();
					}else{				  	
						passRawCam();
					}
					break;
				case 'E': // avr cam pass thru
				  	enableTracking();
					// get rid of T
					serialRead();
					break;
					#endif
				case 'G':
					RSTATE = GO;
					#ifdef TGT_HAS_CLOCK
					unPulseEyes();
					#endif
					ACK();
					break;
				case 'H':
					RSTATE = HALT;
					#ifdef TGT_HAS_CAMERA
					if(camStatus==CAM_TRACKING){
						disableTracking();
					}
					#endif
					ACK();
					break;
				#ifdef TGT_HAS_CAMERA
				case 'P':
				  	pingCam();
					// get rid of G
					serialRead();
					break;
					#endif
				case 'Q':
					// query for behaviors;
					sendBehList();
					break;
				//case 'R':
				// restart system with watchdog timer
				//case 'S':
				//	mode = PARSE_BEHS;
				//	break;
                #ifdef TGT_HAS_CAMERA
				case 'S':// avr cam pass thru
                  while(serialAvailable() == 0)
					;
					if(serialRead() == 'M'){
						setColorMap();
					}
                    break;
                    #endif
				case 'W':
					mode = PARSE_WRITE;
					break;
				case '\n':
				case '\r':
					// do nothing, extra console input
					break;
				default:
					NCK();
					break;
			}
		}
	}else if(mode == PARSE_WRITE){
		if(serialAvailable() > 0){
			byte data = serialRead();
			if(data == 'R'){
				saveState();
			}else{
				NCK();
			}
			mode = PARSE_WAIT;
		}
	}else if(mode == PARSE_REQ){		
		if(serialAvailable() > 0){
			byte data = serialRead();
			dname[drecieved] = data;
			drecieved++;
			// we have get or set
			if(drecieved > 2){
				if(dname[0] == 'g'){
					mode = PARSE_GET;
				}else if(dname[0] == 's'){
					mode = PARSE_SET;
				}else{
					mode = PARSE_WAIT;
					NCK();
				}
				drecieved = 0;
				dname[0] = ' ';
				dname[1] = ' ';
				dname[2] = ' ';	
			}
		}
	}else if((mode == PARSE_SET) || (mode == PARSE_GET)){
		if(serialAvailable() > 0){
			byte data = serialRead();
			if((vrecieved < 3) && (data != '=') && (data != '\n')){
				vname[vrecieved] = data;
				vrecieved++;
			}else{
				vrecieved = 3;
				if((drecieved < 4) && (data != '\n')){
					if(data != '='){
						dname[drecieved] = data;
						drecieved++;
					}
				}else{
					int idx= 0;
					int dir= BACKWARD;
					// we have all data, now process
					switch(vname[0]){
						case 'M':
							// motor command
							if(dname[0] == 'F'){
								dir = FORWARD;
							}
							if(mode == PARSE_SET){
								int idx = 0, spd = 0;
								for(idx = 1; idx< (drecieved); idx++){
									spd = (spd * 10) + (dname[idx] - 48);
								}
								if(vname[1] == 'L'){
									//state[DIRL] = dir;
									//state[SPDL] = spd;
									motorLeft(dir,spd);
								}else{
									//state[DIRR] = dir;
									//state[SPDR] = spd;
									motorRight(dir,spd);
								}
							}else{
								// no get!
							}
							// we did our own processing
							mode = PARSE_WAIT;
							break;
						case 'R':
							// ranger command, read only
							sysReading(SONAR);
							mode = PARSE_WAIT;
							break;
						case 'I': 
							// IR commands, read only
							if(vname[1] == 'L'){
								sysReading(LEFT_IR);
							}else if(vname[1] == 'R'){
								sysReading(RIGHT_IR);
							}else{
								sysReading(HEAD_IR);
							}
							mode = PARSE_WAIT;
							break;
						case 'S':
							// servo command
							if(vname[1] == 'C'){
								// center command
								if(vname[2] == 'P'){
									// set or get pan center
									idx = PANC;
								}else{
									// set or get tilt center
									idx = TLTC;
								}
							}else{
								if(vname[1] == 'P'){
									// set or get pan
									idx = PAN;
								}else{
									// set or get tilt
									idx = TILT;
								}
							}
							break;
						default:
							mode = PARSE_WAIT;
							NCK();										
					}
					if(mode == PARSE_GET){
						sysReading(state[idx]);
					}	
					if(mode == PARSE_SET){
						int i = 0, b = 0, invB = 0;
						for(i = 0; i< drecieved; i++){
							if(dname[i] != '-'){
								b = (b * 10) + (dname[i] - 48);
							}else{
								invB = 1;
							}
						}
						if(invB > 0){
							b = -b;
						}
						state[idx] = b;
						ACK();
						// update servos!
					  #ifdef TGT_HAS_HEAD
						servoSetPosition(PIN_PAN_SERVO, state[PAN]);
						servoSetPosition(PIN_TILT_SERVO, state[TILT]);
						#endif
					}
					mode = PARSE_WAIT;
					drecieved = 0;
					dname[0] = ' ';
					dname[1] = ' ';
					dname[2] = ' ';	
					vrecieved = 0;
					vname[0] = ' ';
					vname[1] = ' ';
					vname[2] = ' ';	
				}
			}
		}
	}
	
	// behavior parsing pauses all arbitration and planning
	while((mode == PARSE_BEHB) || (mode == PARSE_BEHS)){
		// need to get number
		if(serialAvailable() > 0){
			byte data = serialRead();
			if((data == '\n') || (drecieved > 2)){
				// start or stop behavior
				int i = 0, b = 0;
				for(i = 0; i< drecieved; i++){
					b = (b * 10) + (dname[i] - 48);
				}
				if(mode == PARSE_BEHB){
					behSwitch(b);
				}else{
					behSwitch(-1);
				}
				mode = PARSE_WAIT;
				drecieved = 0;
				dname[0] = ' ';
				dname[1] = ' ';
				dname[2] = ' ';			
			}else{
				// make sure it is a number
				if((data >= '0') && (data <= '9')){
					dname[drecieved] = data;
					drecieved++;
				}
			}
		}
	}	

	return 0;
}

/** This function starts the parser and sets up behaviors. */
void xrinit(){
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
			if(digitalGetData(PIN_GO) == 0){
				// button pressed
				behSwitch(GO_BEH);
				RSTATE= GO;
			  #ifdef TGT_HAS_CLOCK
				unPulseEyes();
				#endif
			}
			delayms(5);
		}
	}	
}


