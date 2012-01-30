/******************************************************************************
 * AVRRA: The AVR Robotics API
 * clock.h - device driver for clock, eyes on AVRRA PRO Board using Timer2
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
 * - Neither the name of the AVRRA nor the names of its contributors 
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

#ifndef AVRRA_CLOCK
#define AVRRA_CLOCK

#define TGT_HAS_CLOCK

#include <avr/interrupt.h>

#define t_time	unsigned long

volatile t_time systime;	// system uptime (60 hz)	
int sysalive;
int syseyes; 				// 0 means off, 255 = pulse, between is intensity

/** Starts timer 2, sets up alive/eyes output */
void clockInit(){
	digitalSetDirection(0x1E, AVRRA_OUTPUT);
	TCNT2 = 0;

#ifdef TGT_HAS_EYES
	systime = 0;
	sysalive = 0;
	syseyes = 255;
	OCR2B = sysalive;
#endif
	OCR2A = 240;			// this defines 60 Hz Clock

	TCCR2A = 0x23;
	TCCR2B = 0x0F;
	
	TIMSK2 |= 0x01; 		// interrupt on overflow
}

void clockDisable(){
	TIMSK2 &= 0XFE;
}

void clockEnable(){
	TIMSK2 |= 0x01;
}

/** updates systime, alive output */
ISR(TIMER2_OVF_vect){
	systime++;
	if(syseyes == 255){ 
		sysalive++;
		// this defines 40 cycles/min
		if(sysalive > 45){
			if(sysalive < 90){
				OCR2B = (90 - sysalive)*5;
			}else{
				sysalive = 0;
				OCR2B = sysalive*5;
			}
		}else{
			OCR2B = sysalive*5;
		}
	}
}

t_time getClock(){
	return systime;
}

void wait(t_time ms){
	t_time curtime = systime;
	// now wait
	while((systime-curtime) < (ms/16));
}

void waitUntil(t_time time){
	while(systime < time);
}

#ifdef TGT_HAS_EYES
void setEyes(int intensity){
	syseyes = intensity;
	if(syseyes != 255){
		OCR2B = syseyes;
	}
}

int getEyes(){
	return syseyes;
}

/** Turns on eye pulsing*/
void pulseEyes(){
	setEyes(255);
	// set to increment
}

/** Turns off eye pulsing */
void unPulseEyes(){
	setEyes(0);
	// unset to change
}
#endif

#endif


/*********************************End of clock.h*******************************
 * REVISIONS: */

