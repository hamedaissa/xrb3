/******************************************************************************
 * AVRRA: The AVR Robotics API
 * default.h - a basic behavior that puts hello world to console. Also
 *  includes several helper routines useful for the XRSERIES robots.
 * 
 * Copyright (c) 2007-2008, Michael E. Ferguson
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

static int done= 0;

int defaultInit(){
	// do nothing
	return 0;
}

/** = one run of Arbitration loop */
int defaultArbitrate(){
	if(done == 0){
		Print("?Hello World!\n");
	}
	done++;
	if(done > 10)
		done = 0;
	
	// turn this delay down as we add more stuff above
	delayms(30);
	return 0;
}

/** Such a basic behavior, no planning is required. */
int defaultPlan(){
	return 1;
}

//****************************OTHER HELPER ROUTINES**************************//

/** avoid a left obstacle, backs up a set time, adding some angle at end */
void leftBump(int backup){
	// save previous motor speeds
	motorStop();
	// get out of dodge (Ballistic)
	motorRight(BACKWARD*REG_SPEED);
	motorLeft(BACKWARD*REG_SPEED);
	delayms(backup/2);
	motorRight(BACKWARD*REG_SPEED);
	motorLeft(BACKWARD*REG_SPEED/2);
	delayms(backup/2);	
	// restore motor speeds
	motorResume();
}

/** avoid a right obstacle, backs up a set time, adding some angle at end */
void rightBump(int backup){
	// save previous motor speeds
	motorStop();
	// get out of dodge (Ballistic)
	motorRight(BACKWARD*REG_SPEED);
	motorLeft(BACKWARD*REG_SPEED);
	delayms(backup/2);
	motorRight(BACKWARD*REG_SPEED/2);
	motorLeft(BACKWARD*REG_SPEED);
	delayms(backup/2);	
	// restore motor speeds
	motorResume();
}

/** Bang-bang iterative follows, should be called at least a minimum of 10-15Hz */
int followLeft(int dist){
	if(GET_LEFT_IR < dist){
		if(GET_LEFT_IR < ((dist * 4) / 5)){
			// motor way to the right
			motorLeft(REG_SPEED + 40);
			motorRight(REG_SPEED - 40);
		}else{
			// motor to the right...
			motorLeft(REG_SPEED+20);
			motorRight(REG_SPEED-20);
		}
	}else{
		// motor to the left...
		motorLeft(REG_SPEED-20);
		motorRight(REG_SPEED+20);
	}
	return 0;
}
int followRight(int dist){
	if(GET_RIGHT_IR < dist){ 
		if(GET_RIGHT_IR < ((dist *4) / 5)){
			// motor way to the left
			motorLeft(REG_SPEED-40);
			motorRight(REG_SPEED+40);
		}else{
			// motor to the left...	
			motorLeft(REG_SPEED-20);
			motorRight(REG_SPEED+20);
		}
	}else{
		// motor to the right...
		motorLeft(REG_SPEED+20);
		motorRight(REG_SPEED-20);
	}
	return 0;
}


