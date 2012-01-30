/******************************************************************************
 * AVRRA: The AVR Robotics API
 * imath.h- integer math routines, returns floating point value for integer 
 *   angle (within 5 degrees) 
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

// sinval[1] = value for degrees 1-5
// sinval[2] = value for degrees 6-10
float sinvals[] = {0,0.09,0.17,0.26,0.34,0.42,0.5,0.57,
				   0.64,0.71,0.77,0.82,0.87,0.91,0.94,
                   0.97,0.98,0.99,1};

/** = sin of */
float isin(int degrees){
	if(degrees < 0)
		return -isin(-degrees);	
	if(degrees == 0)
		return 0;	
	if(degrees <= 90)
		return sinvals[(degrees+4)/5];
	// else we need to do some conversion
	if(degrees <= 180)
	 	return isin(180-degrees);
	if(degrees <= 360)
		return -isin(degrees -180); 
	// else
	return isin(degrees - 360);
}

float icos(int degrees){
	return isin(90 + degrees);
}
