/* cam follow demo */

int camInit(){
	pingCam();
	delayms(500);
	enableTracking();
	return 0;
}

/* Follow an object visually */
int camArb(){
	if(state[OBJ_X] > 0){
		int error = (85 - state[OBJ_X])/6;
		if((servoGetPosition(PIN_PAN_SERVO) + error) > 75){
			servoSetPosition(PIN_PAN_SERVO, 75);
		}else if((servoGetPosition(PIN_PAN_SERVO) + error) < -90){
			servoSetPosition(PIN_PAN_SERVO, -90);
		}else{
			servoSetPosition(PIN_PAN_SERVO, servoGetPosition(PIN_PAN_SERVO) + error);
		}
	}
	if(state[OBJ_Y] > 0){
		int error = (75 - state[OBJ_Y])/8;
		if((servoGetPosition(PIN_TILT_SERVO) + error) > 25){
			servoSetPosition(PIN_TILT_SERVO, 25);
		}else if((servoGetPosition(PIN_TILT_SERVO) + error) < -10){
			servoSetPosition(PIN_TILT_SERVO, -10);
		}else{
			servoSetPosition(PIN_TILT_SERVO, servoGetPosition(PIN_TILT_SERVO) + error);
		}
	}
	
	sysReading(state[OBJ_X]);
	
	delayms(50);
	return 0;
}
