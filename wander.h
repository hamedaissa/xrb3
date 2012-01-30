//***************************************************************************//
//                                                                           //
//                                xrb3 - wander                              //
//                                                                           //
//***************************************************************************//
//                     Copyright 2008 Michael E. Ferguson                    //
//***************************************************************************//

/* wander around, trying not to get stuck... */
int wanderArb(){
	// head leftish
	servoSetPosition(PIN_PAN_SERVO,-15);
	if(SONAR < 10){
		// little left hit, stop, turn right
		motorStop();
		turnX(-35);
		sysMsg("LeftHit");
		return 0;
	}
	delayms(50);
	// head forward
	servoSetPosition(PIN_PAN_SERVO,0);	
	if(SONAR < 15){
		// left was clear, turn left
		motorStop();
		clearCounters;
		//moveX(-3);
		turnX(90);
		sysMsg("CenterHit");
		return 0;
	}
	delayms(50);
	// head right
	servoSetPosition(PIN_PAN_SERVO,15);	
	if(SONAR < 10){
		// left was clear, turn left
		motorStop();
		turnX(35);
		sysMsg("RightHit");
		return 0;
	}
	delayms(50);

	// nothing - go forward
	motorLeft(REG_SPEED);
	motorRight(REG_SPEED);	

	// return head to center
	servoSetPosition(PIN_PAN_SERVO,0);	
	return 0;
}
