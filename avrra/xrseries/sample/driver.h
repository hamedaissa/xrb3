/*
 * driver.h - behavior that allows desktop driving
 * Copyright 2008, Michael E Ferguson
 */

/** = one run of Arbitration loop */
int driveArbitrate(){
	// maybe add some obstacle avoidance?
	motorLeft(BACKWARD,240);
	motorRight(BACKWARD,240);
	
	
	delayms(5);
	return 0;
}

/** Such a basic behavior, no planning is required. */
int drivePlan(){
	return 1;
}
