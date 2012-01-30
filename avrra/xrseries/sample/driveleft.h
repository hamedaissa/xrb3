/*
 * driveleft.h - drive to the left
 */

/** = one run of Arbitration loop, 0 if cruise, >0 otherwise */
int dleftArbitrate(){
	motorLeft(FORWARD,100);
	motorRight(FORWARD,200);
	
	// turn this delay down as we add more stuff above
	delayms(15);
	return 0;

}

int dleftPlan(){
	return 1;
}

