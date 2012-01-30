/*
 * driveleft.h - drive to the left
 */

/** = one run of Arbitration loop, 0 if cruise, >0 otherwise */
int drightArbitrate(){
	motorLeft(FORWARD,200);
	motorRight(FORWARD,100);
	
	// turn this delay down as we add more stuff above
	delayms(15);
	return 0;

}

int drightPlan(){
	return 1;
}

