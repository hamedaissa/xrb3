
int drivetestArb(){
	
	if(HEAD_IR > 54){
		motorLeft(FORWARD,150);
		motorRight(FORWARD,150);
	}else{
		motorStop();
	}
	
	// turn this delay down as we add more stuff above
	delayms(30);
	return 0;
	
}

