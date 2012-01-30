/* I2C compass demo */

int cmpsInit(){
	cmps03Init();
    sysMsg("CMPS Ver:");
    sysReading(cmps03GetVer());
	return 0;
}

/* get a reading, print it */
int cmpsArb(){
	sysReading(cmps03GetData());	
	delayms(100);
	return 0;
}
