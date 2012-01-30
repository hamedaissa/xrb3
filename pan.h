//***************************************************************************//
//                                                                           //
//                  xrb3 - pan & sonar scan, metric mapping                  //
//                                                                           //
//***************************************************************************//
//                     Copyright 2008 Michael E. Ferguson                    //
//***************************************************************************//
// Behavior pans the area (about 120 degrees) and builds a forward map       //
//***************************************************************************// 

#define NORTH			0
#define WEST			1
#define SOUTH			2
#define EAST			3

// each m_map byte is mapped as:
//  bit0..3 - no. of hits
//  bit   6 - left wall
//  bit   7 - upper wall oR NOT???

// address in the map: lower left corner is (0,0), NORTH is UP
//  each m_map byte is 3x3 inches.
#define MAP_X       26     // 6'3" of space + 3" gray buffer
#define MAP_Y       26     //

#define MAP_CUTOFF  120    // maximum IR reading we use...
#define HEAD_OFFSET 13     // compensation for head IR off center of rotation

// vertical gray line is located half the map to the right of robot_X
#define m_gray_X   ((robot_X + 13) % MAP_X)

// horizontal gray line is located half the map up from robot_Y  
#define m_gray_Y   ((robot_Y + 13) % MAP_Y)    

unsigned char m_map[MAP_X][MAP_Y];

// robot current X,Y coordinates(inches), and heading(N,W,S,E)
unsigned char robot_X;    // note, divide by 3 to get box numbers
unsigned char robot_Y;
unsigned char robot_heading;     

/* Creates the map, places the robot. */
int panInit(){
    int i,j;
    // We start in 12 of [0..24]
    // gray_X = 25, gray_Y = 25...
    robot_X = 12;
    robot_Y = 12;
    robot_heading = NORTH;
    
    for(i=0; i < MAP_X; i++){
        for(j=0; j < MAP_Y; j++){
            m_map[i][j] = 0;
        }
    }
    return 0;
}

/* */
void mapInc(signed char X, signed char Y){ 
    /*if(X < 0){
        X = MAP_X + X;
    }
    if(Y < 0){
        Y = MAP_Y + Y;
    }
    X = X%MAP_X;
    Y = Y%MAP_Y;*/
    // Values limited, now post them
    if(m_map[X][Y] < 255){
        m_map[X][Y]++;    
    }
} 

/* Takes an X,Y hit and converts for robot orientation 
    Right, and Forward are positive */
void mapHit(signed char X, signed char Y){
    if(robot_heading == NORTH){
        mapInc(robot_X + X,robot_Y + Y);
    }
    if(robot_heading == SOUTH){
        mapInc(robot_X - X,robot_Y - Y);
    }
    if(robot_heading == WEST){
        mapInc(robot_X - Y,robot_Y - X);
    }
    if(robot_heading == EAST){
        mapInc(robot_X + Y,robot_Y + X);
    }       
}

/* Take a pan reading of the area. Convert it into the map
    remember: IR is in URCP. */
int panRead(){
    int reading;
    
    // head at -60 (left)
    servoSetPosition(PIN_PAN_SERVO, -60);
    delayms(100); // delay to remove skew from map (may be able to lower this...)
    reading = GET_HEAD_IR + HEAD_OFFSET;
    if(reading < MAP_CUTOFF) mapHit(-reading/18,reading/32);

    // head at -50
    servoSetPosition(PIN_PAN_SERVO, -50);
    delayms(100);
    reading = GET_HEAD_IR + HEAD_OFFSET;
    if(reading < MAP_CUTOFF) mapHit(-reading/21,reading/25);

    // head at -40
    servoSetPosition(PIN_PAN_SERVO, -40);
    delayms(100);
    reading = GET_HEAD_IR + HEAD_OFFSET;
    if(reading < MAP_CUTOFF) mapHit(-reading/25,reading/21);
    
    // head at -30
    servoSetPosition(PIN_PAN_SERVO, -30);
    delayms(100);
    reading = GET_HEAD_IR + HEAD_OFFSET;
    if(reading < MAP_CUTOFF) mapHit(-reading/32,reading/18);
    
    // head at -20
    servoSetPosition(PIN_PAN_SERVO, -20);
    delayms(100);
    reading = GET_HEAD_IR + HEAD_OFFSET;
    if(reading < MAP_CUTOFF) mapHit(-reading/47,reading/17);
    
    // head at -10
    servoSetPosition(PIN_PAN_SERVO, -10);
    delayms(100);
    reading = GET_HEAD_IR + HEAD_OFFSET;
    if(reading < MAP_CUTOFF) mapHit(-reading/92,(reading*2)/33);
    
    // head at 0
    servoSetPosition(PIN_PAN_SERVO, 0);
    delayms(100);
    reading = GET_HEAD_IR + HEAD_OFFSET;
    if(reading < MAP_CUTOFF) mapHit(0,reading/16);
 
    // head at 10
    servoSetPosition(PIN_PAN_SERVO, 10);
    delayms(100);
    reading = GET_HEAD_IR + HEAD_OFFSET;
    if(reading < MAP_CUTOFF) mapHit(reading/92,(reading*2)/33);
    
    // head at 20
    servoSetPosition(PIN_PAN_SERVO, 20);
    delayms(100);
    reading = GET_HEAD_IR + HEAD_OFFSET;
    if(reading < MAP_CUTOFF) mapHit(reading/47,reading/17);
    
    // head at 30
    servoSetPosition(PIN_PAN_SERVO, 30);
    delayms(100);
    reading = GET_HEAD_IR + HEAD_OFFSET;
    if(reading < MAP_CUTOFF) mapHit(reading/32,reading/18);
    
    // head at 40
    servoSetPosition(PIN_PAN_SERVO, 40);
    delayms(100);
    reading = GET_HEAD_IR + HEAD_OFFSET;
    if(reading < MAP_CUTOFF) mapHit(reading/25,reading/21); 
    
    // head at 50
    servoSetPosition(PIN_PAN_SERVO, 50);
    delayms(100);
    reading = GET_HEAD_IR + HEAD_OFFSET;
    if(reading < MAP_CUTOFF) mapHit(reading/21,reading/25);
    
    // head at 60
    servoSetPosition(PIN_PAN_SERVO, 60);
    delayms(100);
    reading = GET_HEAD_IR + HEAD_OFFSET;
    if(reading < MAP_CUTOFF) mapHit(reading/18,reading/32);
    
	return 0;
}

/* Prints the map to serial0. */
void printMap(){
	int i,j;    
	// send map out    
    for(i=MAP_Y-1; i > 0; i--){
        for(j=0; j < MAP_X; j++){
            if((j == robot_X) && (i== robot_Y)){
                serialWrite('R');
            }else{
                serialWrite(m_map[j][i]);
            }
        }
        serialWrite('\n');
    }
}

/* Testing code... */
int panArbitrate(){
	panRead();
	// send the map out...
	printMap();
	// testing code ... move forward a bit	    
    moveX(3);
    robot_Y++;
	// one run, then halt!
    RSTATE = HALT;
	return 0;
}
