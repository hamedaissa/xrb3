/******************************************************************************
 * Xr-B3: Topological Mapping with Limited Local Metrics
 * Copyright 2008 Michael E Ferguson, mfergs7@gmail.com
 * 
 * Multiple parts:
 *  METRIC MAP ARCHITECTURE
 *   X m_map[MAP_X][MAP_Y] - the metric map
 *   X printMetricMap() - prints out the metric map
 *  TOPOLOGICAL MAP ARCHITECTURE
 *   _ node - flags + a list of pointers
 *   _ nodeptr - node, heading
 *  MOVEMENT API 
 *   X getX(rot,trans) - returns box index given rotation and translation 
 *   X getY(rot,trans) - returns box index given rotation and translation
 *   X turnXupdate() - turns X degrees and updates robot's position
 *   X moveXupdate() - moves X degress and updates robot's position 
 *  METRIC MAP BUILDER
 *   X m_touch(angle,reading) - update our map with a new data point
 * 	 X panUpdate() - makes a forward pan and updates our map
 *   _ rollingUpdate() - updates map with angled L/R IR readings
 *   X rollForward() - move forward helper for explore(). 
 *   X mapThreshold() - applies thresholding to build walls
 *  TOPOLOGICAL MAP BUILDER
 *   _ extract() - extracts new nodes from metric map
 *   _ localize() - where are we in the map (and with what certainty?)
 *  PLANNER
 *   X explore() - moves the robot around and builds map 
 *   _ plan() - moves us towards a goal, using the topological map
 *  OPERATIONAL CODE
 *   X topometricInit() - initializes the map to 0's, robot centered
 *   X topometricArb() - arbitration loop that builds map by exploring
 *                       can be used with the mapbuilder add-in for xr console
 */

#include "imath.h"

/****************************************************************************** 
 * METRIC MAP ARCHITECTURE
 *  m_map[X][Y]
 *  Our metric map is an 2D array of bytes. Each byte represents a 3" square.
 *  We assume an arbitrary start location pointed North. Thus, everything is 
 *    North-indexed. X addresses range west (low) to east. Y addresses range
 *    from south (low) to north:
 * 
 *              (0,1) N         (1,1)
 *                    |
 *                    |
 *     (-1,0) W --- (0,0) --- E (1,0) 
 *                    |
 *                    |
 *     (-1,-1)        S (0,-1)
 *     
 *  Within our map, have limited space (about 6'3" by 6'3"). Thus the map will
 *    roll over and be re-used. We define a gray area around our map.  
 *  Initial Map:
 *
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G   
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G      N
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G      ^
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G      |
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G      |
 *     0 0 0 0 0 0 0 0 0 0 0 0 ^ 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G   
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 G
 *     G G G G G G G G G G G G G G G G G G G G G G G G G G
 * 
 *     *note* the ^ in the center shows that the robot is
 *            located there, and facing north.
 */

#define NORTH			0
#define EAST			90
#define SOUTH			180
#define WEST			270

// address in the map: lower left corner is (0,0), NORTH is UP
//  each m_map byte is 3x3 inches.
#define MAP_X       26     // 6'3" of space + 3" gray buffer
#define MAP_Y       26     //
#define MAP_CUTOFF  120    // maximum IR reading we use... ~2ft
#define HEAD_OFFSET 15     // compensation for head IR off center of rotation

// vertical gray line is located half the map to the right of robot_X
#define m_gray_X   (((robot_X/3) + 13) % MAP_X)

// horizontal gray line is located half the map up from robot_Y  
#define m_gray_Y   (((robot_Y/3) + 13) % MAP_Y)    

// each byte is defined as:
//  + is number of hits
//  - negative is distance from nearest hit
signed char m_map[MAP_X][MAP_Y];

// robot current X,Y coordinates(inches), and heading(0-359 degrees)
int robot_X;    // note, divide by 3 to get box numbers
int robot_Y;
int robot_heading;     
int map_iter;   // number of iterations we have run 

/** printMetricMap() - prints the map to serial0. */
void printMetricMap(){
	int i,j;  
    Print("#map\n");
	// send map out    
    for(i=MAP_Y-1; i > 0; i--){
        for(j=1; j < MAP_X; j++){
            if(((j+m_gray_X)%MAP_X == (robot_X/3)) && ((i+m_gray_Y)%MAP_Y == (robot_Y/3))){
                switch(robot_heading){
                    case NORTH:
                        serialWrite('^');
                        break;
                    case WEST:
                        serialWrite('<');
                        break;
                    case EAST:
                        serialWrite('>');
                        break;
                    default:
                        serialWrite('R');
                        break;     
                }
            /*}else if((i == m_gray_Y) || (j == m_gray_X)){
                serialWrite('G');*/
            }else{
                serialWrite(m_map[(j+m_gray_X)%MAP_X][(i+m_gray_Y)%MAP_Y]);
            }
        }
        serialWrite('\n');
    }    
    Print("#end\n");
}

/****************************************************************************** 
 * TOPOLOGICAL MAP ARCHITECTURE
 *  Our topological map is multiple-linked list of nodes. 
 * 
 */    

/** data structure to .. 
typedef struct node_ptr{
	struct node * dest;
	int heading;
	struct node_ptr * next;
} node_ptr_t;*/

/** data structure to describe the nodes in our map 
typedef struct node{
	// nodes have data to describe them ...
	unsigned char flags;		
	// and headings to describe their location relative to other nodes.
	int[] headings;
    struct node[] nodes;
} node_t;

node_t * start;
node_t * current;
*/


/******************************************************************************
 * MOVEMENT API
 */

/** uses robot_X, robot_Y, and robot_heading, plus a rotation (degrees) about axis, 
      and translation (inches) returns index of box. */
int getX(int rotation, int translation){
	int X = robot_X + ((float) translation * isin(robot_heading + rotation));	
	X = X/3;
	if(X < 0)
		X = MAP_X + X;
	X = X % MAP_X;
	return X;
}
int getY(int rotation, int translation){
	int Y = robot_Y + ((float)translation * icos(robot_heading + rotation));	
    Y = Y/3;
	if(Y < 0)
		Y = MAP_Y + Y;
	Y = Y % MAP_Y;
	return Y;
}

/** turnXupdate() - turns X degrees and updates robot's position */
void turnXupdate(int degrees){
	turnX(degrees);
	robot_heading = (robot_heading + degrees) % 360;
	if(robot_heading < 0)
		robot_heading = 360 + robot_heading;
}

/** moveXupdate() - moves X degress and updates robot's position */
void moveXupdate(int inches){
	moveX(inches);
	// update position in map
	robot_X = robot_X + (inches * isin(robot_heading));	
	if(robot_X < 0)
		robot_X = (3 * MAP_X) + robot_X;
	robot_X = robot_X % (3 * MAP_X);

	robot_Y = robot_Y + (inches * icos(robot_heading));
	if(robot_Y < 0)
		robot_Y = (3 * MAP_Y) + robot_Y;
	robot_Y = robot_Y % (3 * MAP_Y);
    // TODO: zero the gray line
    
} 


/******************************************************************************
 * METRIC MAP BUILDER
 */

/** m_touch() - update our map with a new data point */
void m_touch(int rotation, int distance){
	int X = (((float) distance * isin(robot_heading + rotation))*3)/16; // inches
    int Y = (((float) distance * icos(robot_heading + rotation))*3)/16;	// inches
    X = (X + robot_X)/3;
	if(X < 0)
		X = MAP_X + X;
	X = X % MAP_X;
    Y = (Y + robot_Y)/3;
	if(Y < 0)
		Y = MAP_Y + Y;
	Y = Y % MAP_Y;
    if(m_map[X][Y] < 32){
        m_map[X][Y]++;    
    }
}

/** panUpdate() - take reading of the area, using X data points. 
	remember: IR is in URCP. */
void panUpdate(int Xpoints){
    int reading;
	int rotation;
	// default is 7 data points
	if(Xpoints == 0)
		Xpoints = 7;
	// take data points
	for(rotation = -60; rotation < 61; rotation += 120/(Xpoints -1)){
		servoSetPosition(PIN_PAN_SERVO, rotation);
		delayms(150);
		Print("?Reading:");
		reading = GET_HEAD_IR + HEAD_OFFSET;
		PrintNumber(reading);
		Print("@");
		PrintNumber(rotation);
		Print("\n");
		if(reading < MAP_CUTOFF) 
			m_touch(rotation,reading); //(reading*3)/16);
    }
    map_iter ++;
}

void rollingUpdate(){
	// read and add left
	
	// read and add right
}

/** rollForward() - move forward a few inches, adjust alignment to wall */
void rollForward(){
    // take a reading before movement
    sysReading(GET_RIGHT_IR);
    // no wall in front, move forward some more
    moveXupdate(3);
    // take a reading after
    sysReading(GET_RIGHT_IR);
    // TODO: adjust angle to follow wall
}

/** mapThreshold() - applies thresholding to build walls */
void mapThreshold(){
    int X, Y;
    for(X=0; X< MAP_X; X++){
        for(Y=0; Y<MAP_Y; Y++){
            if(m_map[X][Y] > 0){
                //if((m_map[(X-1)%MAP_X][Y] > 1) && (m_map[(X+1)%MAP_X][Y] > 1)){
                if(m_map[(X-1)%MAP_X][Y] + m_map[(X+1)%MAP_X][Y] > 1){
                      m_map[X][Y] = 5;
                //}else if((m_map[X][(Y-1)%MAP_Y] > 1) && (m_map[X][(Y+1)%MAP_Y] > 1)){
                }else if(m_map[X][(Y-1)%MAP_Y] + m_map[X][(Y+1)%MAP_Y] > 1){
                      m_map[X][Y] = 5;
                }else{
                    m_map[X][Y] = 0;
                }
            }else{
                m_map[X][Y] = 0;            
            }
        }
    }
}

/******************************************************************************
 * TOPOLOGICAL MAP BUILDER
 */

/** extract() - pull the nodes from the metric map. */
void extract(){
	// we find nodes using a method of maximum likelihood

    // for each block which is not occupied:
	//   we assign a value that is the minimum distance to an occupied block
	//
	//   after dropping values below threshold:
    //   3 3 4 3 3 4 5 3
    //   0 0 0 0 0 0 0 4
	//   0 0 0 0 0 0 0 5
    //   0 0 0 0 0 0 0 5
    //   3 4 5 5 0 0 0 3
    //   3 4 4 4 0 0 0 4
	//  
    //   after finding minimum distances:
	//   X X X X X X X X 
    //   1 1 1 1 1 1 1 X
    //   2 2 2 2 2 2 1 X
    //   1 1 1 1 1 2 1 X
    //   X X X X 1 2 1 X
	//   X X X X 1 2 1 X
	//   
	//   after final summation:	
	//   X X X X X X X X 	   X X X X X X X X
    //   9 9 9 9 9 8 5 X	                 X
    //   B B B B B C 8 X                 N   X
    //   9 9 9 A C C 9 X  -->          N N   X
    //   X X X X A B 9 X       X X X X       X
	//   X X X X 9 B 9 X       X X X X       X
    //       metric              topological

	//   X X X X X X X X 	   X X X X X X X X
    //   9 9 9 9 9 8 5 X	                 X
    //   B B B B B C 8 X                 N   X
    //   9 9 9 A C C 9 X  -->          N N   X
    //   X X X X A B 9 X       X X X X       X
	//   X X X X 9 B 9 X       X X X X       X
    //       metric              topological
	
}

/** localize() */


/******************************************************************************
 * PLANNER
 */

/** explore() - arbitration loop, right coastal navigation and map building */
void explore(){
	int X,Y,X2,Y2;
    // make a pan
    panUpdate(0);
    // threshold if neccessary
    if(map_iter > 3){     
        mapThreshold(); 
        map_iter = 0;
    }
    // Basic Planner - we should later add node detection here (forward wall requires it,
    //   if we have no left wall we should do it)
    // forward wall detection
    X = getX(0,9);
    Y = getY(0,9);	
    X2 = getX(0,6);
    Y2 = getY(0,6);	
    if((m_map[X][Y] > 1) || (m_map[X2][Y2] > 1)){
        // map shows forward wall,  but..
        // last check - head forward, get a sonar reading
        servoSetPosition(PIN_PAN_SERVO, 0);
        if(SONAR > 10){
            // bad map - clear cone in front
            for(X2 = -5; X2 < 9; X2+=5){
                for(Y2 = 3; Y2<10; Y2+=3){      
                    X = getX(X2,Y2);
                    Y = getY(X2,Y2);
                    m_map[X][Y] = 0;
                }
            }
            // go forward
            rollForward();
            sysMsg("Fixed map");
        }else{
            // try to go right!
            X = getX(90,9);
            Y = getY(90,9);
            X2 = getX(90,6);
            Y2 = getY(90,6);
            if((m_map[X][Y] > 0) || (m_map[X2][Y2] > 0)){
                // map shows right side wall,  but..
                // last check - head to right, get a sonar reading
                servoSetPosition(PIN_PAN_SERVO, 80);
                if(SONAR > 10){
                    // bad map - clear cone to side
                    for(X2 = -5; X2 < 9; X2+=5){
                        for(Y2 = 3; Y2<10; Y2+=3){      
                            X = getX(X2+90,Y2);
                            Y = getY(X2+90,Y2);
                            m_map[X][Y] = 0;
                        }
                    }
                    turnXupdate(90);
                }else{
                    // try to go left 
                    X = getX(-90,9);
                    Y = getY(-90,9);
                    X2 = getX(-90,6);
                    Y2 = getY(-90,6);
                    if((m_map[X][Y] > 0) || (m_map[X2][Y2] > 0)){
                        // map shows left side wall,  but..
                        // last check - head to left, get a sonar reading
                        servoSetPosition(PIN_PAN_SERVO, -80);
                        if(SONAR > 10){
                            // bad map - clear cone to side
                            for(X2 = -5; X2 < 9; X2+=5){
                                for(Y2 = 3; Y2<10; Y2+=3){      
                                    X = getX(X2-90,Y2);
                                    Y = getY(X2-90,Y2);
                                    m_map[X][Y] = 0;
                                }
                            }
                            turnXupdate(-90);
                        }else{
                            // have to go back
                            turnXupdate(180);
                        }
                    }else{   
                        turnXupdate(-90);
                    }
                }
            }else{
                turnXupdate(90);
            }
        }
    }else{
        rollForward();   
    }
}

/** plan() - moves us towards a goal, using the topological map */


/******************************************************************************
 * Operational Code
 */

/* Creates the map, places the robot. */
int topometricInit(){
    int i,j;
    // We start in 12 of [0..24]
    // gray_X = 25, gray_Y = 25...
    robot_X = 37;
    robot_Y = 38;
    robot_heading = 0; // NORTH
    for(i=0; i < MAP_X; i++){
        for(j=0; j < MAP_Y; j++){
            m_map[i][j] = 0;
        }
    }
    map_iter = 0;
    return 0;
}

/* Testing code... right wall follower, with map building */
int topometricArb(){
    explore();
    printMetricMap();    
    delayms(500);
    return 0;
}	
