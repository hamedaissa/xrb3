/******************************************************************************
 * Xr-B3: Topological Mapping and Planning, with sparse global metrics
 * Copyright 2008 Michael E Ferguson, mfergs7@gmail.com
 * 
 * This is an right-coast orthogonal maze solver. Rolls forward, following a  
 *   right wall. If right wall dissappears, or a forward wall appears, will    
 *   stop and call planner.                                                  
 * If we have run out of space to go, we enter backtrack mode, which follows
 *   the left wall.                                                         
 */

/* Behavior Status Register - default to 0x00
 *  bit 0:      hi = JUST_TURNED
 *  bit 7:      hi = KNOWN MAP 
 */
char BREG;

#define JUST_TURNED 		(BREG&0x01)	
#define SET_JUST_TURNED		BREG |= 0x01; 
#define CLR_JUST_TURNED		BREG &= 0xFE; 

/** Setting this keeps us forever in BACKTRACK */
#define KNOWN_MAP			(BREG & 0x80)

#define SLOWER_SPEED		160
#define LOOK_LEFT			-80
#define LOOK_CENTER			0
#define LOOK_RIGHT			80

/** approximate grid size, in inches, used to calculate free space reqs. */
#define GRID_SIZE           18
#define FWD_DIST            7
/** reading that defines loss of wall */
#define NO_WALL 			50
#define FOLLOW_DIST         30      // was 18 for UALBANY

int JTdistance;				        // how far to go before enabling JT

/******************************************************************************
 * lotsa constants for all things mapalicious
 */

#define NORTH			0
#define WEST			1
#define SOUTH			2
#define EAST			3

#define VISITED			-1
#define UNKNOWN			0
#define UNVISITED		1
#define GOAL			4
#define mNULL			-1

#define pNULL           -1

#define MODE_EXPLORE	0
#define MODE_BACKTRACK	1

#define tNodes			50


/******************************************************************************
 * node definition
 */

typedef struct{
	signed char dirs[4];        // direction index links
    signed char path[4];        // path index links
	signed char flags;          // flags for this node
	signed char dist;           // distance to goal (only used in return)
} mapNode_t; 

/** map and state variables */
mapNode_t map[tNodes];
int paths[tNodes];

signed char lheading;			// our heading when we left node (N,W,S,E)
signed char lnode;				// this is that last node we left

int exp_mode;
int nCount;


/******************************************************************************
 * Debugging info routines
 */

void PrintHeading(signed char heading){
    switch(heading){
		case 0:
			Print("N");
			break;
		case 1:
			Print("W");
			break;
		case 2:
			Print("S");
			break;
		case 3:
			Print("E");
			break;
	}
}

void PrintNewHeading(signed char heading, signed char node){
	Print("?");
	PrintHeading(heading);
	Print(",n");
	PrintNumber(node);
	Print("\n");
}

void PrintNewNode(signed char nodeP, signed char nodeN, signed char heading){
	Print("?n");
	PrintNumber(nodeN);
	PrintHeading(heading);
	Print(" of n");
	PrintNumber(nodeP);
	Print("\n");
}


/******************************************************************************
 * helper functions
 */

/** = new heading, after shifting current heading left */
signed char shiftHeadingL(signed char heading){
	if(heading<EAST){
		heading = heading + 1;
	}else{
		heading = NORTH;
	}
	return heading;
}

/** = new heading, after shifting current heading right */
signed char shiftHeadingR(signed char heading){
	if(heading>NORTH){
		heading = heading - 1;
	}else{
		heading = EAST;
	}
	return heading;
}

/** physically rotates robot to a new heading */
void moveToHeading(oldHeading, newHeading){
	// logic to turn from cheading to nheading!
	if(newHeading == shiftHeadingL(oldHeading)){
		turnX(90);
	}else if(newHeading == shiftHeadingR(oldHeading)){
		turnX(-90);
	}else if(newHeading == shiftHeadingR(shiftHeadingR(oldHeading))){
		turnX(180);
	}
}

/** = index of nearest unvisited node, ignore heading direction */
int findGoal(int start, int heading){
	int goal;
	// this is an unvisited node
	if(map[start].flags > VISITED){
		return start;
	}else{
		// we have to go back and find a unvisited node
		if((map[start].dirs[NORTH] != mNULL) && (heading!=NORTH)){
			goal = findGoal(map[start].dirs[NORTH], SOUTH);
			if(goal > -1){
				return goal;
			}
		}
		if((map[start].dirs[WEST] != mNULL) && (heading!=WEST)){
			goal = findGoal(map[start].dirs[WEST], EAST);
			if(goal > -1){
				return goal;
			}
		}
		if((map[start].dirs[EAST] != mNULL) && (heading!=EAST)){
			goal = findGoal(map[start].dirs[EAST], WEST);
			if(goal > -1){
				return goal;
			}
		}
		if((map[start].dirs[SOUTH] != mNULL) && (heading!=SOUTH)){
			goal = findGoal(map[start].dirs[SOUTH], NORTH);
			if(goal > -1){
				return goal;
			}
		}
		return -1;
	}
} 

/** assigns distance from goal to nodes, spreads out from begin */
int wave(int begin, int end, int x){
	if(map[begin].dist == 100){
		map[begin].dist = x;
		if(begin == end){
			// debug message here?
			return 1;
		}else{
			// not finished, gotta continue
			if(map[begin].dirs[NORTH] != mNULL){
				if(wave(map[begin].dirs[NORTH], end, x+1) > 0)
					return 1;
			}
			if(map[begin].dirs[WEST] != mNULL){
				if(wave(map[begin].dirs[WEST], end, x+1) > 0)
					return 1;
			}
			if(map[begin].dirs[EAST] != mNULL){
				if(wave(map[begin].dirs[EAST], end, x+1) > 0)
					return 1;
			}
			if(map[begin].dirs[SOUTH] != mNULL){
				if(wave(map[begin].dirs[SOUTH], end, x+1) > 0)
					return 1;
			}	
			return 0;
		}
	}else{
		return 0;
	}
}

/** finds last unvisited node, sets distance to goal for each node */
int wavePlan(int start, int heading){
	int goal = 0;
	// reset distances
	while(goal < tNodes){
		map[goal].dist = 100;
		goal++;
	}
	// find goal
	goal = findGoal(start, heading);
    if(goal > -1){
    	return wave(goal,start,0);
    }else{
        return -1;
    }
}

/******************************************************************************
 * This is the map setup - we start the map. 
 */
int topoInit(){
	int x;
	signed char nHeading = -1;
	// general setup
	for(x= 0; x<tNodes; x++){
		map[x].dirs[NORTH] = mNULL;
		map[x].dirs[WEST] = mNULL;
		map[x].dirs[SOUTH] = mNULL;
		map[x].dirs[EAST] = mNULL;
        map[x].path[NORTH] = pNULL;
        map[x].path[WEST] = pNULL;
        map[x].path[SOUTH] = pNULL;
        map[x].path[EAST] = pNULL;
		map[x].flags = UNKNOWN;
		map[x].dist = 0;
	}
    
	// we have one node so far
	map[0].flags = VISITED;
	nCount = 1;	
	
	// add our next node(s)
	servoSetPosition(PIN_PAN_SERVO,LOOK_RIGHT);
	delayms(100);
	if(SONAR > GRID_SIZE){
        // add a right node
		map[0].dirs[EAST] = nCount;
		map[nCount].dirs[WEST] = 0;
		PrintNewNode(0,nCount,EAST);
		nCount++;
		nHeading = EAST;
	}
	servoSetPosition(PIN_PAN_SERVO,LOOK_CENTER);
	delayms(100);
	if(SONAR > GRID_SIZE){
        // add a forward node
		map[0].dirs[NORTH] = nCount;
		map[nCount].dirs[SOUTH] = 0;
		PrintNewNode(0,nCount,NORTH);
		nCount++;
		nHeading = NORTH;
	}
    if(nHeading == -1){
        // we can't go right or forward!
        servoSetPosition(PIN_PAN_SERVO,LOOK_LEFT);
        delayms(100);
        if(SONAR > GRID_SIZE){
        	map[0].dirs[WEST] = nCount;
        	map[nCount].dirs[EAST] = 0;
        	PrintNewNode(0,nCount,WEST);
        	nCount++;
        	nHeading = WEST;
        }else{
            sysMsg("Map Failed!");
            RSTATE = HALT;
            return 0;
        }
    }
    
	moveToHeading(NORTH,nHeading);
	delayms(100);
	PrintNewHeading(nHeading,map[0].dirs[nHeading]);
	
	exp_mode= MODE_EXPLORE;
	
	lnode = 0;
	lheading = nHeading;

	BREG= 0;
	return 0;
}

/******************************************************************************
 * arbitration and planning
 */

/** if exploring, follow left wall, checking left, forward,
    if backtracking, follow right wall, checking right, forward */
int topoArb(){
	/* priority 3 = forward wall detection */
	servoSetPosition(PIN_PAN_SERVO,0);
	if(SONAR < FWD_DIST){
		motorStop();
		delayms(100);
		// once stopped, check again
		if(SONAR < FWD_DIST){
			sysMsg("Fwall");
			// return and cause the planner to be called
			return 3;
		}
		// wall not really there, start again
		motorResume();
		// no return, false hit, we want to check lower levels still
	}
	
	/* priority 2 = turn if there is an opening, if not JT */
	if(JUST_TURNED == 0){
		if(exp_mode == MODE_EXPLORE){
			// following right wall, turn right if opening
			if(GET_RIGHT_IR > NO_WALL){
				// check to see if opening
				moveX(12);       // was 8
				servoSetPosition(PIN_PAN_SERVO,LOOK_RIGHT);
				if((SONAR > (GRID_SIZE/2)) || (GET_HEAD_IR > 90)){
					sysMsg("No R Wall");
                    delayms(25);
					// return and cause the planner to be called
                    return 2;
				}
			}
		}else{
			// following left wall, turn left if opening
			if(GET_LEFT_IR > NO_WALL){
				// check to see if opening
				moveX(12);
				servoSetPosition(PIN_PAN_SERVO,LOOK_LEFT);
				if((SONAR > (GRID_SIZE/2)) || (GET_HEAD_IR > 90)){
					sysMsg("No L Wall");
                    delayms(25);
					// return and cause the planner to be called
					return 2;
				}
			}
		}
	}	
	
	/* priority 1 = follow wall, if present*/
	if(JUST_TURNED == 0){
		if(exp_mode == MODE_EXPLORE){
			// follow right
			followRight(FOLLOW_DIST);
		}else{
			// follow left
			followLeft(FOLLOW_DIST);
		}
	}else{
		/* priority 0 = reset JT, if we have gotten to JTdistance */
		if(exp_mode == MODE_EXPLORE){
			// arc left a little
			motorLeft(REG_SPEED-5);
			motorRight(REG_SPEED+5);
		}else{
			// arc right a little
			motorLeft(REG_SPEED+5);
			motorRight(REG_SPEED-5);
		}
		if(getRcount > JTdistance)
			CLR_JUST_TURNED;
	}
	
	delayms(10);
	return 0;
}

/******************************************************************************
 * The real action. Robot is parked when we hit this. 
 */
int topoPlan(){
	CLR_JUST_TURNED;
    // plan and move us to new heading
		signed char nnode=0;		// next node
	signed char nheading=0;		// next heading
	signed char tnode;			// temp node
	signed char theading;		// temp heading
	signed char priority;		// used in search pattern
	
	// this should be our new current node
	signed char curnode = map[lnode].dirs[lheading];
	// update current node flags to show visited
	map[curnode].flags= VISITED;
	
	if((exp_mode == MODE_BACKTRACK) && (map[curnode].dist == 0)){
		if(KNOWN_MAP > 0){
			// if in known map, stop
			RSTATE = HALT;
			// we are done!
			return 0;
		}else{
			exp_mode = MODE_EXPLORE;
		}
	}
	
	// priority is to go left if possible... then forward, then right
	if(exp_mode == MODE_EXPLORE){
		// add new node(s)
		nheading = -1;
		// if this was triggered by missing wall, we are looking right
		if(servoGetPosition(PIN_PAN_SERVO) == LOOK_RIGHT){
			if(SONAR > GRID_SIZE){
				map[curnode].dirs[shiftHeadingR(lheading)] = nCount;
				map[nCount].dirs[shiftHeadingL(lheading)] = curnode;
				PrintNewNode(curnode,nCount,shiftHeadingR(lheading));
				nCount++;
				nheading = shiftHeadingR(lheading);
			}	
		}
		servoSetPosition(PIN_PAN_SERVO,LOOK_CENTER);
		delayms(200);
		if(SONAR > GRID_SIZE){
			map[curnode].dirs[lheading] = nCount;
			map[nCount].dirs[shiftHeadingR(shiftHeadingR(lheading))] = curnode;
			PrintNewNode(curnode,nCount,lheading);
			nCount++;
			if(nheading == -1)
				nheading = lheading;
		}else{
			// if no forward heading, look to left
			servoSetPosition(PIN_PAN_SERVO,LOOK_LEFT);
			delayms(200);
			if(SONAR > GRID_SIZE){
				map[curnode].dirs[shiftHeadingL(lheading)] = nCount;
				map[nCount].dirs[shiftHeadingR(lheading)] = curnode;
				PrintNewNode(curnode,nCount,shiftHeadingL(lheading));
				nCount++;
				if(nheading == -1)
					nheading = shiftHeadingL(lheading);
			}
		}
	
		if(nheading == -1){
			// no choices, gotta backtrack
			exp_mode = MODE_BACKTRACK;
            sysMsg("BackTrack");
			// call the wave planner to set a new goal
			if(wavePlan(curnode,lheading) == -1){
                // no goal found, end mapping
                sysMsg("Map Complete");
                RSTATE = HALT;
                return 0;
            }
		}
	}
	
	if(exp_mode == MODE_BACKTRACK){
		theading= shiftHeadingL(lheading);
		priority = map[curnode].dist;		// this is new, was just 100
		do{
			// find our next node and heading
			theading= shiftHeadingR(theading);
			tnode= map[curnode].dirs[theading];
			// check if it is a node
			if(tnode != mNULL){
				// is it of higher priority?
				if(map[tnode].dist < priority){
					// yep, update
					nnode= tnode;
					nheading= theading;
					priority = map[nnode].dist;
				}
			}
			// run 4 times (one full circle)
		}while(theading!=shiftHeadingL(lheading));
        
        if(KNOWN_MAP == 0){
            if(map[curnode].dist == 1){
                // switch back to left side, no JT
                exp_mode = MODE_EXPLORE;
                BREG = 0;
            }
        }
	}
	
	SET_JUST_TURNED;

	// logic to turn from cheading to nheading!
	moveToHeading(lheading,nheading);
	delayms(100);
	PrintNewHeading(nheading,map[curnode].dirs[nheading]);
	
	// update globals
	lheading= nheading;
	lnode= curnode;
	
    // if JT set by planner, we go GRID_SIZE before checking...
    if(JUST_TURNED){    
        // 16 counts per inch...        was GRID_SIZE/2 = GRID_SIZE*8
        JTdistance = getRcount + 80; //(GRID_SIZE * 16);
	}else{
        JTdistance = 0;
    }
	// return control (to arbitrate)
    return (int) curnode;
}
