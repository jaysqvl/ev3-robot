#pragma config(Sensor, S1,     ultra_sonic,     sensorEV3_Ultrasonic)
#pragma config(Sensor, S4,     gyro,           sensorEV3_Gyro)
#pragma config(Motor,  motorD,          motor_L,       tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorA,          motor_R,       tmotorEV3_Large, PIDControl, encoder)

const int ScreenHeight = 127;
const int ScreenWidth  = 177;

const int numOfRows = 4;
const int numOfCols = 6;
const int size_of_route = 58;

typedef struct{
	int NorthWall;  // 0 = door, 1 = wall
	int EastWall;
	int SouthWall;
	int WestWall;
}Cell;

typedef struct{
	int x;
	int y;
}point;

Cell Grid[numOfRows][numOfCols];

point route_records[size_of_route];
point shortest_routes[size_of_route];

int RobotDirection = 0; // 0=North, 1=East, 2=South, 3=West
int index_records = 0;

// SET TARGET AND HOME
int StartPosRow = 3;
int StartPosCol = 1;
int CurrentPosRow = StartPosRow;
int CurrentPosCol = StartPosCol;
int TargetPosRow = 0;
int TargetPosCol = 3;
int realsize = 0;

// PID Variable Initialization
float Kp;
float Ki;
float Kd;
float gyro_data;
float error;
float power_L;
float power_R;
float previous_error;
int detect_distance = 4;

// ----------------------------------------------------
// ROBOT MOVEMENT
// ----------------------------------------------------

// No PID Forward, moves the robot forward without tuning
void move_no_PID_bump(){
	setMotorSpeed(motor_L,10);
	setMotorSpeed(motor_R,10);
}

// No PID Reverse, moves the robot backwards without tuning
void move_reverse_nopid(){
	setMotorSpeed(motor_L,-10);
	setMotorSpeed(motor_R,-10);
}

// -----------------------------------------------------
// True 90 Degree PID Turning Error Handling
// Robot Self-aligning algorithm using Calculated Error
// -----------------------------------------------------

// Accumulated error from Gyro
float Error_turning(int goal){
	gyro_data = getGyroDegrees(gyro);
	float offset = gyro_data - goal;
	return offset;
}

// Returns reported proportioned error (from total distance off goal)
float get_proportional_turning(int goal){ // return proportional gain which is the angle error
	error = Error_turning(goal);
	return error*Kp;
}

// Derivative error gain, rate of error between current and previous turning error
float get_derivative_turning(int goal){
	float derivative = (Error_turning(goal) - previous_error)*Kd;
	previous_error = Error_turning(goal);
	return derivative;
}

// Integral error gain,
float get_integral_turning(int goal){  // return the integral gain which is the difference between the proportional gain and the derivative gain
	return -(get_proportional_turning(goal)- get_derivative_turning(goal))*Ki;
}

// Uses sum of errors above to return to true 90 degrees, doesn't reset gyro upon correction
void turn_PID_without_reset(int goal){
	// Determined Constants
	Kp =5;
	Ki = 0.9;
	Kd = 5;
	power_L= 0;
	power_R = 0;
	int count = 0;

	while(true){
		float adjust = get_proportional_turning(goal)+get_derivative_turning(goal)+get_integral_turning(goal);
		if (adjust < 0 && adjust >-1){
			adjust = -2;
		}
		if (adjust > 0 && adjust <1){
			adjust =2;
		}
		setMotorSpeed(motor_L,adjust);
		setMotorSpeed(motor_R,-adjust);
		if (gyro_data == goal){
			count++;
			// COUNT DETERMINES HOW MANY ITERATIONS/TIMES THE PROGRAM CHECKS ITS ALIGNMENT (Increase 200 for more checks)
			if(count >200){
				setMotorSpeed(motor_L,0);
				setMotorSpeed(motor_R,0);
				previous_error = 0;
				break;
			}
		}
	}
}

// Uses sum of errors above to return to true 90 degrees, resets gyro upon correction
void turn_PID_with_reset(int goal){
	// Determined Constants
	Kp =5;
	Ki = 0.9;
	Kd = 5;
	power_L= 0;
	power_R = 0;

	int count = 0;	// variable to let the device auto adjust itself at the end of movement
	resetGyro(gyro);
	while(true){
		float adjust = get_proportional_turning(goal)+get_derivative_turning(goal)+get_integral_turning(goal);
		if (adjust < 0 && adjust >-1){
			adjust = -2;
		}
		if (adjust > 0 && adjust <1){
			adjust =2;
		}
		setMotorSpeed(motor_L,adjust);
		setMotorSpeed(motor_R,-adjust);
		if (gyro_data == goal){
			count++;
			// COUNT DETERMINES HOW MANY ITERATIONS/TIMES THE PROGRAM CHECKS ITS ALIGNMENT (Increase 200 for more checks)
			if(count >200){
				setMotorSpeed(motor_L,0);
				setMotorSpeed(motor_R,0);
				previous_error = 0;
				break;
			}
		}
	}
}

// -----------------------------------------------------
// True Forward PID Error Handling
// Forward movement error minimization using gyro to catch error
// -----------------------------------------------------

float Error(){
	gyro_data = getGyroDegrees(gyro);
	return gyro_data;
}

float get_proportional(){
	error = Error();
	return error*Kp;
}

float get_derivative(){
	float derivative = (Error() - previous_error) * Kd;
	previous_error = Error();
	return derivative;
}

float get_integral(){
	return -(get_proportional()- get_derivative()) * Ki;
}

void move_PID(){
	float adjust = get_proportional() + get_derivative() + get_integral();
	power_L += adjust;
	power_R -= adjust;
	setMotorSpeed(motor_L, power_L);
	setMotorSpeed(motor_R, power_R);
}


// ---------------------------------------------------------------
//  Moves forward by static cell distance
// 	Distance Between Cells = 23 cm
//	Wheel Radius = 2.8 cm
// 	Wheel Circumference = (2)(pi)(2.8cm) = 17.5929189 cm
//  Required Rotations (# of 360 degree turns) = 1.30734418 rots
//  Required Angle of Rotation =~ 471 Deg
// ---------------------------------------------------------------
void move_fwd_cell(){
	resetGyro(gyro);
	resetMotorEncoder(motor_L);
	resetMotorEncoder(motor_R);
	power_L = 20;
	power_R = 20;
	Kp =0.000005;
	Ki = 0.001;
	Kd = 0.021;
	while(true){
		move_PID();
		if(getMotorEncoder(motor_L) >= 471){
			setMotorSpeed(motor_L, 0);
		}
		if(getMotorEncoder(motor_R) >= 471){
			setMotorSpeed(motor_R,0);
		}
		if((getMotorEncoder(motor_R) >= 471) && (getMotorEncoder(motor_R) >= 471)){
			break;
		}
		if (getUSDistance(ultra_sonic) <= detect_distance){
			setMotorSpeed(motor_R,0);
			setMotorSpeed(motor_L, 0);
			break;
		}
	}
	turn_PID_without_reset(0);
	resetMotorEncoder(motor_L);
	resetMotorEncoder(motor_R);
}

// re-calibrate the gyroscopes position by aligning with the back and front wall
// as benchmarks for relative angle
void recallibrate(){
	int timer_limit_2 = 3000;
	int timer_2 = 0;

	// reset gyro
	// Reverses into wall behind it to align backwards
	resetGyro(gyro);
	while(true){
		move_reverse_nopid();
		timer_2++;
		if(timer_2 == timer_limit_2 ){
			break;
		}
	}

	// Bumps the wall infront to align forwards
	resetGyro(gyro);
	timer_limit_2 = 15000;
	timer_2 = 0;
	while(true){
		timer_2++;
		move_no_PID_bump();
		if(timer_2 == timer_limit_2 ){
			break;
		}
	}
	setMotorSpeed(motor_L,0);
	setMotorSpeed(motor_R,0);
	delay(500);

	// Reverses to recenter itself
	resetGyro(gyro);
	while((getUSDistance(ultra_sonic)<detect_distance)||(getUSDistance(ultra_sonic)>50) ){
		move_reverse_nopid();
	}
	setMotorSpeed(motor_L,0);
	setMotorSpeed(motor_R,0);
	turn_PID_without_reset(0);
}

// Easy Of Access Commmands
void turnRobotLeft(){  // turn left 90 degree
	resetGyro(gyro);
	turn_PID_with_reset(-90);
}

void turnRobotRight(){ // turn right 90 degree
	resetGyro(gyro);
	turn_PID_with_reset(90);
};

// --------------------------------------------------------------------------------------------------------------------
// Grid Drawing Stuff Below
// --------------------------------------------------------------------------------------------------------------------

// DrawBot
// Draws the current position, direction, as well as the target position
// Combines DisplayStartandEnd and DrawBot
void DrawBotDisplayStartandEnd(){
	int RobotXpixelPos=0;
	int RobotYpixelPos=0;

	int targetXpixelPos=0;
	int targetYpixelPos=0;

	if(CurrentPosCol==0){
		RobotXpixelPos=ScreenWidth/12;
	}
	else{
		RobotXpixelPos=(2*CurrentPosCol+1)*ScreenWidth/12;
	}

	if(CurrentPosRow==0){
		RobotYpixelPos=ScreenHeight/8;
	}
	else{
		RobotYpixelPos=(2*CurrentPosRow+1)*ScreenHeight/8;
	}

	if(TargetPosCol==0){
		targetXpixelPos=ScreenWidth/12;
	}
	else{
		targetXpixelPos=(2*TargetPosCol+1)*ScreenWidth/12;
	}

	if(TargetPosRow==0){
		targetYpixelPos=ScreenHeight/8;
	}
	else{
		targetYpixelPos=(2*TargetPosRow+1)*ScreenHeight/8;
	}

	displayStringAt(targetXpixelPos, targetYpixelPos,"*");
	switch(RobotDirection){
	case 0: displayStringAt(RobotXpixelPos,RobotYpixelPos,"^");	break; // Shows That The Robot Is Facing North
	case 1: displayStringAt(RobotXpixelPos,RobotYpixelPos,">"); break; // Facing East
	case 2: displayStringAt(RobotXpixelPos,RobotYpixelPos,"V"); break; // Facing South
	case 3: displayStringAt(RobotXpixelPos,RobotYpixelPos,"<"); break; // Facing West
	default: break;
	}

}


// Draws grid from maze matrix
void GridDraw(){
	int XStart=0;
	int YStart=0;
	int XEnd  =0;
	int YEnd  =0;
	for(int i=0;i<numOfRows;i++){
		for(int j=0;j<numOfCols;j++){
			if(Grid[i][j].NorthWall==1){
				XStart= j   *ScreenWidth/numOfCols;
				YStart=(i+1)*ScreenHeight/numOfRows;
				XEnd  =(j+1)*ScreenWidth/numOfCols;
				YEnd  =(i+1)*ScreenHeight/numOfRows;
				drawLine(XStart,YStart,XEnd,YEnd);
			}
			if (Grid[i][j].EastWall==1){
				XStart=(j+1)*ScreenWidth/numOfCols;
				YStart=(i)*ScreenHeight/numOfRows;
				XEnd  =(j+1)*ScreenWidth/numOfCols;
				YEnd  =(i+1)*ScreenHeight/numOfRows;
				drawLine(XStart,YStart,XEnd,YEnd);
			}
			if (Grid[i][j].WestWall==1){
				XStart= j   *ScreenWidth/numOfCols;
				YStart=(i)*ScreenHeight/numOfRows;
				XEnd  =(j)*ScreenWidth/numOfCols;
				YEnd  =(i+1)*ScreenHeight/numOfRows;
				drawLine(XStart,YStart,XEnd,YEnd);
			}
			if(Grid[i][j].SouthWall==1){
				XStart= j   *ScreenWidth/numOfCols;
				YStart=(i)*ScreenHeight/numOfRows;
				XEnd  =(j+1)*ScreenWidth/numOfCols;
				YEnd  =(i)*ScreenHeight/numOfRows;
				drawLine(XStart,YStart,XEnd,YEnd);
			}
		}
	}
}

// Resets display to update robot on screen
void refreshScreen(){
	eraseDisplay();
	GridDraw();
	DrawBotDisplayStartandEnd();
}

// Turns the robot right and callibrates within the cell
void turnRight(){
	refreshScreen();
	if (RobotDirection < 3){
		RobotDirection++;
	}
	else RobotDirection=0;
	if(getUSDistance(ultra_sonic) < 15){
		recallibrate();
	}
	refreshScreen();
}

// Turns the robot left and callibrates within the cell
void turnLeft(){
	refreshScreen();
	if (RobotDirection > 0){
		RobotDirection--;
	}
	else RobotDirection=3;
	if(getUSDistance(ultra_sonic) < 15){
		recallibrate();
	}
	refreshScreen();
}

// Updates robots position on grid
void goFwd(){
	refreshScreen();
	if (RobotDirection==0)	{   // Going Fwd North
		CurrentPosRow++;
	}
	else if (RobotDirection==1)	{   // Going Fwd East
		CurrentPosCol++;
	}
	else if (RobotDirection==2)	{   // Going Fwd South
		CurrentPosRow--;
	}
	else if (RobotDirection==3)	{   // Going Fwd West
		CurrentPosCol--;
	}
	else {
		CurrentPosRow = 99;
		CurrentPosCol = 99;
	}
	refreshScreen();
}

// Initializes inside grid as empty with wall boundaries on outside
void GridInit(){
	for(int col=0; col<numOfCols; col++){
		Grid[0][col].SouthWall = 1;
		Grid[numOfRows-1][col].NorthWall = 1;
	}

	for(int row=0; row<numOfRows; row++){
		Grid[row][0].WestWall = 1;
		Grid[row][numOfCols-1].EastWall = 1;
	}
}


// Solver
void Solver(){
	// Initial cell position must pre-determined to find the shortest path
	route_records[index_records].x = CurrentPosRow;
	route_records[index_records].y = CurrentPosCol;
	index_records = index_records + 1;

	//turn right every time when starting
	turnRobotRight();
	turnRight();

	while((CurrentPosCol != TargetPosCol) || (CurrentPosRow != TargetPosRow) ){
		// Robot initialized to always look north
		switch(RobotDirection){
		case 0: // facing north
			if(getUSDistance(ultra_sonic) < 15){
				turnRobotLeft();
				turnLeft();
				continue;
			}
			break;
		case 1:  // facing east
			if(getUSDistance(ultra_sonic) < 15){
				turnRobotLeft();
				turnLeft();
				continue;
			}
			break;
		case 2: // facing south
			if(getUSDistance(ultra_sonic) < 15){
				turnRobotLeft();
				turnLeft();
				continue;
			}
			break;
		case 3: // facing west
			if(getUSDistance(ultra_sonic) < 15){
				turnRobotLeft();
				turnLeft();
				continue;
			}
			break;
		default: break;
		}
		// move the robot in both visual and physical world
		move_fwd_cell();
		goFwd();

		// record the cell you visit
		route_records[index_records].x = CurrentPosRow;
		route_records[index_records].y = CurrentPosCol;
		index_records = index_records + 1;

		// turn the robot in both visual and physical world
		turnRobotRight();
		turnRight();
	}
}

// This is the move to target function
void move_to_target(int target){
	while(RobotDirection != target){
		if(RobotDirection > target){
			if(RobotDirection == 3 && target == 0){
				turnRobotRight();
				turnRight();
				}else{
				turnRobotLeft();
				turnLeft();
			}
		}
		else{
			if(RobotDirection == 0 && target == 3){
				turnRobotLeft();
				turnLeft();
				}else{
				turnRobotRight();
				turnRight();
			}
		}
	}
	move_fwd_cell();
	if(getUSDistance(ultra_sonic) < 15){
		recallibrate();
	}
	goFwd();
}

// Initializes route records array and sets all to -1
void init_record(){
	for (int i = 0; i < 58; ++i){
		route_records[i].x = -1;
		route_records[i].y = -1;
	}
}

// Determines the shortest route by pruning
void find_shortest_route(){
	for(int i = 0; i < 58; ++i){
		int counter = 57;
		while(route_records[counter].x == -1 &&  route_records[counter].y == -1){
			counter --;
		}
		while((route_records[i].x != route_records[counter].x || route_records[i].y != route_records[counter].y )){
			counter = counter - 1;
		}
		for(int j = i + 1; j <= counter; ++j){
			route_records[j].x = route_records[j].y = -1;
		}
	}

	for(int i = 0; i < 58; ++i){
		if(route_records[i].x != -1 && route_records[i].y != -1){
			shortest_routes[realsize].x = route_records[i].x;
			shortest_routes[realsize].y = route_records[i].y;
			realsize = realsize + 1;
		}
	}
}


// Determines which direction to return to home after pruning by taking the opposite direction travelled to current spot
void cell_moving(int point_1_x, int point_1_y, int point_2_x, int point_2_y){
	int com = 0;
	if(point_1_x == point_2_x){
		if(point_1_y > point_2_y){
			com = 3;
			//turn west
			}else{
			com = 1;
			//turn east
		}
	}
	else if(point_1_y == point_2_y){
		if(point_1_x > point_2_x){
			com = 2;
			//turn south
			}else{
			com = 0;
			//turn north
		}
	}
	move_to_target(com);
}

// Return to home algorithm
void backward_progressing(){
	for(int i = realsize - 1; i > 0; i--){
		cell_moving(shortest_routes[i].x, shortest_routes[i].y, shortest_routes[i-1].x, shortest_routes[i-1].y);
	}
	int tmp_counter = 0;
	while(tmp_counter <= 10){

		eraseDisplay();
		sleep(500);
		displayCenteredTextLine(3,"Reach The Target");
		sleep(500);
		tmp_counter = tmp_counter + 1;
	}
}

/*----------------------------------------------------------*/
/*            					WallGen                              */
/*----------------------------------------------------------*/

void WallGen (){
	Grid[0][1].EastWall = Grid[0][2].WestWall = 1;
	Grid[0][2].EastWall = Grid[0][3].WestWall = 1;
	Grid[0][3].NorthWall = Grid[1][3].SouthWall = 1;
	Grid[0][4].EastWall = Grid[0][5].WestWall = 1;

	Grid[1][0].EastWall = Grid[1][1].WestWall = 1;
	Grid[1][1].NorthWall = Grid[2][1].SouthWall = 1;
	Grid[1][1].EastWall = Grid[1][2].WestWall = 1;
	Grid[1][3].NorthWall = Grid[2][3].SouthWall = 1;
	Grid[1][4].EastWall = Grid[1][5].WestWall = 1;

	Grid[2][1].NorthWall = Grid[3][1].SouthWall = 1;
	Grid[2][2].EastWall = Grid[2][3].WestWall = 1;
	Grid[2][3].EastWall = Grid[2][4].WestWall = 1;
	Grid[2][5].NorthWall = Grid[3][5].SouthWall = 1;

	Grid[3][1].EastWall = Grid[3][2].WestWall = 1;
	Grid[3][3].EastWall = Grid[3][4].WestWall = 1;
}

// MAIN FUNCTION
task main()
{
	playTone(300, 200);
	sleep(200);
	init_record(); // Begin recording paths
	GridInit(); // Generate surrounding outer Walls
	WallGen();

	Solver();
	find_shortest_route();
	int turn_back = 0;
	while(turn_back <= 3){
		eraseDisplay();
		sleep(100);
		displayCenteredTextLine(3,"Returning");
		playTone(300, 200);

		sleep(200);
		turn_back = turn_back + 1;
	}

	backward_progressing();
	sleep(1000);
}
