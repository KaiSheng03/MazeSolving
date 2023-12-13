#ifndef INITIALIZATION_H
#define INITIALIZATION_H

void funcFrontSensor();
void funcLeftSensor();
void funcRightSensor();

void funcForward();
void funcLeft();
void funcRight();
void funcBackward();

void funcRobotState();

// (Speed, in1, in2, en1_2)
Motor leftMotor(255, 2, 3, 6);
Motor rightMotor(255, 4, 5, 7);

Motion motion(leftMotor, rightMotor);

FrontSensor frontsensor(9, A1, 11);
LeftSensor leftsensor(8, A0, 12);
RightSensor rightsensor(10, A2, 13);

int robotState;
int motionIndex = 0;
const int forwardState = 1;
const int rightState = 2;
const int leftState = 3;
const int backwardState = 4;
const int stopState = 5;
const int returnCallerState = 6;

bool forwardClear;
bool rightClear;
bool leftClear;
bool backwardClear;
bool stopAllowed;

bool forwardAllowed;
bool rightAllowed;
bool leftAllowed;

int row = 0;
int column = 0;
 
const int mazeRow = 12;
const int mazeColumn = 12;

Cell maze[mazeRow][mazeColumn];

static int count = 0;

#endif