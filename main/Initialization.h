#ifndef INITIALIZATION_H
#define INITIALIZATION_H

void funcFrontSensor();
void funcLeftSensor();
void funcRightSensor();

void funcForward();
void funcLeft();
void funcRight();

void funcRobotState();
void setMaze(const Cell&);

Cell& differenceForPossible(Cell&, Cell&);
//Cell& differenceForPossible(Cell&, Cell&, Cell&);

// (Speed, in1, in2, en1_2)
Motor leftMotor(255, 2, 3, 6);
Motor rightMotor(255, 4, 5, 9);
//IMU imu;

//Motion motion(leftMotor, rightMotor, imu);
Motion motion(leftMotor, rightMotor);

FrontSensor frontsensor(7, A1);
LeftSensor leftsensor(8, A0);
RightSensor rightsensor(10, A2);

int robotState;
int motionIndex = 0;
const PROGMEM int turnLeft = 200;
const PROGMEM int goBack = 300;
const PROGMEM int turnRight = 100;

const PROGMEM int forwardState = 1;
const PROGMEM int rightState = 2;
const PROGMEM int leftState = 3;
const PROGMEM int backwardState = 4;
const PROGMEM int returnCallerState = 6;

bool forwardClear;
bool rightClear;
bool leftClear;

bool needToRight;
bool needToLeft;
bool needToBack;

bool frontPossibleFlag;
bool leftPossibleFlag;
bool rightPossibleFlag;

int robotMode = 0;
const PROGMEM int leftMode = 1;
const PROGMEM int rightMode = 2;

int row = 0;
int column = 0;

int possibleCellCount = 0;
int differenceRow;
int differenceColumn;

const PROGMEM int mazeRow = 12;
const PROGMEM int mazeColumn = 12;

Cell maze[mazeRow][mazeColumn];
Cell target[4];
Cell frontPossible;
Cell leftPossible;
Cell rightPossible;
Cell route;

int columnIncrement;
const PROGMEM int leftModeColumnIncrement = 1;
const PROGMEM int rightModeColumnIncrement = -1;
int targetIndex = 0;

#endif
