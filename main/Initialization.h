#ifndef INITIALIZATION_H
#define INITIALIZATION_H

void funcFrontSensor();
void funcLeftSensor();
void funcRightSensor();

void funcForward();
void funcLeft();
void funcRight();

void funcIMU();
void driveForward();
void driveBackward();

void funcRobotState();
void setMaze(const Cell&);

Cell& differenceForPossible(Cell&, Cell&);

// (Speed, in1, in2, en1_2)
Motor leftMotor(230, 2, 3, 6);
Motor rightMotor(230, 4, 5, 9);

Motion motion(leftMotor, rightMotor);

FrontSensor frontsensor(30, A1);
LeftSensor leftsensor(13, A0);
RightSensor rightsensor(14, A2);

IMU imu;  // Create IMU object
unsigned long startTime = 0;

int robotState;
int motionIndex = 0;
const PROGMEM int checkRobotMode = 100;
const PROGMEM int drive = 200;
const PROGMEM int checkState = 300;
const PROGMEM int returnCaller = 400;
const PROGMEM int turnLeft = 500;
const PROGMEM int goBack = 600;
const PROGMEM int adjustmentAfterTurn = 700;
const PROGMEM int turnRight = 800;

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

double targetAngle = 0.0;

const PROGMEM int mazeRow = 12;
const PROGMEM int mazeColumn = 12;

//const PROGMEM int mazeRow = 6;
//const PROGMEM int mazeColumn = 6;

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
int interval = 775;
int backInterval = 775;
#endif
