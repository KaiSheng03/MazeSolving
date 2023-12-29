#include "Motor.h"
#include "Motion.h"
#include "FrontSensor.h"
#include "LeftSensor.h"
#include "RightSensor.h"
#include "Cell.h"
#include "IMU.h"
#include "Initialization.h"
#include "TimedAction.h"

TimedAction taskFrontSensor = TimedAction(1, funcFrontSensor);
TimedAction taskLeftSensor = TimedAction(1, funcLeftSensor);
TimedAction taskRightSensor = TimedAction(1, funcRightSensor);

TimedAction taskForward = TimedAction(1, funcForward);
TimedAction taskLeft = TimedAction(1, funcLeft);
TimedAction taskRight = TimedAction(1, funcRight);

TimedAction taskIMU = TimedAction(1, funcIMU);

TimedAction taskRobotState = TimedAction(1, funcRobotState);

void setup() {
  Serial.begin(115200);
  
  leftMotor.setup();
  rightMotor.setup();
  frontsensor.setup();
  leftsensor.setup();
  rightsensor.setup();
  imu.setup();

  //Set the maze target coordinates
  //target[0].setCoordinate(5, 6);
  //target[1].setCoordinate(5, 7);
  //target[2].setCoordinate(6, 6);
  //target[3].setCoordinate(6, 7);
  
  target[0].setCoordinate(2, 2);
  target[1].setCoordinate(2, 3);
  target[2].setCoordinate(3, 2);
  target[3].setCoordinate(3, 3);
  
  //Set the maze coordinates
  for(int i=0; i<mazeRow; i++){
    for(int j=0; j<mazeColumn; j++){
      maze[i][j].setCoordinate(i, j);
    }
  }
}


void loop(){ 
  taskRobotState.check();
  taskFrontSensor.check();
  taskLeftSensor.check();
  taskRightSensor.check();
  taskIMU.check();
  taskForward.check();
  taskLeft.check();
  taskRight.check();
}

void funcRobotState(){
  unsigned long currentTime = millis();
  switch(motionIndex){
    case 0: // Initialization 
      robotState = forwardState;
      motionIndex = checkRobotMode;
      break;

    case drive: // Drive forward state
      driveForward();
      if(currentTime - startTime >= interval){ // Time reached
        if(robotState == forwardState){ // Robot facing north direction
          row++;
          //Serial.println("Forward State");
          if(!maze[row][column].getVisited()){ // Check if current coordinate is a visited cell
            maze[row][column].setCaller(row-1, column); // Only set the caller when it is not visited
            setMaze(maze[row][column]);
          }
        }

        else if(robotState == rightState){ // Robot facing east direction
          column -= columnIncrement;
          if(!maze[row][column].getVisited()){
            maze[row][column].setCaller(row, column+columnIncrement);
            //Serial.println("Right State");
            setMaze(maze[row][column]);
          }
        }

        else if(robotState == leftState){ // Robot facing west direction
          column += columnIncrement;
          if(!maze[row][column].getVisited()){
            maze[row][column].setCaller(row, column-columnIncrement);
            //Serial.println("Left State");
            setMaze(maze[row][column]);
          }
        }

        else if(robotState == backwardState){ // Robot facing south direction
          row--;
          //Serial.println("Backward State");
          if(!maze[row][column].getVisited()){
            maze[row][column].setCaller(row+1, column);
            setMaze(maze[row][column]);
          }
        }

        // Check robot mode
        if(robotMode == 0 ){ // Neither left nor right mode
          motionIndex = checkRobotMode;
        }
        else{ // Robot is set to either left or right mode
          motionIndex = checkState;
        }
        startTime = currentTime;
      }
      break;

    // Check and set the robot to left or right mode
    case checkRobotMode:
      motion.stop();
      if(currentTime - startTime >= interval){
        if(rightClear){ // If right is cleared, then the robot is set to right mode
          robotMode = rightMode;
          columnIncrement = rightModeColumnIncrement;
          Serial.println("RIGHT MODE");
        }

        else if(leftClear){ // If left is cleared, then the robot is set to left mode
          robotMode = leftMode;
          columnIncrement = leftModeColumnIncrement;
          Serial.println("LEFT MODE");
        }

        if(robotMode == 0){ // Neither left nor right mode
          motionIndex = drive;
        }
        else{
          motionIndex = checkState;
        }
        startTime = currentTime;
      }
      break;

    case checkState: // Buffer stage for condition checking
      motion.stop(); // Stop the robot for buffer session
      for(int i=0; i<4; i++){
        if(maze[row][column] == target[i]){
          motion.stop();
          taskRobotState.disable();
        }
      }
      if(currentTime - startTime >= interval){
        if(robotState == forwardState){ // Robot facing north
          if(forwardClear && !maze[row+1][column].getVisited()){ // If robot's forward is cleared and the cell infront of it is not visited yet
            frontPossibleFlag = true; // Set the front possible as true
            frontPossible = maze[row+1][column];
            possibleCellCount += 1; // Increment the number of junction
          }
          if(leftClear && !maze[row][column+columnIncrement].getVisited()){ // If robot's left is cleared and the cell left of it is not visited
            leftPossibleFlag = true; // Set the left possible as true
            leftPossible = maze[row][column+columnIncrement]; 
            possibleCellCount += 1; // Increment the number of junction
          }
          if(rightClear && !maze[row][column-columnIncrement].getVisited()){ // If robot's right is cleared and the cell right of it is not visited
            rightPossibleFlag = true; // Set the right possible as true
            rightPossible = maze[row][column-columnIncrement];
            possibleCellCount +=1; // Increment the number of junction
          }
          if(possibleCellCount>1){ // If number of junction is more than 1
          // If number of junction is 2
            if(frontPossibleFlag && leftPossibleFlag && !rightPossibleFlag){ // If junction is front and left of the robot
              route = differenceForPossible(frontPossible, leftPossible); // Calculate which cell is nearest to the target
            }
            else if(frontPossibleFlag && rightPossibleFlag && !leftPossibleFlag){ // If junction is front and right of the robot
              route = differenceForPossible(frontPossible, rightPossible); // Calculate which cell is nearest to the target
            }
            else if(leftPossibleFlag && rightPossibleFlag && !frontPossibleFlag){ // If junction is left and right of the robot
              route = differenceForPossible(leftPossible, rightPossible); // Calculate which cell is nearest to the target
            }
            else{ // IF THE NUMBER OF JUNCTION IS 3
              robotState = forwardState; // Remain the direction
            } 

            // Check which decision cell has been made
            if(route == frontPossible){ // If front of the robot is the decision
              robotState = forwardState; // Set the direction of robot towards north
            }
            else if(route == leftPossible){ // If left of the robot is the decision
              robotState = leftState; // Set the direction of robot towards west
              needToLeft = true; // The robot need to turn left
            }
            else if(route == rightPossible){ // If right of the robot is the decision
              robotState = rightState; // Set the direction of the robot towards east
              needToRight = true; // The robot need to turn right
            }
            else{
              robotState = forwardState;
            }         
          }

          // If number of junction is 1
          else if(possibleCellCount == 1){
            if(frontPossibleFlag){ // If front is the only possible cell
              robotState = forwardState; // Set the robot direction to north
            }
            else if(leftPossibleFlag){ // If left is the only possible cell
              robotState = leftState; // Set the robot direction to west
              needToLeft = true; // The robot need to turn left
            }
            else if(rightPossibleFlag){ // If right is the only possible cell
              robotState = rightState; // Set the robot direction to east
              needToRight = true; // The robot need to turn right
            }
          }

          // If number of junction is 0 (NO WAY TO GO)
          else if(possibleCellCount == 0){
            maze[row][column].markUseless();
            needToBack = true; // Need to go back to PREVIOUS CELL (CALLER)
          }
        }

        else if(robotState == rightState){ // Robot facing east
          if(leftClear && !maze[row+1][column].getVisited()){
            leftPossibleFlag = true;
            leftPossible = maze[row+1][column];
            possibleCellCount += 1;
          }
          if(forwardClear && !maze[row][column-columnIncrement].getVisited()){
            frontPossibleFlag = true;
            frontPossible = maze[row][column-columnIncrement];
            possibleCellCount += 1;
          }
          if(rightClear && !maze[row-1][column].getVisited()){
            rightPossibleFlag = true;
            rightPossible = maze[row-1][column];
            possibleCellCount +=1;
          }
          
          if(possibleCellCount>1){
            if(frontPossibleFlag && leftPossibleFlag){
              route = differenceForPossible(frontPossible, leftPossible);
            }
            else if(frontPossibleFlag && rightPossibleFlag){
              route = differenceForPossible(frontPossible, rightPossible);
            }
            else if(leftPossibleFlag && rightPossibleFlag){
              route = differenceForPossible(leftPossible, rightPossible);
            }
            else{
              robotState = rightState;
            }  

            if(route == frontPossible){
              robotState = rightState;
            }
            else if(route == leftPossible){
              robotState = forwardState;
              needToLeft = true;
            }
            else if(route == rightPossible){
              robotState = backwardState;
              needToRight = true;
            }
            else{
              robotState = rightState;
            }        
          }

          else if(possibleCellCount == 1){
            if(frontPossibleFlag){
              robotState = rightState;
            }
            else if(leftPossibleFlag){
              robotState = forwardState;
              needToLeft = true;
            }
            else if(rightPossibleFlag){
              robotState = backwardState;
              needToRight = true;
            }
          }

          else if(possibleCellCount == 0){
            maze[row][column].markUseless();
            needToBack = true;
          }
        }

        else if(robotState == leftState){
          if(rightClear && !maze[row+1][column].getVisited()){
            rightPossibleFlag = true;
            rightPossible = maze[row+1][column];
            possibleCellCount +=1;
          }  
          if(leftClear && !maze[row-1][column].getVisited()){
            leftPossibleFlag = true;
            leftPossible = maze[row-1][column];
            possibleCellCount += 1;
          }
          if(forwardClear && !maze[row][column+columnIncrement].getVisited()){
            frontPossibleFlag = true;
            frontPossible = maze[row][column+columnIncrement];
            possibleCellCount += 1;
          }  

          if(possibleCellCount>1){
            if(frontPossibleFlag && leftPossibleFlag && !rightPossibleFlag){
              route = differenceForPossible(frontPossible, leftPossible);
            }
            else if(frontPossibleFlag && rightPossibleFlag && !leftPossibleFlag){
              route = differenceForPossible(frontPossible, rightPossible);
            }
            else if(leftPossibleFlag && rightPossibleFlag && !frontPossibleFlag){
              route = differenceForPossible(leftPossible, rightPossible);
            }
            else{
              robotState = leftState;
            }

            if(route == frontPossible){
              robotState = leftState;
            }
            else if(route == leftPossible){
              robotState = backwardState;
              needToLeft = true;
            }
            else if(route == rightPossible){
              robotState = forwardState;
              needToRight = true;
            }
            else{
              robotState = leftState;
            }          
          }
          else if(possibleCellCount == 1){
            if(frontPossibleFlag){
              robotState = leftState;
            }
            else if(leftPossibleFlag){
              robotState = backwardState;
              needToLeft = true;
            }
            else if(rightPossibleFlag){
              robotState = forwardState;
              needToRight = true;
            }
          }

          else if(possibleCellCount == 0){
            maze[row][column].markUseless();
            needToBack = true;
          }
        }

        else if(robotState == backwardState){
          if(leftClear && !maze[row][column-columnIncrement].getVisited()){
            leftPossibleFlag = true;
            leftPossible = maze[row][column-columnIncrement];
            possibleCellCount += 1;
          }
          if(forwardClear && !maze[row-1][column].getVisited()){
            frontPossibleFlag = true;
            frontPossible = maze[row-1][column];
            possibleCellCount += 1;
          }
          if(rightClear && !maze[row][column+columnIncrement].getVisited()){
            rightPossibleFlag = true;
            rightPossible = maze[row][column+columnIncrement];
            possibleCellCount +=1; 
          }

          if(possibleCellCount>1){
            if(frontPossibleFlag && leftPossibleFlag && !rightPossibleFlag){
              route = differenceForPossible(frontPossible, leftPossible);
            }
            else if(frontPossibleFlag && rightPossibleFlag && !leftPossibleFlag){
              route = differenceForPossible(frontPossible, rightPossible);
            }
            else if(leftPossibleFlag && rightPossibleFlag && !frontPossibleFlag){
              route = differenceForPossible(leftPossible, rightPossible);
            }
            else{
              robotState = backwardState;
            }

            if(route == frontPossible){
              robotState = backwardState;
            }
            else if(route == leftPossible){
              robotState = rightState;
              needToLeft = true;
            }
            else if(route == rightPossible){
              robotState = leftState;
              needToRight = true;
            }
            else{
              robotState =  backwardState;
            }          
          }
          else if(possibleCellCount == 1){
            if(frontPossibleFlag){
              robotState = backwardState;
            }
            else if(leftPossibleFlag){
              robotState = rightState;
              needToLeft = true;
            }
            else if(rightPossibleFlag){
              robotState = leftState;
              needToRight = true;
            }
          }
          else if(possibleCellCount == 0){
            maze[row][column].markUseless();
            needToBack = true;
          }
        }
        
        // CHECK WHERE TO ROBOT NEED TO TURN OR REVERSE BACK, OR JUST KEEP GOING FORWARD
        if(needToLeft){ // If robot need to turn left
          imu.setTarget(targetAngle+=89); // Set the target of the imu sensor
          motionIndex = turnLeft; // Change the motion to turn left
          Serial.println(imu.getTarget()); 
        }
        else if(needToRight){ // If robot need to turn right
          imu.setTarget(targetAngle-=89); // Set the target of the imu sensor
          motionIndex = turnRight; // Change the motion to turn right
          Serial.println(imu.getTarget());
        }
        else if(needToBack){ // If robot need to reverse back
          motionIndex = returnCaller; // Motion index 3 is for the robot to go back to its PREVIOUS CELL (CALLER)
        }
        else{
          motionIndex = drive;  // Motion index 1 for the robot to keep driving forward
        }

        // RESET
        needToLeft = false;
        needToRight = false;
        needToBack = false;
        frontPossibleFlag = false;
        leftPossibleFlag = false;
        rightPossibleFlag = false;
        possibleCellCount = 0;
        startTime = currentTime;
      }
    break;

    // TURN RIGHT
    case turnRight:
      if(imu.getZ()>imu.getTarget()){
        motion.right();
        Serial.println("Turn Right");
        Serial.println(imu.getTarget());
      }
      else{
        motion.stop();
        motionIndex = adjustmentAfterTurn;
        startTime = currentTime;
      }
      break;

    // TURN LEFT
    case turnLeft:
      if(imu.getZ()<imu.getTarget()){
        //motion.left(165, 220);
        motion.left();
        Serial.println("Turn Left");
        Serial.println(imu.getTarget());
      }
      else{
        motion.stop();
        startTime = currentTime;
        motionIndex = adjustmentAfterTurn;
      }
      break;

    // ADJUSTMENT AFTER TURNING LEFT OR RIGHT
    case adjustmentAfterTurn: 
      driveBackward();
      Serial.println("adjust");
      if(currentTime - startTime >= 640){
        startTime = currentTime;
        motion.stop();
        motionIndex = checkState;
      }
      break;

    // REVERSE THE ROBOT 
    case goBack:
      driveBackward(); // Drive robot backward
      if(currentTime - startTime >= backInterval){
        Serial.println("Go Back");
        if(robotState == forwardState){ // If robot is facing north
          row--;
        }
        else if(robotState == rightState){ // If robot is facing east
          column += columnIncrement;
        }
        else if(robotState == leftState){ // If robot is facing west
          column -= columnIncrement;
        }
        else if(robotState == backwardState){ // If robot is facing south
          row++;
        }
        motionIndex = checkState;
        startTime = currentTime;
      }
      break;

    case returnCaller: // Called when the robot return back to its caller cell
      Cell caller = maze[row][column].getCaller().operator-(maze[row][column]); // CALCULATE WHERE THE DIRECTION OF PREVIOUS CELL (CALLER) IS 
      if(robotState == forwardState){ // If robot is facing north
        if(caller.getRow() == -1){ 
          needToBack = true;
        }
        else if(caller.getRow() == 1){
          robotState = forwardState;
        }
        else if(caller.getColumn() == -columnIncrement){ 
          needToRight = true;
          robotState = rightState;
        }
        else if(caller.getColumn() == columnIncrement){
          needToLeft = true;
          robotState = leftState;
        }
      }

      else if(robotState == rightState){ // If robot is facing east
        if(caller.getColumn() == columnIncrement){
          needToBack = true;
        }
        else if(caller.getColumn() == -columnIncrement){
          robotState = rightState;
        }
        else if(caller.getRow() == 1){
          needToLeft = true;
          robotState = forwardState;
        }
        else if(caller.getRow() == -1){
          needToRight = true;
          robotState = backwardState;
        }
      }

      else if(robotState == leftState){ // If robot is facing west
        if(caller.getColumn() == -columnIncrement){
          needToBack = true;
        }
        else if(caller.getColumn() == columnIncrement){
          robotState = leftState;
        }
        else if(caller.getRow() == 1){
          needToRight = true;
          robotState = forwardState;
        }
        else if(caller.getRow() == -1){
          needToLeft = true;
          robotState = backwardState;
        }
      }

      else if(robotState == backwardState){ // If robot is facing south
        if(caller.getRow() == 1){
          needToBack = true;
        }
        else if(caller.getRow() == -1){
          robotState = backwardState;
        }
        else if(caller.getColumn() == -columnIncrement){
          needToLeft = true;
          robotState = rightState;
        }
        else if(caller.getColumn() == columnIncrement){
          needToRight = true;
          robotState = leftState;
        }
      }
      if(currentTime - startTime >= interval){
        if(needToLeft){
          imu.setTarget(targetAngle+=89);
          motionIndex = turnLeft;
        }
        else if(needToRight){
          imu.setTarget(targetAngle-=89);
          motionIndex = turnRight;
        }
        else if(needToBack){
          motionIndex = goBack;
        }
        else{
          motionIndex = drive;
        }
        needToLeft = false;
        needToRight = false;
        needToBack = false;
        startTime = currentTime;
      }
      break;
  }  
}

void setMaze(const Cell & inputCell){
  inputCell.setCoordinate(row, column);
  inputCell.markVisited();
  inputCell.printCoordinate();
}

Cell& differenceForPossible(Cell& inputCell, Cell& inputCell2){
  for(int i=0; i<4; i++){
    int differenceRow1 = abs(target[i].getRow() - inputCell.getRow());
    int differenceRow2 = abs(target[i].getRow() - inputCell2.getRow());

    int differenceColumn1 = abs(target[i].getColumn() - inputCell.getColumn());
    int differenceColumn2 = abs(target[i].getColumn() - inputCell2.getColumn());

    int difference1 = differenceRow1 + differenceColumn1;
    int difference2 = differenceRow2 + differenceColumn2;

    if(difference1 < difference2){
      return inputCell;
    }
    else if(difference2 < difference1){
      return inputCell2;
    }
  }
  return inputCell;
}

void funcFrontSensor(){
  frontsensor.readDuration();
  frontsensor.calculateDistance();
}

void funcLeftSensor(){
  leftsensor.readDuration();
  leftsensor.calculateDistance();
}

void funcRightSensor(){
  rightsensor.readDuration();
  rightsensor.calculateDistance();
}

void funcForward(){
  if(!frontsensor.checkObstacle()){
    forwardClear = true;
  }
  else{
    forwardClear = false;
  }
}

void funcLeft(){
  if(!leftsensor.checkObstacle()){
    leftClear = true;
  }
  else{
    leftClear = false;
  }
}

void funcRight(){
  if(!rightsensor.checkObstacle()){
    rightClear = true;
  }
  else{
    rightClear = false;
  }
}

void funcIMU(){
  imu.updateAngles();
}

void driveForward(){
  imu.setTarget(targetAngle);
  Serial.println(imu.getTarget());
  imu.calculatePID();
  imu.calculateMotorSpeed();
  motion.forward(imu.getLeftSpeed(), imu.getRightSpeed());
}

void driveBackward(){
  imu.setTarget(targetAngle);
  imu.calculatePID();
  imu.calculateMotorSpeed();
  motion.backward(imu.getRightSpeed(), imu.getLeftSpeed());
}