#include "Motor.h"
#include "Motion.h"
#include "FrontSensor.h"
#include "LeftSensor.h"
#include "RightSensor.h"
#include "Cell.h"
#include "Initialization.h"
#include "TimedAction.h"

TimedAction taskFrontSensor = TimedAction(1, funcFrontSensor);
TimedAction taskLeftSensor = TimedAction(1, funcLeftSensor);
TimedAction taskRightSensor = TimedAction(1, funcRightSensor);

TimedAction taskForward = TimedAction(1, funcForward);
TimedAction taskLeft = TimedAction(1, funcLeft);
TimedAction taskRight = TimedAction(1, funcRight);
//TimedAction taskBackward = TimedAction(1000, funcBackward);

TimedAction taskRobotState = TimedAction(2000, funcRobotState);

void setup() {
  leftMotor.setup();
  rightMotor.setup();
  frontsensor.setup();
  leftsensor.setup();
  rightsensor.setup();

  target.setCoordinate(7, 7);

  Serial.begin(115200);
}

unsigned long startTime = 0;
unsigned long startTime2 = 0;

void loop(){ 
  taskRobotState.check();
  taskFrontSensor.check();
  taskLeftSensor.check();
  taskRightSensor.check();
  taskForward.check();
  taskLeft.check();
  taskRight.check();
  //taskBackward.check();
}

void funcRobotState(){
  unsigned long currentTime = millis();
  switch(motionIndex){
    case 0:
      robotState = forwardState;
      motionIndex = 1;
      break;

    case 1: // Drive state
      if(robotState == forwardState){
        Serial.println("Forward State");
        motion.forward();
        row++;
        maze[row][column].setCaller(row-1, column);
        setMaze(maze[row][column]);
      }

      else if(robotState == rightState){
        if(robotMode == leftMode){
          column--;
          maze[row][column].setCaller(row, column+1);
        }
        else if(robotMode == rightMode){
          column++;
          maze[row][column].setCaller(row, column-1);
        }
        Serial.println("Right State");
        motion.forward();
        column++;
        setMaze(maze[row][column]);
      }

      else if(robotState == leftState){
        if(robotMode == leftMode){
          column++;
          maze[row][column].setCaller(row, column-1);
        }
        else if(robotMode == rightMode){
          column--;
          maze[row][column].setCaller(row, column+1);
        }
        Serial.println("Left State");
        motion.forward();
        column--;
        setMaze(maze[row][column]);
      }

      else if(robotState == backwardState){
        Serial.println("Backward State");
        motion.forward();
        row--;
        
        maze[row][column].setCaller(row+1, column);
        setMaze(maze[row][column]);
      }
      if(currentTime - startTime >= 2000){
        motionIndex = 2;
        startTime = currentTime;
      }
      break;

    case 1000:
      motion.stop();
      if(rightClear){
        motion.right();
        robotMode = rightMode;
      }
      else if(leftClear){
        motion.left();
        robotMode = leftMode;
      }
      else{
        motionIndex = 1;
      }
      break;

    case 2: // Buffer stage for condition checking
      motion.stop();
      if(target.getRow() - maze[row][column].getRow() > 0){
        //Going upward
      }

      else if(target.getRow() - maze[row][column].getRow() < 0){
        //Going downward
      }

      else if(target.getColumn() - maze[row][column].getColumn() > 0){
        //Going right
      }

      else if(target.getColumn() - maze[row][column].getColumn() < 0){
        //Going left
      }

      if(robotState == forwardState){
        if(forwardClear && !maze[row+1][column].getVisited()){
          robotState = forwardState;
        }
        else if(rightClear && leftClear){
          if(target.getColumn() - maze[row][column].getColumn() > 0){
            //Going right
            needToRight = true;
            robotState = rightState;
          }
          else if(target.getColumn() - maze[row][column].getColumn() < 0){
            //Going left
            needToLeft = true;
            robotState = leftState;
          }
        }
        else if(rightClear && !maze[row][column+1].getVisited()){
          //motion.right();
          needToRight = true;
          robotState = rightState;
        }
        else if(leftClear && !maze[row][column-1].getVisited()){
          //motion.left();
          needToLeft = true;
          robotState = leftState;
        }
        else{
          maze[row][column].markUseless();
        }
      }

      else if(robotState == rightState){
        if(leftClear && !maze[row+1][column].getVisited()){
          //motion.left();
          needToLeft = true;
          robotState = forwardState;
        }
        else if(forwardClear && !maze[row][column+1].getVisited()){
          robotState = rightState;
        }
        else if(rightClear && !maze[row-1][column].getVisited()){
          //motion.right();
          needToRight = true;
          robotState = backwardState;
        }
        else{
          maze[row][column].markUseless();
          motionIndex = 3;
        }
      }

      else if(robotState == leftState){
        if(rightClear && !maze[row+1][column].getVisited()){
          //motion.right();
          needToRight = true;
          robotState = forwardState;
        }  
        else if(leftClear && !maze[row-1][column].getVisited()){
          //motion.left();
          needToLeft = true;
          robotState = backwardState;
        }
        else if(forwardClear && !maze[row][column-1].getVisited()){
          robotState = leftState;
        }      
        else{
          maze[row][column].markUseless();
          motionIndex = 3;
        }
      }

      else if(robotState == backwardState){
        if(leftClear && !maze[row][column+1].getVisited()){
          //motion.left();
          needToLeft = true;
          robotState = rightState;
        }
        else if(forwardClear && !maze[row-1][column].getVisited()){
          robotState = backwardState;
        }
        else if(rightClear && !maze[row][column-1].getVisited()){
          //motion.right();
          needToRight = true;
          robotState = leftState; 
        }
      }
      if(currentTime - startTime >= 2000){
        if(needToLeft){
          motionIndex = turnLeft;
        }
        else if(needToRight){
          motionIndex = turnRight;
        }
        else{
          motionIndex = 1;
        }
        needToLeft = false;
        needToRight = false;
        startTime = currentTime;
        Serial.println("NOW");
      }
      break;

    case turnRight:
      Serial.println("Turn Right");
      motion.right();
      if(currentTime - startTime >= 2000){
        motionIndex = 2;
        startTime = currentTime;
        Serial.println("NOW");
      }
      break;

    case turnLeft:
      Serial.println("Turn Left");
      motion.left();
      if(currentTime - startTime >= 2000){
        motionIndex = 2;
        startTime = currentTime;
        Serial.println("NOW");
      }
      break;

    case goBack:
      Serial.println("Go Back");
      motion.backward();
      if(currentTime - startTime >= 2000){
        motionIndex = 2;
        startTime = currentTime;
        Serial.println("NOW");
      }
      break;

    case 3: // Called when the robot return back to its caller cell
      Cell caller = maze[row][column].getCaller().operator-(maze[row][column]);
      if(robotState == forwardState){
        if(caller.getRow() == -1){
          //motion.backward();
          needToBack = true;
        }
        else if(caller.getRow() == 1){
          //motion.right();
          needToRight = true;
          robotState = rightState;
        }
        else if(caller.getRow() == -1){
          //motion.left();
          needToLeft = true;
          robotState = leftState;
        }
      }

      else if(robotState == rightState){
        if(caller.getColumn() == -1){
          //motion.backward();
          needToBack = true;
        }
        else if(caller.getRow() == 1){
          //motion.left();
          needToLeft = true;
          robotState = forwardState;
        }
        else if(caller.getRow() == -1){
          //motion.right();
          needToRight = true;
          robotState = backwardState;
        }
      }

      else if(robotState == leftState){
        if(caller.getColumn() == 1){
          //motion.backward();
          needToBack = true;
        }
        else if(caller.getRow() == 1){
          //motion.right();
          needToRight = true;
          robotState = forwardState;
        }
        else if(caller.getRow() == -1){
          //motion.left();
          needToLeft = true;
          robotState = backwardState;
        }
      }

      else if(robotState == backwardState){
        if(caller.getRow() == 1){
          //motion.backward();
          needToBack = true;
        }
        else if(caller.getColumn() == 1){
          //motion.left();
          needToLeft = true;
          robotState = rightState;
        }
        else if(caller.getColumn() == -1){
          //motion.right();
          needToRight = true;
          robotState = leftState;
        }
      }
      if(currentTime - startTime >= 2000){
        if(needToLeft){
          motionIndex = turnLeft;
        }
        else if(needToRight){
          motionIndex = turnRight;
        }
        else if(needToBack){
          motionIndex = goBack;
        }
        motionIndex = 2;
        startTime = currentTime;
        Serial.println("NOW");
      }
      break;
  }
}

void setMaze(const Cell & inputCell){
  inputCell.setCoordinate(row, column);
  inputCell.markVisited();
  inputCell.printCoordinate();
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
