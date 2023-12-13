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

TimedAction taskRobotState = TimedAction(1, funcRobotState);

void setup() {
  leftMotor.setup();
  rightMotor.setup();
  frontsensor.setup();
  leftsensor.setup();
  rightsensor.setup();
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
        motion.forward();
        row++;
        Serial.println(row);
        if(row>=mazeRow){
          motion.stop();
          robotState = rightState;
          motionIndex = 1;
          break;
        }
        maze[row][column].setCaller(row-1, column);
        maze[row][column].setCoordinate(row, column);
        maze[row][column].markVisited();
      }

      else if(robotState == rightState){
        motion.forward();
        column++;
        maze[row][column].setCaller(row, column-1);
        maze[row][column].setCoordinate(row, column);
        maze[row][column].markVisited();
      }

      else if(robotState == leftState){
        motion.forward();
        column--;
        maze[row][column].setCaller(row, column+1);
        maze[row][column].setCoordinate(row, column);
        maze[row][column].markVisited();
      }

      else if(robotState == backwardState){
        motion.forward();
        row--;
        maze[row][column].setCaller(row+1, column);
        maze[row][column].setCoordinate(row, column);
        maze[row][column].markVisited();
      }
      if(currentTime - startTime >= 6000){
        motionIndex = 2;
        startTime = currentTime;
      }
      break;

    case 2:
      motion.stop();
      if(robotState == forwardState){
        if(forwardClear && !maze[row+1][column].getVisited()){
          robotState = forwardState;
        }
        else if(rightClear && !maze[row][column+1].getVisited()){
          motion.right();
          robotState = rightState;
        }
        else if(leftClear && maze[row][column-1].getVisited()){
          motion.left();
          robotState = leftState;
        }
        else{
          maze[row][column].markUseless();
        }
      }

      else if(robotState == rightState){
        if(leftClear && !maze[row+1][column].getVisited()){
          motion.left();
          robotState = forwardState;
        }
        else if(forwardClear && !maze[row][column+1].getVisited()){
          robotState = rightState;
        }
        else if(rightClear && !maze[row-1][column].getVisited()){
          motion.right();
          robotState = backwardState;
        }
        else{
          maze[row][column].markUseless();
          motionIndex = 3;
        }
      }

      else if(robotState == leftState){
        if(rightClear && !maze[row+1][column].getVisited()){
          motion.right();
          robotState = forwardState;
        }  
        else if(leftClear && !maze[row-1][column].getVisited()){
          motion.left();
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
          motion.left();
          robotState = rightState;
        }
        else if(forwardClear && !maze[row-1][column].getVisited()){
          robotState = backwardState;
        }
        else if(rightClear && !maze[row][column-1].getVisited()){
          motion.right();
          robotState = leftState; 
        }
      }
      if(currentTime - startTime >= 6000){
        motionIndex = 1;
        startTime = currentTime;
        Serial.println("NOW");
      }
      break;

    case 3: // Called when the robot return back to its caller cell
      Cell caller = maze[row][column].getCaller().operator-(maze[row][column]);
      if(robotState == forwardState){
        if(caller.getRow() == -1){
          motion.backward();
        }
        else if(caller.getRow() == 1){
          motion.right();
          robotState = rightState;
        }
        else if(caller.getRow() == -1){
          motion.left();
          robotState = leftState;
        }
      }

      else if(robotState == rightState){
        if(caller.getColumn() == -1){
          motion.backward();
        }
        else if(caller.getRow() == 1){
          motion.left();
          robotState = forwardState;
        }
        else if(caller.getRow() == -1){
          motion.right();
          robotState = backwardState;
        }
      }

      else if(robotState == leftState){
        if(caller.getColumn() == 1){
          motion.backward();
        }
        else if(caller.getRow() == 1){
          motion.right();
          robotState = forwardState;
        }
        else if(caller.getRow() == -1){
          motion.left();
          robotState = backwardState;
        }
      }

      else if(robotState == backwardState){
        if(caller.getRow() == 1){
          motion.backward();
        }
        else if(caller.getColumn() == 1){
          motion.left();
          robotState = rightState;
        }
        else if(caller.getColumn() == -1){
          motion.right();
          robotState = leftState;
        }
      }
      motionIndex = 1;
      break;
  }
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
