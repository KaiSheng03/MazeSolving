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

TimedAction taskRobotState = TimedAction(interval, funcRobotState);

void setup() {
  //imu.setup();
  leftMotor.setup();
  rightMotor.setup();
  frontsensor.setup();
  leftsensor.setup();
  rightsensor.setup();
  imu.setup();


  //target[0].setCoordinate(5, 6);
  //target[1].setCoordinate(5, 7);
  //target[2].setCoordinate(6, 6);
  //target[3].setCoordinate(6, 7);
  
  target[0].setCoordinate(2, 2);
  target[1].setCoordinate(3, 2);
  target[2].setCoordinate(3, 3);
  target[3].setCoordinate(2, 3);
  

  for(int i=0; i<mazeRow; i++){
    for(int j=0; j<mazeColumn; j++){
      maze[i][j].setCoordinate(i, j);
    }
  }
  Serial.begin(115200);
}

unsigned long startTime = 0;

void loop(){ 
  taskRobotState.check();
  taskFrontSensor.check();
  taskLeftSensor.check();
  taskRightSensor.check();
  taskForward.check();
  taskLeft.check();
  taskRight.check();
}

void funcRobotState(){
  unsigned long currentTime = millis();
  switch(motionIndex){
    case 0:
      robotState = forwardState;
      motionIndex = 1000;
      break;

    case 1: // Drive state
      if(robotState == forwardState){
        row++;
        motion.forward();
        //Serial.println("Forward State");
        maze[row][column].setCaller(row-1, column);
        setMaze(maze[row][column]);
      }

      else if(robotState == rightState){
        column -= columnIncrement;
        maze[row][column].setCaller(row, column+columnIncrement);
        //Serial.println("Right State");
        motion.forward();
        setMaze(maze[row][column]);
      }

      else if(robotState == leftState){
        column += columnIncrement;
        maze[row][column].setCaller(row, column-columnIncrement);
        //Serial.println("Left State");
        motion.forward();
        setMaze(maze[row][column]);
      }

      else if(robotState == backwardState){
        row--;
        //Serial.println("Backward State");
        motion.forward();
        maze[row][column].setCaller(row+1, column);
        setMaze(maze[row][column]);
      }

      if(currentTime - startTime >= interval){
        if(robotMode == 0 ){
          motionIndex = 1000;
        }
        else{
          motionIndex = 2;
        }
        startTime = currentTime;
      }
      break;

    case 1000:
      motion.stop();
      if(rightClear){
        robotMode = rightMode;
        columnIncrement = rightModeColumnIncrement;
        Serial.println("RIGHT MODE");
      }
      else if(leftClear){
        robotMode = leftMode;
        columnIncrement = leftModeColumnIncrement;
        Serial.println("LEFT MODE");
      }
      if(currentTime - startTime >= interval){
        if(robotMode == 0){
          motionIndex = 1;
        }
        else{
          motionIndex = 2;
        }
        startTime = currentTime;
      }
      break;

    case 2: // Buffer stage for condition checking
      targetIndex = 0;
      motion.stop();
      if(robotState == forwardState){
        if(forwardClear && !maze[row+1][column].getVisited()){
          frontPossibleFlag = true;
          frontPossible = maze[row+1][column];
          possibleCellCount += 1;
        }
        if(leftClear && !maze[row][column+columnIncrement].getVisited()){
          leftPossibleFlag = true;
          leftPossible = maze[row][column+columnIncrement];
          possibleCellCount += 1;
        }
        if(rightClear && !maze[row][column-columnIncrement].getVisited()){
          rightPossibleFlag = true;
          rightPossible = maze[row][column-columnIncrement];
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
            //route = differenceForPossible(frontPossible, leftPossible, rightPossible);
            robotState = forwardState;
          } 

          if(route == frontPossible){
            robotState = forwardState;
          }
          else if(route == leftPossible){
            robotState = leftState;
            needToLeft = true;
          }
          else if(route == rightPossible){
            robotState = rightState;
            needToRight = true;
          }
          else{
            robotState = forwardState;
          }         
        }
        else if(possibleCellCount == 1){
          if(frontPossibleFlag){
            robotState = forwardState;
          }
          else if(leftPossibleFlag){
            robotState = leftState;
            needToLeft = true;
          }
          else if(rightPossibleFlag){
            robotState = rightState;
            needToRight = true;
          }
        }
        else{
          maze[row][column].markUseless();
          needToBack = true;
        }
      }

      else if(robotState == rightState){
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
            //route = differenceForPossible(frontPossible, leftPossible, rightPossible);
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
        else if(leftClear && !maze[row-1][column].getVisited()){
          leftPossibleFlag = true;
          leftPossible = maze[row-1][column];
          possibleCellCount += 1;
        }
        else if(forwardClear && !maze[row][column+columnIncrement].getVisited()){
          frontPossibleFlag = true;
          frontPossible = maze[row][column-1];
          possibleCellCount += 1;
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
            //route = differenceForPossible(frontPossible, leftPossible, rightPossible);
            robotState = leftState;
            break;
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
        else if(forwardClear && !maze[row-1][column].getVisited()){
          frontPossibleFlag = true;
          frontPossible = maze[row][column+columnIncrement];
          possibleCellCount += 1;
        }
        else if(rightClear && !maze[row][column+columnIncrement].getVisited()){
          rightPossibleFlag = true;
          rightPossible = maze[row+1][column];
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
            //route = differenceForPossible(frontPossible, leftPossible, rightPossible);
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
        else{
          maze[row][column].markUseless();
          needToBack = true;
        }
      }
      if(currentTime - startTime >= interval){
        if(needToLeft){
          motionIndex = turnLeft;
        }
        else if(needToRight){
          motionIndex = turnRight;
        }
        else if(needToBack){
          motionIndex = 3;
        }
        else{
          motionIndex = 1;
        }
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

    case turnRight:
      Serial.println("Turn Right");
      motion.right();
      if(currentTime - startTime >= interval){
        motionIndex = 1;
        startTime = currentTime;
      }
      break;

    case turnLeft:
      Serial.println("Turn Left");
      motion.left();
      if(currentTime - startTime >= interval){
        motionIndex = 1;
        startTime = currentTime;
      }
      break;

    case goBack:
      Serial.println("Go Back");
      motion.backward();
      if(currentTime - startTime >= interval){
        motionIndex = 2;
        startTime = currentTime;
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
      if(currentTime - startTime >= interval){
        if(needToLeft){
          motionIndex = turnLeft;
        }
        else if(needToRight){
          motionIndex = turnRight;
        }
        else if(needToBack){
          motionIndex = goBack;
        }
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
  int differenceRow1 = abs(target[targetIndex].getRow() - inputCell.getRow());
  int differenceRow2 = abs(target[targetIndex].getRow() - inputCell2.getRow());

  int differenceColumn1 = abs(target[targetIndex].getColumn() - inputCell.getColumn());
  int differenceColumn2 = abs(target[targetIndex].getColumn() - inputCell2.getColumn());

  int difference1 = differenceRow1 + differenceColumn1;
  int difference2 = differenceRow2 + differenceColumn2;

  if(targetIndex < 3){
    if(difference1 < difference2){
      return inputCell;
    }
    else if(difference2 < difference1){
      return inputCell2;
    }
    else if(difference1 == difference2){
      targetIndex++;
      return differenceForPossible(inputCell, inputCell2);
    }
  }
  return inputCell;
}

/*
Cell& differenceForPossible(Cell& inputCell, Cell& inputCell2, Cell& inputCell3){
  bool routeFound = false;
  int targetIndex = 0;
  int smallest;
  int differenceRow1 = abs(target[targetIndex].getRow() - inputCell.getRow());
  int differenceRow2 = abs(target[targetIndex].getRow() - inputCell2.getRow());
  int differenceRow3 = abs(target[targetIndex].getRow() - inputCell3.getRow());

  int differenceColumn1 = abs(target[targetIndex].getColumn() - inputCell.getColumn());
  int differenceColumn2 = abs(target[targetIndex].getColumn() - inputCell2.getColumn());
  int differenceColumn3 = abs(target[targetIndex].getColumn() - inputCell3.getColumn());

  int difference1 = differenceRow1 + differenceColumn1;
  int difference2 = differenceRow2 + differenceColumn2;
  int difference3 = differenceRow3 + differenceColumn3;

  smallest = difference1;
  if(difference2 < smallest){
    smallest = difference2;
  }
  if(difference3 < smallest){
    smallest = difference3;
  }
  
  if(smallest == difference1){
    return inputCell;
  }
  else if(smallest == difference2){
    return inputCell2;
  }
  else if(smallest == difference3){
    return inputCell3;
  }
}
*/
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
