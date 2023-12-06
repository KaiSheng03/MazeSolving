#include "Motor.h"
#include "Motion.h"
#include "FrontSensor.h"
#include "LeftSensor.h"
#include "RightSensor.h"
#include <TimedAction.h>

void funcFrontSensor();
void funcLeftSensor();
void funcRightSensor();

void funcForward();
void funcLeft();
void funcRight();
void funcBackward();

TimedAction taskFrontSensor = TimedAction(1000, funcFrontSensor);
TimedAction taskLeftSensor = TimedAction(1000, funcLeftSensor);
TimedAction taskRightSensor = TimedAction(1000, funcRightSensor);

TimedAction taskForward = TimedAction(1000, funcForward);
TimedAction taskLeft = TimedAction(1000, funcLeft);
TimedAction taskRight = TimedAction(1000, funcRight);
TimedAction taskBackward = TimedAction(1000, funcBackward);

// (Speed, in1, in2, en1_2)
Motor leftMotor(1200, 2, 3, 6);
Motor rightMotor(1200, 4, 5, 7);

Motion motion(leftMotor, rightMotor);

FrontSensor frontsensor(9, A1, 11);
LeftSensor leftsensor(8, A0, 12);
RightSensor rightsensor(10, A2, 13);

void setup() {
  leftMotor.setup();
  rightMotor.setup();
  frontsensor.setup();
  leftsensor.setup();
  rightsensor.setup();
  Serial.begin(9600);
}

void loop() { //this is just a draft(i cincai write it hahaha), algorithm will be discussed and rewrite soon
  // put your main code here, to run repeatedly:
  taskFrontSensor.check();
  taskLeftSensor.check();
  taskRightSensor.check();

  //taskForward.check();
  //taskLeft.check();
  //taskRight.check();
  //taskBackward.check();
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
  if(!frontsensor.checkObstacle() && !rightsensor.checkObstacle() && !leftsensor.checkObstacle()){
    motion.forward();
    Serial.println("Forward");
  }
}

void funcLeft(){
  if(frontsensor.checkObstacle() && rightsensor.checkObstacle() && !leftsensor.checkObstacle()){
    motion.left();
    Serial.println("Left");
  }
}

void funcRight(){
  if(frontsensor.checkObstacle() && !rightsensor.checkObstacle() && leftsensor.checkObstacle()){
    motion.right();
    Serial.println("Right");
  }
}

void funcBackward(){
  if(frontsensor.checkObstacle() && rightsensor.checkObstacle() && leftsensor.checkObstacle()){
    motion.backward();
    Serial.println("Backward");
  }
}
