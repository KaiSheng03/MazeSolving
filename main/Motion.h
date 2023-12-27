#include <Arduino.h>
#include "Motor.h"
class Motion{
  public:
    Motion(Motor, Motor);
    void forward(int&, int&);
    void backward();
    void right();
    void left();
    void stop();

  private:
    Motor leftMotor;
    Motor rightMotor;
};


/*
class ForwardMotion{
  public:
    ForwardMotion(Motor, Motor);
    void forward();
  private:
    Motor motor1;
    Motor motor2;
};

ForwardMotion::ForwardMotion(Motor m1, Motor m2){
  motor1 = m1;
  motor2 = m2;
}

void ForwardMotion::forward(){  //function to move the robot forward
  analogWrite(motor1.getEnable1_2(), motor1.getSpeed());
  analogWrite(motor2.getEnable1_2(), motor2.getSpeed());

  digitalWrite(motor1.getInput1(), HIGH);
  digitalWrite(motor1.getInput2(), LOW);
  digitalWrite(motor2.getInput1(), HIGH);
  digitalWrite(motor2.getInput2(), LOW);
}

class BackwardMotion{
  public:
    BackwardMotion(Motor, Motor);
    void backward();
  private:
    Motor motor1;
    Motor motor2;
};

BackwardMotion::BackwardMotion(Motor m1, Motor m2){
  motor1 = m1;
  motor2 = m2;
}

void BackwardMotion::backward(){  //function to move the robot backward
  analogWrite(motor1.getEnable1_2(), motor1.getSpeed());
  analogWrite(motor2.getEnable1_2(), motor2.getSpeed());

  digitalWrite(motor1.getInput1(), LOW);
  digitalWrite(motor1.getInput2(), HIGH);
  digitalWrite(motor2.getInput1(), LOW);
  digitalWrite(motor2.getInput2(), HIGH);
}

class TurnLeft{
  public:
    TurnLeft(Motor, Motor);
    void left();
  private:
    Motor motor1;
    Motor motor2;
};

TurnLeft::TurnLeft(Motor m1, Motor m2){
  motor1 = m1;
  motor2 = m2;
}

void TurnLeft::left(){  //function to turn the robot left
  analogWrite(motor1.getEnable1_2(), motor1.getSpeed());
  analogWrite(motor2.getEnable1_2(), motor2.getSpeed());

  digitalWrite(motor1.getInput1(), LOW);
  digitalWrite(motor1.getInput2(), HIGH);
  digitalWrite(motor2.getInput1(), HIGH);
  digitalWrite(motor2.getInput2(), LOW);
}

class TurnRight{
  public:
    TurnRight(Motor,Motor);
    void right();
  private:
    Motor motor1;
    Motor motor2;
};

TurnRight::TurnRight(Motor m1, Motor m2){
  motor1 = m1;
  motor2 = m2;
}

void TurnRight::right(){ //function to turn the robot right
  analogWrite(motor1.getEnable1_2(), motor1.getSpeed());
  analogWrite(motor2.getEnable1_2(), motor2.getSpeed());

  digitalWrite(motor1.getInput1(), HIGH);
  digitalWrite(motor1.getInput2(), LOW);
  digitalWrite(motor2.getInput1(), LOW);
  digitalWrite(motor2.getInput2(), HIGH);
}

class StopMoving{
  public:
    StopMoving(Motor, Motor);
    void stop();
  private:
    Motor motor1;
    Motor motor2;
};

StopMoving::StopMoving(Motor m1, Motor m2){
  motor1 = m1;
  motor2 = m2;
}

void StopMoving::stop(){ //function to stop the robot
  analogWrite(motor1.getEnable1_2(), motor1.getSpeed());
  analogWrite(motor2.getEnable1_2(), motor2.getSpeed());

  digitalWrite(motor1.getInput1(), LOW);
  digitalWrite(motor1.getInput2(), LOW);
  digitalWrite(motor2.getInput1(), LOW);
  digitalWrite(motor2.getInput2(), LOW);
}
*/

