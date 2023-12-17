#include "Motor.h"

Motor::Motor(){
  speed = 0;
  input1 = 0;
  input2 = 0;
  enable1_2 = 0;
}

Motor::Motor(int s, int in1, int in2, int en){
  speed = s;
  input1 = in1;
  input2 = in2;
  enable1_2 = en;
}

int& Motor::getSpeed(){
  return speed;
}

void Motor::setSpeed(int s){
  speed = s;
}

int Motor::getInput1()const{
  return input1;
}

int Motor::getInput2()const{
  return input2;
}

int Motor::getEnable1_2()const{
  return enable1_2;
}

void Motor::setup(){
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);
  pinMode(enable1_2, OUTPUT);
}
