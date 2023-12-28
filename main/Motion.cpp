#include "Motion.h"

Motion::Motion(Motor lm, Motor rm){
  leftMotor = lm;
  rightMotor = rm;
  leftSpeed = leftMotor.getSpeed();
  rightSpeed = rightMotor.getSpeed();
}

void Motion::forward(int left, int right){
  analogWrite(leftMotor.getEnable1_2(), left);
  analogWrite(rightMotor.getEnable1_2(), right);

  digitalWrite(leftMotor.getInput1(), HIGH);
  digitalWrite(leftMotor.getInput2(), LOW);
  digitalWrite(rightMotor.getInput1(), HIGH);
  digitalWrite(rightMotor.getInput2(), LOW);

  Serial.print("Left Motor Speed: ");
  Serial.println(left);
  Serial.print("Right Motor Speed: ");
  Serial.println(right);
}

void Motion::backward(int left, int right){
  analogWrite(leftMotor.getEnable1_2(), left);
  analogWrite(rightMotor.getEnable1_2(), right);

  digitalWrite(leftMotor.getInput1(), LOW);
  digitalWrite(leftMotor.getInput2(), HIGH);
  digitalWrite(rightMotor.getInput1(), LOW);
  digitalWrite(rightMotor.getInput2(), HIGH);
}

void Motion::right(int left, int right){
  analogWrite(leftMotor.getEnable1_2(), left);
  analogWrite(rightMotor.getEnable1_2(), right);

  digitalWrite(leftMotor.getInput1(), HIGH);
  digitalWrite(leftMotor.getInput2(), LOW);
  digitalWrite(rightMotor.getInput1(), LOW);
  digitalWrite(rightMotor.getInput2(), HIGH);
}

void Motion::right(){
  analogWrite(leftMotor.getEnable1_2(), 160);
  analogWrite(rightMotor.getEnable1_2(), 0);

  digitalWrite(leftMotor.getInput1(), HIGH);
  digitalWrite(leftMotor.getInput2(), LOW);
  digitalWrite(rightMotor.getInput1(), LOW);
  digitalWrite(rightMotor.getInput2(), HIGH);
}

void Motion::left(int left, int right){
  analogWrite(leftMotor.getEnable1_2(), left);
  analogWrite(rightMotor.getEnable1_2(), right);

  digitalWrite(leftMotor.getInput1(), LOW);
  digitalWrite(leftMotor.getInput2(), HIGH);
  digitalWrite(rightMotor.getInput1(), HIGH);
  digitalWrite(rightMotor.getInput2(), LOW);
}

void Motion::left(){
  analogWrite(leftMotor.getEnable1_2(), 0);
  analogWrite(rightMotor.getEnable1_2(), 160);

  digitalWrite(leftMotor.getInput1(), LOW);
  digitalWrite(leftMotor.getInput2(), HIGH);
  digitalWrite(rightMotor.getInput1(), HIGH);
  digitalWrite(rightMotor.getInput2(), LOW);
}

void Motion::stop(){
  analogWrite(leftMotor.getEnable1_2(), leftMotor.getSpeed());
  analogWrite(rightMotor.getEnable1_2(), rightMotor.getSpeed());

  digitalWrite(leftMotor.getInput1(), LOW);
  digitalWrite(leftMotor.getInput2(), LOW);
  digitalWrite(rightMotor.getInput1(), LOW);
  digitalWrite(rightMotor.getInput2(), LOW);
}