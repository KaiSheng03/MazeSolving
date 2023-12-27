#include "Motion.h"

Motion::Motion(Motor lm, Motor rm){
  leftMotor = lm;
  rightMotor = rm;
}

void Motion::forward(int& leftSpeed, int& rightSpeed){
  analogWrite(leftMotor.getEnable1_2(), leftSpeed);
  analogWrite(rightMotor.getEnable1_2(), rightSpeed);

  digitalWrite(leftMotor.getInput1(), HIGH);
  digitalWrite(leftMotor.getInput2(), LOW);
  digitalWrite(rightMotor.getInput1(), HIGH);
  digitalWrite(rightMotor.getInput2(), LOW);

  Serial.print("Left Motor Speed: ");
  Serial.println(leftSpeed);
  Serial.print("Right Motor Speed: ");
  Serial.println(rightSpeed);
}

void Motion::backward(){
  analogWrite(leftMotor.getEnable1_2(), leftMotor.getSpeed());
  analogWrite(rightMotor.getEnable1_2(), rightMotor.getSpeed());

  digitalWrite(leftMotor.getInput1(), LOW);
  digitalWrite(leftMotor.getInput2(), HIGH);
  digitalWrite(rightMotor.getInput1(), LOW);
  digitalWrite(rightMotor.getInput2(), HIGH);
}

void Motion::right(){
  analogWrite(leftMotor.getEnable1_2(), leftMotor.getSpeed());
  analogWrite(rightMotor.getEnable1_2(), rightMotor.getSpeed());

  digitalWrite(leftMotor.getInput1(), HIGH);
  digitalWrite(leftMotor.getInput2(), LOW);
  digitalWrite(rightMotor.getInput1(), LOW);
  digitalWrite(rightMotor.getInput2(), HIGH);
}

void Motion::left(){
  analogWrite(leftMotor.getEnable1_2(), leftMotor.getSpeed());
  analogWrite(rightMotor.getEnable1_2(), rightMotor.getSpeed());

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