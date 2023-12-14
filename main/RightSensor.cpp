#include "RightSensor.h"

RightSensor::RightSensor(int trigPin, int echoPin, int ledPin)
  : UltrasonicSensor(trigPin, echoPin), ledPin(ledPin) {}

void RightSensor::setup(){
  UltrasonicSensor::setup();
  pinMode(ledPin, OUTPUT);
}

void RightSensor::readDuration(){
  UltrasonicSensor::readDuration();
}

void RightSensor::calculateDistance(){
  UltrasonicSensor::calculateDistance();
  //Serial.print("Right Distance: ");
  //Serial.println(distance);
}

double RightSensor::getDistance()const{
  return UltrasonicSensor::getDistance();
}

bool RightSensor::checkObstacle() const{
  return UltrasonicSensor::checkObstacle();
}