#include "LeftSensor.h"

LeftSensor::LeftSensor(int trigPin, int echoPin)
  : UltrasonicSensor(trigPin, echoPin){}

void LeftSensor::setup(){
  UltrasonicSensor::setup();
}

void LeftSensor::readDuration(){
  UltrasonicSensor::readDuration();
}

void LeftSensor::calculateDistance(){
  UltrasonicSensor::calculateDistance();
  //Serial.print("Left Distance: ");
  //Serial.println(distance);
}

double LeftSensor::getDistance()const{
  return UltrasonicSensor::getDistance();
}

bool LeftSensor::checkObstacle() const{
  return UltrasonicSensor::checkObstacle();
}