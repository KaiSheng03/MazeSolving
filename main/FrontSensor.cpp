#include "FrontSensor.h"

FrontSensor::FrontSensor(int trigpin, int echopin)
  : UltrasonicSensor(trigpin, echopin){}

void FrontSensor::setup(){
  UltrasonicSensor::setup();
}

void FrontSensor::readDuration(){
  UltrasonicSensor::readDuration();
}

void FrontSensor::calculateDistance(){
  UltrasonicSensor::calculateDistance();
  //Serial.print("Front Distance: ");
  //Serial.println(distance);
}

double FrontSensor::getDistance()const{
  return UltrasonicSensor::getDistance();
}

bool FrontSensor::checkObstacle() const{
  return UltrasonicSensor::checkObstacle();
}
