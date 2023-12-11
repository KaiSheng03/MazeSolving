#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(){
  trigPin = 0;
  echoPin = 0;
  duration = 0;
  distance = 0;
}

UltrasonicSensor::UltrasonicSensor(int trigpin, int echopin)
  : trigPin(trigpin), echoPin(echopin) {}
  
void UltrasonicSensor::setup(){
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void UltrasonicSensor::readDuration(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

void UltrasonicSensor::calculateDistance(){
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
}

double UltrasonicSensor::getDistance()const{
  return distance;
}

bool UltrasonicSensor::checkObstacle() const{
  if(distance <= 5){
    return true;
  }
  else{
    return false;
  }
}