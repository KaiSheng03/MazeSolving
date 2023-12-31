#include "IMU.h"

IMU::IMU():mpu(Wire){
  Kp = 15.0;
  Ki = 0.01;
  Kd = 2.0;

  error, lastError = 0;
  integral = 0;
  target = 0;
  derivative = 0;
  motorSpeed = 250;
  leftSpeed = 200;
  rightSpeed = 200;
}

void IMU::setup(){
  // Initialize MPU6050
  Wire.begin();          // Initialize I2C communication
  int status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println("Calculating offsets, do not move MPU6050");
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

void IMU::updateAngles(){
  mpu.update();
  
  Serial.print("X : ");
  Serial.print(mpu.getAngleX());
  Serial.print("\tY : ");
  Serial.print(mpu.getAngleY());
  Serial.print("\tZ : ");
  Serial.println(mpu.getAngleZ());
  
  angleZ = mpu.getAngleZ();
}

void IMU::calculatePID(){
  error = target - mpu.getAngleZ(); // Desired angle is 0 for straight motion
  integral += error;
  derivative = error - lastError;
}

void IMU::calculateMotorSpeed(){
  leftSpeed = motorSpeed - (Kp * error) + (Ki * integral) + (Kd * derivative);
  rightSpeed = motorSpeed + (Kp * error) - (Ki * integral) - (Kd * derivative);
  if(leftSpeed > 255){
    leftSpeed = 255;
  }
  if(rightSpeed > 255){
    rightSpeed = 255;
  }
  if(leftSpeed < 0){
    leftSpeed = 0;
  }
  if(rightSpeed < 0){
    rightSpeed = 0;
  }
}

int IMU::getLeftSpeed() const{
  return leftSpeed;
}

int IMU::getRightSpeed() const{
  return rightSpeed;
}

void IMU::setTarget(int targetAngle){
  target = targetAngle;
}

double IMU::getZ() const{
  return angleZ;
}

double IMU::getTarget() const{
  return target;
}
