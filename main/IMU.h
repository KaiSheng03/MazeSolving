#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <MPU6050_light.h>

#define MPU6050_ADDR 0x68 // MPU6050 I2C address

class IMU{
  private:
    MPU6050 mpu;
    unsigned long timer;
    double Kp; // Proportional constant
    double Ki; // Integral constant
    double Kd; // Derivative constant
    int motorSpeed;
    int leftSpeed;
    int rightSpeed;

    // Variables for PID control
    double error, lastError;
    double integral;
    double derivative;
    double target;
    double angleZ;

  public:
    IMU();
    void updateAngles();
    void calculatePID();
    void calculateMotorSpeed();
    void setTarget(int);
    void setup();
    int getLeftSpeed() const;
    int getRightSpeed() const;
    double getZ() const;
    double getTarget() const;
};

#endif