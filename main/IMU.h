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

    // Variables for PID control
    double error, lastError;
    double integral;
    double derivative;
    int target;
    double angleZ;

  public:
    IMU();
    void updateAngles();
    void calculatePID();
    void calculateMotorSpeed(int&, int&);
    void setTarget(int);
    void setup();
    double getAngleZ() const;
};

#endif