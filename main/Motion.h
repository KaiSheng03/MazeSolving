#include <Arduino.h>
#include "Motor.h"
class Motion{
  public:
    Motion(Motor, Motor);
    void forward(int, int);
    void backward(int, int);
    void left();
    void right();
    void stop();

    int leftSpeed;
    int rightSpeed;

    Motor leftMotor;
    Motor rightMotor;
};