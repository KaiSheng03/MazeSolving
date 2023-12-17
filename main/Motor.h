#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

class Motor{  //to declare pin number of one motor driver on Arduino board
  public:
    Motor();
    Motor(int, int, int, int);
    int& getSpeed();
    void setSpeed(int);
    int getInput1()const;
    int getInput2()const;
    int getEnable1_2()const;
    void setup();

  private:
    int speed;
    int input1;
    int input2;
    int enable1_2;
};
#endif