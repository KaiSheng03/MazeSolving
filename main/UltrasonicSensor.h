// BASE CLASS OF ULTRASONIC SENSOR
#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H
#include <Arduino.h>

class UltrasonicSensor{
  public:
    UltrasonicSensor();
    UltrasonicSensor(int, int);
    void setup();
    void readDuration();
    virtual void calculateDistance();
    double getDistance()const;
    virtual bool checkObstacle() const;

  private:
    int trigPin;
    int echoPin;
    long duration;
  
  protected:
    double distance;
};
#endif

