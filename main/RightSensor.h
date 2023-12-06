#include "UltrasonicSensor.h"

class RightSensor : public UltrasonicSensor{
  public:
    RightSensor(int, int, int);
    void setup();
    void readDuration();
    void calculateDistance();
    double getDistance()const;
    bool checkObstacle() const;

  private:
    const int ledPin;
};