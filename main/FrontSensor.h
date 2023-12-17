#include "UltrasonicSensor.h"

class FrontSensor : public UltrasonicSensor{
  public:
    FrontSensor(int, int);
    void setup();
    void readDuration();
    void calculateDistance();
    double getDistance()const;
    bool checkObstacle() const;
};