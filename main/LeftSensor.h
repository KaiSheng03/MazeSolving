#include "UltrasonicSensor.h"

class LeftSensor : public UltrasonicSensor{
  public:
    LeftSensor(int, int);
    void setup();
    void readDuration();
    void calculateDistance();
    double getDistance()const;
    bool checkObstacle() const;
};