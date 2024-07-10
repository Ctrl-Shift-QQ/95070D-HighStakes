#include "vex.h"
#include <iostream>

class odom {
    private:
        double sidewaysTrackerValue;
        double forwardTrackerValue;
    public:
        double xPosition;
        double yPosition;
        double orientation;
        void setPosition();
        void updatePosition();
        void setTrackerToCenterDistances();
};