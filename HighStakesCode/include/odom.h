#include "vex.h"

class odom {
    private:
        double previousSidewaysPosition; //Keeps track of 
        double previousForwardPosition;
        double previousOrientation;
        double wheelSize;
        double sidewaysToCenterDistance;
        double forwardToCenterDistance;
    public:
        double xPosition; //Inches
        double yPosition; //Inches
        double orientation; //Degrees
        void setPhysicalMeasurements(double wheelSize, double sidewaysToCenterDistance, double forwardToCenterDistance);
        void setPosition(double xPosition, double yPosition, double orientation);
        void updatePosition();
};