#include "vex.h"

class Odom {
    private:
        double previousSidewaysPosition;
        double previousForwardPosition;
        double previousOrientationRad;
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