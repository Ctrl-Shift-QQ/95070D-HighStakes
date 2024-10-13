#include "util.h"

class Odometry {
    //In inches and degrees unless otherwise specified
    private:
        double previousSidewaysPosition;
        double previousForwardPosition;
        double previousOrientationRad;
        double wheelDiameter;
        double sidewaysToCenterDistance;
        double forwardToCenterDistance;
    public:
        Odometry(double wheelDiameter, double sidewaysToCenterDistance, double forwardToCenterDistance); //Constructor

        double xPosition;
        double yPosition;
        double orientation;
        
        void setPosition(double xPosition, double yPosition, double orientation);
        void updatePosition();
};