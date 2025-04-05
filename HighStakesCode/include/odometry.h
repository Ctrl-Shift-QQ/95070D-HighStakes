#pragma once

#include "mech-config.h"
#include "util.h"

class Odometry {
    //Units: inches and degrees unless otherwise specified
    private:
        double previousHorizontalPosition;
        double previousVerticalPosition;
        double previousOrientationRad;
        double horizontalWheelDiameter;
        double verticalWheelDiameter;
        double horizontalToCenterDistance;
        double verticalToCenterDistance;
        double inertialScale; //Accounts for IMU inaccuracy
    public:
        Odometry(double horizontalToCenterDistance, double verticalToCenterDistance); //Constructor

        double xPosition;
        double yPosition;
        double orientation;
        
        void setPosition(double xPosition, double yPosition, double orientation);
        void updatePosition(double horizontalTrackerPosition, double verticalHorizontalPosition, double orientation);
};