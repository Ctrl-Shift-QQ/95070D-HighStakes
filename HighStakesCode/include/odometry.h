#pragma once

#include "util.h"

class Odometry {
    //In inches and degrees unless otherwise specified
    private:
        double previousSidewaysPosition;
        double previousForwardPosition;
        double previousOrientationRad;
        double sidewaysWheelDiameter;
        double forwardWheelDiameter;
        double sidewaysToCenterDistance;
        double forwardToCenterDistance;
        double inertialScale; //Accounts for IMU inaccuracy
    public:
        Odometry(double sidewaysWheelDiameter, double forwardWheelDiameter, double sidewaysToCenterDistance, double forwardToCenterDistance, double inertialScale); //Constructor

        double xPosition;
        double yPosition;
        double orientation;
        
        void setPosition(double xPosition, double yPosition, double orientation);
        void updatePosition();
};