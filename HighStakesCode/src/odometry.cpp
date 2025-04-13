#include "vex.h"
#include "odometry.h"
#include <iostream>

Odometry::Odometry(double horizontalToCenterDistance, double verticalToCenterDistance):
    horizontalToCenterDistance(horizontalToCenterDistance),
    verticalToCenterDistance(verticalToCenterDistance)
{};

void Odometry::setPosition(double xPosition, double yPosition, double orientation){
    //Sets variables accordingly
    this->xPosition = xPosition;
    this->yPosition = yPosition;
    this->orientation = orientation;
    this->previousHorizontalPosition = 0;
    this->previousVerticalPosition = 0;
    this->previousOrientationRad = degToRad(orientation);
}

void Odometry::updatePosition(double horizontalTrackerPosition, double verticalTrackerPosition, double orientation){
    //Saves values for consistency within cycle
    double horizontalPositionDelta = horizontalTrackerPosition - previousHorizontalPosition; //Gets change in inches
    double verticalPositionDelta = verticalTrackerPosition - previousVerticalPosition; //Gets change in inches
    this->orientation = orientation;
    double orientationRad = degToRad(orientation);
    double orientationDeltaRad = orientationRad - previousOrientationRad;
    double localXPosition = 0;
    double localYPosition = 0;
    double localPolarAngle = 0;
    double globalPolarAngle = 0;
    double polarLength = 0;
    double xPositionDelta = 0;
    double yPositionDelta = 0;

    //Computes new local Cartesian translation coordinates
    if (orientationDeltaRad == 0){
        localXPosition = horizontalPositionDelta;
        localYPosition = verticalPositionDelta;
    }
    else {
        localXPosition = 2 * (horizontalPositionDelta / orientationDeltaRad + horizontalToCenterDistance) * sin(orientationDeltaRad / 2);
        localYPosition = 2 * (verticalPositionDelta / orientationDeltaRad + verticalToCenterDistance) * sin(orientationDeltaRad / 2);
    }

    //Converts Cartesian translation coordinates to polar and local to global
    localPolarAngle = atan2(localYPosition, localXPosition);
    globalPolarAngle = localPolarAngle - previousOrientationRad - orientationDeltaRad / 2;
    polarLength = sqrt(pow(localXPosition, 2) + pow(localYPosition, 2));

    //Computes global offset
    xPositionDelta = cos(globalPolarAngle) * polarLength;
    yPositionDelta = sin(globalPolarAngle) * polarLength;

    //Applies offset to update values
    this->xPosition += xPositionDelta;
    this->yPosition += yPositionDelta;
    
    //Updates previous cycle values for next cycle
    this->previousHorizontalPosition = horizontalTrackerPosition;
    this->previousVerticalPosition = verticalTrackerPosition;
    this->previousOrientationRad = orientationRad;
}