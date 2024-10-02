#include "vex.h"
#include "odom.h"
#include <iostream>

void Odom::setPhysicalMeasurements(double wheelSize, double sidewaysToCenterDistance, double forwardToCenterDistance){
    this->wheelSize = wheelSize;
    this->sidewaysToCenterDistance = sidewaysToCenterDistance;
    this->forwardToCenterDistance = forwardToCenterDistance;
}

void Odom::setPosition(double xPosition, double yPosition, double orientation){
    this->xPosition = xPosition;
    this->yPosition = yPosition;
    this->orientation = orientation;
    this->previousSidewaysPosition = 0;
    this->previousForwardPosition = 0;
    this->previousOrientationRad = orientation * M_PI / 180;

    Inertial.setHeading(orientation, degrees);
    SidewaysTracker.resetRotation();
    ForwardTracker.resetRotation();
}

void Odom::updatePosition(){
    //Saves values so that they don't change during the same cycle
    double sidewaysTrackerPosition = SidewaysTracker.position(turns);
    double forwardTrackerPosition = ForwardTracker.position(turns);
    double orientationRad = Inertial.heading(degrees) * M_PI / 180;
    double sidewaysPositionDelta = (sidewaysTrackerPosition - previousSidewaysPosition) * wheelSize * M_PI; //Gets change in inches
    double forwardPositionDelta = (forwardTrackerPosition - previousForwardPosition) * wheelSize * M_PI; //Gets change in inches
    double orientationDeltaRad = orientationRad - previousOrientationRad;
    
    double localXPosition;
    double localYPosition;

    //Gets local Cartesian coordinates of new position
    if (orientationDeltaRad == 0){
        localXPosition = sidewaysPositionDelta;
        localYPosition = forwardPositionDelta;
    }
    else {
        localXPosition = 2 * (sidewaysPositionDelta / orientationDeltaRad + sidewaysToCenterDistance) * sin(orientationDeltaRad / 2);
        localYPosition = 2 * (forwardPositionDelta / orientationDeltaRad + forwardToCenterDistance) * sin(orientationDeltaRad / 2);
    }

    //Converts Cartesian coordinates to polar coordinates and local to global
    double localPolarAngle = atan2(localYPosition, localXPosition);
    double globalPolarAngle = localPolarAngle - previousOrientationRad - orientationDeltaRad / 2;
    double polarLength = sqrt(pow(localXPosition, 2) + pow(localYPosition, 2));

    //Calculates global offset
    double xPositionDelta = cos(globalPolarAngle) * polarLength;
    double yPositionDelta = sin(globalPolarAngle) * polarLength;

    //Adds change to total value to get actual values
    this->xPosition += xPositionDelta;
    this->yPosition += yPositionDelta;
    this->orientation = orientationRad * 180 / M_PI;

    //Sets previous values of next cycle to current values
    this->previousSidewaysPosition = sidewaysTrackerPosition;
    this->previousForwardPosition = forwardTrackerPosition;
    this->previousOrientationRad = orientationRad;
}