#include "vex.h"
#include "odometry.h"
#include <iostream>

Odometry::Odometry(double wheelDiameter, double sidewaysToCenterDistance, double forwardToCenterDistance) :
    wheelDiameter(wheelDiameter),
    sidewaysToCenterDistance(sidewaysToCenterDistance),
    forwardToCenterDistance(forwardToCenterDistance)
{};

void Odometry::setPosition(double xPosition, double yPosition, double orientation){
    //Sets variables accordingly
    this->xPosition = xPosition;
    this->yPosition = yPosition;
    this->orientation = orientation;
    this->previousSidewaysPosition = 0;
    this->previousForwardPosition = 0;
    this->previousOrientationRad = orientation * M_PI / 180;

    //Sets sensor values accordingly
    Inertial.setHeading(orientation, degrees);
    SidewaysTracker.resetRotation();
    ForwardTracker.resetRotation();
}

void Odometry::updatePosition(){
    //Saves values so that they don't change during the same cycle
    double sidewaysTrackerPosition = SidewaysTracker.position(turns);
    double forwardTrackerPosition = ForwardTracker.position(turns);
    double orientationRad = degToRad(Inertial.heading(degrees));
    double sidewaysPositionDelta = (sidewaysTrackerPosition - previousSidewaysPosition) * wheelDiameter * M_PI; //Gets change in inches
    double forwardPositionDelta = (forwardTrackerPosition - previousForwardPosition) * wheelDiameter * M_PI; //Gets change in inches
    double orientationDeltaRad = orientationRad - previousOrientationRad;
    
    double localXPosition;
    double localYPosition;

    //Gets local Cartesian translation coordinates of new position
    if (orientationDeltaRad == 0){
        localXPosition = sidewaysPositionDelta;
        localYPosition = forwardPositionDelta;
    }
    else {
        localXPosition = 2 * (sidewaysPositionDelta / orientationDeltaRad + sidewaysToCenterDistance) * sin(orientationDeltaRad / 2);
        localYPosition = 2 * (forwardPositionDelta / orientationDeltaRad + forwardToCenterDistance) * sin(orientationDeltaRad / 2);
    }

    //Converts Cartesian translation coordinates to polar and local to global
    double localPolarAngle = atan2(localYPosition, localXPosition);
    double globalPolarAngle = localPolarAngle - previousOrientationRad - orientationDeltaRad / 2;
    double polarLength = sqrt(pow(localXPosition, 2) + pow(localYPosition, 2));

    //Calculates global offset
    double xPositionDelta = cos(globalPolarAngle) * polarLength;
    double yPositionDelta = sin(globalPolarAngle) * polarLength;

    //Adds change to total value to get actual values
    this->xPosition += xPositionDelta;
    this->yPosition += yPositionDelta;
    this->orientation = radToDeg(orientationRad);

    //Sets previous values of next cycle to current values
    this->previousSidewaysPosition = sidewaysTrackerPosition;
    this->previousForwardPosition = forwardTrackerPosition;
    this->previousOrientationRad = orientationRad;
}