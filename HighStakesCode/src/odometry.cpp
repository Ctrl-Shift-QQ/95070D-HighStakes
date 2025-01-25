#include "vex.h"
#include "odometry.h"
#include <iostream>

Odometry::Odometry(double sidewaysWheelDiameter, double forwardWheelDiameter, double sidewaysToCenterDistance, double forwardToCenterDistance, double inertialScale) :
    sidewaysWheelDiameter(sidewaysWheelDiameter),
    forwardWheelDiameter(forwardWheelDiameter),
    sidewaysToCenterDistance(sidewaysToCenterDistance),
    forwardToCenterDistance(forwardToCenterDistance),
    inertialScale(inertialScale)
{};

void Odometry::setPosition(double xPosition, double yPosition, double orientation){
    //Sets variables accordingly
    this->xPosition = xPosition;
    this->yPosition = yPosition;
    this->orientation = orientation;
    this->previousSidewaysPosition = 0;
    this->previousForwardPosition = 0;
    this->previousOrientationRad = degToRad(orientation);

    //Sets sensor values accordingly
    Inertial.setRotation(orientation / inertialScale, degrees);
    SidewaysTracker.resetPosition();
    RightDrive.resetPosition();
}

void Odometry::updatePosition(){
    //Saves values so that they don't change during the same cycle
    double sidewaysTrackerPosition = SidewaysTracker.position(turns);
    double forwardTrackerPosition = RightDrive.position(turns) * DRIVETRAIN_GEAR_RATIO;
    double orientationRad = degToRad(Inertial.rotation(degrees) * inertialScale);
    double sidewaysPositionDelta = (sidewaysTrackerPosition - previousSidewaysPosition) * sidewaysWheelDiameter * M_PI; //Gets change in inches
    double forwardPositionDelta = (forwardTrackerPosition - previousForwardPosition) * forwardWheelDiameter * M_PI; //Gets change in inches
    double orientationDeltaRad = orientationRad - previousOrientationRad;
    double localXPosition;
    double localYPosition;
    double localPolarAngle;
    double globalPolarAngle;
    double polarLength;
    double xPositionDelta;
    double yPositionDelta;

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
    localPolarAngle = atan2(localYPosition, localXPosition);
    globalPolarAngle = localPolarAngle - previousOrientationRad - orientationDeltaRad / 2;
    polarLength = sqrt(pow(localXPosition, 2) + pow(localYPosition, 2));

    //Calculates global offset
    xPositionDelta = cos(globalPolarAngle) * polarLength;
    yPositionDelta = sin(globalPolarAngle) * polarLength;

    //Applies offset to get updated values
    this->xPosition += xPositionDelta;
    this->yPosition += yPositionDelta;
    this->orientation = fmod(fmod(radToDeg(orientationRad), 360) + 360, 360);

    //Sets previous values of next cycle to current values
    this->previousSidewaysPosition = sidewaysTrackerPosition;
    this->previousForwardPosition = forwardTrackerPosition;
    this->previousOrientationRad = orientationRad;
}