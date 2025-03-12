#include "vex.h"
#include "odometry.h"
#include <iostream>

Odometry::Odometry(double horizontalWheelDiameter, double verticalWheelDiameter, double horizontalToCenterDistance, double verticalToCenterDistance, double inertialScale) :
    horizontalWheelDiameter(horizontalWheelDiameter),
    verticalWheelDiameter(verticalWheelDiameter),
    horizontalToCenterDistance(horizontalToCenterDistance),
    verticalToCenterDistance(verticalToCenterDistance),
    inertialScale(inertialScale)
{};

void Odometry::setPosition(double xPosition, double yPosition, double orientation){
    //Sets variables accordingly
    this->xPosition = xPosition;
    this->yPosition = yPosition;
    this->orientation = orientation;
    this->previousHorizontalPosition = 0;
    this->previousVerticalPosition = 0;
    this->previousOrientationRad = degToRad(orientation);

    //Sets sensor values accordingly
    Inertial.setRotation(orientation / (360 / inertialScale), degrees);
    HorizontalTracker.resetPosition();
    LeftDrive.resetPosition();
    RightDrive.resetPosition();
}

void Odometry::updatePosition(){
    //Saves values so that they don't change during the same cycle
    double horizontalTrackerPosition = HorizontalTracker.position(turns);
    double verticalTrackerPosition = (LeftDrive.position(turns) + RightDrive.position(turns)) / 2 * DRIVETRAIN_GEAR_RATIO;
    this->orientation = fmod(fmod(Inertial.rotation(degrees) * (360 / inertialScale), 360) + 360, 360);
    double orientationRad = degToRad(orientation);
    double horizontalPositionDelta = (horizontalTrackerPosition - previousHorizontalPosition) * M_PI * horizontalWheelDiameter; //Gets change in inches
    double verticalPositionDelta = (verticalTrackerPosition - previousVerticalPosition) * M_PI * verticalWheelDiameter ; //Gets change in inches
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

    //Calculates global offset
    xPositionDelta = cos(globalPolarAngle) * polarLength;
    yPositionDelta = sin(globalPolarAngle) * polarLength;

    //Applies offset to get updated values
    this->xPosition += xPositionDelta;
    this->yPosition += yPositionDelta;
    
    //Sets previous values of next cycle to current values
    this->previousHorizontalPosition = horizontalTrackerPosition;
    this->previousVerticalPosition = verticalTrackerPosition;
    this->previousOrientationRad = orientationRad;
}