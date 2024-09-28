#include "vex.h"
#include "odom.h"
#include <iostream>

void odom::setPhysicalMeasurements(double wheelSize, double sidewaysToCenterDistance, double forwardToCenterDistance){
    this->wheelSize = wheelSize;
    this->sidewaysToCenterDistance = sidewaysToCenterDistance;
    this->forwardToCenterDistance = forwardToCenterDistance;
}

void odom::setPosition(double xPosition, double yPosition, double orientation){
    this->xPosition = xPosition;
    this->yPosition = yPosition;
    this->orientation = orientation;
}

void odom::updatePosition(){
    //Saves values so that they don't change during the same cycle
    double sidewaysTrackerPosition = SidewaysTracker.position(turns);
    double forwardTrackerPosition = ForwardTracker.position(turns);
    double orientationRadians = Inertial.heading(degrees);

    double sidewaysPositionDelta = (sidewaysTrackerPosition - previousSidewaysPosition) * 2.75 * M_PI; //Gets change in inches (position variable is value from last cycle)
    double forwardPositionDelta = (sidewaysTrackerPosition - previousForwardPosition) * 2.75 * M_PI;
    previousSidewaysPosition = sidewaysTrackerPosition;
    previousForwardPosition = forwardTrackerPosition;
    
    
    double localXPosition;
    double localYposition;
    double localPlaneOffset; 

    
}