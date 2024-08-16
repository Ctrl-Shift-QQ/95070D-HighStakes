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
    double currentSidewaysTrackerValue = sidewaysTrackerValue;
    double currentForwardTrackerValue = forwardTrackerValue;
}