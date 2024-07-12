#include "vex.h"
#include "odom.h"
#include <iostream>

void odom::setTrackerToCenterDistances(double sidewaysToCenterDistance, double forwardToCenterDistance){
    this->sidewaysToCenterDistance = sidewaysToCenterDistance;
    this->forwardToCenterDistance = forwardToCenterDistance;
}

void odom::setPosition(double xPosition, double yPosition, double orientation){
    this->xPosition = xPosition;
    this->yPosition = yPosition;
    this->orientation = orientation;
}

void odom::updatePosition(){ 
    
}