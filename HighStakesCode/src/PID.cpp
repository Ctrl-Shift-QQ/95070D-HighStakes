#include "vex.h"
#include <PID.h>
#include <iostream>

PID::PID(double startError, double kp, double ki, double kd, double startI, double deadband, double loopCycleTime, double settleTime, double timeout):
    startError(startError),
    kp(kp),
    ki(ki),
    kd(kd),
    startI(startI),
    integral(0),
    previousError(startError),
    deadband(deadband),
    loopCycleTime(loopCycleTime),
    timeSpent(0),
    settleTime(settleTime),
    timeout(timeout)
{};

double PID::output(double error){
    double loopsPerSec = 1000 / loopCycleTime;

    if (fabs(error) < startI){
        this->integral += error;
    }

    if (getSign(error) != getSign(startError)){ //Checks if error has crossed 0
        this->integral = 0;
    }
    
    this->derivative = previousError - error;
    this->previousError = error;
    
    return error * kp + integral / loopsPerSec * ki - derivative * loopsPerSec * kd;
}

bool PID::isSettled(double error){
    if ((fabs(error) < deadband && timeSpent > settleTime) || (timeSpent > timeout && timeout > 0)){
        return true;
    }
    
    this->timeSpent += loopCycleTime;

    return false;
}