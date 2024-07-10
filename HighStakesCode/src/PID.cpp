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
    if (fabs(error) < startI){
        integral += error;
    }
    
    derivative = previousError - error;
    previousError = error;

    return error * kp + integral * ki - derivative * kd;
}

bool PID::isSettled(double error){
    if ((fabs(error) < deadband && timeSpent > settleTime) || timeSpent > timeout){
        return true;
    }

    timeSpent += loopCycleTime;

    return false;
}