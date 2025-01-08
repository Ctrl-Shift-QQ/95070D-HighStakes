#pragma once

#include "util.h"

class PID {
    public:
        double startError;

        //Tunings
        const double kp;
        const double ki;
        const double kd;

        //For settling calculations
        const double deadband;
        const double loopCycleTime;
        double timeSpent;
        const double settleTime;
        const double timeout;
        
        //For output calculations
        const double startI;
        double integral;
        double previousError;
        double derivative;

        PID(double startError, double kp, double ki, double kd, double startI, double deadband, double loopCycleTime, double settleTime, double timeout); //Constructor

        double output(double error); //Returns the output of the PID loop
        bool isSettled(double error); //Returns whether or not the PID is settled
};