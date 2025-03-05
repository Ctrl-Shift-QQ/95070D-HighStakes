#pragma once

#include "mech-config.h"
#include "PID.h"

typedef enum { //For pressed function
    Up = 0,
    Left,
    Right,
    Down,
    X,
    Y,
    A,
    B,
    L1,
    L2,
    R1,
    R2,
    Null,
} ButtonID;
const controller::button& IDToButton(ButtonID buttonID);
bool pressed(ButtonID controlButtonID);
bool released(ButtonID controlButtonID);

void motorSeperateButton(double percentSpeed, motor_group &controlMotor, ButtonID spinForwardButtonID, ButtonID spinReverseButtonID, 
                         ButtonID stopButtonID);
void pistonSeperateButton(digital_out &controlPiston, ButtonID extendButtonID, ButtonID retractButtonID);
void motorHold(double percentSpeed, motor_group &controlMotor, ButtonID forwardButtonID, ButtonID reverseButtonID);
void pistonHold(bool reverse, digital_out &controlPiston, ButtonID controlButtonID);
void motorToggle(directionType motorDirection, double percentSpeed, motor_group &controlMotor, ButtonID controlButtonID);
void pistonToggle(digital_out &controlPiston, ButtonID controlButtonID);

/******************** Controls ********************/

void runTankDrive(double percentSpeed, bool toggleSpeed = false, ButtonID toggleSpeedButtonID = A, double slowPercentSpeed = 0);
void runArcadeDrive(double percentSpeed, double steerPercentSpeed, bool toggleSpeed = false, ButtonID toggleSpeedButtonID = A, double slowPercentSpeed = 0);
void runIntake();
void runArm();
void runMogo();
void runDoinker();