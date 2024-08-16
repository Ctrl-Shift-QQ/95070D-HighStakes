#include "vex.h"

void motorSeperateButton(double motorVelocity, motor &controlMotor, const controller::button &spinForwardButton, const controller::button &spinReverseButton, 
                         const controller::button &stopButton);
void pistonSeperateButton(digital_out &controlPiston, const controller::button &extendButton, const controller::button &retractButton);
void motorHold(directionType motorDirection, double motorVelocity, motor &controlMotor, const controller::button &controlButton);
void pistonHold(bool reverse, digital_out &controlPiston, const controller::button &controlButton);
bool pressed(const controller::button &controlButton);
void motorToggle(directionType motorDirection, double motorVelocity, motor &controlMotor, const controller::button &controlButton);
void pistonToggle(digital_out &controlPiston, const controller::button &controlButton);

/********** Controls **********/

void runTankDrive(double percentSpeed, bool toggleSpeed = false, const controller::button &toggleSpeedButton = Controller1.ButtonA, double slowPercentSpeed = 0);
void runArcadeDrive(double percentSpeed, double steerPercentSpeed, bool toggleSpeed = false, const controller::button &toggleSpeedButton = Controller1.ButtonA, double slowPercentSpeed = 0);
void runIntake();
void runMogo();
void runRedirect();