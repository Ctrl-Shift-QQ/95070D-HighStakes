#include "vex.h"
#include "controls.h"
#include <iostream>

controller dummyController;
controller::button nullButton = dummyController.ButtonA; //Placeholder for button when valid button not wanted

void motorSeperateButton(double motorVelocity, motor_group &controlMotor, const controller::button &spinForwardButton, const controller::button &spinReverseButton, 
                         const controller::button &stopButton){ //Tap seperate buttons for spin forward, spin reverse, and stop

    if (spinForwardButton.pressing()){
        controlMotor.spin(forward, motorVelocity, percent);
    }
    else if (spinReverseButton.pressing()){
        controlMotor.spin(reverse, motorVelocity, percent);
    }
    else if (stopButton.pressing()){
        controlMotor.stop(brake);
    }
}

void pistonSeperateButton(digital_out &controlPiston, const controller::button &extendButton, const controller::button &retractButton){ //Tap seperate buttons for extend and retract
    if (extendButton.pressing()){
        controlPiston.set(true);
    }
    else if (retractButton.pressing()){
        controlPiston.set(false);
    }
}

void motorHold(double motorVelocity, motor_group &controlMotor, const controller::button &forwardButton, const controller::button &reverseButton){ //Hold button to spin

    if (forwardButton.pressing()){
        controlMotor.spin(forward, motorVelocity, percent);
    }
    else if (reverseButton.pressing()){
        controlMotor.spin(reverse, motorVelocity, percent);
    }
    else {
        controlMotor.stop(brake);
    }
}

void pistonHold(bool reverse, digital_out &controlPiston, const controller::button &controlButton){ //Hold button to extend/retract
    if ((!reverse && controlButton.pressing()) || (reverse && !controlButton.pressing())){
        controlPiston.set(true);
    }
    else {
        controlPiston.set(false);
    }
}

bool pressed(ButtonID controlButtonID){ //Detects new presses
    static bool buttonWasAlreadyPressed[] = {false, false, false, false, false, false, false, false, false, false, false, false}; //Saves statuses of all buttons
    controller::button controlButton = Controller1.ButtonA; //Use button A as a placeholder

    switch (controlButtonID){ //Assigns ID to actual button
        case Up: {
            controlButton = Controller1.ButtonUp;
            break;
        }
        case Left: {
            controlButton = Controller1.ButtonLeft;
            break;
        }
        case Right: {
            controlButton = Controller1.ButtonRight;
            break;
        }
        case Down: {
            controlButton = Controller1.ButtonDown;
            break;
        }
        case X: {
            controlButton = Controller1.ButtonX;
            break;
        }
        case Y: {
            controlButton = Controller1.ButtonY;
            break;
        }
        case A: {
            controlButton = Controller1.ButtonA;
            break;
        }
        case B: {
            controlButton = Controller1.ButtonB;
            break;
        }
        case L1: {
            controlButton = Controller1.ButtonL1;
            break;
        }
        case L2: {
            controlButton = Controller1.ButtonL2;
            break;
        }
        case R1: {
            controlButton = Controller1.ButtonR1;
            break;
        }
        case R2: {
            controlButton = Controller1.ButtonR2;
            break;
        }
        default: {
            break;
        }
    }

    if (controlButton.pressing() && !buttonWasAlreadyPressed[static_cast<int> (controlButtonID)]){ //Button Pressed
        buttonWasAlreadyPressed[static_cast<int> (controlButtonID)] = true; //Sets corresponding value in array to true

        return true;
    }
    else if (!controlButton.pressing() && buttonWasAlreadyPressed[static_cast<int> (controlButtonID)]){ //Button Released
        buttonWasAlreadyPressed[static_cast<int> (controlButtonID)] = false; //Sets corresponding value in array to false
    }
    
    return false;
}

void motorToggle(directionType motorDirection, double motorVelocity, motor_group &controlMotor, ButtonID controlButtonID){ //Toggle button to spin/stop
    static bool motorState = false;

    if (pressed(controlButtonID)){
        motorState = !motorState;

        if (motorState){
            controlMotor.spin(motorDirection, motorVelocity, percent);
        }
        else {
            controlMotor.stop(brake);
        }
    }
}

void pistonToggle(digital_out &controlPiston, ButtonID controlButtonID){ //Toggle button to extend/retract
    static bool pistonState = false;

    if (pressed(controlButtonID)){
        pistonState = !pistonState;
        controlPiston.set(pistonState);
    }
}

/********** Controls **********/

void runTankDrive(double percentSpeed, bool toggleSpeed, const controller::button &toggleSpeedButton, double slowPercentSpeed){ //Each joystick controls the movement of the corresponding side of the drivetrain
    double leftDriveSpeed = Controller1.Axis3.position(percent) * percentSpeed / 100;
    double rightDriveSpeed = Controller1.Axis2.position(percent) * percentSpeed / 100;

    if (toggleSpeedButton.pressing()){ //Makes drivetrain move slower when in slowmode
        leftDriveSpeed *= slowPercentSpeed / 100;
        rightDriveSpeed *= slowPercentSpeed / 100;
    }

    LeftDrive.spin(forward, leftDriveSpeed, percent);
    RightDrive.spin(forward, rightDriveSpeed, percent);
}

void runArcadeDrive(double percentSpeed, double steerPercentSpeed, bool toggleSpeed, const controller::button &toggleSpeedButton, double slowPercentSpeed){ //Left joystick up/down controls forward backward, right joystick left/right controls turning
    double leftDriveSpeed = (Controller1.Axis3.position(percent) + (Controller1.Axis1.position(percent) * steerPercentSpeed / 100)) * percentSpeed / 100;
    double rightDriveSpeed = (Controller1.Axis3.position(percent) - (Controller1.Axis1.position(percent) * steerPercentSpeed / 100)) * percentSpeed / 100;

    if (toggleSpeedButton.pressing()){ //Makes drivetrain move slower when in slowmode
        leftDriveSpeed *= slowPercentSpeed / 100;
        rightDriveSpeed *= slowPercentSpeed / 100;
    }

    LeftDrive.spin(forward, leftDriveSpeed, percent);
    RightDrive.spin(forward, rightDriveSpeed, percent);
}

void runIntake(double percentSpeed){
    motorSeperateButton(percentSpeed, Intake, Controller1.ButtonR1, Controller1.ButtonR2, Controller1.ButtonB);
}

void runArm(double loadingPosition, double manualSpeed, double macroKp, double intakeSpeed){
    static bool loadingState = false; //True = loading, False = down
    static double targetPosition = 0;

    if (Controller1.ButtonL1.pressing()){ //Manual
        motorHold(manualSpeed, Arm, Controller1.ButtonL1, nullButton);
        motorHold(intakeSpeed, Intake, nullButton, Controller1.ButtonL1);
    }
    else { //Macro
        if (pressed(Up)){
            loadingState = !loadingState;
        }
        
        if (loadingState){
            targetPosition = loadingPosition;
        }
        else {
            targetPosition = 0;
        }

        Arm.spin(forward, macroKp * (targetPosition - ArmRotation.position(degrees)), percent);
    }
}

void runMogo(){
    pistonToggle(MogoMech, A);
}

void runDoinker(){
    pistonToggle(Doinker, X);
}

