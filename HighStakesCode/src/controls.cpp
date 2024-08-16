#include "vex.h"
#include <controls.h>
#include <iostream>

void motorSeperateButton(double motorVelocity, motor &controlMotor, const controller::button &spinForwardButton, const controller::button &spinReverseButton, 
                         const controller::button &stopButton){ //Tap seperate buttons for spin forward, spin reverse, and stop
    controlMotor.setVelocity(motorVelocity, percent);

    if (spinForwardButton.pressing()){
        controlMotor.spin(forward);
    }
    else if (spinReverseButton.pressing()){
        controlMotor.spin(reverse);
    }
    else if (stopButton.pressing()){
        controlMotor.stop();
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

void motorHold(directionType motorDirection, double motorVelocity, motor &controlMotor, const controller::button &controlButton){ //Hold button to spin
    controlMotor.setVelocity(motorVelocity, percent);

    if (controlButton.pressing()){
        controlMotor.spin(motorDirection);
    }
    else {
        controlMotor.stop();
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

bool pressed(const controller::button &controlButton){ //Detects new presses
    static bool buttonWasAlreadyPressed = false;

    if (controlButton.pressing() && !buttonWasAlreadyPressed){ //Button Pressed
        buttonWasAlreadyPressed = true;

        return true;
    }
    else if (!controlButton.pressing() && buttonWasAlreadyPressed){ //Button Released
        buttonWasAlreadyPressed = false;
    }

    return false;
}

void motorToggle(directionType motorDirection, double motorVelocity, motor &controlMotor, const controller::button &controlButton){ //Toggle button to spin/stop
    static bool motorState = false;
    controlMotor.setVelocity(motorVelocity, percent);

    if (pressed(controlButton)){
        motorState = !motorState;

        if (motorState){
            controlMotor.spin(motorDirection);
        }
        else {
            controlMotor.stop();
        }
    }
}

void pistonToggle(digital_out &controlPiston, const controller::button &controlButton){ //Toggle button to extend/retract
    static bool pistonState = false;

    if (pressed(controlButton)){
        pistonState = !pistonState;
        controlPiston.set(pistonState);
    }
}

/********** Controls **********/

void runTankDrive(double percentSpeed, bool toggleSpeed = false, const controller::button &toggleSpeedButton = Controller1.ButtonA, 
                  double slowPercentSpeed = 0){ //Each joystick controls the movement of the corresponding side of the drivetrain
    static bool slowMode;
    double leftDriveSpeed = Controller1.Axis3.position(percent) * percentSpeed;
    double rightDriveSpeed = Controller1.Axis2.position(percent) * percentSpeed;

    if (toggleSpeed){
        if (pressed(toggleSpeedButton)){
            slowMode = !slowMode;
        }

        if (slowMode){
            leftDriveSpeed *= slowPercentSpeed;
            rightDriveSpeed *= slowPercentSpeed;
        }
    }

    LeftDrive.spin(forward, leftDriveSpeed, percent);
    RightDrive.spin(forward, rightDriveSpeed, percent);
}

void runArcadeDrive(double percentSpeed, double steerPercentSpeed, bool toggleSpeed = false, const controller::button &toggleSpeedButton = Controller1.ButtonA, 
                    double slowPercentSpeed = 0){ //
    static bool slowMode;
    double leftDriveSpeed = Controller1.Axis3.position(percent) + (Controller1.Axis1.position(percent) * steerPercentSpeed / 100) * percentSpeed;
    double rightDriveSpeed = Controller1.Axis3.position(percent) - (Controller1.Axis1.position(percent) * steerPercentSpeed / 100) * percentSpeed;

    if (toggleSpeed){
        if (pressed(toggleSpeedButton)){
            slowMode = !slowMode;
        }

        if (slowMode){
            leftDriveSpeed *= slowPercentSpeed;
            rightDriveSpeed *= slowPercentSpeed;
        }
    }

    LeftDrive.spin(forward, leftDriveSpeed, percent);
    RightDrive.spin(forward, rightDriveSpeed, percent);
}

void runIntake(){

}

void runMogo(){
    pistonToggle(MogoMech, Controller1.ButtonA);
}

void runRedirect(){
    
} 

