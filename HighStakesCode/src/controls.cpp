#include "vex.h"
#include "controls.h"
#include <iostream>

static controller dummyController;
static controller::button nullButton = dummyController.ButtonA; //Placeholder for button when valid button not wanted

void motorSeperateButton(double percentSpeed, motor_group &controlMotor, const controller::button &spinForwardButton, const controller::button &spinReverseButton, 
                         const controller::button &stopButton){ //Tap seperate buttons for spin forward, spin reverse, and stop
    if (spinForwardButton.pressing()){
        controlMotor.spin(forward, percentToVolts(percentSpeed), volt);
    }
    else if (spinReverseButton.pressing()){
        controlMotor.spin(reverse, percentToVolts(percentSpeed), volt);
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

void motorHold(double percentSpeed, motor_group &controlMotor, const controller::button &forwardButton, const controller::button &reverseButton){ //Hold button to spin
    if (forwardButton.pressing()){
        controlMotor.spin(forward, percentToVolts(percentSpeed), volt);
    }
    else if (reverseButton.pressing()){
        controlMotor.spin(reverse, percentToVolts(percentSpeed), volt);
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

void motorToggle(directionType motorDirection, double percentSpeed, motor_group &controlMotor, ButtonID controlButtonID){ //Toggle button to spin/stop
    static bool motorState = false;

    if (pressed(controlButtonID)){
        motorState = !motorState;

        if (motorState){
            controlMotor.spin(motorDirection, percentToVolts(percentSpeed), volt);
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

/******************** Controls ********************/

void runTankDrive(double percentSpeed, bool toggleSpeed, const controller::button &toggleSpeedButton, double slowPercentSpeed){ //Each joystick controls the movement of the corresponding side of the drivetrain
    double leftDriveSpeed = Controller1.Axis3.position(percent) * percentSpeed / 100;
    double rightDriveSpeed = Controller1.Axis2.position(percent) * percentSpeed / 100;

    if (toggleSpeedButton.pressing()){ //Makes drivetrain move slower when in slowmode
        leftDriveSpeed *= slowPercentSpeed / 100;
        rightDriveSpeed *= slowPercentSpeed / 100;
    }

    LeftDrive.spin(forward, percentToVolts(leftDriveSpeed), volt);
    RightDrive.spin(forward, percentToVolts(rightDriveSpeed), volt);
}

void runArcadeDrive(double percentSpeed, double steerPercentSpeed, bool toggleSpeed, const controller::button &toggleSpeedButton, double slowPercentSpeed){ //Left joystick up/down controls forward backward, right joystick left/right controls turning
    double leftDriveSpeed = (Controller1.Axis3.position(percent) + (Controller1.Axis1.position(percent) * steerPercentSpeed / 100)) * percentSpeed / 100;
    double rightDriveSpeed = (Controller1.Axis3.position(percent) - (Controller1.Axis1.position(percent) * steerPercentSpeed / 100)) * percentSpeed / 100;

    if (toggleSpeedButton.pressing()){ //Makes drivetrain move slower when in slowmode
        leftDriveSpeed *= slowPercentSpeed / 100;
        rightDriveSpeed *= slowPercentSpeed / 100;
    }

    LeftDrive.spin(forward, percentToVolts(leftDriveSpeed), volt);
    RightDrive.spin(forward, percentToVolts(rightDriveSpeed), volt);
}

void runIntake(){
    motorSeperateButton(INTAKE_DEFAULT_SPEED, Intake, INTAKE_FORWARD_BUTTON, INTAKE_REVERSE_BUTTON, INTAKE_STOP_BUTTON);
}

void runArm(){
    static int currentMacro = 0; //Used for indexing
    static double macroPositions[] = {0, ARM_LOADING_POSITION};
    static double targetPosition = 0;
    
    static PID armPID(0, ARM_MACRO_KP, ARM_MACRO_KI, 0, ARM_MACRO_START_I, 0, DEFAULT_LOOP_CYCLE_TIME, 0, 0);

    if (ARM_SPIN_BUTTON.pressing()){ //Manual
        motorHold(ARM_MANUAL_SPEED, Arm, ARM_SPIN_BUTTON, nullButton);
        motorHold(ARM_INTAKE_SPEED, Intake, nullButton, ARM_SPIN_BUTTON);

        armPID.integral = 0;
    }
    else { //Macro
        if (pressed(ARM_TOGGLE_STATE_BUTTON_ID)){
            if (currentMacro == (sizeof(macroPositions) / sizeof(macroPositions[0])) - 1){
                currentMacro = 0;
            }
            else {
                currentMacro++;
            }
        }
        
        targetPosition = macroPositions[currentMacro];
        Arm.spin(forward, percentToVolts(armPID.output(targetPosition - ArmRotation.position(degrees))), volt);
    }
}

void runMogo(){
    pistonToggle(MogoMech, MOGO_BUTTON_ID);
}

void runDoinker(){
    pistonToggle(Doinker, DOINKER_BUTTON_ID);
}

