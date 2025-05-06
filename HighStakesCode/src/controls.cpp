#include "vex.h"
#include "controls.h"
#include <iostream>

static controller dummyController;
static controller::button nullButton = dummyController.ButtonA; //Placeholder for button when valid button not wanted

const controller::button& IDToButton(ButtonID buttonID){
    switch (buttonID){ //Returns corresponding button
        case Up: {
            return Controller1.ButtonUp;
        }
        case Left: {
            return Controller1.ButtonLeft;
        }
        case Right: {
            return Controller1.ButtonRight;
        }
        case Down: {
            return Controller1.ButtonDown;
        }
        case X: {
            return Controller1.ButtonX;
        }
        case Y: {
            return Controller1.ButtonY;
        }
        case A: {
            return Controller1.ButtonA;
        }
        case B: {
            return Controller1.ButtonB;
        }
        case L1: {
            return Controller1.ButtonL1;
        }
        case L2: {
            return Controller1.ButtonL2;
        }
        case R1: {
            return Controller1.ButtonR1;
        }
        case R2: {
            return Controller1.ButtonR2;
        }
        default: {
            return nullButton;
        }
    }
}

bool pressed(ButtonID controlButtonID){ //Detects new presses
    static bool buttonWasAlreadyPressed[] = {false, false, false, false, false, false, false, false, false, false, false, false}; //Saves statuses of all buttons
    controller::button controlButton = IDToButton(controlButtonID);


    if (controlButton.pressing() && !buttonWasAlreadyPressed[static_cast<int> (controlButtonID)]){ //Button Pressed
        buttonWasAlreadyPressed[static_cast<int> (controlButtonID)] = true; //Sets corresponding value in array to true

        return true;
    }
    else if (!controlButton.pressing() && buttonWasAlreadyPressed[static_cast<int> (controlButtonID)]){ //Button Released
        buttonWasAlreadyPressed[static_cast<int> (controlButtonID)] = false; //Sets corresponding value in array to false
    }
    
    return false;
}

bool released(ButtonID controlButtonID){ //Detects new releases
    static bool buttonWasAlreadyReleased[] = {false, false, false, false, false, false, false, false, false, false, false, false}; //Saves statuses of all buttons
    controller::button controlButton = IDToButton(controlButtonID);

    if (!controlButton.pressing() && !buttonWasAlreadyReleased[static_cast<int> (controlButtonID)]){ //Button Released
        buttonWasAlreadyReleased[static_cast<int> (controlButtonID)] = true; //Sets corresponding value in array to true

        return true;
    }
    else if (controlButton.pressing() && buttonWasAlreadyReleased[static_cast<int> (controlButtonID)]){ //Button Pressed
        buttonWasAlreadyReleased[static_cast<int> (controlButtonID)] = false; //Sets corresponding value in array to false
    }
    
    return false;
}

void motorSeperateButton(double percentSpeed, motor_group &controlMotor, ButtonID spinForwardButtonID, ButtonID spinReverseButtonID, 
                         ButtonID stopButtonID){ //Tap seperate buttons for spin forward, spin reverse, and stop
    if (IDToButton(spinForwardButtonID).pressing()){
        controlMotor.spin(forward, percentToVolts(percentSpeed), volt);
    }
    else if (IDToButton(spinReverseButtonID).pressing()){
        controlMotor.spin(reverse, percentToVolts(percentSpeed), volt);
    }
    else if (IDToButton(stopButtonID).pressing()){
        controlMotor.stop(brake);
    }
}

void pistonSeperateButton(digital_out &controlPiston, ButtonID extendButtonID, ButtonID retractButtonID){ //Tap seperate buttons for extend and retract
    if (IDToButton(extendButtonID).pressing()){
        controlPiston.set(true);
    }
    else if (IDToButton(retractButtonID).pressing()){
        controlPiston.set(false);
    }
}

void motorHold(double percentSpeed, motor_group &controlMotor, ButtonID forwardButtonID, ButtonID reverseButtonID){ //Hold button to spin
    if (IDToButton(forwardButtonID).pressing()){
        controlMotor.spin(forward, percentToVolts(percentSpeed), volt);
    }
    else if (IDToButton(reverseButtonID).pressing()){
        controlMotor.spin(reverse, percentToVolts(percentSpeed), volt);
    }
    else if (released(forwardButtonID) || released(reverseButtonID)){
        controlMotor.stop(brake);
    }
}

void pistonHold(bool reverse, digital_out &controlPiston, ButtonID controlButtonID){ //Hold button to extend/retract
    if ((!reverse && IDToButton(controlButtonID).pressing()) || (reverse && !IDToButton(controlButtonID).pressing())){
        controlPiston.set(true);
    }
    else if (released(controlButtonID)){
        controlPiston.set(false);
    }
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


void runTankDrive(double percentSpeed, bool toggleSpeed, ButtonID toggleSpeedButtonID, double slowPercentSpeed){ //Each joystick controls the movement of the corresponding side of the drivetrain
    double leftDriveSpeed = Controller1.Axis3.position(percent) * percentSpeed / 100;
    double rightDriveSpeed = Controller1.Axis2.position(percent) * percentSpeed / 100;

    if (toggleSpeed && IDToButton(toggleSpeedButtonID).pressing()){ //Makes drivetrain move slower when in slowmode
        leftDriveSpeed *= slowPercentSpeed / 100;
        rightDriveSpeed *= slowPercentSpeed / 100;
    }

    LeftDrive.spin(forward, percentToVolts(leftDriveSpeed), volt);
    RightDrive.spin(forward, percentToVolts(rightDriveSpeed), volt);
}

void runArcadeDrive(double percentSpeed, double steerPercentSpeed, bool toggleSpeed, ButtonID toggleSpeedButtonID, double slowPercentSpeed){ //Left joystick up/down controls forward backward, right joystick left/right controls turning
    double leftDriveSpeed = (Controller1.Axis3.position(percent) + (Controller1.Axis1.position(percent) * steerPercentSpeed / 100)) * percentSpeed / 100;
    double rightDriveSpeed = (Controller1.Axis3.position(percent) - (Controller1.Axis1.position(percent) * steerPercentSpeed / 100)) * percentSpeed / 100;

    if (toggleSpeed && IDToButton(toggleSpeedButtonID).pressing()){ //Makes drivetrain move slower when in slowmode
        leftDriveSpeed *= slowPercentSpeed / 100;
        rightDriveSpeed *= slowPercentSpeed / 100;
    }

    LeftDrive.spin(forward, percentToVolts(leftDriveSpeed), volt);
    RightDrive.spin(forward, percentToVolts(rightDriveSpeed), volt);
}

void runIntake(){
    IntakeLift.set(false);
    motorSeperateButton(INTAKE_DEFAULT_SPEED, Intake, INTAKE_FORWARD_BUTTON_ID, INTAKE_REVERSE_BUTTON_ID, INTAKE_STOP_BUTTON_ID);
}

void runArm(){
    static int currentMacro = 0; //Used for indexing
    static double macroPositions[] = {0, ARM_LOADING_POSITION, ARM_UP_POSITION};
    static double targetPosition = 0;
    static double PIDOutput = 0;
    static double FFOutput = 0;
    static double output = 0;

    static PID armPID(targetPosition, ARM_MACRO_KP, 0, 0, 0, 0, DEFAULT_LOOP_CYCLE_TIME, 0, 0);

    if (IDToButton(ARM_SPIN_BUTTON_ID).pressing()){ //Manual
        motorHold(ARM_MANUAL_SPEED, Arm, ARM_SPIN_BUTTON_ID, Null);
        motorHold(ARM_INTAKE_SPEED, Intake, Null, ARM_SPIN_BUTTON_ID);
    }
    else { //Macro
        if (pressed(ARM_TOGGLE_STATE_BUTTON_ID)){
            if (currentMacro == (sizeof(macroPositions) / sizeof(macroPositions[0])) - 1){
                currentMacro = 0;
            }
            else {
                currentMacro++;
            }

            targetPosition = macroPositions[currentMacro];
            armPID.startError = targetPosition;

            if (targetPosition > ARM_LOADING_POSITION){
                FirstIntake.stop(brake);
                SecondIntake.spin(reverse, ARM_INTAKE_SPEED, percent);
            } 
        }
        else if (pressed(ARM_DESCORE_BUTTON_ID)){
            currentMacro = 0; //Sets next position to load

            targetPosition = ARM_DESCORE_POSITION;
            armPID.startError = targetPosition;

            if (targetPosition > ARM_LOADING_POSITION){
                SecondIntake.spin(reverse, ARM_INTAKE_SPEED, percent);
            }
        }

        PIDOutput = armPID.output(targetPosition - ArmRotation.position(degrees));
        FFOutput = ARM_MACRO_KCOS * cos(degToRad(ArmRotation.position(degrees)));

        output = PIDOutput + FFOutput;

        Arm.spin(forward, percentToVolts(output), volt);
    }
}

void runMogo(){
    pistonToggle(MogoMech, MOGO_BUTTON_ID);
}

void runDoinker(){
    pistonToggle(RightDoinker, DOINKER_BUTTON_ID);
}

