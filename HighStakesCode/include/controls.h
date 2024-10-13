void motorSeperateButton(double motorVelocity, motor_group &controlMotor, const controller::button &spinForwardButton, const controller::button &spinReverseButton, 
                         const controller::button &stopButton);
void pistonSeperateButton(digital_out &controlPiston, const controller::button &extendButton, const controller::button &retractButton);
void motorHold(double motorVelocity, motor_group &controlMotor, const controller::button &forwardButton, const controller::button &reverseButton);
void pistonHold(bool reverse, digital_out &controlPiston, const controller::button &controlButton);
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
} ButtonID;
bool pressed(ButtonID controlButtonID);
void motorToggle(directionType motorDirection, double motorVelocity, motor_group &controlMotor, ButtonID controlButtonID);
void pistonToggle(digital_out &controlPiston, ButtonID controlButtonID);

/********** Controls **********/

void runTankDrive(double percentSpeed, bool toggleSpeed = false, ButtonID toggleSpeedButtonID = A, double slowPercentSpeed = 0);
void runArcadeDrive(double percentSpeed, double steerPercentSpeed, bool toggleSpeed = false, ButtonID toggleSpeedButtonID = A, double slowPercentSpeed = 0);
void runIntake(double percentSpeed);
void runArm();
void runMogo();
void runDoinker();