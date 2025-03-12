#include "vex.h"
#include "drivetrain.h"
#include <iostream>

Drivetrain::Drivetrain(double horizontalWheelDiameter, double verticalWheelDiameter, double horizontalToCenterDistance, double verticalToCenterDistance, double inertialScale):
    odom(horizontalWheelDiameter, verticalWheelDiameter, horizontalToCenterDistance, verticalToCenterDistance, inertialScale)
{};

/******************** Odometry ********************/

int Drivetrain::trackPosition(){
    while (true){
        chassis.odom.updatePosition();

        wait(5, msec);
    }

    return 0;
}

void Drivetrain::setCoordinates(double startPositionX, double startPositionY, double startPositionOrientation){
    odom.setPosition(startPositionX, startPositionY, startPositionOrientation);
    positionTrackTask = task(trackPosition);
}

/******************** Motion Algorithms ********************/

void Drivetrain::driveToPoint(double targetX, double targetY){
    driveToPoint(targetX, targetY, defaultDriveClampConstants, defaultDriveSettleConstants, defaultDriveOutputConstants, defaultHeadingClampConstants, defaultHeadingSettleConstants, defaultHeadingOutputConstants); 
}

void Drivetrain::driveToPoint(double targetX, double targetY, clampConstants driveClampConstants){
    driveToPoint(targetX, targetY, driveClampConstants, defaultDriveSettleConstants, defaultDriveOutputConstants, defaultHeadingClampConstants, defaultHeadingSettleConstants, defaultHeadingOutputConstants); 
}

void Drivetrain::driveToPoint(double targetX, double targetY, clampConstants driveClampConstants, settleConstants driveSettleConstants){
    driveToPoint(targetX, targetY, driveClampConstants, driveSettleConstants, defaultDriveOutputConstants, defaultHeadingClampConstants, defaultHeadingSettleConstants, defaultHeadingOutputConstants);
}

void Drivetrain::driveToPoint(double targetX, double targetY, clampConstants driveClampConstants, settleConstants driveSettleConstants, outputConstants driveOutputConstants){
    driveToPoint(targetX, targetY, driveClampConstants, driveSettleConstants, driveOutputConstants, defaultHeadingClampConstants, defaultHeadingSettleConstants, defaultHeadingOutputConstants);
}

void Drivetrain::driveToPoint(double targetX, double targetY, clampConstants driveClampConstants, settleConstants driveSettleConstants, outputConstants driveOutputConstants, clampConstants turnClampConstants, settleConstants turnSettleConstants, outputConstants turnOutputConstants){
    double driveError = hypot(targetX - odom.xPosition, targetY - odom.yPosition); //Straight line distance from current coordinates to target
    double turnTarget = fmod(radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition)) + 360, 360); //Heading that faces target
    double turnError = 0;
    if (fabs(headingError(turnTarget, odom.orientation)) > 90){ //Reverses target heading if driving backwards
        turnError = headingError(fmod(turnTarget + 180, 360), odom.orientation);
    }
    else {
        turnError = headingError(turnTarget, odom.orientation);
    }
    double driveOutput = 0;
    double turnOutput = 0;
    double leftSideOutput = 0;
    double rightSideOutput = 0;
    
    //Double PID (drives and turns toward the target simultaneously)
    PID drivePID(driveError, driveOutputConstants.kp, driveOutputConstants.ki, driveOutputConstants.kd, driveOutputConstants.startI, driveSettleConstants.deadband, driveSettleConstants.loopCycleTime, driveSettleConstants.settleTime, driveSettleConstants.timeout);
    PID turnPID(turnError, turnOutputConstants.kp, turnOutputConstants.ki, turnOutputConstants.kd, turnOutputConstants.startI, turnSettleConstants.deadband, driveSettleConstants.loopCycleTime, 0, 0);

    while (!drivePID.isSettled(driveError)){
        driveError = hypot(targetX - odom.xPosition, targetY - odom.yPosition); //Straight line distance from current coordinates to target
        turnTarget = fmod(radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition)) + 360, 360); //Heading that faces target
        turnError = headingError(turnTarget, odom.orientation);

        driveOutput = drivePID.output(driveError) * cos(degToRad(turnError));
        if (fabs(headingError(turnTarget, odom.orientation)) > 90){ //Reverses target heading if driving backwards
            turnError = headingError(fmod(turnTarget + 180, 360), odom.orientation);
        }
        turnOutput = turnPID.output(turnError);

        if (fabs(driveError) < turnSettleConstants.deadband){ //Prevents drastic turning when near the target
            turnOutput = 0;
        }

        //Clamps the output speeds to stay within the specified minimum and maximum speeds
        driveOutput *= driveOutputScale(driveClampConstants.minimumSpeed, driveClampConstants.maximumSpeed, driveOutput, driveOutput);
        turnOutput *= driveOutputScale(turnClampConstants.minimumSpeed, turnClampConstants.maximumSpeed, turnOutput, turnOutput);

        leftSideOutput = driveOutput + turnOutput;
        rightSideOutput = driveOutput - turnOutput;

        LeftDrive.spin(forward, percentToVolts(leftSideOutput), volt);
        RightDrive.spin(forward, percentToVolts(rightSideOutput), volt);

        wait(driveSettleConstants.loopCycleTime, msec);
    }
}


void Drivetrain::driveDistance(double targetDistance){
    driveDistance(targetDistance, odom.orientation, defaultDriveClampConstants, defaultDriveSettleConstants, defaultDriveOutputConstants);
}

void Drivetrain::driveDistance(double targetDistance, double targetHeading){
    driveDistance(targetDistance, targetHeading, defaultDriveClampConstants, defaultDriveSettleConstants, defaultDriveOutputConstants);
}

void Drivetrain::driveDistance(double targetDistance, double targetHeading, clampConstants driveClampConstants){
    driveDistance(targetDistance, targetHeading, driveClampConstants, defaultDriveSettleConstants, defaultDriveOutputConstants);
}

void Drivetrain::driveDistance(double targetDistance, double targetHeading, clampConstants driveClampConstants, settleConstants driveSettleConstants){
    driveDistance(targetDistance, targetHeading, driveClampConstants, driveSettleConstants, defaultDriveOutputConstants);
}

void Drivetrain::driveDistance(double targetDistance, double targetHeading, clampConstants driveClampConstants, settleConstants driveSettleConstants, outputConstants driveOutputConstants){
    double startPosition = RightBack.position(turns);
    double driveError = targetDistance - (DRIVETRAIN_GEAR_RATIO * (RightBack.position(turns) - startPosition) * M_PI * DRIVETRAIN_VERTICAL_WHEEL_DIAMETER);
    double turnError = headingError(targetHeading, odom.orientation);
    double driveOutput = 0;
    double turnOutput = 0;
    double leftSideOutput = 0;
    double rightSideOutput = 0;

    //Double PID (drives and aligns towards target simultaneously)
    PID drivePID(driveError, driveOutputConstants.kp, driveOutputConstants.ki, driveOutputConstants.kd, driveOutputConstants.startI, driveSettleConstants.deadband, driveSettleConstants.loopCycleTime, driveSettleConstants.settleTime, driveSettleConstants.timeout);
    PID turnPID(turnError, defaultDriveDistanceTurnOutputConstants.kp, defaultDriveDistanceTurnOutputConstants.ki, defaultDriveDistanceTurnOutputConstants.kd, defaultDriveDistanceTurnOutputConstants.startI, 0, defaultTurnSettleConstants.loopCycleTime, 0, 0);

    while (!drivePID.isSettled(driveError)){
        driveError = targetDistance - (DRIVETRAIN_GEAR_RATIO * (RightBack.position(turns) - startPosition) * M_PI * DRIVETRAIN_VERTICAL_WHEEL_DIAMETER);
        turnError = headingError(targetHeading, odom.orientation);
        
        driveOutput = drivePID.output(driveError);
        turnOutput = turnPID.output(turnError);

        //Clamps the output speeds to stay within the specified minimum and maximum speeds
        driveOutput *= driveOutputScale(driveClampConstants.minimumSpeed, driveClampConstants.maximumSpeed, driveOutput, driveOutput);
        turnOutput *= driveOutputScale(defaultDriveDistanceTurnClampConstants.minimumSpeed, defaultDriveDistanceTurnClampConstants.maximumSpeed, turnOutput, turnOutput);

        leftSideOutput = driveOutput + turnOutput;
        rightSideOutput = driveOutput - turnOutput;

        LeftDrive.spin(forward, percentToVolts(leftSideOutput), volt);
        RightDrive.spin(forward, percentToVolts(rightSideOutput), volt);

        wait(driveSettleConstants.loopCycleTime, msec);
    }
}


void Drivetrain::turnToHeading(double targetHeading){
    turnToHeading(targetHeading, defaultTurnClampConstants, defaultTurnSettleConstants, defaultTurnOutputConstants);
}

void Drivetrain::turnToHeading(double targetHeading, clampConstants turnClampConstants){
    turnToHeading(targetHeading, turnClampConstants, defaultTurnSettleConstants, defaultTurnOutputConstants);
}

void Drivetrain::turnToHeading(double targetHeading, clampConstants turnClampConstants, settleConstants turnSettleConstants){
    turnToHeading(targetHeading, turnClampConstants, turnSettleConstants, defaultTurnOutputConstants);
}

void Drivetrain::turnToHeading(double targetHeading, clampConstants turnClampConstants, settleConstants turnSettleConstants, outputConstants turnOutputConstants){
    double turnError = headingError(targetHeading, odom.orientation);
    double turnOutput = 0;

    PID turnPID(turnError, turnOutputConstants.kp, turnOutputConstants.ki, turnOutputConstants.kd, turnOutputConstants.startI, 
                           turnSettleConstants.deadband, turnSettleConstants.loopCycleTime, turnSettleConstants.settleTime, turnSettleConstants.timeout);

    while (!turnPID.isSettled(turnError)){
        turnError = headingError(targetHeading, odom.orientation);

        turnOutput = turnPID.output(turnError);

        //Clamps the output speeds to stay within the specified minimum and maximum speeds
        turnOutput *= driveOutputScale(turnClampConstants.minimumSpeed, turnClampConstants.maximumSpeed, turnOutput, -turnOutput);

        LeftDrive.spin(forward, percentToVolts(turnOutput), volt);
        RightDrive.spin(reverse, percentToVolts(turnOutput), volt);
        
        wait(turnSettleConstants.loopCycleTime, msec);
    }
}


void Drivetrain::turnToPoint(bool reversed, double targetX, double targetY){
    turnToPoint(reversed, targetX, targetY, defaultTurnClampConstants, defaultTurnSettleConstants, defaultTurnOutputConstants);
}

void Drivetrain::turnToPoint(bool reversed, double targetX, double targetY, clampConstants turnClampConstants){
    turnToPoint(reversed, targetX, targetY, turnClampConstants, defaultTurnSettleConstants, defaultTurnOutputConstants);
}

void Drivetrain::turnToPoint(bool reversed, double targetX, double targetY, clampConstants turnClampConstants, settleConstants turnSettleConstants){
    turnToPoint(reversed, targetX, targetY, turnClampConstants, turnSettleConstants, defaultTurnOutputConstants);
}

void Drivetrain::turnToPoint(bool reversed, double targetX, double targetY, clampConstants turnClampConstants, settleConstants turnSettleConstants, outputConstants turnOutputConstants){
    double targetHeading = 0;

    if (reversed){
        targetHeading = fmod(radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition)) + 540, 360);
    }
    else {
        targetHeading = fmod(radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition)) + 360, 360);
    }

    turnToHeading(targetHeading, turnClampConstants, turnSettleConstants, turnOutputConstants);
}


void Drivetrain::swingToHeading(std::string driveSide, double targetHeading){
    swingToHeading(driveSide, targetHeading, defaultSwingClampConstants, defaultSwingSettleConstants, defaultSwingOutputConstants);
}

void Drivetrain::swingToHeading(std::string driveSide, double targetHeading, clampConstants swingClampConstants){
    swingToHeading(driveSide, targetHeading, swingClampConstants, defaultSwingSettleConstants, defaultSwingOutputConstants);
}

void Drivetrain::swingToHeading(std::string driveSide, double targetHeading, clampConstants swingClampConstants, settleConstants swingSettleConstants){
    swingToHeading(driveSide, targetHeading, swingClampConstants, swingSettleConstants, defaultSwingOutputConstants);
}

void Drivetrain::swingToHeading(std::string driveSide, double targetHeading, clampConstants swingClampConstants, settleConstants swingSettleConstants, outputConstants swingOutputConstants){
    double swingError = headingError(targetHeading, odom.orientation);
    double swingOutput = 0;

    PID swingPID(swingError, swingOutputConstants.kp, swingOutputConstants.ki, swingOutputConstants.kd, swingOutputConstants.startI, swingSettleConstants.deadband, swingSettleConstants.loopCycleTime, swingSettleConstants.settleTime, swingSettleConstants.timeout);

    while (!swingPID.isSettled(swingError)){
        swingError = headingError(targetHeading, odom.orientation);

        swingOutput = swingPID.output(swingError);

        //Clamps the output speeds to stay within the specified minimum and maximum speeds
        swingOutput *= driveOutputScale(swingClampConstants.minimumSpeed, swingClampConstants.maximumSpeed, swingOutput, swingOutput);

        if (driveSide == "Left"){
            LeftDrive.spin(forward, percentToVolts(swingOutput), volt);
            RightDrive.stop(brake);
        }
        else {
            LeftDrive.stop(brake);
            RightDrive.spin(forward, percentToVolts(-swingOutput), volt);
        }

        wait(swingSettleConstants.loopCycleTime, msec);
    }
}


void Drivetrain::stopDrive(brakeType brakeType){
    LeftDrive.stop(brakeType);
    RightDrive.stop(brakeType);
}