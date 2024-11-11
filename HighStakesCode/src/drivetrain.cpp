#include "vex.h"
#include "drivetrain.h"
#include <iostream>

Drivetrain::Drivetrain(double wheelDiameter, double sidewaysToCenterDistance, double forwardToCenterDistance):
    odom(wheelDiameter, sidewaysToCenterDistance, forwardToCenterDistance)
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
    driveToPoint(targetX, targetY, defaultDriveClampConstants, defaultDriveSettleConstants, defaultDriveOutputConstants, defaultTurnClampConstants, defaultTurnSettleConstants, defaultTurnOutputConstants); 
}

void Drivetrain::driveToPoint(double targetX, double targetY, clampConstants driveClampConstants){
    driveToPoint(targetX, targetY, driveClampConstants, defaultDriveSettleConstants, defaultDriveOutputConstants, defaultTurnClampConstants, defaultTurnSettleConstants, defaultTurnOutputConstants); 
}

void Drivetrain::driveToPoint(double targetX, double targetY, clampConstants driveClampConstants, settleConstants driveSettleConstants){
    driveToPoint(targetX, targetY, driveClampConstants, driveSettleConstants, defaultDriveOutputConstants, defaultTurnClampConstants, defaultTurnSettleConstants, defaultTurnOutputConstants);
}

void Drivetrain::driveToPoint(double targetX, double targetY, clampConstants driveClampConstants, settleConstants driveSettleConstants, outputConstants driveOutputConstants){
    driveToPoint(targetX, targetY, driveClampConstants, driveSettleConstants, driveOutputConstants, defaultTurnClampConstants, defaultTurnSettleConstants, defaultTurnOutputConstants);
}

void Drivetrain::driveToPoint(double targetX, double targetY, clampConstants driveClampConstants, settleConstants driveSettleConstants, outputConstants driveOutputConstants, clampConstants turnClampConstants, settleConstants turnSettleConstants, outputConstants turnOutputConstants){
    double driveError = hypot(targetX - odom.xPosition, targetY - odom.yPosition); //Straight line distance from current coordinates to target
    double turnTarget = radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition)); //Heading that faces target
    bool turnDirection = true; //True = forward, false = reverse
    double turnError = 0;
    if (cos(degToRad(headingError(turnTarget, odom.orientation))) < 0){ //Reverses target heading if driving backwards
        turnDirection = false;
        turnError = headingError(fmod(turnTarget + 180, 360), odom.orientation);
    }
    else {
        turnDirection = true;
        turnError = headingError(turnTarget, odom.orientation);
    }
    double driveOutput = 0;
    double turnOutput = 0;
    double leftSideOutput = 0;
    double rightSideOutput = 0;
    
    //Double PID (drives and turns toward the target simultaneously)
    PID drivePID(driveError, driveOutputConstants.kp, driveOutputConstants.ki, driveOutputConstants.kd, driveOutputConstants.startI, driveSettleConstants.deadband, driveSettleConstants.loopCycleTime, driveSettleConstants.settleTime, driveSettleConstants.timeout);
    PID turnPID(turnError, turnOutputConstants.kp, turnOutputConstants.ki, turnOutputConstants.kd, turnOutputConstants.startI, turnSettleConstants.deadband, 0, 0, 0);

    while (!drivePID.isSettled(driveError)){
        driveError = hypot(targetX - odom.xPosition, targetY - odom.yPosition); //Straight line distance from current coordinates to target
        turnTarget = radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition)); //Heading that faces target
        if (turnDirection){
            turnError = headingError(turnTarget, odom.orientation);
        }
        else {
            turnError = headingError(fmod(turnTarget + 180, 360), odom.orientation);
        }
        
        driveOutput = drivePID.output(driveError) * cos(degToRad(headingError(turnTarget, odom.orientation)));
        turnOutput = turnPID.output(turnError);
        if (fabs(driveError) < turnPID.deadband){ //Prevents drastic turning when near the target
            turnOutput = 0;
        }

        //Clamps the output speeds to stay within the specified minimum and maximum speeds
        driveOutput *= driveOutputScale(driveClampConstants.minimumSpeed, driveClampConstants.maximumSpeed, driveOutput, driveOutput);
        turnOutput *= driveOutputScale(turnClampConstants.minimumSpeed, turnClampConstants.maximumSpeed, turnOutput, turnOutput);

        leftSideOutput = driveOutput + turnOutput;
        rightSideOutput = driveOutput - turnOutput;

        LeftDrive.spin(forward, leftSideOutput, percent);
        RightDrive.spin(forward, rightSideOutput, percent);

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
    double startX = odom.xPosition;
    double startY = odom.yPosition;
    double polarHeadingRad = degToRad(90 - fmod(headingToPolar(targetHeading), 360));
    double distanceTraveled = (odom.xPosition - startX) * sin(polarHeadingRad) + (odom.yPosition - startY) * cos(polarHeadingRad); //Straight line distance traveled
    double driveError = targetDistance - distanceTraveled;
    double turnError = headingError(targetHeading, odom.orientation);
    double driveOutput = 0;
    double turnOutput = 0;
    double leftSideOutput = 0;
    double rightSideOutput = 0;

    //Double PID (drives and aligns towards target simultaneously)
    PID drivePID(driveError, driveOutputConstants.kp, driveOutputConstants.ki, driveOutputConstants.kd, driveOutputConstants.startI, driveSettleConstants.deadband, driveSettleConstants.loopCycleTime, driveSettleConstants.settleTime, driveSettleConstants.timeout);
    PID turnPID(turnError, defaultDriveDistanceTurnOutputConstants.kp, defaultDriveDistanceTurnOutputConstants.kp, defaultDriveDistanceTurnOutputConstants.kp, defaultDriveDistanceTurnOutputConstants.kp, 0, 0, 0, 0);

    while (!drivePID.isSettled(driveError)){
        distanceTraveled = (odom.xPosition - startX) * sin(polarHeadingRad) + (odom.yPosition - startY) * cos(polarHeadingRad); //Straight line distance traveled
        driveError = targetDistance - distanceTraveled;
        turnError = headingError(targetHeading, odom.orientation);
        
        driveOutput = drivePID.output(driveError);
        turnOutput = turnPID.output(turnError);

        //Clamps the output speeds to stay within the specified minimum and maximum speeds
        driveOutput *= driveOutputScale(driveClampConstants.minimumSpeed, driveClampConstants.maximumSpeed, driveOutput, driveOutput);
        turnOutput *= driveOutputScale(defaultDriveDistanceTurnClampConstants.minimumSpeed, defaultDriveDistanceTurnClampConstants.maximumSpeed, turnOutput, turnOutput);
        
        leftSideOutput = driveOutput + turnError;
        rightSideOutput = driveOutput - turnError;

        LeftDrive.spin(forward, leftSideOutput, percent);
        RightDrive.spin(forward, rightSideOutput, percent);

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

        LeftDrive.spin(forward, turnOutput, percent);
        RightDrive.spin(reverse, turnOutput, percent);

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
        targetHeading = fmod(radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition)) + 180, 360);
    }
    else {
        targetHeading = radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition));
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
        swingOutput *= driveOutputScale(swingClampConstants.minimumSpeed, swingClampConstants.maximumSpeed, swingOutput, -swingOutput);

        if (driveSide == "Left"){
            LeftDrive.spin(forward, swingOutput, percent);
            RightDrive.stop(brake);
        }
        else {
            LeftDrive.stop(brake);
            RightDrive.spin(forward, -swingOutput, percent);
        }

        wait(swingSettleConstants.loopCycleTime, msec);
    }
}


void Drivetrain::stopDrive(brakeType brakeType){
    LeftDrive.stop(brakeType);
    RightDrive.stop(brakeType);
}