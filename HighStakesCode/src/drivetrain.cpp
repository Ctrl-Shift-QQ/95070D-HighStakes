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
    driveToPoint(targetX, targetY, defaultDriveOutputConstants, defaultTurnOutputConstants, defaultDriveSettleConstants); 
}

void Drivetrain::driveToPoint(double targetX, double targetY, outputConstants driveOutputConstants, settleConstants driveSettleConstants){
    driveToPoint(targetX, targetY, driveOutputConstants, defaultTurnOutputConstants, driveSettleConstants);
}

void Drivetrain::driveToPoint(double targetX, double targetY, outputConstants driveOutputConstants, outputConstants turnOutputConstants, settleConstants driveSettleConstants){
    double driveError = hypot(targetX - odom.xPosition, targetY - odom.yPosition); //Straight line distance from current coordinates to target
    double turnTarget = radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition)); //Heading that faces target
    double turnError;
    if (cos(degToRad(headingError(turnTarget, odom.orientation))) < 0){ //Reverses target heading if driving backwards
        turnError = headingError(fmod(turnTarget + 180, 360), odom.orientation);
    }
    else {
        turnError = headingError(turnTarget, odom.orientation);
    }
    
    //Double PID (drives and turns toward the target simultaneously)
    PID drivePID(driveError, driveOutputConstants.kp, driveOutputConstants.ki, driveOutputConstants.kd, driveOutputConstants.startI, driveSettleConstants.deadband, driveSettleConstants.loopCycleTime, driveSettleConstants.settleTime, driveSettleConstants.timeout);
    PID turnPID(turnError, turnOutputConstants.kp, turnOutputConstants.ki, turnOutputConstants.kd, turnOutputConstants.startI, driveSettleConstants.deadband, driveSettleConstants.loopCycleTime, driveSettleConstants.settleTime, driveSettleConstants.timeout);

    while (!drivePID.isSettled(driveError)){
        driveError = hypot(targetX - odom.xPosition, targetY - odom.yPosition); //Straight line distance from current coordinates to target
        turnTarget = radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition)); //Heading that faces target
        if (cos(degToRad(headingError(turnTarget, odom.orientation))) < 0){ //Reverses target heading if driving backwards
            turnError = headingError(fmod(turnTarget + 180, 360), odom.orientation);
        }
        else {
            turnError = headingError(turnTarget, odom.orientation);
        }

        double leftDriveOutput = drivePID.output(driveError) * cos(degToRad(headingError(turnTarget, odom.orientation))) + turnPID.output(turnError);
        double rightDriveOutput = drivePID.output(driveError) * cos(degToRad(headingError(turnTarget, odom.orientation))) - turnPID.output(turnError);

        //Clamps the output speeds to stay within the specified minimum and maximum speeds
        leftDriveOutput *= driveOutputScale(driveOutputConstants.minimumSpeed, leftDriveOutput, rightDriveOutput);
        rightDriveOutput *= driveOutputScale(driveOutputConstants.minimumSpeed, leftDriveOutput, rightDriveOutput);
        
        LeftDrive.spin(forward, leftDriveOutput, percent);
        RightDrive.spin(forward, rightDriveOutput, percent);

        wait(driveSettleConstants.loopCycleTime, msec);
    }
}


void Drivetrain::driveDistance(double targetDistance){
    driveDistance(targetDistance, defaultDriveOutputConstants, defaultTurnOutputConstants, defaultDriveSettleConstants);
}

void Drivetrain::driveDistance(double targetDistance, outputConstants driveOutputConstants, settleConstants driveSettleConstants){
    driveDistance(targetDistance, driveOutputConstants, defaultTurnOutputConstants, driveSettleConstants);
}

void Drivetrain::driveDistance(double targetDistance, outputConstants driveOutputConstants, outputConstants turnOutputConstants, settleConstants driveSettleConstants){
    double targetX = odom.xPosition + targetDistance * cos(degToRad(headingToPolar(odom.orientation)));
    double targetY = odom.yPosition + targetDistance * sin(degToRad(headingToPolar(odom.orientation)));

    driveToPoint(targetX, targetY, driveOutputConstants, turnOutputConstants, driveSettleConstants);
}


void Drivetrain::turnToHeading(double targetHeading){
    turnToHeading(targetHeading, defaultTurnOutputConstants, defaultTurnSettleConstants);
}

void Drivetrain::turnToHeading(double targetHeading, outputConstants turnOutputConstants){
    turnToHeading(targetHeading, turnOutputConstants, defaultTurnSettleConstants);
}

void Drivetrain::turnToHeading(double targetHeading, outputConstants turnOutputConstants, settleConstants turnSettleConstants){
    double turnError = headingError(targetHeading, odom.orientation);

    PID turnPID(turnError, turnOutputConstants.kp, turnOutputConstants.ki, turnOutputConstants.kd, turnOutputConstants.startI, turnSettleConstants.deadband, turnSettleConstants.loopCycleTime, turnSettleConstants.settleTime, turnSettleConstants.timeout);

    while (!turnPID.isSettled(turnError)){
        turnError = headingError(targetHeading, odom.orientation);

        double driveOutput = turnPID.output(turnError);

        //Clamps the output speeds to stay within the specified minimum and maximum speeds
        driveOutput *= driveOutputScale(turnOutputConstants.minimumSpeed, driveOutput, -driveOutput);

        LeftDrive.spin(forward, driveOutput, percent);
        RightDrive.spin(reverse, driveOutput, percent);

        wait(turnSettleConstants.loopCycleTime, msec);
    }
}


void Drivetrain::turnToPoint(double targetX, double targetY){
    turnToPoint(targetX, targetY, defaultTurnOutputConstants, defaultTurnSettleConstants);
}

void Drivetrain::turnToPoint(double targetX, double targetY, outputConstants turnOutputConstants){
    turnToPoint(targetX, targetY, turnOutputConstants, defaultTurnSettleConstants);
}

void Drivetrain::turnToPoint(double targetX, double targetY, outputConstants turnOutputConstants, settleConstants turnSettleConstants){
    double targetHeading = radToDeg(atan2(targetX, targetY));

    turnToHeading(targetHeading, turnOutputConstants, turnSettleConstants);
}


void Drivetrain::stopDrive(vex::brakeType brakeType){
    LeftDrive.stop(brakeType);
    RightDrive.stop(brakeType);
}











