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
    driveToPoint(targetX, targetY, defaultDriveSettleConstants, defaultDriveOutputConstants, defaultTurnOutputConstants); 
}

void Drivetrain::driveToPoint(double targetX, double targetY, settleConstants driveSettleConstants){
    driveToPoint(targetX, targetY, driveSettleConstants, defaultDriveOutputConstants, defaultTurnOutputConstants); 
}

void Drivetrain::driveToPoint(double targetX, double targetY, settleConstants driveSettleConstants, outputConstants driveOutputConstants){
    driveToPoint(targetX, targetY, driveSettleConstants, driveOutputConstants, defaultTurnOutputConstants);
}

void Drivetrain::driveToPoint(double targetX, double targetY, settleConstants driveSettleConstants, outputConstants driveOutputConstants, outputConstants turnOutputConstants){
    double driveError = hypot(targetX - odom.xPosition, targetY - odom.yPosition); //Straight line distance from current coordinates to target
    double turnTarget = radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition)); //Heading that faces target
    double turnError = 0;
    if (cos(degToRad(headingError(turnTarget, odom.orientation))) < 0){ //Reverses target heading if driving backwards
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
    PID turnPID(turnError, turnOutputConstants.kp, turnOutputConstants.ki, turnOutputConstants.kd, turnOutputConstants.startI, driveSettleConstants.deadband, 0, 0, 0);

    while (!drivePID.isSettled(driveError)){
        driveError = hypot(targetX - odom.xPosition, targetY - odom.yPosition); //Straight line distance from current coordinates to target
        turnTarget = radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition)); //Heading that faces target
        if (cos(degToRad(headingError(turnTarget, odom.orientation))) < 0){ //Reverses target heading if driving backwards
            turnError = headingError(fmod(turnTarget + 180, 360), odom.orientation);
        }
        else {
            turnError = headingError(turnTarget, odom.orientation);
        }

        driveOutput = drivePID.output(driveError) * cos(degToRad(headingError(turnTarget, odom.orientation)));
        turnOutput = turnPID.output(turnError);
        if (fabs(driveError) < turnPID.deadband){ //Prevents drastic turning when near the target
            turnOutput = 0;
        }

        //Clamps the output speeds to stay within the specified minimum and maximum speeds
        driveOutput *= driveOutputScale(driveOutputConstants.minimumSpeed, driveOutputConstants.maximumSpeed, driveOutput, driveOutput);
        turnOutput *= driveOutputScale(turnOutputConstants.minimumSpeed, turnOutputConstants.maximumSpeed, turnOutput, turnOutput);

        leftSideOutput = driveOutput + turnOutput;
        rightSideOutput = driveOutput - turnOutput;

        LeftDrive.spin(forward, leftSideOutput, percent);
        RightDrive.spin(forward, rightSideOutput, percent);

        wait(driveSettleConstants.loopCycleTime, msec);
    }
}


void Drivetrain::driveDistance(double targetDistance){
    driveDistance(targetDistance, odom.orientation, defaultDriveSettleConstants, defaultDriveOutputConstants);
}

void Drivetrain::driveDistance(double targetDistance, double targetHeading){
    driveDistance(targetDistance, targetHeading, defaultDriveSettleConstants, defaultDriveOutputConstants);
}

void Drivetrain::driveDistance(double targetDistance, double targetHeading, settleConstants driveSettleConstants){
    driveDistance(targetDistance, targetHeading, driveSettleConstants, defaultDriveOutputConstants);
}

void Drivetrain::driveDistance(double targetDistance, double targetHeading, settleConstants driveSettleConstants, outputConstants driveOutputConstants){
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
        driveOutput *= driveOutputScale(driveOutputConstants.minimumSpeed, driveOutputConstants.maximumSpeed, driveOutput, driveOutput);
        turnOutput *= driveOutputScale(defaultDriveDistanceTurnOutputConstants.minimumSpeed, defaultDriveDistanceTurnOutputConstants.maximumSpeed, turnOutput, turnOutput);
        
        leftSideOutput = driveOutput + turnError;
        rightSideOutput = driveOutput - turnError;

        LeftDrive.spin(forward, leftSideOutput, percent);
        RightDrive.spin(forward, rightSideOutput, percent);

        wait(driveSettleConstants.loopCycleTime, msec);
    }
}


void Drivetrain::turnToHeading(double targetHeading){
    turnToHeading(targetHeading, defaultTurnSettleConstants, defaultTurnOutputConstants);
}

void Drivetrain::turnToHeading(double targetHeading, settleConstants turnSettleConstants){
    turnToHeading(targetHeading, turnSettleConstants, defaultTurnOutputConstants);
}

void Drivetrain::turnToHeading(double targetHeading, settleConstants turnSettleConstants, outputConstants turnOutputConstants){
    double turnError = headingError(targetHeading, odom.orientation);
    double turnOutput = 0;

    PID turnPID(turnError, turnOutputConstants.kp, turnOutputConstants.ki, turnOutputConstants.kd, turnOutputConstants.startI, 
                           turnSettleConstants.deadband, turnSettleConstants.loopCycleTime, turnSettleConstants.settleTime, turnSettleConstants.timeout);

    while (!turnPID.isSettled(turnError)){
        turnError = headingError(targetHeading, odom.orientation);

        turnOutput = turnPID.output(turnError);

        //Clamps the output speeds to stay within the specified minimum and maximum speeds
        turnOutput *= driveOutputScale(turnOutputConstants.minimumSpeed, turnOutputConstants.maximumSpeed, turnOutput, -turnOutput);

        LeftDrive.spin(forward, turnOutput, percent);
        RightDrive.spin(reverse, turnOutput, percent);

        wait(turnSettleConstants.loopCycleTime, msec);
    }
}


void Drivetrain::turnToPoint(bool reversed, double targetX, double targetY){
    turnToPoint(reversed, targetX, targetY, defaultTurnSettleConstants, defaultTurnOutputConstants);
}

void Drivetrain::turnToPoint(bool reversed, double targetX, double targetY, settleConstants turnSettleConstants){
    turnToPoint(reversed, targetX, targetY, turnSettleConstants, defaultTurnOutputConstants);
}

void Drivetrain::turnToPoint(bool reversed, double targetX, double targetY, settleConstants turnSettleConstants, outputConstants turnOutputConstants){
    double targetHeading = 0;
    if (reversed){
        targetHeading = fmod(radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition)) + 180, 360);
    }
    else {
        targetHeading = radToDeg(atan2(targetX - odom.xPosition, targetY - odom.yPosition));
    }

    turnToHeading(targetHeading, turnSettleConstants, turnOutputConstants);
}


void Drivetrain::swingToHeading(std::string driveSide, double target){
    swingToHeading(driveSide, target, defaultSwingSettleConstants, defaultSwingOutputConstants);
}

void Drivetrain::swingToHeading(std::string driveSide, double target, settleConstants swingSettleConstants){
    swingToHeading(driveSide, target, swingSettleConstants, defaultSwingOutputConstants);
}

void Drivetrain::swingToHeading(std::string driveSide, double target, settleConstants swingSettleConstants, outputConstants swingOutputConstants){
    double swingError = headingError(target, odom.orientation);

    double swingOutput = 0;

    PID swingPID(swingError, swingOutputConstants.kp, swingOutputConstants.ki, swingOutputConstants.kd, swingOutputConstants.startI, swingSettleConstants.deadband, swingSettleConstants.loopCycleTime, swingSettleConstants.settleTime, swingSettleConstants.timeout);

    while (!swingPID.isSettled(swingError)){
        swingError = headingError(targetHeading, odom.orientation);

        swingOutput = turnPID.output(turnError);

        //Clamps the output speeds to stay within the specified minimum and maximum speeds
        swingOutput *= driveOutputScale(swingOutputConstants.minimumSpeed, swingOutputConstants.maximumSpeed, swingOutput, -swingOutput);

        if (driveSide == "Left"){
            LeftDrive.spin(forward, turnOutput, percent);
            RightDrive.stop(brake);
        }
        else {
            LeftDrive.stop(brake);
            RightDrive.spin(forward, -turnOutput, percent);
        }

        wait(turnSettleConstants.loopCycleTime, msec);
    }
    
}


void Drivetrain::stopDrive(brakeType brakeType){
    LeftDrive.stop(brakeType);
    RightDrive.stop(brakeType);
}