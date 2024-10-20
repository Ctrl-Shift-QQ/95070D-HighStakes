#include "vex.h"
#include "autons.h"
#include <iostream>

/******************** PID Tunings ********************/

void setDefaultPIDConstants(){
    //Drive constants
    chassis.defaultDriveOutputConstants.kp = 2.5;
    chassis.defaultDriveOutputConstants.ki = 0;
    chassis.defaultDriveOutputConstants.kd = 0.1;
    chassis.defaultDriveOutputConstants.startI = 0;
    chassis.defaultDriveOutputConstants.minimumSpeed = 15;
    chassis.defaultDriveOutputConstants.maximumSpeed = 40;
    chassis.defaultDriveSettleConstants.deadband = 0.75;
    chassis.defaultDriveSettleConstants.loopCycleTime = 20;
    chassis.defaultDriveSettleConstants.settleTime = 1000;
    chassis.defaultDriveSettleConstants.timeout = 5000;

    //Drive Distance Turn Constants
    chassis.defaultDriveDistanceTurnOutputConstants.kp = 0.3;
    chassis.defaultDriveDistanceTurnOutputConstants.ki = 0;
    chassis.defaultDriveDistanceTurnOutputConstants.kd = 0;
    chassis.defaultDriveDistanceTurnOutputConstants.startI = 0;
    chassis.defaultDriveDistanceTurnOutputConstants.minimumSpeed = 0;
    chassis.defaultDriveDistanceTurnOutputConstants.maximumSpeed = 5;
    chassis.defaultDriveDistanceTurnSettleConstants.deadband = 0.25;
    chassis.defaultDriveDistanceTurnSettleConstants.loopCycleTime = 0;
    chassis.defaultDriveDistanceTurnSettleConstants.settleTime = 0;
    chassis.defaultDriveDistanceTurnSettleConstants.timeout = 0;

    //Turn Constants
    chassis.defaultTurnOutputConstants.kp = 0.6;
    chassis.defaultTurnOutputConstants.ki = 0.01;
    chassis.defaultTurnOutputConstants.kd = 1.75;
    chassis.defaultTurnOutputConstants.startI = 5;
    chassis.defaultTurnOutputConstants.minimumSpeed = 3;
    chassis.defaultTurnOutputConstants.maximumSpeed = 75;
    chassis.defaultTurnSettleConstants.deadband = 2;
    chassis.defaultTurnSettleConstants.loopCycleTime = 20;
    chassis.defaultTurnSettleConstants.settleTime = 1000;
    chassis.defaultTurnSettleConstants.timeout = 2500;
}

/******************** Tasks ********************/

int colorSort(){
    while (true){
        if ((redAlliance && 210 < IntakeOptical.hue() < 230) || (!redAlliance && 10 < IntakeOptical.hue() < 30)){ //Intaking opposite color ring
            Intake.stop(brake);
        }

        wait(20, msec);
    }

    return 0;
}

int armDown(){
    while (true){
        if (fabs(0 - ArmRotation.position(degrees)) > 2){
            Arm.spin(forward, 0.5 * -ArmRotation.position(degrees), percent); //Turns torwards down position
        }
        else {
            Arm.stop(brake);
        }
        
        wait(20, msec);
    }

    return 0;
}

int armLoad(){
    while (true){
        if (fabs(32 - ArmRotation.position(degrees)) > 2){
            Arm.spin(forward, 0.5 * (32 - ArmRotation.position(degrees)), percent); //Turns towards load position
        }
        else {
            Arm.stop(brake);
        }
        
        wait(20, msec);
    }

    return 0;
}

/******************** Autons ********************/

void runOdomTest(){
    Inertial.calibrate();
    wait(3, sec);
    chassis.setCoordinates(0, 0, 0);

    while (true){
        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(1, 3);
        Controller1.Screen.print("X: ");
        Controller1.Screen.print(chassis.odom.xPosition);
        Controller1.Screen.setCursor(2, 3);
        Controller1.Screen.print("Y: ");
        Controller1.Screen.print(chassis.odom.yPosition);
        Controller1.Screen.setCursor(3, 3);
        Controller1.Screen.print("O: ");
        Controller1.Screen.print(chassis.odom.orientation);
        wait(5, msec);
    }
}

void runAutonRedSoloAWP(){
    chassis.setCoordinates(-55, 17.5, 270);
    setDefaultPIDConstants();

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 2, shortTurnSettle.loopCycleTime = 20, shortTurnSettle.settleTime = 500, shortTurnSettle.timeout = 1500;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 0.5, shortDriveSettle.loopCycleTime = 20, shortDriveSettle.settleTime = 0, shortDriveSettle.timeout = 1500;

    Drivetrain::outputConstants shortDriveOutput;
    shortDriveOutput.kp = 1, shortDriveOutput.ki = 0, shortDriveOutput.kd = 0, shortDriveOutput.startI = 0, shortDriveOutput.minimumSpeed = 15, shortDriveOutput.maximumSpeed = 25;

    Drivetrain::outputConstants maxDriveOutput;
    maxDriveOutput.kp = 1, maxDriveOutput.ki = 0, maxDriveOutput.kd = 0, maxDriveOutput.startI = 0, maxDriveOutput.minimumSpeed = 100, maxDriveOutput.maximumSpeed = 100;


    chassis.turnToHeading(105, shortTurnSettle);
    chassis.driveDistance(-28);
    MogoMech.set(true); //Stack side mogo clamped

    chassis.stopDrive(brake);
    Intake.spin(forward, 85, percent);
    wait(0.2, sec);
    chassis.turnToHeading(0);
    chassis.driveDistance(22, 0); //Two rings scored on mogo
    
    chassis.driveToPoint(-56, 30);
    Intake.spin(forward, 85, percent);
    MogoMech.set(false); //Mogo released

    chassis.turnToHeading(335, shortTurnSettle);
    Intake.spin(forward, 85, percent);
    chassis.driveDistance(12);
    chassis.stopDrive(brake);
    wait(0.1, sec);
    Intake.stop(brake);
    chassis.turnToHeading(40, shortTurnSettle);
    chassis.driveDistance(4, chassis.odom.orientation, shortDriveSettle, shortDriveOutput); //Alliance pushed off line

    chassis.driveToPoint(-60, -1);
    chassis.turnToHeading(90);
    MogoMech.set(true);
    chassis.driveDistance(-3, chassis.odom.orientation, shortDriveSettle, shortDriveOutput);
    chassis.stopDrive(brake);
    Intake.spin(forward, 85, percent);
    wait(0.5, sec); //One ring scored on alliance stake

    chassis.driveDistance(30, 90, chassis.defaultDriveSettleConstants, maxDriveOutput);
    chassis.stopDrive(brake); //Ladder Touched
}

void runAutonRedRushAWP(){

}

void runAutonRedStackAWP(){
    chassis.setCoordinates(-59.5, 17.5, 0);
    setDefaultPIDConstants();

    Drivetrain::settleConstants slowDriveSettle;
    slowDriveSettle.deadband = 0.5, slowDriveSettle.loopCycleTime = 20, slowDriveSettle.settleTime = 0, slowDriveSettle.timeout = 2500;

    Drivetrain::outputConstants slowDriveOutput;
    slowDriveOutput.kp = 2.5, slowDriveOutput.ki = 0, slowDriveOutput.kd = 0.1, slowDriveOutput.startI = 0, slowDriveOutput.minimumSpeed = 5, slowDriveOutput.maximumSpeed = 25;

    chassis.driveDistance(-18);
    chassis.turnToHeading(270);
    chassis.driveDistance(-3, 90, slowDriveSettle, slowDriveOutput);
    MogoMech.set(true);
    chassis.stopDrive(brake);
    Intake.spin(forward, 85, percent);
    wait(0.5, sec); //One ring scored on alliance stake

    Intake.stop(brake);
    chassis.driveDistance(12, 90, slowDriveSettle, slowDriveOutput);
    chassis.turnToHeading(210);
    MogoMech.set(false);
    chassis.stopDrive(brake);
    wait(0.1, sec);
    chassis.driveDistance(-32);
    MogoMech.set(true); //Stack side mogo clmaped

    chassis.stopDrive(brake);
    Intake.spin(forward, 85, percent);
    wait(0.2, sec);
    chassis.turnToHeading(0);
    chassis.driveDistance(22, 0); //One rings scored on mogo

    chassis.turnToHeading(75);
    chassis.driveDistance(12, 75, slowDriveSettle, slowDriveOutput);
    chassis.stopDrive(brake); //Two rings scored on mogo

    chassis.driveDistance(-12, 75, slowDriveSettle, slowDriveOutput);
    chassis.stopDrive(brake);
    chassis.turnToHeading(210);
    chassis.driveDistance(50); //Ladder touched
}

void runAutonRedGoalRush(){
    chassis.setCoordinates(-55, -24, 270);
    setDefaultPIDConstants();

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 0.5, shortDriveSettle.loopCycleTime = 20, shortDriveSettle.settleTime = 0, shortDriveSettle.timeout = 1500;

    Drivetrain::outputConstants shortDriveOutput;
    shortDriveOutput.kp = 1, shortDriveOutput.ki = 0, shortDriveOutput.kd = 0, shortDriveOutput.startI = 0, shortDriveOutput.minimumSpeed = 15, shortDriveOutput.maximumSpeed = 25;

    Drivetrain::outputConstants maxDriveOutput;
    maxDriveOutput.kp = 2.5, maxDriveOutput.ki = 0, maxDriveOutput.kd = 0.1, maxDriveOutput.startI = 0, maxDriveOutput.minimumSpeed = 25, maxDriveOutput.maximumSpeed = 50;

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 2, shortTurnSettle.loopCycleTime = 20, shortTurnSettle.settleTime = 500, shortTurnSettle.timeout = 1500;

    chassis.driveDistance(-28, 270);
    MogoMech.set(true); //Rush side mogo clamped

    Intake.spin(forward, 85, percent); //One ring scored on mogo

    chassis.stopDrive(brake);
    wait(0.5, sec);
    chassis.turnToHeading(180);
    chassis.driveDistance(-4, chassis.odom.orientation, shortDriveSettle, shortDriveOutput);
    MogoMech.set(false); //Mogo released

    chassis.driveDistance(23);
    Intake.stop(brake);
    chassis.turnToHeading(270);
    chassis.driveDistance(-15);
    MogoMech.set(true); //Neutral mogo clamped

    Intake.spin(forward, 85, percent); //One ring scored on mogo

    chassis.driveToPoint(-48, -60);
    chassis.turnToPoint(false, -66, -60);
    Doinker.set(true);
    chassis.driveDistance(16, chassis.odom.orientation, shortDriveSettle, shortDriveOutput);
    chassis.turnToHeading(45); //Corner cleared

    chassis.driveDistance(-6); 
    chassis.stopDrive(brake); //Mogo in positive corner
}

void runAutonBlueSoloAWP(){
    chassis.setCoordinates(55, 17.5, 90);
    setDefaultPIDConstants();

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 2, shortTurnSettle.loopCycleTime = 20, shortTurnSettle.settleTime = 500, shortTurnSettle.timeout = 1500;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 0.5, shortDriveSettle.loopCycleTime = 20, shortDriveSettle.settleTime = 0, shortDriveSettle.timeout = 1500;

    Drivetrain::outputConstants shortDriveOutput;
    shortDriveOutput.kp = 1, shortDriveOutput.ki = 0, shortDriveOutput.kd = 0, shortDriveOutput.startI = 0, shortDriveOutput.minimumSpeed = 15, shortDriveOutput.maximumSpeed = 25;

    Drivetrain::outputConstants maxDriveOutput;
    maxDriveOutput.kp = 1, maxDriveOutput.ki = 0, maxDriveOutput.kd = 0, maxDriveOutput.startI = 0, maxDriveOutput.minimumSpeed = 100, maxDriveOutput.maximumSpeed = 100;

    chassis.turnToPoint(true, 24, 24, shortTurnSettle);
    chassis.driveDistance(-28);
    MogoMech.set(true); //Stack side mogo clamped

    chassis.stopDrive(brake);
    Intake.spin(forward, 85, percent);
    wait(0.2, sec);
    chassis.turnToHeading(0);
    chassis.driveDistance(22, 0); //Two rings scored on mogo
    
    chassis.driveToPoint(56, 30);
    Intake.spin(forward, 85, percent);
    MogoMech.set(false); //Mogo released

    chassis.turnToHeading(25, shortTurnSettle);
    Intake.spin(forward, 85, percent);
    chassis.driveDistance(12);
    chassis.stopDrive(brake);
    wait(0.1, sec);
    Intake.stop(brake);
    chassis.turnToHeading(320, shortTurnSettle);
    chassis.driveDistance(4, chassis.odom.orientation, shortDriveSettle, shortDriveOutput); //Alliance pushed off line

    chassis.driveToPoint(60, -1);
    chassis.turnToHeading(270);
    MogoMech.set(true);
    chassis.driveDistance(-3, chassis.odom.orientation, shortDriveSettle, shortDriveOutput);
    chassis.stopDrive(brake);
    Intake.spin(forward, 85, percent);
    wait(0.5, sec); //One ring scored on alliance stake

    chassis.driveDistance(30, 270, chassis.defaultDriveSettleConstants, maxDriveOutput);
    chassis.stopDrive(brake); //Ladder Touched
    
}

void runAutonBlueRushAWP(){

}

void runAutonBlueStackAWP(){
    chassis.setCoordinates(59.5, 17.5, 0);
    setDefaultPIDConstants();

    Drivetrain::settleConstants slowDriveSettle;
    slowDriveSettle.deadband = 0.5, slowDriveSettle.loopCycleTime = 20, slowDriveSettle.settleTime = 0, slowDriveSettle.timeout = 2500;

    Drivetrain::outputConstants slowDriveOutput;
    slowDriveOutput.kp = 2.5, slowDriveOutput.ki = 0, slowDriveOutput.kd = 0.1, slowDriveOutput.startI = 0, slowDriveOutput.minimumSpeed = 5, slowDriveOutput.maximumSpeed = 25;

    chassis.driveDistance(-18);
    chassis.turnToHeading(270);
    chassis.driveDistance(-3, 270, slowDriveSettle, slowDriveOutput);
    MogoMech.set(true);
    chassis.stopDrive(brake);
    Intake.spin(forward, 85, percent);
    wait(0.5, sec); //One ring scored on alliance stake

    Intake.stop(brake);
    chassis.driveDistance(12, 270, slowDriveSettle, slowDriveOutput);
    chassis.turnToHeading(135);
    MogoMech.set(false);
    chassis.stopDrive(brake);
    wait(0.1, sec);
    chassis.driveDistance(-32);
    MogoMech.set(true); //Stack side mogo clmaped

    chassis.stopDrive(brake);
    Intake.spin(forward, 85, percent);
    wait(0.2, sec);
    chassis.turnToHeading(0);
    chassis.driveDistance(22, 0); //One rings scored on mogo

    chassis.turnToHeading(285);
    chassis.driveDistance(12, 285, slowDriveSettle, slowDriveOutput);
    chassis.stopDrive(brake); //Two rings scored on mogo

    chassis.driveDistance(-12, 285, slowDriveSettle, slowDriveOutput);
    chassis.stopDrive(brake);
    chassis.turnToHeading(150);
    chassis.driveDistance(50); //Ladder touched
}

void runAutonBlueGoalRush(){
    chassis.setCoordinates(55, -24, 90);
    setDefaultPIDConstants();

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 0.5, shortDriveSettle.loopCycleTime = 20, shortDriveSettle.settleTime = 0, shortDriveSettle.timeout = 1500;

    Drivetrain::outputConstants shortDriveOutput;
    shortDriveOutput.kp = 1, shortDriveOutput.ki = 0, shortDriveOutput.kd = 0, shortDriveOutput.startI = 0, shortDriveOutput.minimumSpeed = 15, shortDriveOutput.maximumSpeed = 25;

    Drivetrain::outputConstants maxDriveOutput;
    maxDriveOutput.kp = 2.5, maxDriveOutput.ki = 0, maxDriveOutput.kd = 0.1, maxDriveOutput.startI = 0, maxDriveOutput.minimumSpeed = 25, maxDriveOutput.maximumSpeed = 50;

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 2, shortTurnSettle.loopCycleTime = 20, shortTurnSettle.settleTime = 500, shortTurnSettle.timeout = 1500;

    chassis.driveDistance(-28, 90);
    MogoMech.set(true); //Rush side mogo clamped

    Intake.spin(forward, 85, percent); //One ring scored on mogo

    chassis.stopDrive(brake);
    wait(0.5, sec);
    chassis.turnToHeading(180);
    chassis.driveDistance(-4, chassis.odom.orientation, shortDriveSettle, shortDriveOutput);
    MogoMech.set(false); //Mogo released

    chassis.driveDistance(23);
    Intake.stop(brake);
    chassis.turnToHeading(90);
    chassis.driveDistance(-15);
    MogoMech.set(true); //Neutral mogo clamped

    Intake.spin(forward, 85, percent); //One ring scored on mogo

    chassis.driveToPoint(60, -24);
    chassis.turnToHeading(180);
    chassis.driveDistance(26);
    chassis.turnToPoint(false, 66, -66);
    Doinker.set(true);
    chassis.driveDistance(4, chassis.odom.orientation, shortDriveSettle, shortDriveOutput);
    chassis.turnToHeading(315); //Corner cleared

    chassis.driveDistance(-6); 
    chassis.stopDrive(brake); //Mogo in positive corner
}

/******************** Skills ********************/

void runProgSkills(){
    chassis.setCoordinates(-61.5, 0, 90);
    setDefaultPIDConstants();

    Drivetrain::settleConstants slowDriveSettle;
    slowDriveSettle.deadband = 0.5, slowDriveSettle.loopCycleTime = 20, slowDriveSettle.settleTime = 0, slowDriveSettle.timeout = 2500;

    Drivetrain::outputConstants slowDriveOutput;
    slowDriveOutput.kp = 2.5, slowDriveOutput.ki = 0, slowDriveOutput.kd = 0.1, slowDriveOutput.startI = 0, slowDriveOutput.minimumSpeed = 5, slowDriveOutput.maximumSpeed = 25;

    Intake.spin(forward, 85, percent);
    wait(0.5, sec); //One ring scored on alliance stake

    chassis.driveDistance(12, 90, slowDriveSettle, slowDriveOutput);
    chassis.turnToHeading(180);
    chassis.driveDistance(-21);
    MogoMech.set(true); //Left mogo clamped

    chassis.stopDrive(brake);
    wait(0.1, sec);
    chassis.turnToHeading(90);
    chassis.driveDistance(22); //One ring scored on mogo

    chassis.turnToHeading(0);
    chassis.driveDistance(22); //Two rings scored on mogo

    chassis.turnToHeading(270);
    chassis.driveDistance(38, 270, slowDriveSettle, slowDriveOutput); //Four rings scored on mogo

    chassis.stopDrive(brake);
    wait(0.5, sec);
    chassis.turnToHeading(45);
    chassis.driveDistance(16); //Five rings scored on mogo

    chassis.turnToPoint(true, -66, 66); 
    chassis.driveDistance(-12);
    MogoMech.set(false); //Mogo deposited in corner

    chassis.driveToPoint(-48, 0);
    chassis.turnToHeading(345);
    chassis.driveDistance(-16);
    MogoMech.set(true); //Right mogo clamped
    
    chassis.turnToHeading(90);
    chassis.driveDistance(24); //One ring scored on mogo

    chassis.turnToHeading(180);
    chassis.driveDistance(20); //Two ring scored on mogo

    chassis.turnToHeading(270);
    chassis.driveDistance(38, 270, slowDriveSettle, slowDriveOutput); //Four rings scored on mogo

    wait(0.5, sec);
    chassis.turnToHeading(135);
    chassis.driveDistance(16); //Five rings scored on mogo

    chassis.turnToPoint(true, -66, -66); 
    chassis.driveDistance(-15);
    MogoMech.set(false); //Mogo deposited in corner

    chassis.driveToPoint(-12, -60);
    chassis.turnToHeading(90);
    chassis.driveDistance(18);
    Intake.stop(brake);
    chassis.driveToPoint(48, -24);
    chassis.turnToHeading(180);
    chassis.driveDistance(-24);
    MogoMech.set(true);
    Intake.spin(forward, 85, percent); //One ring scored

    chassis.turnToHeading(90);
    chassis.driveDistance(8);
    chassis.turnToPoint(false, 66, 66);
    MogoMech.set(false);
    chassis.driveDistance(60); //Mogo pushed to corner

    chassis.stopDrive(brake);
}