#include "vex.h"
#include "autons.h"
#include <iostream>

/******************** PID Tunings ********************/

void setDefaultPIDConstants(){
    //Drive constants
    chassis.defaultDriveOutputConstants.kp = 2.5;
    chassis.defaultDriveOutputConstants.ki = 0;
    chassis.defaultDriveOutputConstants.kd = 0.35;
    chassis.defaultDriveOutputConstants.startI = 0;
    chassis.defaultDriveClampConstants.minimumSpeed = 10;
    chassis.defaultDriveClampConstants.maximumSpeed = 75;
    chassis.defaultDriveSettleConstants.deadband = 0.25;
    chassis.defaultDriveSettleConstants.loopCycleTime = 20;
    chassis.defaultDriveSettleConstants.settleTime = 1000;
    chassis.defaultDriveSettleConstants.timeout = 5000;

    //Drive Distance Turn Constants
    chassis.defaultDriveDistanceTurnOutputConstants.kp = 0.3;
    chassis.defaultDriveDistanceTurnOutputConstants.ki = 0;
    chassis.defaultDriveDistanceTurnOutputConstants.kd = 0;
    chassis.defaultDriveDistanceTurnOutputConstants.startI = 0;
    chassis.defaultDriveDistanceTurnClampConstants.minimumSpeed = 0;
    chassis.defaultDriveDistanceTurnClampConstants.maximumSpeed = 5;

    //Turn Constants
    chassis.defaultTurnOutputConstants.kp = 0.6;
    chassis.defaultTurnOutputConstants.ki = 0.005;
    chassis.defaultTurnOutputConstants.kd = 1.25; //With mogo: 1.6
    chassis.defaultTurnOutputConstants.startI = 10;
    chassis.defaultTurnClampConstants.minimumSpeed = 0;
    chassis.defaultTurnClampConstants.maximumSpeed = 75;
    chassis.defaultTurnSettleConstants.deadband = 2;
    chassis.defaultTurnSettleConstants.loopCycleTime = 20;
    chassis.defaultTurnSettleConstants.settleTime = 750;
    chassis.defaultTurnSettleConstants.timeout = 3000;

    //Swing Constants
    chassis.defaultSwingOutputConstants.kp = 0.6;
    chassis.defaultSwingOutputConstants.ki = 0.0025;
    chassis.defaultSwingOutputConstants.kd = 0.2;
    chassis.defaultSwingOutputConstants.startI = 10;
    chassis.defaultSwingClampConstants.minimumSpeed = 7;
    chassis.defaultSwingClampConstants.maximumSpeed = 75;
    chassis.defaultSwingSettleConstants.deadband = 2;
    chassis.defaultSwingSettleConstants.loopCycleTime = 20;
    chassis.defaultSwingSettleConstants.settleTime = 1000;
    chassis.defaultSwingSettleConstants.timeout = 3000;
}

/******************** Tasks ********************/

int colorSort(){
    return 0;
}

int armDown(){
    while (true){
        Arm.spin(forward, 0.7 * (-ArmRotation.position(degrees)), percent);

        wait(20, msec);
    }

    return 0;
}

int armLoad(){
    while (true){
        Arm.spin(forward, 0.7 * (20 - ArmRotation.position(degrees)), percent);

        wait(20, msec);
    }

    return 0;
}

/******************** Tests ********************/

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

void runDriveTest(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

    chassis.driveDistance(12);
    chassis.driveDistance(24);
    chassis.driveDistance(-24);
    chassis.driveDistance(-12);
    chassis.stopDrive(brake);
}

void runTurnTest(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

    chassis.turnToPoint(false, 24, 24);
    chassis.turnToHeading(90);
    chassis.turnToHeading(180);
    chassis.turnToHeading(0);
    chassis.stopDrive(brake);
}

void runSwingTest(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

    chassis.swingToHeading("Left", 90);
    chassis.swingToHeading("Right", 270);
    chassis.swingToHeading("Right", 90);
    chassis.swingToHeading("Left", 0);
    chassis.stopDrive(brake);
}

/******************** Autons ********************/

void runAutonRedSoloAWP(){
    chassis.setCoordinates(-59.75, 18.75, 0);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 5;
    shortDriveClamp.maximumSpeed = 10;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 0.25;
    shortDriveSettle.loopCycleTime = 20;
    shortDriveSettle.settleTime = 750;
    shortDriveSettle.timeout = 1500;

    Drivetrain::clampConstants driveToPointClamp;
    driveToPointClamp.minimumSpeed = 5;
    driveToPointClamp.maximumSpeed = 50;

    Drivetrain::settleConstants driveToPointDriveSettle;
    driveToPointDriveSettle.deadband = 1.5;
    driveToPointDriveSettle.loopCycleTime = 20;
    driveToPointDriveSettle.settleTime = 1000;
    driveToPointDriveSettle.timeout = 5000;

    Drivetrain::settleConstants driveToPointTurnSettle;
    driveToPointTurnSettle.deadband = 5;
    driveToPointTurnSettle.loopCycleTime = 20;
    driveToPointTurnSettle.settleTime = 0;
    driveToPointTurnSettle.timeout = 0;

    // FirstIntake.spin(forward, 85, percent);
    // chassis.driveDistance(10); 
    // Intake.stop();
    // chassis.turnToHeading(345);
    // chassis.driveDistance(6); //Alliance pushed off starting line

    // chassis.driveDistance(-4, 345, shortDriveClamp, shortDriveSettle);
    // chassis.turnToHeading(72);
    // MogoMech.set(true);
    // chassis.driveDistance(-36);
    // MogoMech.set(false); //Mogo Clamped

    // chassis.stopDrive(brake);
    // Intake.spin(forward, 85, percent);
    // wait(1, sec); //Two rings scored on mogo

    // MogoMech.set(true);
    // chassis.driveToPoint(48, 0);
    // chassis.turnToHeading(50);
    // chassis.driveDistance(-27);
    // MogoMech.set(false); //Mogo clamped

    // chassis.turnToHeading(180);
    // chassis.driveDistance(20); //One ring scored on mogo

    // chassis.turnToHeading(0);
    // chassis.driveDistance(50);
    // chassis.stopDrive(coast); //Ladder touched


    chassis.driveDistance(-17);
    chassis.turnToHeading(90);
    chassis.driveDistance(-4, 90, shortDriveClamp, shortDriveSettle);
    Intake.spin(forward, 85, percent);
    chassis.stopDrive(brake); //One ring scored on alliance stake

    wait(0.5, sec);
    chassis.driveDistance(3, 90, shortDriveClamp, shortDriveSettle);
    chassis.turnToHeading(0);
    FirstIntake.spin(forward, 85, percent);
    chassis.driveDistance(27); 
    Intake.stop();
    chassis.turnToHeading(15);
    chassis.driveDistance(6); //Alliance pushed off starting line

    chassis.driveDistance(-4, 15, shortDriveClamp, shortDriveSettle);
    chassis.turnToHeading(283);
    MogoMech.set(true);
    chassis.driveDistance(-32.5);
    MogoMech.set(false); //Mogo Clamped

    chassis.stopDrive(brake);
    Intake.spin(forward, 85, percent); //One ring scored on mogo

    chassis.turnToHeading(0);
    chassis.driveDistance(18); //Two rings scored on mogo

    chassis.turnToHeading(180);
    chassis.driveDistance(50);
    chassis.stopDrive(coast);
}

void runAutonRedRushAWP(){

}

void runAutonRedStackAWP(){

}

void runAutonRedGoalRush(){
    chassis.setCoordinates(55, -24, 270);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 5;
    shortDriveClamp.maximumSpeed = 10;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 0.5, shortDriveSettle.loopCycleTime = 20, shortDriveSettle.settleTime = 0, shortDriveSettle.timeout = 1500;

    Drivetrain::outputConstants shortDriveOutput;
    shortDriveOutput.kp = 1, shortDriveOutput.ki = 0, shortDriveOutput.kd = 0, shortDriveOutput.startI = 0;

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 2, shortTurnSettle.loopCycleTime = 20, shortTurnSettle.settleTime = 500, shortTurnSettle.timeout = 1500;

    MogoMech.set(true);
    chassis.driveDistance(-28, 270);
    MogoMech.set(false); //Rush side mogo clamped

    Intake.spin(forward, 85, percent); //One ring scored on mogo

    chassis.stopDrive(brake);
    wait(0.5, sec);
    chassis.turnToHeading(180);
    chassis.driveDistance(-4, chassis.odom.orientation, shortDriveClamp, shortDriveSettle, shortDriveOutput);
    MogoMech.set(true); //Mogo released

    chassis.driveDistance(23);
    Intake.stop(brake);
    chassis.turnToHeading(270);
    chassis.driveDistance(-15);
    MogoMech.set(false); //Neutral mogo clamped

    Intake.spin(forward, 85, percent); //One ring scored on mogo

    chassis.driveToPoint(-60, -60);
    chassis.turnToHeading(180);
    chassis.driveDistance(26);
    chassis.turnToPoint(false, -66, -66);
    chassis.turnToHeading(225);
    Doinker.set(true);
    chassis.driveDistance(4, chassis.odom.orientation, shortDriveClamp, shortDriveSettle, shortDriveOutput);
    chassis.turnToHeading(45); //Corner cleared

    chassis.driveDistance(-6); 
    chassis.stopDrive(brake); //Mogo in positive corner
}

void runAutonBlueSoloAWP(){
    chassis.setCoordinates(59.75, 18.75, 0);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 5;
    shortDriveClamp.maximumSpeed = 10;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 0.25;
    shortDriveSettle.loopCycleTime = 20;
    shortDriveSettle.settleTime = 750;
    shortDriveSettle.timeout = 1500;

    Drivetrain::clampConstants driveToPointClamp;
    driveToPointClamp.minimumSpeed = 5;
    driveToPointClamp.maximumSpeed = 50;

    Drivetrain::settleConstants driveToPointDriveSettle;
    driveToPointDriveSettle.deadband = 1.5;
    driveToPointDriveSettle.loopCycleTime = 20;
    driveToPointDriveSettle.settleTime = 1000;
    driveToPointDriveSettle.timeout = 5000;

    Drivetrain::settleConstants driveToPointTurnSettle;
    driveToPointTurnSettle.deadband = 5;
    driveToPointTurnSettle.loopCycleTime = 20;
    driveToPointTurnSettle.settleTime = 0;
    driveToPointTurnSettle.timeout = 0;

    // FirstIntake.spin(forward, 85, percent);
    // chassis.driveDistance(10); 
    // Intake.stop();
    // chassis.turnToHeading(345);
    // chassis.driveDistance(6); //Alliance pushed off starting line

    // chassis.driveDistance(-4, 345, shortDriveClamp, shortDriveSettle);
    // chassis.turnToHeading(72);
    // MogoMech.set(true);
    // chassis.driveDistance(-36);
    // MogoMech.set(false); //Mogo Clamped

    // chassis.stopDrive(brake);
    // Intake.spin(forward, 85, percent);
    // wait(1, sec); //Two rings scored on mogo

    // MogoMech.set(true);
    // chassis.driveToPoint(48, 0);
    // chassis.turnToHeading(50);
    // chassis.driveDistance(-27);
    // MogoMech.set(false); //Mogo clamped

    // chassis.turnToHeading(180);
    // chassis.driveDistance(20); //One ring scored on mogo

    // chassis.turnToHeading(0);
    // chassis.driveDistance(50);
    // chassis.stopDrive(coast); //Ladder touched


    chassis.driveDistance(-17);
    chassis.turnToHeading(270);
    chassis.driveDistance(-4, 270, shortDriveClamp, shortDriveSettle);
    Intake.spin(forward, 85, percent);
    chassis.stopDrive(brake); //One ring scored on alliance stake

    wait(0.5, sec);
    chassis.driveDistance(3, 270, shortDriveClamp, shortDriveSettle);
    chassis.turnToHeading(0);
    FirstIntake.spin(forward, 85, percent);
    chassis.driveDistance(27);
    Intake.stop();
    chassis.turnToHeading(345);
    chassis.driveDistance(6); //Alliance pushed off starting line

    chassis.driveDistance(-4, 345, shortDriveClamp, shortDriveSettle);
    chassis.turnToHeading(77);
    MogoMech.set(true);
    chassis.driveDistance(-32.5);
    MogoMech.set(false); //Mogo Clamped

    chassis.stopDrive(brake);
    Intake.spin(forward, 85, percent); //One ring scored on mogo

    chassis.turnToHeading(0);
    chassis.driveDistance(18); //Two rings scored on mogo

    chassis.turnToHeading(180);
    chassis.driveDistance(50);
    chassis.stopDrive(coast);
}

void runAutonBlueRushAWP(){

}

void runAutonBlueStackAWP(){
   
}

void runAutonBlueGoalRush(){
    chassis.setCoordinates(55, -24, 90);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 5;
    shortDriveClamp.maximumSpeed = 10;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 0.5, shortDriveSettle.loopCycleTime = 20, shortDriveSettle.settleTime = 0, shortDriveSettle.timeout = 1500;

    Drivetrain::outputConstants shortDriveOutput;
    shortDriveOutput.kp = 1, shortDriveOutput.ki = 0, shortDriveOutput.kd = 0, shortDriveOutput.startI = 0;

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 2, shortTurnSettle.loopCycleTime = 20, shortTurnSettle.settleTime = 500, shortTurnSettle.timeout = 1500;

    MogoMech.set(true);
    chassis.driveDistance(-28, 90);
    MogoMech.set(false); //Rush side mogo clamped

    Intake.spin(forward, 85, percent); //One ring scored on mogo

    chassis.stopDrive(brake);
    wait(0.5, sec);
    chassis.turnToHeading(180);
    chassis.driveDistance(-4, chassis.odom.orientation, shortDriveClamp, shortDriveSettle, shortDriveOutput);
    MogoMech.set(true); //Mogo released

    chassis.driveDistance(23);
    Intake.stop(brake);
    chassis.turnToHeading(90);
    chassis.driveDistance(-15);
    MogoMech.set(false); //Neutral mogo clamped

    Intake.spin(forward, 85, percent); //One ring scored on mogo

    chassis.driveToPoint(60, -24);
    chassis.turnToHeading(180);
    chassis.driveDistance(26);
    chassis.turnToPoint(false, 66, -66);
    Doinker.set(true);
    chassis.driveDistance(4, chassis.odom.orientation, shortDriveClamp, shortDriveSettle, shortDriveOutput);
    chassis.turnToHeading(315); //Corner cleared

    chassis.driveDistance(-6); 
    chassis.stopDrive(brake); //Mogo in positive corner
}

/******************** Prog Skills ********************/

void runProgSkills(){
    chassis.setCoordinates(-61.5, 0, 90);
    setDefaultPIDConstants();

    Drivetrain::clampConstants slowDriveClamp;
    slowDriveClamp.minimumSpeed = 10;
    slowDriveClamp.maximumSpeed = 30;

    Intake.spin(forward, 85, percent);
    wait(0.5, sec); //One ring scored on alliance stake

    chassis.driveDistance(11.5, 90);
    chassis.turnToHeading(0);
    MogoMech.set(true);
    chassis.driveDistance(-21);
    MogoMech.set(false); //Mogo clamped

    chassis.turnToHeading(90);
    chassis.driveDistance(19, 90); //One ring scored on mogo

    chassis.turnToHeading(180);
    chassis.driveDistance(21, 180); //Two rings scored on mogo

    chassis.turnToHeading(270);
    chassis.driveDistance(38, 270, slowDriveClamp);

    chassis.turnToHeading(135);
    chassis.driveDistance(12); //Five rings scored on mogo

    chassis.turnToPoint(true, -66, -66);
    chassis.driveDistance(-10);
    MogoMech.set(true); //Mogo deposited in corner

    // chassis.driveToPoint(-48, 0);
    // chassis.turnToHeading(0);
    // chassis.driveDistance(-21);
    // MogoMech.set(false); //Mogo clamped

    // chassis.turnToHeading(90);
    // chassis.driveDistance(21); //One ring scored on mogo

    // chassis.turnToHeading(0);
    // chassis.driveDistance(21); //Two rings scored on mogo

    // chassis.turnToHeading(270);
    // chassis.driveDistance(38, 270, slowDriveClamp);

    // chassis.turnToHeading(45);
    // chassis.driveDistance(12); //Five rings scored on mogo

    // chassis.turnToPoint(true, -66, 66);
    // chassis.driveDistance(-10);
    // MogoMech.set(true); //Mogo deposited in corner

    chassis.stopDrive(brake);
}