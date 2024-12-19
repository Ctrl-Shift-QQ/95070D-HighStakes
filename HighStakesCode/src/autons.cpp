#include "vex.h"
#include "autons.h"
#include <iostream>

/******************** PID Tunings ********************/

void setDefaultPIDConstants(){
    //Drive constants
    chassis.defaultDriveOutputConstants.kp = 3.5;
    chassis.defaultDriveOutputConstants.ki = 0;
    chassis.defaultDriveOutputConstants.kd = 9;
    chassis.defaultDriveOutputConstants.startI = 0;
    chassis.defaultDriveClampConstants.minimumSpeed = 20;
    chassis.defaultDriveClampConstants.maximumSpeed = 75;
    chassis.defaultDriveSettleConstants.deadband = 0.5;
    chassis.defaultDriveSettleConstants.loopCycleTime = 20;
    chassis.defaultDriveSettleConstants.settleTime = 1000;
    chassis.defaultDriveSettleConstants.timeout = 5000;

    //Drive Distance Turn Constants
    chassis.defaultDriveDistanceTurnOutputConstants.kp = 1;
    chassis.defaultDriveDistanceTurnOutputConstants.ki = 0;
    chassis.defaultDriveDistanceTurnOutputConstants.kd = 0;
    chassis.defaultDriveDistanceTurnOutputConstants.startI = 0;
    chassis.defaultDriveDistanceTurnClampConstants.minimumSpeed = 0;
    chassis.defaultDriveDistanceTurnClampConstants.maximumSpeed = 5;

    //Turn Constants
    chassis.defaultTurnOutputConstants.kp = 0.8;
    chassis.defaultTurnOutputConstants.ki = 0;
    chassis.defaultTurnOutputConstants.kd = 0.75;
    chassis.defaultTurnOutputConstants.startI = 10;
    chassis.defaultTurnClampConstants.minimumSpeed = 12;
    chassis.defaultTurnClampConstants.maximumSpeed = 75;
    chassis.defaultTurnSettleConstants.deadband = 3;
    chassis.defaultTurnSettleConstants.loopCycleTime = 20;
    chassis.defaultTurnSettleConstants.settleTime = 750;
    chassis.defaultTurnSettleConstants.timeout = 3000;

    //Swing Constants
    chassis.defaultSwingOutputConstants.kp = 3;
    chassis.defaultSwingOutputConstants.ki = 0.025;
    chassis.defaultSwingOutputConstants.kd = 4;
    chassis.defaultSwingOutputConstants.startI = 10;
    chassis.defaultSwingClampConstants.minimumSpeed = 12;
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
        Arm.spin(forward, ARM_MACRO_KP * (-ArmRotation.position(degrees)), percent);

        wait(20, msec);
    }

    return 0;
}

int armLoad(){
    while (true){
        Arm.spin(forward, ARM_MACRO_KP * (ARM_LOADING_POSITION - ArmRotation.position(degrees)), percent);

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

        wait(100, msec);
    }
}

void runDriveTest(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();
    chassis.driveDistance(12, 0);
    chassis.driveDistance(24, 0);
    chassis.driveDistance(-24, 0);
    chassis.driveDistance(-12, 0);
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
    chassis.setCoordinates(-53.25, -24, 270);
    setDefaultPIDConstants();

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 1;
    shortTurnSettle.loopCycleTime = 20;
    shortTurnSettle.settleTime = 300;
    shortTurnSettle.timeout = 1000;

    Drivetrain::clampConstants withMogoClamp;
    withMogoClamp.minimumSpeed = 20;
    withMogoClamp.maximumSpeed = 80;

    Drivetrain::clampConstants dtp1Clamp;
    dtp1Clamp.minimumSpeed = 0;
    dtp1Clamp.maximumSpeed = 25;

    Drivetrain::settleConstants dtp1Settle;
    dtp1Settle.deadband = 2;
    dtp1Settle.loopCycleTime = 20;
    dtp1Settle.settleTime = 1000;
    dtp1Settle.timeout = 3000;

    MogoMech.set(true);
    chassis.driveDistance(-26, 270, withMogoClamp);
    MogoMech.set(false); //Mogo clamped

    chassis.turnToHeading(180);
    chassis.turnToHeading(180);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent); 
    chassis.driveDistance(26); //Two rings scored on mogo

    chassis.turnToHeading(315);
    MogoMech.set(true); //Mogo released

    Intake.stop(brake);
    chassis.driveToPoint(-48, 0, withMogoClamp, dtp1Settle, chassis.defaultDriveOutputConstants, dtp1Clamp, chassis.defaultTurnOutputConstants);
    chassis.turnToHeading(230);
    chassis.turnToHeading(230);
    chassis.driveDistance(-34);
    MogoMech.set(false); //Mogo clamped

    chassis.turnToHeading(0);
    chassis.driveDistance(10); //Three rings scored on mogo

    chassis.turnToHeading(180);
    chassis.turnToHeading(180);
    Intake.stop(brake);
    chassis.driveDistance(28); //Laddr touched

    chassis.stopDrive(brake);
}

void runAutonRedRushAWP(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

}

void runAutonRedStackAWP(){
    chassis.setCoordinates(-53.25, 24, 270);
    setDefaultPIDConstants();

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 1;
    shortTurnSettle.loopCycleTime = 20;
    shortTurnSettle.settleTime = 300;
    shortTurnSettle.timeout = 1000;

    Drivetrain::clampConstants withMogoClamp;
    withMogoClamp.minimumSpeed = 16;
    withMogoClamp.maximumSpeed = 80;

    Drivetrain::settleConstants dtp1Settle;
    dtp1Settle.deadband = 2;
    dtp1Settle.loopCycleTime = 20;
    dtp1Settle.settleTime = 1000;
    dtp1Settle.timeout = 3000;

    Drivetrain::clampConstants dtp1Clamp;
    dtp1Clamp.minimumSpeed = 0;
    dtp1Clamp.maximumSpeed = 25;

    Drivetrain::clampConstants dtp2Clamp;
    dtp1Clamp.minimumSpeed = 0;
    dtp1Clamp.maximumSpeed = 75;

    MogoMech.set(true);
    chassis.driveDistance(-26, 270, withMogoClamp);
    MogoMech.set(false); //Mogo clamped

    chassis.turnToHeading(60, withMogoClamp);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent); 
    chassis.driveToPoint(-10.5, 40, withMogoClamp, dtp1Settle, chassis.defaultDriveOutputConstants, dtp1Clamp, chassis.defaultTurnOutputConstants);
    chassis.turnToHeading(0, withMogoClamp, shortTurnSettle, chassis.defaultTurnOutputConstants);
    chassis.driveDistance(12, 0, withMogoClamp); //Three rings scored on mogo

    chassis.driveToPoint(-24, 32, withMogoClamp, dtp1Settle, chassis.defaultDriveOutputConstants, dtp2Clamp, chassis.defaultSwingOutputConstants);
    chassis.turnToHeading(0);
    chassis.driveDistance(10, 0, withMogoClamp); //Four rings scored on mogo

    chassis.turnToHeading(180);
    chassis.turnToHeading(180);
    chassis.driveDistance(32); //Ladder touched

    chassis.stopDrive(brake);
}

void runAutonRedGoalRush(){
    chassis.setCoordinates(-53.25, -24, 270);
    setDefaultPIDConstants();

    Drivetrain::clampConstants withMogoClamp;
    withMogoClamp.minimumSpeed = 20;
    withMogoClamp.maximumSpeed = 80;

    Drivetrain::clampConstants dtp1Clamp;
    dtp1Clamp.minimumSpeed = 0;
    dtp1Clamp.maximumSpeed = 25;

    Drivetrain::settleConstants dtp1Settle;
    dtp1Settle.deadband = 2;
    dtp1Settle.loopCycleTime = 20;
    dtp1Settle.settleTime = 1000;
    dtp1Settle.timeout = 3000;

    MogoMech.set(true);
    chassis.driveDistance(-26, 270, withMogoClamp);
    MogoMech.set(false); //Mogo clamped

    chassis.turnToHeading(180);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent); 
    chassis.driveDistance(-4, 180, withMogoClamp);
    chassis.stopDrive(brake);
    wait(250, msec); //One ring scored on mogo

    MogoMech.set(true); //Mogo released

    chassis.driveToPoint(-24, -60, dtp1Clamp, dtp1Settle);
    chassis.turnToHeading(270);
    chassis.driveDistance(26);
    chassis.turnToPoint(false, -66, -66);
    Doinker.set(true);
    chassis.driveDistance(4, chassis.odom.orientation, withMogoClamp);
    chassis.turnToHeading(45); //Corner cleared

    chassis.driveDistance(-6); 
    chassis.stopDrive(brake); //Mogo in positive corner
}

void runAutonBlueSoloAWP(){
    chassis.setCoordinates(53.25, -24, 90);
    setDefaultPIDConstants();

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 1;
    shortTurnSettle.loopCycleTime = 20;
    shortTurnSettle.settleTime = 300;
    shortTurnSettle.timeout = 1000;

    Drivetrain::clampConstants withMogoClamp;
    withMogoClamp.minimumSpeed = 20;
    withMogoClamp.maximumSpeed = 80;

    Drivetrain::clampConstants dtp1Clamp;
    dtp1Clamp.minimumSpeed = 0;
    dtp1Clamp.maximumSpeed = 25;

    Drivetrain::settleConstants dtp1Settle;
    dtp1Settle.deadband = 2;
    dtp1Settle.loopCycleTime = 20;
    dtp1Settle.settleTime = 1000;
    dtp1Settle.timeout = 3000;

    MogoMech.set(true);
    chassis.driveDistance(-26, 90, withMogoClamp);
    MogoMech.set(false); //Mogo clamped

    chassis.turnToHeading(180);
    chassis.turnToHeading(180);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent); 
    chassis.driveDistance(26); //Two rings scored on mogo

    chassis.turnToHeading(45);
    MogoMech.set(true); //Mogo released

    Intake.stop(brake);
    chassis.driveToPoint(48, 0, withMogoClamp, dtp1Settle, chassis.defaultDriveOutputConstants, dtp1Clamp, chassis.defaultTurnOutputConstants);
    chassis.turnToHeading(130);
    chassis.turnToHeading(130);
    chassis.driveDistance(-34);
    MogoMech.set(false); //Mogo clamped

    chassis.turnToHeading(0);
    chassis.driveDistance(10); //Three rings scored on mogo

    chassis.turnToHeading(180);
    chassis.turnToHeading(180);
    Intake.stop(brake);
    chassis.driveDistance(28); //Ladder touched

    chassis.stopDrive(brake);
}

void runAutonBlueRushAWP(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

}

void runAutonBlueStackAWP(){
    chassis.setCoordinates(53.25, 24, 90);
    setDefaultPIDConstants();

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 1;
    shortTurnSettle.loopCycleTime = 20;
    shortTurnSettle.settleTime = 300;
    shortTurnSettle.timeout = 1000;

    Drivetrain::clampConstants withMogoClamp;
    withMogoClamp.minimumSpeed = 16;
    withMogoClamp.maximumSpeed = 80;

    Drivetrain::settleConstants dtp1Settle;
    dtp1Settle.deadband = 2;
    dtp1Settle.loopCycleTime = 20;
    dtp1Settle.settleTime = 1000;
    dtp1Settle.timeout = 3000;

    Drivetrain::clampConstants dtp1Clamp;
    dtp1Clamp.minimumSpeed = 0;
    dtp1Clamp.maximumSpeed = 25;

    Drivetrain::clampConstants dtp2Clamp;
    dtp1Clamp.minimumSpeed = 0;
    dtp1Clamp.maximumSpeed = 75;

    MogoMech.set(true);
    chassis.driveDistance(-26, 90, withMogoClamp);
    MogoMech.set(false); //Mogo clamped

    chassis.turnToHeading(300, withMogoClamp);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent); 
    chassis.driveToPoint(10.5, 40, withMogoClamp, dtp1Settle, chassis.defaultDriveOutputConstants, dtp1Clamp, chassis.defaultTurnOutputConstants);
    chassis.turnToHeading(0, withMogoClamp, shortTurnSettle, chassis.defaultTurnOutputConstants);
    chassis.driveDistance(12, 0, withMogoClamp); //Three rings scored on mogo

    chassis.driveToPoint(24, 32, withMogoClamp, dtp1Settle, chassis.defaultDriveOutputConstants, dtp2Clamp, chassis.defaultSwingOutputConstants);
    chassis.turnToHeading(0);
    chassis.driveDistance(10, 0, withMogoClamp); //Four rings scored on mogo

    chassis.turnToHeading(180);
    chassis.turnToHeading(180);
    chassis.driveDistance(32); //Ladder touched

    chassis.stopDrive(brake);
}

void runAutonBlueGoalRush(){
    chassis.setCoordinates(53.25, -24, 90);
    setDefaultPIDConstants();

    Drivetrain::clampConstants withMogoClamp;
    withMogoClamp.minimumSpeed = 20;
    withMogoClamp.maximumSpeed = 80;

    Drivetrain::clampConstants dtp1Clamp;
    dtp1Clamp.minimumSpeed = 0;
    dtp1Clamp.maximumSpeed = 25;

    Drivetrain::settleConstants dtp1Settle;
    dtp1Settle.deadband = 2;
    dtp1Settle.loopCycleTime = 20;
    dtp1Settle.settleTime = 1000;
    dtp1Settle.timeout = 3000;

    MogoMech.set(true);
    chassis.driveDistance(-26, 90, withMogoClamp);
    MogoMech.set(false); //Mogo clamped

    chassis.turnToHeading(180);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent); 
    chassis.driveDistance(-4, 180, withMogoClamp);
    chassis.stopDrive(brake);
    wait(250, msec); //One ring scored on mogo

    MogoMech.set(true); //Mogo released

    chassis.driveToPoint(60, -24, dtp1Clamp, dtp1Settle);
    chassis.turnToHeading(180);
    chassis.driveDistance(26);
    chassis.turnToPoint(false, 66, -66);
    Doinker.set(true);
    chassis.driveDistance(4, chassis.odom.orientation, withMogoClamp);
    chassis.turnToHeading(315); //Corner cleared

    chassis.driveDistance(-6); 
    chassis.stopDrive(brake); //Mogo in positive corner
}