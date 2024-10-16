#include "vex.h"
#include "autons.h"
#include <iostream>

void setDefaultPIDConstants(){
    //Drive constants
    chassis.defaultDriveOutputConstants.kp = 2.5;
    chassis.defaultDriveOutputConstants.ki = 0;
    chassis.defaultDriveOutputConstants.kd = 0.1;
    chassis.defaultDriveOutputConstants.startI = 0;
    chassis.defaultDriveOutputConstants.minimumSpeed = 15;
    chassis.defaultDriveOutputConstants.maximumSpeed = 40;
    chassis.defaultDriveSettleConstants.deadband = 0.5;
    chassis.defaultDriveSettleConstants.loopCycleTime = 20;
    chassis.defaultDriveSettleConstants.settleTime = 1000;
    chassis.defaultDriveSettleConstants.timeout = 5000;

    //Drive Distance Turn Constants
    chassis.defaultDriveDistanceTurnOutputConstants.kp = 0.4;
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
    chassis.defaultTurnOutputConstants.kd = 1.5;
    chassis.defaultTurnOutputConstants.startI = 5;
    chassis.defaultTurnOutputConstants.minimumSpeed = 3;
    chassis.defaultTurnOutputConstants.maximumSpeed = 75;
    chassis.defaultTurnSettleConstants.deadband = 2;
    chassis.defaultTurnSettleConstants.loopCycleTime = 20;
    chassis.defaultTurnSettleConstants.settleTime = 1000;
    chassis.defaultTurnSettleConstants.timeout = 2500;
}

int colorSortTask(){
    
}

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
    Inertial.calibrate();
    wait(3, sec);
    chassis.setCoordinates(55, 17.5, 90);
    setDefaultPIDConstants();

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 2, shortTurnSettle.loopCycleTime = 20, shortTurnSettle.settleTime = 500, shortTurnSettle.timeout = 1500;

    chassis.turnToPoint(true, 24, 24, shortTurnSettle);
    MogoMech.set(false);
    chassis.driveDistance(-28);
    MogoMech.set(true); //Rush side mogo clamped

    chassis.turnToHeading(0);
    Intake.spin(forward, 85, percent);
    chassis.driveDistance(26, 0);
    wait(0.5, sec); //Two rings scored on mogo
    
    Intake.stop(brake); 
    chassis.driveToPoint(58, 30);
    MogoMech.set(false); //Mogo released

    chassis.turnToHeading(0);
    Intake.spin(forward, 85, percent);
    chassis.driveDistance(10, 0);
    chassis.stopDrive(brake);
    wait(0.2, sec);
    Intake.stop(brake);
    chassis.driveToPoint(50, 51); //Alliance pushed off line

    chassis.turnToHeading(0);
    chassis.driveDistance(-54, 0);
    chassis.turnToHeading(270);
    chassis.driveDistance(-10);
    Intake.spin(forward, 85, percent); //One ring scored on alliance stake

    chassis.driveDistance(24); //Bar Touched
}

void runAutonRedRushAWP(){

}

void runAutonRedStackAWP(){

}

void runAutonRedGoalRush(){

}

void runAutonBlueSoloAWP(){

}

void runAutonBlueRushAWP(){

}

void runAutonBlueStackAWP(){

}

void runAutonBlueGoalRush(){
    
}