#include "vex.h"
#include "autons.h"
#include <iostream>

void setDefaultPIDConstants(){
    //Drive constants
    chassis.defaultDriveOutputConstants.kp = 2.5;
    chassis.defaultDriveOutputConstants.ki = 0;
    chassis.defaultDriveOutputConstants.kd = 0;
    chassis.defaultDriveOutputConstants.startI = 0;
    chassis.defaultDriveOutputConstants.minimumSpeed = 10;
    chassis.defaultDriveSettleConstants.deadband = 0.5;
    chassis.defaultDriveSettleConstants.loopCycleTime = 20;
    chassis.defaultDriveSettleConstants.settleTime = 1000;
    chassis.defaultDriveSettleConstants.timeout = 5000;

    //Turn Constants
    chassis.defaultTurnOutputConstants.kp = 0.7;
    chassis.defaultTurnOutputConstants.ki = 0;
    chassis.defaultTurnOutputConstants.kd = 1;
    chassis.defaultTurnOutputConstants.startI = 0;
    chassis.defaultTurnOutputConstants.minimumSpeed = 0;
    chassis.defaultTurnSettleConstants.deadband = 2;
    chassis.defaultTurnSettleConstants.loopCycleTime = 20;
    chassis.defaultTurnSettleConstants.settleTime = 1250;
    chassis.defaultTurnSettleConstants.timeout = 2000;
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

void driveToPointTest(){
    Inertial.calibrate();
    wait(3, sec);
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

    // chassis.driveDistance(24);
    // chassis.driveDistance(-24);
    // chassis.driveDistance(48);
    // chassis.driveDistance(-48);
    // chassis.stopDrive(brake);
    chassis.turnToHeading(90);
    chassis.turnToHeading(180);
    chassis.turnToHeading(270);
    chassis.turnToHeading(0);
    chassis.turnToHeading(180);
    chassis.turnToHeading(0);
    chassis.stopDrive(brake);
}

void runAutonRedSolo(){

}

void runAutonRedLeft(){

}

void runAutonRedRight(){

}

void runAutonRedRush(){

}

void runAutonBlueSolo(){

}

void runAutonBlueLeft(){

}

void runAutonBlueRight(){

}

void runAutonBlueRush(){
    
}