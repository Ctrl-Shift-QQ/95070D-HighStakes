#include "vex.h"
#include "autons.h"
#include <iostream>

/******************** PID Tunings ********************/

void setDefaultPIDConstants(){
    //Drive constants
    chassis.defaultDriveOutputConstants.kp = 10;
    chassis.defaultDriveOutputConstants.ki = 0;
    chassis.defaultDriveOutputConstants.kd = 0.75;
    chassis.defaultDriveOutputConstants.startI = 0;
    chassis.defaultDriveClampConstants.minimumSpeed = 0;
    chassis.defaultDriveClampConstants.maximumSpeed = 90;
    chassis.defaultDriveSettleConstants.deadband = 1;
    chassis.defaultDriveSettleConstants.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    chassis.defaultDriveSettleConstants.settleTime = 1000;
    chassis.defaultDriveSettleConstants.timeout = 3000;

    //Drive Distance Turn Constants
    chassis.defaultDriveDistanceTurnOutputConstants.kp = 2;
    chassis.defaultDriveDistanceTurnOutputConstants.ki = 0;
    chassis.defaultDriveDistanceTurnOutputConstants.kd = 0;
    chassis.defaultDriveDistanceTurnOutputConstants.startI = 0;
    chassis.defaultDriveDistanceTurnClampConstants.minimumSpeed = 0;
    chassis.defaultDriveDistanceTurnClampConstants.maximumSpeed = 5;

    //Turn Constants
    chassis.defaultTurnOutputConstants.kp = 3;
    chassis.defaultTurnOutputConstants.ki = 0;
    chassis.defaultTurnOutputConstants.kd = 0.15;
    chassis.defaultTurnOutputConstants.startI = 0;
    chassis.defaultTurnClampConstants.minimumSpeed = 0;
    chassis.defaultTurnClampConstants.maximumSpeed = 100;
    chassis.defaultTurnSettleConstants.deadband = 3;
    chassis.defaultTurnSettleConstants.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    chassis.defaultTurnSettleConstants.settleTime = 750;
    chassis.defaultTurnSettleConstants.timeout = 3000;

    //Swing Constants
    chassis.defaultSwingOutputConstants.kp = 9;
    chassis.defaultSwingOutputConstants.ki = 0.75;
    chassis.defaultSwingOutputConstants.kd = 0.6;
    chassis.defaultSwingOutputConstants.startI = 10;
    chassis.defaultSwingClampConstants.minimumSpeed = 0;
    chassis.defaultSwingClampConstants.maximumSpeed = 75;
    chassis.defaultSwingSettleConstants.deadband = 3;
    chassis.defaultSwingSettleConstants.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    chassis.defaultSwingSettleConstants.settleTime = 1000;
    chassis.defaultSwingSettleConstants.timeout = 3000;
}

/******************** Tasks ********************/

int colorSort(){
    return 0;
}

// void spinArmTo(double targetPosition){
//     PID armPID(targetPosition, ARM_MACRO_KP, ARM_MACRO_KI, 0, ARM_MACRO_START_I, 0, DEFAULT_LOOP_CYCLE_TIME, 0, 0);

//     while (true){
//         Arm.spin(forward, percentToVolts(armPID.output(targetPosition - Arm.position(degrees))), volt);
//         wait(DEFAULT_LOOP_CYCLE_TIME, msec);
//     }
// }

// int armToDown(){
//     spinArmTo(0);

//     return 0;
// }

// int armToLoad(){
//     spinArmTo(ARM_LOADING_POSITION);

//     return 0;
// }

// int armToAllianceStake(){
//     spinArmTo(ARM_ALLIANCE_STAKE_POSITION);

//     return 0;
// }

// int armToWallStake(){
//     spinArmTo(ARM_WALL_STAKE_POSITION);

//     return 0;
// }

int armToDown(){
    while (true){
        double output = ARM_MACRO_KP * -ArmRotation.position(degrees); 

        if (fabs(output) < ARM_MACRO_MINIMUM_SPEED){
            output = ARM_MACRO_MINIMUM_SPEED * getSign(output);
        }

        Arm.spin(forward, percentToVolts(output), volt);
        Intake.spin(reverse, ARM_INTAKE_SPEED, percent);

        wait(20, msec);
    }

    return 0;
}

int armToLoad(){
    while (true){
        double output = ARM_MACRO_KP * (ARM_LOADING_POSITION - ArmRotation.position(degrees));

        if (fabs(output) < ARM_MACRO_MINIMUM_SPEED){
            output = ARM_MACRO_MINIMUM_SPEED * getSign(output);
        }

        Arm.spin(forward, percentToVolts(output), volt);
        Intake.spin(reverse, ARM_INTAKE_SPEED, percent);

        wait(20, msec);
    }

    return 0;
}

int armToAllianceStake(){
    while (true){
        double output = ARM_MACRO_KP * (ARM_ALLIANCE_STAKE_POSITION - ArmRotation.position(degrees));
        Intake.spin(reverse, ARM_INTAKE_SPEED, percent);

        if (fabs(output) < ARM_MACRO_MINIMUM_SPEED){
            output = ARM_MACRO_MINIMUM_SPEED * getSign(output);
        }

        Arm.spin(forward, percentToVolts(output), volt);
        Intake.spin(reverse, ARM_INTAKE_SPEED, percent);

        wait(20, msec);
    }
}

int armToWallStake(){
    while (true){
        double output = ARM_MACRO_KP * (ARM_WALL_STAKE_POSITION - ArmRotation.position(degrees));

        if (fabs(output) < ARM_MACRO_MINIMUM_SPEED){
            output = ARM_MACRO_MINIMUM_SPEED * getSign(output);
        }

        Arm.spin(forward, percentToVolts(output), volt);
        Intake.spin(reverse, ARM_INTAKE_SPEED, percent);

        wait(20, msec);
    }
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

    chassis.driveToPoint(0, 12);
    chassis.driveToPoint(0, 36);
    chassis.driveToPoint(0, 12);
    chassis.driveToPoint(0, 0);
    chassis.stopDrive(brake);
}

void runTurnTest(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

    chassis.turnToPoint(false, 24, 24);
    chassis.turnToPoint(false, 0, 24);
    chassis.turnToPoint(false, 0, -24);
    chassis.turnToPoint(false, 0, 24);
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
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

}

void runAutonRedRushAWP(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

}

void runAutonRedStackAWP(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

}

void runAutonRedGoalRush(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

}

void runAutonBlueSoloAWP(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

}

void runAutonBlueRushAWP(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

}

void runAutonBlueStackAWP(){
    chassis.setCoordinates(57.5, 17, 180);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 50;

    Drivetrain::clampConstants clampMogoClamp;
    shortDriveClamp.minimumSpeed = 20;
    shortDriveClamp.maximumSpeed = 70;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 500;
    shortDriveSettle.timeout = 1500;

    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    task loadForAlly = task(armToLoad);
    chassis.driveToPoint(57.5, 7, shortDriveClamp, shortDriveSettle, chassis.defaultDriveOutputConstants);
    Intake.stop();
    loadForAlly.stop();
    task scoreOnAlly = task(armToAllianceStake);
    chassis.turnToPoint(false, 71, 0); //One ring scored on ally stake;

    wait(500, msec);
    scoreOnAlly.stop();
    task resetArm = task(armToDown);
    MogoMech.set(true);
    chassis.driveToPoint(26, 23, clampMogoClamp);
    chassis.stopDrive(brake);
    wait(100, msec);
    MogoMech.set(false); //Mogo clamped

    chassis.turnToPoint(false, 3, 45);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.driveToPoint(12, 36);
    // chassis.swingToHeading("Right", 0);

    std::cout << chassis.odom.xPosition << " " << chassis.odom.yPosition << std::endl;

    chassis.stopDrive(brake);
}

void runAutonBlueGoalRush(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();
    
}