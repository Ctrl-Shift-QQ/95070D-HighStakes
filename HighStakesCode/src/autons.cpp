#include "vex.h"
#include "autons.h"
#include <iostream>

/******************** PID Tunings ********************/

void setDefaultPIDConstants(){
    //Drive Constants
    chassis.defaultDriveOutputConstants.kp = 10;
    chassis.defaultDriveOutputConstants.ki = 0;
    chassis.defaultDriveOutputConstants.kd = 0.75;
    chassis.defaultDriveOutputConstants.startI = 0;
    chassis.defaultDriveClampConstants.minimumSpeed = 0;
    chassis.defaultDriveClampConstants.maximumSpeed = 90;
    chassis.defaultDriveSettleConstants.deadband = 2;
    chassis.defaultDriveSettleConstants.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    chassis.defaultDriveSettleConstants.settleTime = 1000;
    chassis.defaultDriveSettleConstants.timeout = 3000;

    //Heading Adjustment Constants
    chassis.defaultHeadingOutputConstants.kp = 3;
    chassis.defaultHeadingOutputConstants.ki = 0;
    chassis.defaultHeadingOutputConstants.kd = 0.24;
    chassis.defaultHeadingOutputConstants.startI = 0;
    chassis.defaultHeadingClampConstants.minimumSpeed = 0;
    chassis.defaultHeadingClampConstants.maximumSpeed = 100;
    chassis.defaultHeadingSettleConstants.deadband = 5;

    //Drive Distance Turn Constants
    chassis.defaultDriveDistanceTurnOutputConstants.kp = 2;
    chassis.defaultDriveDistanceTurnOutputConstants.ki = 0;
    chassis.defaultDriveDistanceTurnOutputConstants.kd = 0;
    chassis.defaultDriveDistanceTurnOutputConstants.startI = 0;
    chassis.defaultDriveDistanceTurnClampConstants.minimumSpeed = 0;
    chassis.defaultDriveDistanceTurnClampConstants.maximumSpeed = 5;

    //Turn Constants
    chassis.defaultTurnOutputConstants.kp = 3;
    chassis.defaultTurnOutputConstants.ki = 30;
    chassis.defaultTurnOutputConstants.kd = 0.24;
    chassis.defaultTurnOutputConstants.startI = 10;
    chassis.defaultTurnClampConstants.minimumSpeed = 0;
    chassis.defaultTurnClampConstants.maximumSpeed = 100;
    chassis.defaultTurnSettleConstants.deadband = 2;
    chassis.defaultTurnSettleConstants.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    chassis.defaultTurnSettleConstants.settleTime = 750;
    chassis.defaultTurnSettleConstants.timeout = 1250;

    //Swing Constants
    chassis.defaultSwingOutputConstants.kp = 6;
    chassis.defaultSwingOutputConstants.ki = 10;
    chassis.defaultSwingOutputConstants.kd = 0.4;
    chassis.defaultSwingOutputConstants.startI = 10;
    chassis.defaultSwingClampConstants.minimumSpeed = 0;
    chassis.defaultSwingClampConstants.maximumSpeed = 75;
    chassis.defaultSwingSettleConstants.deadband = 3;
    chassis.defaultSwingSettleConstants.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    chassis.defaultSwingSettleConstants.settleTime = 700;
    chassis.defaultSwingSettleConstants.timeout = 3000;
}

/******************** Tasks ********************/

int colorSort(){
    return 0;
}

void spinArmTo(double targetPosition, bool outtake){
    double armOutput;
    double intakeOutput;

    PID armPID(targetPosition, ARM_MACRO_KP, 0, 0, 0, ARM_MACRO_DEADBAND, DEFAULT_LOOP_CYCLE_TIME, 0, 0);

    while (true){
        if (!armPID.isSettled(targetPosition - ArmRotation.position(degrees))){
            armOutput = armPID.output(targetPosition - ArmRotation.position(degrees));
            intakeOutput = 0;

            if (outtake){
                intakeOutput = ARM_INTAKE_SPEED;
                Intake.spin(reverse, intakeOutput, percent);
            }

            Arm.spin(forward, percentToVolts(armOutput), volt);
        }
        else {
            Arm.stop(brake);
        }

        wait(armPID.loopCycleTime, msec);
    }
}

int armToDown(){
    spinArmTo(0, false);

    return 0;
}

int armToLoad(){
    spinArmTo(ARM_LOADING_POSITION, false);

    return 0;
}

int armToAllianceStake(){
    spinArmTo(ARM_ALLIANCE_STAKE_POSITION, true);

    return 0;
}

int armToWallStake(){
    spinArmTo(ARM_WALL_STAKE_POSITION, true);

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

    chassis.turnToHeading(22.5);
    chassis.turnToHeading(45);
    chassis.turnToHeading(90);
    chassis.turnToHeading(180);
    chassis.turnToHeading(270);
    chassis.turnToHeading(45);
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
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

}

void runAutonRedRushAWP(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

}

void runAutonRedStackAWP(){
    chassis.setCoordinates(-58.5, 17, 180);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 50;
    
    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 600;
    shortDriveSettle.timeout = 1500;

    Drivetrain::settleConstants noMogoTurnSettle;
    noMogoTurnSettle.deadband = 2;
    noMogoTurnSettle.loopCycleTime = 20;
    noMogoTurnSettle.settleTime = 1500;
    noMogoTurnSettle.timeout = 2000;

    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    task loadForAlly = task(armToLoad);
    chassis.driveToPoint(-59, 8, shortDriveClamp, shortDriveSettle, chassis.defaultDriveOutputConstants);
    Intake.stop(brake);
    loadForAlly.stop();
    task scoreOnAlly = task(armToAllianceStake);
    chassis.turnToPoint(false, -70.5, 0, chassis.defaultTurnClampConstants, noMogoTurnSettle); //One ring scored on ally stake;

    std::cout << "Final Coords:" << chassis.odom.xPosition << " " << chassis.odom.yPosition << std::endl;

    chassis.stopDrive(brake);
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
    chassis.setCoordinates(55.25, -24, 90);
    setDefaultPIDConstants();

}

void runAutonBlueStackAWP(){
    chassis.setCoordinates(57.5, 17, 180);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 60;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 0;
    clampMogoDriveClamp.maximumSpeed = 50;
    
    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 400;
    shortDriveSettle.timeout = 1250;

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 2;
    shortTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortTurnSettle.settleTime = 500;
    shortTurnSettle.timeout = 1250;

    Drivetrain::settleConstants allyStakeTurnSettle;
    allyStakeTurnSettle.deadband = 2;
    allyStakeTurnSettle.loopCycleTime = 20;
    allyStakeTurnSettle.settleTime = 1500;
    allyStakeTurnSettle.timeout = 2000;

    Drivetrain::settleConstants noMogoTurnSettle;
    noMogoTurnSettle.deadband = 2;
    noMogoTurnSettle.loopCycleTime = 20;
    noMogoTurnSettle.settleTime = 750;
    noMogoTurnSettle.timeout = 2000;

    Drivetrain::settleConstants clampMogoDriveSettle;
    clampMogoDriveSettle.deadband = 2;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::outputConstants noMogoTurnOutput;
    noMogoTurnOutput.kp = 2;
    noMogoTurnOutput.ki = 30;
    noMogoTurnOutput.kd = 0.125;
    noMogoTurnOutput.startI = 10;

    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    task loadForAlly = task(armToLoad);
    chassis.driveToPoint(57.5, 7, shortDriveClamp, shortDriveSettle);
    Intake.stop(brake);
    loadForAlly.stop();
    task scoreOnAlly = task(armToAllianceStake);
    chassis.turnToPoint(false, 71, 2, chassis.defaultTurnClampConstants, allyStakeTurnSettle, noMogoTurnOutput); //One ring scored on ally stake;

    scoreOnAlly.stop();
    // MogoMech.set(true);
    // chassis.driveToPoint(24, 24, clampMogoDriveClamp, clampMogoDriveSettle);
    // task resetArm = task(armToDown);
    // MogoMech.set(false); //Mogo clamped

    // wait(300, msec);
    // chassis.turnToPoint(false, 3, 43);
    // Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    // chassis.driveToPoint(10, 36);
    // chassis.turnToHeading(0, chassis.defaultTurnClampConstants, shortTurnSettle);
    // chassis.driveDistance(10, 0, chassis.defaultDriveClampConstants, shortDriveSettle); 
    // chassis.stopDrive(brake);
    // wait(150, msec); //Two rings scored on mogo

    // chassis.driveToPoint(18, 30, shortDriveClamp, shortDriveSettle);
    // chassis.turnToPoint(false, 28, 48, chassis.defaultTurnClampConstants, shortTurnSettle);
    // chassis.driveToPoint(27, 43); //Three rings scored on mogo
 
    // chassis.turnToPoint(false, 52, -10);
    // MogoMech.set(true);
    // chassis.driveToPoint(52, -10);
    // chassis.turnToPoint(true, 24, -30, chassis.defaultTurnClampConstants, noMogoTurnSettle);
    // chassis.driveToPoint(24, -30, clampMogoDriveClamp, clampMogoDriveSettle);
    // MogoMech.set(false); //Mogo clamped
    
    // wait(300, msec);
    // chassis.turnToPoint(false, 24, -48);

    std::cout << "Final Coords: " << chassis.odom.xPosition << " " << chassis.odom.yPosition << std::endl;

    chassis.stopDrive(brake);
}

void runAutonBlueGoalRush(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();
    
}

/******************** Prog Skills ********************/

void runProgSkills(){
    chassis.setCoordinates(-61.25, 0, 90);
    setDefaultPIDConstants();

    chassis.defaultDriveClampConstants.maximumSpeed = 60;

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 50;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 0;
    clampMogoDriveClamp.maximumSpeed = 60;

    Drivetrain::clampConstants slowDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 0;
    clampMogoDriveClamp.maximumSpeed = 40;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 600;
    shortDriveSettle.timeout = 1500;

    Drivetrain::settleConstants clampMogoDriveSettle;
    clampMogoDriveSettle.deadband = 2;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::outputConstants clampMogoDriveOutput;
    clampMogoDriveOutput.kp = 4;
    clampMogoDriveOutput.ki = 0;
    clampMogoDriveOutput.kd = 0.1;
    clampMogoDriveOutput.startI = 0;
    
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    wait(0.5, sec); //One ring scored on alliance stake

    Intake.spin(reverse, INTAKE_DEFAULT_SPEED, percent);
    wait(0.1, sec);
    chassis.driveToPoint(-50, 0, shortDriveClamp, shortDriveSettle);
    Intake.stop(brake);
    chassis.turnToPoint(true, -48, 20);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    MogoMech.set(true);
    chassis.driveToPoint(-48, 18, clampMogoDriveClamp, clampMogoDriveSettle, clampMogoDriveOutput);
    MogoMech.set(false); //Mogo clamped
    
    wait(500, msec);
    chassis.turnToPoint(false, -24, 24);
    chassis.driveToPoint(-22, 24); //One ring scored on mogo

    chassis.turnToPoint(false, 0, 48);
    chassis.driveToPoint(24, 48); //Two rings scored on mogo

    task firstStakeLoad = task(armToLoad);
    chassis.turnToPoint(false, 0, 40);
    chassis.driveToPoint(0, 40);
    chassis.stopDrive(brake);
    wait(100, msec);
    chassis.turnToHeading(0);
    chassis.driveDistance(20, 0, slowDriveClamp);
    // chassis.stopDrive(brake);
    // wait(5000, msec);
    // chassis.setCoordinates(0, 61, chassis.odom.orientation); //Wall stakes shenanigans

    // chassis.driveToPoint(0, 50);
    // chassis.turnToHeading(270);
    // chassis.driveToPoint(-58, 48, slowDriveClamp);

    // wait(300, msec);
    // chassis.stopDrive(brake);
    // chassis.turnToPoint(false, -48, 60);
    // chassis.driveToPoint(-48, 60);

    Intake.stop(brake);
    chassis.stopDrive(brake);
}