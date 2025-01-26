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
    chassis.defaultDriveSettleConstants.settleTime = 750;
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
    chassis.defaultTurnSettleConstants.settleTime = 700;
    chassis.defaultTurnSettleConstants.timeout = 1000;

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
    chassis.setCoordinates(-57.5, 17, 180);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 60;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 0;
    clampMogoDriveClamp.maximumSpeed = 50;

    Drivetrain::clampConstants longDriveDriveClamp;
    longDriveDriveClamp.minimumSpeed = 0;
    longDriveDriveClamp.maximumSpeed = 70;

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
    chassis.driveToPoint(-57.5, 8, shortDriveClamp, shortDriveSettle);
    Intake.stop(brake);
    loadForAlly.stop();
    task scoreOnAlly = task(armToAllianceStake);
    chassis.turnToPoint(false, -71, 2, chassis.defaultTurnClampConstants, allyStakeTurnSettle, noMogoTurnOutput); //One ring scored on ally stake;

    scoreOnAlly.stop();
    MogoMech.set(true);
    chassis.driveToPoint(-22, 24, clampMogoDriveClamp, clampMogoDriveSettle);
    task resetArm = task(armToDown);
    MogoMech.set(false); //Mogo clamped

    wait(300, msec);
    chassis.stopDrive(brake);
    chassis.turnToPoint(false, -3, 43);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.driveToPoint(-2, 46);
    chassis.turnToHeading(0, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveDistance(16, 0, chassis.defaultDriveClampConstants, shortDriveSettle); //Two rings scored on mogo

    chassis.driveToPoint(-10, 36, shortDriveClamp, shortDriveSettle);
    chassis.turnToHeading(345);
    chassis.driveDistance(20); //Three rings scored on mogo
 
    chassis.turnToHeading(210);
    MogoMech.set(true);
    Intake.stop(brake);
    chassis.driveDistance(50);
    chassis.turnToHeading(310);
    chassis.driveDistance(-36, 310, clampMogoDriveClamp, clampMogoDriveSettle);
    MogoMech.set(false); //Mogo clamped

    chassis.stopDrive(brake);
    wait(300, msec);
    chassis.turnToHeading(180);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.driveDistance(22);
    chassis.turnToHeading(0);
    chassis.driveDistance(48); //Ladder touched
}

void runAutonRedRushAWP(){
    chassis.setCoordinates(-57.5, -17, 0);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 60;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 0;
    clampMogoDriveClamp.maximumSpeed = 50;

    Drivetrain::clampConstants longDriveDriveClamp;
    longDriveDriveClamp.minimumSpeed = 0;
    longDriveDriveClamp.maximumSpeed = 70;

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
    std::cout << chassis.odom.xPosition << " " << chassis.odom.yPosition << std::endl;
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    task loadForAlly = task(armToLoad);
    chassis.driveToPoint(-57.5, -8, shortDriveClamp, shortDriveSettle);
    Intake.stop(brake);
    loadForAlly.stop();
    task scoreOnAlly = task(armToAllianceStake);
    chassis.turnToPoint(false, -71, -3, chassis.defaultTurnClampConstants, allyStakeTurnSettle, noMogoTurnOutput); //One ring scored on ally stake;

    scoreOnAlly.stop();
    MogoMech.set(true);
    chassis.driveToPoint(-22, -24, clampMogoDriveClamp, clampMogoDriveSettle);
    task resetArm = task(armToDown);
    MogoMech.set(false); //Mogo clamped

    chassis.stopDrive(brake);
    wait(300, msec);
    chassis.turnToHeading(180);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.driveDistance(22);
    chassis.turnToHeading(0);
    chassis.driveDistance(40, 0, shortDriveClamp); //Ladder Touched
}

void runAutonRedStackAWP(){
    chassis.setCoordinates(-57.5, 17, 180);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 60;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 0;
    clampMogoDriveClamp.maximumSpeed = 50;

    Drivetrain::clampConstants longDriveDriveClamp;
    longDriveDriveClamp.minimumSpeed = 0;
    longDriveDriveClamp.maximumSpeed = 70;

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
    chassis.driveToPoint(-57.5, 8, shortDriveClamp, shortDriveSettle);
    Intake.stop(brake);
    loadForAlly.stop();
    task scoreOnAlly = task(armToAllianceStake);
    chassis.turnToPoint(false, -71, 2, chassis.defaultTurnClampConstants, allyStakeTurnSettle, noMogoTurnOutput); //One ring scored on ally stake;

    scoreOnAlly.stop();
    MogoMech.set(true);
    chassis.driveToPoint(-22, 24, clampMogoDriveClamp, clampMogoDriveSettle);
    task resetArm = task(armToDown);
    MogoMech.set(false); //Mogo clamped

    wait(300, msec);
    chassis.stopDrive(brake);
    chassis.turnToPoint(false, -3, 43);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.driveToPoint(-2, 46);
    chassis.turnToHeading(0, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveDistance(16, 0, chassis.defaultDriveClampConstants, shortDriveSettle); //Two rings scored on mogo

    chassis.driveToPoint(-10, 36, shortDriveClamp, shortDriveSettle);
    chassis.turnToHeading(345);
    chassis.driveDistance(20); 
    chassis.stopDrive(brake);
    wait(300, msec); //Three rings scored on mogo

    chassis.turnToHeading(180);
    chassis.driveDistance(44, 180, shortDriveClamp); //Ladder touched
}

void runAutonRedGoalRush(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();

}

void runAutonBlueSoloAWP(){
    chassis.setCoordinates(57.5, 17, 180);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 60;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 0;
    clampMogoDriveClamp.maximumSpeed = 50;

    Drivetrain::clampConstants longDriveDriveClamp;
    longDriveDriveClamp.minimumSpeed = 0;
    longDriveDriveClamp.maximumSpeed = 70;

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
    chassis.driveToPoint(57.5, 8, shortDriveClamp, shortDriveSettle);
    Intake.stop(brake);
    loadForAlly.stop();
    task scoreOnAlly = task(armToAllianceStake);
    chassis.turnToPoint(false, 71, 3, chassis.defaultTurnClampConstants, allyStakeTurnSettle, noMogoTurnOutput); //One ring scored on ally stake;

    scoreOnAlly.stop();
    MogoMech.set(true);
    chassis.driveToPoint(22, 24, clampMogoDriveClamp, clampMogoDriveSettle);
    task resetArm = task(armToDown);
    MogoMech.set(false); //Mogo clamped

    wait(300, msec);
    chassis.stopDrive(brake);
    chassis.turnToPoint(false, 3, 43);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.driveToPoint(11, 35, shortDriveClamp);
    chassis.turnToHeading(0, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveDistance(14, 0, chassis.defaultDriveClampConstants, shortDriveSettle); //Two rings scored on mogo

    chassis.driveToPoint(18, 30, shortDriveClamp, shortDriveSettle);
    chassis.turnToPoint(false, 26, 48, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveDistance(20); //Three rings scored on mogo
 
    chassis.turnToPoint(false, 48, -4, chassis.defaultTurnClampConstants, shortTurnSettle, noMogoTurnOutput);
    Intake.stop(brake);
    MogoMech.set(true);
    chassis.driveToPoint(48, -4, longDriveDriveClamp);
    chassis.turnToPoint(true, 28, -40);
    chassis.driveToPoint(28, -40, clampMogoDriveClamp, clampMogoDriveSettle);
    MogoMech.set(false); //Mogo Clamped
    
    chassis.stopDrive(brake);
    wait(300, msec);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.turnToHeading(180);
    chassis.driveDistance(18, 180); //One ring scored on mogo

    chassis.turnToHeading(0);
    chassis.driveDistance(38, 0); 
    chassis.stopDrive(brake); //Ladder touched
}

void runAutonBlueRushAWP(){
    chassis.setCoordinates(57.5, -17, 0);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 60;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 0;
    clampMogoDriveClamp.maximumSpeed = 50;

    Drivetrain::clampConstants longDriveDriveClamp;
    longDriveDriveClamp.minimumSpeed = 0;
    longDriveDriveClamp.maximumSpeed = 70;

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
    chassis.driveToPoint(57.5, -8, shortDriveClamp, shortDriveSettle);
    Intake.stop(brake);
    loadForAlly.stop();
    task scoreOnAlly = task(armToAllianceStake);
    chassis.turnToPoint(false, 71, -2, chassis.defaultTurnClampConstants, allyStakeTurnSettle, noMogoTurnOutput); //One ring scored on ally stake;

    scoreOnAlly.stop();
    MogoMech.set(true);
    chassis.driveToPoint(22, -24, clampMogoDriveClamp, clampMogoDriveSettle);
    task resetArm = task(armToDown);
    MogoMech.set(false); //Mogo clamped

    wait(300, msec);
    chassis.stopDrive(brake);
    chassis.turnToHeading(180);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.driveDistance(22);
    chassis.turnToHeading(0);
    chassis.driveDistance(40, 0, shortDriveClamp); //Ladder touched
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

    Drivetrain::clampConstants longDriveDriveClamp;
    longDriveDriveClamp.minimumSpeed = 0;
    longDriveDriveClamp.maximumSpeed = 70;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 400;
    shortDriveSettle.timeout = 1250;

    Drivetrain::settleConstants touchLadderDriveSettle;
    touchLadderDriveSettle.deadband = 4;
    touchLadderDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    touchLadderDriveSettle.settleTime = 1500;
    touchLadderDriveSettle.timeout = 3000;

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
    chassis.driveToPoint(57.5, 8, shortDriveClamp, shortDriveSettle);
    Intake.stop(brake);
    loadForAlly.stop();
    task scoreOnAlly = task(armToAllianceStake);
    chassis.turnToPoint(false, 71, 3, chassis.defaultTurnClampConstants, allyStakeTurnSettle, noMogoTurnOutput); //One ring scored on ally stake;

    scoreOnAlly.stop();
    MogoMech.set(true);
    chassis.driveToPoint(22, 24, clampMogoDriveClamp, clampMogoDriveSettle);
    task resetArm = task(armToDown);
    MogoMech.set(false); //Mogo clamped

    wait(300, msec);
    chassis.stopDrive(brake);
    chassis.turnToPoint(false, 3, 43);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.driveToPoint(11, 35, shortDriveClamp);
    chassis.turnToHeading(0, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveDistance(17, 0, chassis.defaultDriveClampConstants, shortDriveSettle); //Two rings scored on mogo

    chassis.driveToPoint(18, 30, shortDriveClamp, shortDriveSettle);
    chassis.turnToPoint(false, 26, 48, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveDistance(20); //Three rings scored on mogo

    chassis.turnToPoint(false, 28, 0);
    chassis.driveDistance(40, chassis.odom.orientation, shortDriveClamp, touchLadderDriveSettle); //Ladder touched
}

void runAutonBlueGoalRush(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();
    
}

/******************** Prog Skills ********************/

void runProgSkills(){
    chassis.setCoordinates(-59.5, 0, 90);
    setDefaultPIDConstants();

    chassis.defaultDriveClampConstants.maximumSpeed = 60;
    chassis.defaultDriveSettleConstants.deadband = 1;

    chassis.defaultTurnSettleConstants.deadband = 1.5;
    chassis.defaultTurnSettleConstants.settleTime = 750;

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 50;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 0;
    clampMogoDriveClamp.maximumSpeed = 60;

    Drivetrain::clampConstants slowDriveClamp;
    slowDriveClamp.minimumSpeed = 0;
    slowDriveClamp.maximumSpeed = 40;

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
    chassis.driveToPoint(-47, 0, shortDriveClamp, shortDriveSettle);
    Intake.stop(brake);
    chassis.turnToHeading(180);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    MogoMech.set(true);
    chassis.driveDistance(-20, 180, clampMogoDriveClamp, clampMogoDriveSettle, clampMogoDriveOutput);
    MogoMech.set(false); //Mogo clamped
    
    wait(500, msec);
    chassis.turnToPoint(false, -24, 20);
    chassis.driveToPoint(-24, 20); //One ring scored on mogo

    chassis.turnToPoint(false, 0, 48);
    chassis.driveToPoint(24, 44); //Two rings scored on mogo

    chassis.turnToHeading(270);
    chassis.driveToPoint(-24, 46, slowDriveClamp);
    chassis.stopDrive(brake);

    chassis.driveToPoint(-58, 46, slowDriveClamp); //Five rings scored on mogo

    chassis.stopDrive(brake);
    wait(300, msec);
    chassis.turnToHeading(45);
    chassis.driveDistance(6); 
    wait(500, msec);
    chassis.stopDrive(brake); //Six rings scored on mogo

    chassis.turnToPoint(true, -72, 75);
    chassis.driveDistance(-7);
    MogoMech.set(true); //Mogo deposited in corner

    Intake.spin(reverse, INTAKE_DEFAULT_SPEED, percent);
    chassis.stopDrive(brake);
    wait(2000, msec);
    chassis.driveDistance(10);
    chassis.turnToHeading(0);

    chassis.driveDistance(-32, 0);
    chassis.driveDistance(-44, 0, clampMogoDriveClamp, clampMogoDriveSettle, clampMogoDriveOutput);
    MogoMech.set(false); //Mogo clamped

    wait(500, msec);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.turnToHeading(90);
    chassis.driveDistance(24); //One ring scored on mogo

    chassis.turnToHeading(180);
    chassis.driveDistance(24); //Two rings scored on mogo

    chassis.turnToHeading(90);
    chassis.driveDistance(46); //Three rings scored on mogo

    chassis.turnToHeading(270);
    chassis.driveDistance(46, 270, slowDriveClamp);
    chassis.driveDistance(38, 270, slowDriveClamp); //Five rings scored on mogo

    chassis.turnToHeading(135);
    chassis.driveDistance(8); 
    wait(500, msec); 
    chassis.stopDrive(brake); //Six rings scored on mogo

    chassis.turnToHeading(50);
    chassis.driveDistance(-10);
    MogoMech.set(true); //Mogo deposited in corner

    Intake.spin(reverse, INTAKE_DEFAULT_SPEED, percent);
    chassis.stopDrive(brake);
    wait(2000, msec);

    chassis.turnToHeading(80);
    chassis.driveDistance(78);
    chassis.turnToHeading(0);
    chassis.turnToHeading(0);
    Intake.stop(brake);
    chassis.driveDistance(24);

    chassis.turnToHeading(225);
    chassis.driveDistance(-32, 225, clampMogoDriveClamp, clampMogoDriveSettle, clampMogoDriveOutput);
    MogoMech.set(false); //Mogo clamped

    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent); //One ring scored on mogo
    chassis.turnToHeading(0);
    chassis.driveDistance(60); //Three rings scored on mogo

    chassis.turnToHeading(240);
    chassis.driveDistance(-8);
    MogoMech.set(true);
    chassis.stopDrive(brake);
    wait(500, msec); //Mogo deposited in corner

    chassis.driveDistance(20);
    chassis.turnToHeading(160);
    chassis.driveDistance(48);
    chassis.turnToHeading(180);
    chassis.driveDistance(80); //Mogo pushed into corner

    Intake.stop(brake);
    chassis.stopDrive(brake);
}