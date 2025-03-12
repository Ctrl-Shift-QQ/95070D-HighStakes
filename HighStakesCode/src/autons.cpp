#include "vex.h"
#include "autons.h"
#include <iostream>

/******************** PID Tunings ********************/

void setDefaultPIDConstants(){
    //Drive Constants
    chassis.defaultDriveOutputConstants.kp = 9;
    chassis.defaultDriveOutputConstants.ki = 0;
    chassis.defaultDriveOutputConstants.kd = 0.7;
    chassis.defaultDriveOutputConstants.startI = 0;
    chassis.defaultDriveClampConstants.minimumSpeed = 0;
    chassis.defaultDriveClampConstants.maximumSpeed = 85;
    chassis.defaultDriveSettleConstants.deadband = 1.5;
    chassis.defaultDriveSettleConstants.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    chassis.defaultDriveSettleConstants.settleTime = 750;
    chassis.defaultDriveSettleConstants.timeout = 3000;

    //Heading Adjustment Constants
    chassis.defaultHeadingOutputConstants.kp = 3;
    chassis.defaultHeadingOutputConstants.ki = 0;
    chassis.defaultHeadingOutputConstants.kd = 0.25;
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
    chassis.defaultTurnOutputConstants.ki = 25;
    chassis.defaultTurnOutputConstants.kd = 0.225;
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
    int hue;
    int hueRange;
    double intakeVelocity;

    if (allianceColor == "Red"){
        hue = SORT_BLUE_HUE;
        hueRange = SORT_BLUE_HUE_RANGE;
    }
    else {
        hue = SORT_RED_HUE;
        hueRange = SORT_RED_HUE_RANGE;
    }

    IntakeOptical.integrationTime(5);
    IntakeOptical.setLightPower(100, percent);

    while (true){
        std::cout << IntakeOptical.hue() << " " << hue << " " << hueRange << std::endl;
        if (headingError(hue, IntakeOptical.hue()) < hueRange){
            std::cout << "wtf" << std::endl;
            wait(SORT_DETECT_TO_SORT_DELAY, msec);

            Intake.stop(brake);

            wait(SORT_SORT_TO_CONTINUE_DELAY, msec);
            Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
        }

        wait(DEFAULT_LOOP_CYCLE_TIME / 2, msec);
    }

    return 0;
}

void spinArmTo(double targetPosition, bool outtake){
    double PIDOutput;
    double FFOutput;
    double output;

    PID armPID(targetPosition, ARM_MACRO_KP, 0, 0, 0, 0, DEFAULT_LOOP_CYCLE_TIME, 0, 0);

    while (true){
        PIDOutput = armPID.output(targetPosition - ArmRotation.position(degrees));
        FFOutput = ARM_MACRO_KCOS * cos(degToRad(ArmRotation.position(degrees)));

        output = PIDOutput + FFOutput;

        if (outtake){
            SecondIntake.spin(reverse, ARM_INTAKE_SPEED, percent);
        }

        Arm.spin(forward, percentToVolts(output), volt);

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

int armToLadder(){
    spinArmTo(ARM_UP_POSITION, false);

    return 0;
}

int armToUp(){
    spinArmTo(ARM_UP_POSITION, true);

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
    chassis.setCoordinates(-60, 12, 225);

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
    clampMogoDriveSettle.deadband = 1.5;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::outputConstants noMogoTurnOutput;
    noMogoTurnOutput.kp = 2;
    noMogoTurnOutput.ki = 30;
    noMogoTurnOutput.kd = 0.125;
    noMogoTurnOutput.startI = 10;

    task sort = task(colorSort);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
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
    shortDriveSettle.deadband = 2;
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
    clampMogoDriveSettle.deadband = 1.5;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::outputConstants noMogoTurnOutput;
    noMogoTurnOutput.kp = 2;
    noMogoTurnOutput.ki = 30;
    noMogoTurnOutput.kd = 0.125;
    noMogoTurnOutput.startI = 10;
}

void runAutonRedStackAWP(){
    chassis.setCoordinates(-60, 12, 225);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 60;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1.5;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 500;
    shortDriveSettle.timeout = 1200;

    Drivetrain::clampConstants shallowCurveTurnClamp;
    shallowCurveTurnClamp.minimumSpeed = 0;
    shallowCurveTurnClamp.maximumSpeed = 25;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 0;
    clampMogoDriveClamp.maximumSpeed = 40;

    Drivetrain::settleConstants clampMogoDriveSettle;
    clampMogoDriveSettle.deadband = 1.5;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 2;
    shortTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortTurnSettle.settleTime = 400;
    shortTurnSettle.timeout = 1250;

    Drivetrain::clampConstants steepCurveDriveClamp;
    steepCurveDriveClamp.minimumSpeed = 0;
    steepCurveDriveClamp.maximumSpeed = 50;

    Drivetrain::settleConstants ramDriveSettle;
    ramDriveSettle.deadband = 6;
    ramDriveSettle.loopCycleTime = 20;
    ramDriveSettle.settleTime = 400;
    ramDriveSettle.timeout = 1500;

    Drivetrain::clampConstants slowerDriveClamp;
    slowerDriveClamp.minimumSpeed = 0;
    slowerDriveClamp.maximumSpeed = 55;

    task scoreOnAlly = task(armToAllianceStake);
    chassis.driveToPoint(-62, 9, shortDriveClamp, shortDriveSettle);
    chassis.stopDrive(coast);
    wait(250, msec);  //One ring scored on alliance stake

    scoreOnAlly.stop();
    task armDown = task(armToDown);
    chassis.driveToPoint(-25, 24, clampMogoDriveClamp, clampMogoDriveSettle, chassis.defaultDriveOutputConstants,
                         shallowCurveTurnClamp, chassis.defaultHeadingSettleConstants, chassis.defaultHeadingOutputConstants); 
    MogoMech.set(true); //Mogo clamped

    wait(250, msec);
    armDown.stop();
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.turnToPoint(false, -6.5, 39, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveToPoint(-6.5, 39); //One ring scored on mogo

    chassis.swingToHeading("Right", 0, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.stopDrive(coast);
    chassis.driveDistance(10, 0, shortDriveClamp, shortDriveSettle); //Two rings scored on mogo

    chassis.driveToPoint(-10, 32, steepCurveDriveClamp, shortDriveSettle);
    chassis.turnToPoint(false, -20, 45, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveToPoint(-22, 45, shortDriveClamp, shortDriveSettle); //Three rings scored on mogo

    chassis.driveToPoint(-55, 55, steepCurveDriveClamp);
    Intake.spin(reverse, INTAKE_DEFAULT_SPEED, percent);
    chassis.turnToPoint(false, -68, 70);
    chassis.driveToPoint(-68, 70, chassis.defaultDriveClampConstants, ramDriveSettle);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    wait(500, msec);
    chassis.driveDistance(-20, chassis.odom.orientation, chassis.defaultDriveClampConstants, shortDriveSettle);
    chassis.driveDistance(15, chassis.odom.orientation, shortDriveClamp, shortDriveSettle); //Five rings scored on mogo

    chassis.driveToPoint(-28, 28, slowerDriveClamp);
    chassis.turnToPoint(false, 0, 0);
    ArmMotor.spin(forward, 95, percent);
    wait(750, msec);
    ArmMotor.stop(brake); //Ladder touched

    chassis.stopDrive(coast);
}

void runAutonRedGoalRush(){
    chassis.setCoordinates(0, 0, 0);
    setDefaultPIDConstants();


}

void runAutonBlueSoloAWP(){
    chassis.setCoordinates(60, 12, 135);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 60;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1.5;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 500;
    shortDriveSettle.timeout = 1200;

    Drivetrain::clampConstants shallowCurveTurnClamp;
    shallowCurveTurnClamp.minimumSpeed = 0;
    shallowCurveTurnClamp.maximumSpeed = 25;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 0;
    clampMogoDriveClamp.maximumSpeed = 40;

    Drivetrain::settleConstants clampMogoDriveSettle;
    clampMogoDriveSettle.deadband = 1.5;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 2;
    shortTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortTurnSettle.settleTime = 400;
    shortTurnSettle.timeout = 1250;

    Drivetrain::clampConstants steepCurveDriveClamp;
    steepCurveDriveClamp.minimumSpeed = 0;
    steepCurveDriveClamp.maximumSpeed = 50;

    Drivetrain::settleConstants ramDriveSettle;
    ramDriveSettle.deadband = 6;
    ramDriveSettle.loopCycleTime = 20;
    ramDriveSettle.settleTime = 400;
    ramDriveSettle.timeout = 1500;

    Drivetrain::clampConstants slowerDriveClamp;
    slowerDriveClamp.minimumSpeed = 0;
    slowerDriveClamp.maximumSpeed = 55;

    task scoreOnAlly = task(armToAllianceStake);
    chassis.driveToPoint(62, 9, shortDriveClamp, shortDriveSettle);
    chassis.stopDrive(coast);
    wait(250, msec);  //One ring scored on alliance stake

    scoreOnAlly.stop();
    task armDown = task(armToDown);
    chassis.driveToPoint(25, 24, clampMogoDriveClamp, clampMogoDriveSettle, chassis.defaultDriveOutputConstants,
                         shallowCurveTurnClamp, chassis.defaultHeadingSettleConstants, chassis.defaultHeadingOutputConstants); 
    MogoMech.set(true); 
    wait(250, msec); //Mogo clamped

    armDown.stop();
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.turnToPoint(false, 6.5, 39, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveToPoint(6.5, 39); //One ring scored on mogo

    chassis.swingToHeading("Left", 0, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.stopDrive(coast);
    chassis.driveDistance(10, 0, shortDriveClamp, shortDriveSettle); //Two rings scored on mogo

    chassis.driveToPoint(10, 32, steepCurveDriveClamp, shortDriveSettle);
    chassis.turnToPoint(false, 20, 45, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveToPoint(22, 45, shortDriveClamp, shortDriveSettle); //Three rings scored on mogo

    chassis.driveToPoint(50, 36, shortDriveClamp);
    chassis.turnToPoint(false, 48, 0, chassis.defaultTurnClampConstants, shortTurnSettle);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED * 0.6, percent);
    chassis.driveToPoint(48, 10, chassis.defaultDriveClampConstants, shortDriveSettle);
    MogoMech.set(false);
    chassis.driveToPoint(48, -20, shortDriveClamp, shortDriveSettle); //Ring intaked

    Intake.stop(brake);
    chassis.turnToPoint(true, 27, -24, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveToPoint(28, -24, clampMogoDriveClamp, clampMogoDriveSettle);
    MogoMech.set(true); 
    wait(250, msec); //Mogo clamped

    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent); //One ring scored on mogo

    chassis.turnToPoint(false, 24, -48, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveToPoint(25, -46, chassis.defaultDriveClampConstants, shortDriveSettle); //Two rings scored on mogo

    task armToLadder = task(armToLadder);
    chassis.driveToPoint(16, -16, steepCurveDriveClamp);
    armToLadder.stop(); //Ladder touched

    chassis.stopDrive(coast);
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
    clampMogoDriveSettle.deadband = 1.5;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::outputConstants noMogoTurnOutput;
    noMogoTurnOutput.kp = 2;
    noMogoTurnOutput.ki = 30;
    noMogoTurnOutput.kd = 0.125;
    noMogoTurnOutput.startI = 10;
}

void runAutonBlueStackAWP(){
    chassis.setCoordinates(60, 12, 135);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 60;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1.5;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 500;
    shortDriveSettle.timeout = 1200;

    Drivetrain::clampConstants shallowCurveTurnClamp;
    shallowCurveTurnClamp.minimumSpeed = 0;
    shallowCurveTurnClamp.maximumSpeed = 25;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 0;
    clampMogoDriveClamp.maximumSpeed = 40;

    Drivetrain::settleConstants clampMogoDriveSettle;
    clampMogoDriveSettle.deadband = 1.5;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 2;
    shortTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortTurnSettle.settleTime = 400;
    shortTurnSettle.timeout = 1250;

    Drivetrain::clampConstants steepCurveDriveClamp;
    steepCurveDriveClamp.minimumSpeed = 0;
    steepCurveDriveClamp.maximumSpeed = 50;

    Drivetrain::settleConstants ramDriveSettle;
    ramDriveSettle.deadband = 6;
    ramDriveSettle.loopCycleTime = 20;
    ramDriveSettle.settleTime = 400;
    ramDriveSettle.timeout = 1500;

    Drivetrain::clampConstants slowerDriveClamp;
    slowerDriveClamp.minimumSpeed = 0;
    slowerDriveClamp.maximumSpeed = 55;

    task scoreOnAlly = task(armToAllianceStake);
    chassis.driveToPoint(62, 9, shortDriveClamp, shortDriveSettle);
    chassis.stopDrive(coast);
    wait(250, msec);  //One ring scored on alliance stake

    scoreOnAlly.stop();
    task armDown = task(armToDown);
    chassis.driveToPoint(25, 24, clampMogoDriveClamp, clampMogoDriveSettle, chassis.defaultDriveOutputConstants,
                         shallowCurveTurnClamp, chassis.defaultHeadingSettleConstants, chassis.defaultHeadingOutputConstants); 
    MogoMech.set(true); //Mogo clamped

    wait(250, msec);
    armDown.stop();
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.turnToPoint(false, 6.5, 39, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveToPoint(6.5, 39); //One ring scored on mogo

    chassis.swingToHeading("Left", 0, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.stopDrive(coast);
    chassis.driveDistance(10, 0, shortDriveClamp, shortDriveSettle); //Two rings scored on mogo

    chassis.driveToPoint(10, 32, steepCurveDriveClamp, shortDriveSettle);
    chassis.turnToPoint(false, 20, 45, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveToPoint(22, 45, shortDriveClamp, shortDriveSettle); //Three rings scored on mogo

    chassis.driveToPoint(55, 55, steepCurveDriveClamp);
    Intake.spin(reverse, INTAKE_DEFAULT_SPEED, percent);
    chassis.turnToPoint(false, 68, 70);
    chassis.driveToPoint(68, 70, chassis.defaultDriveClampConstants, ramDriveSettle);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    wait(500, msec);
    chassis.driveDistance(-20, chassis.odom.orientation, chassis.defaultDriveClampConstants, shortDriveSettle);
    chassis.driveDistance(15, chassis.odom.orientation, shortDriveClamp, shortDriveSettle); //Five rings scored on mogo

    chassis.driveToPoint(28, 28, slowerDriveClamp);
    chassis.turnToPoint(false, 0, 0);
    chassis.stopDrive(coast);
    ArmMotor.spin(forward, 95, percent);
    wait(750, msec);
    ArmMotor.stop(brake); //Ladder touched

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
    chassis.defaultDriveSettleConstants.timeout = 5000;

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
    clampMogoDriveSettle.deadband = 1.5;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::outputConstants clampMogoDriveOutput;
    clampMogoDriveOutput.kp = 4;
    clampMogoDriveOutput.ki = 0;
    clampMogoDriveOutput.kd = 0.1;
    clampMogoDriveOutput.startI = 0;
}