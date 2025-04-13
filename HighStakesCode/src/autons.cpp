#include "vex.h"
#include "autons.h"
#include <iostream>

void setDefaultPIDConstants(){
    chassis.defaultDriveOutputConstants.kp = 9;
    chassis.defaultDriveOutputConstants.ki = 0;
    chassis.defaultDriveOutputConstants.kd = 0.65;
    chassis.defaultDriveOutputConstants.startI = 0;
    chassis.defaultDriveClampConstants.minimumSpeed = 0;
    chassis.defaultDriveClampConstants.maximumSpeed = 90;
    chassis.defaultDriveSettleConstants.deadband = 1.5;
    chassis.defaultDriveSettleConstants.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    chassis.defaultDriveSettleConstants.settleTime = 750;
    chassis.defaultDriveSettleConstants.timeout = 3000;

    chassis.defaultHeadingOutputConstants.kp = 5;
    chassis.defaultHeadingOutputConstants.ki = 0;
    chassis.defaultHeadingOutputConstants.kd = 0.5;
    chassis.defaultHeadingOutputConstants.startI = 0;
    chassis.defaultHeadingClampConstants.minimumSpeed = 0;
    chassis.defaultHeadingClampConstants.maximumSpeed = 100;
    chassis.defaultHeadingSettleConstants.deadband = 5;

    chassis.defaultTurnOutputConstants.kp = 3;
    chassis.defaultTurnOutputConstants.ki = 30;
    chassis.defaultTurnOutputConstants.kd = 0.225;
    chassis.defaultTurnOutputConstants.startI = 10;
    chassis.defaultTurnClampConstants.minimumSpeed = 0;
    chassis.defaultTurnClampConstants.maximumSpeed = 100;
    chassis.defaultTurnSettleConstants.deadband = 2;
    chassis.defaultTurnSettleConstants.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    chassis.defaultTurnSettleConstants.settleTime = 700;
    chassis.defaultTurnSettleConstants.timeout = 1000;

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


static double intakeVelocity = 0;
int controlIntake(){
    int hue = 0;
    int hueRange = 0;
    int previousVelocity = 0;

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
        if (intakeVelocity != previousVelocity){
            Intake.spin(forward, intakeVelocity, percent);

            previousVelocity = intakeVelocity;
        }

        if (fabs(headingError(hue, IntakeOptical.hue())) < hueRange){ //Color sort
            while (IntakeDistance.objectDistance(inches) > SORT_DETECT_RING_RANGE){
                wait(DEFAULT_LOOP_CYCLE_TIME / 4, msec);
            }

            wait(SORT_DEFAULT_DETECT_TO_SORT_DELAY /  (Intake.velocity(percent) / INTAKE_DEFAULT_SPEED), msec);
            Intake.stop(brake);
            wait(SORT_SORT_TO_CONTINUE_DELAY, msec);
        }

        wait(DEFAULT_LOOP_CYCLE_TIME / 2, msec);
    }

    return 0;
}

static double armPosition = 0;
int controlArm(){
    double PIDOutput = 0;
    double FFOutput = 0;
    double output = 0;

    PID armPID(armPosition, ARM_MACRO_KP, 0, 0, 0, 0, DEFAULT_LOOP_CYCLE_TIME, 0, 0);

    while (true){
        PIDOutput = armPID.output(armPosition - ArmRotation.position(degrees));
        FFOutput = ARM_MACRO_KCOS * cos(degToRad(ArmRotation.position(degrees)));

        output = PIDOutput + FFOutput;

        if (armPosition > ARM_LOADING_POSITION){
            FirstIntake.stop(brake);
            SecondIntake.spin(reverse, ARM_INTAKE_SPEED, percent);
        }

        Arm.spin(forward, percentToVolts(output), volt);

        wait(armPID.loopCycleTime, msec);
    }
}


void runOdomTest(){
    chassis.setCoordinates(-60, 14, 220);

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

    chassis.driveToPoint(24, 60);
    chassis.driveToPoint(0, 0);
    // chassis.driveDistance(12, 0);
    // chassis.driveDistance(24, 0);
    // chassis.driveDistance(-24, 0);
    // chassis.driveDistance(-12, 0);
    // chassis.driveToPoint(16, 8);
    // chassis.driveToPoint(0, 16);

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


void runAutonRedSoloAWP(){
//     chassis.setCoordinates(-60, 12, 225);
//     setDefaultPIDConstants();

//     Drivetrain::clampConstants shortDriveClamp;
//     shortDriveClamp.minimumSpeed = 0;
//     shortDriveClamp.maximumSpeed = 70;

//     Drivetrain::settleConstants shortDriveSettle;
//     shortDriveSettle.deadband = 1.5;
//     shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     shortDriveSettle.settleTime = 500;
//     shortDriveSettle.timeout = 1200;

//     Drivetrain::clampConstants shallowCurveTurnClamp;
//     shallowCurveTurnClamp.minimumSpeed = 0;
//     shallowCurveTurnClamp.maximumSpeed = 25;

//     Drivetrain::clampConstants clampMogoDriveClamp;
//     clampMogoDriveClamp.minimumSpeed = 0;
//     clampMogoDriveClamp.maximumSpeed = 40;

//     Drivetrain::settleConstants clampMogoDriveSettle;
//     clampMogoDriveSettle.deadband = 1.5;
//     clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     clampMogoDriveSettle.settleTime = 1000;
//     clampMogoDriveSettle.timeout = 1500;

//     Drivetrain::settleConstants shortTurnSettle;
//     shortTurnSettle.deadband = 2;
//     shortTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     shortTurnSettle.settleTime = 400;
//     shortTurnSettle.timeout = 1250;

//     Drivetrain::clampConstants steepCurveDriveClamp;
//     steepCurveDriveClamp.minimumSpeed = 0;
//     steepCurveDriveClamp.maximumSpeed = 60;

//     Drivetrain::settleConstants ramDriveSettle;
//     ramDriveSettle.deadband = 6;
//     ramDriveSettle.loopCycleTime = 20;
//     ramDriveSettle.settleTime = 400;
//     ramDriveSettle.timeout = 1500;

//     Drivetrain::clampConstants slowerDriveClamp;
//     slowerDriveClamp.minimumSpeed = 0;
//     slowerDriveClamp.maximumSpeed = 55;

//     task scoreOnAlly = task(armToAllianceStake);
//     chassis.driveToPoint(-62, 9, shortDriveClamp, shortDriveSettle);
//     chassis.stopDrive(coast);
//     wait(250, msec);  //One ring scored on alliance stake

//     scoreOnAlly.stop();
//     task armDown = task(armToDown);
//     chassis.driveToPoint(-25, 24, clampMogoDriveClamp, clampMogoDriveSettle, chassis.defaultDriveOutputConstants,
//                          shallowCurveTurnClamp, chassis.defaultHeadingSettleConstants, chassis.defaultHeadingOutputConstants); 
//     MogoMech.set(true); 
//     wait(250, msec); //Mobile goal clamped

//     armDown.stop();
//     Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
//     chassis.turnToPoint(false, -6.5, 39, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.driveToPoint(-6.5, 39); //One ring scored on mogo

//     chassis.swingToHeading("Right", 0, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.stopDrive(coast);
//     chassis.driveDistance(8, 0, shortDriveClamp, shortDriveSettle); //Two rings scored on mogo

//     chassis.driveToPoint(-10, 32, steepCurveDriveClamp, shortDriveSettle);
//     chassis.turnToPoint(false, -20, 45, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.driveToPoint(-22, 45, shortDriveClamp, shortDriveSettle); //Three rings scored on mogo

//     chassis.driveToPoint(-50, 36, shortDriveClamp);
//     chassis.turnToPoint(false, -48, 0, chassis.defaultTurnClampConstants, shortTurnSettle);
//     Intake.spin(forward, INTAKE_DEFAULT_SPEED * 0.6, percent);
//     chassis.driveToPoint(-48, 10, chassis.defaultDriveClampConstants, shortDriveSettle);
//     MogoMech.set(false);
//     chassis.driveToPoint(-48, -20, shortDriveClamp, shortDriveSettle); //Ring intaked

//     Intake.stop(brake);
//     chassis.turnToPoint(true, -27, -24, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.driveToPoint(-28, -24, clampMogoDriveClamp, clampMogoDriveSettle);
//     MogoMech.set(true); 
//     wait(250, msec); //Mobile goal clamped

//     Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent); //One ring scored on mogo

//     chassis.turnToPoint(false, -24, -48, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.driveToPoint(-23, -46, chassis.defaultDriveClampConstants, shortDriveSettle); //Two rings scored on mogo

//     chassis.driveToPoint(-16, -16, steepCurveDriveClamp); //Ladder touched

//     chassis.stopDrive(coast);
}

void runAutonRedRushAWP(){
//     chassis.setCoordinates(-57.5, -17, 0);
//     setDefaultPIDConstants();

//     Drivetrain::clampConstants shortDriveClamp;
//     shortDriveClamp.minimumSpeed = 0;
//     shortDriveClamp.maximumSpeed = 70;

//     Drivetrain::clampConstants clampMogoDriveClamp;
//     clampMogoDriveClamp.minimumSpeed = 0;
//     clampMogoDriveClamp.maximumSpeed = 50;

//     Drivetrain::clampConstants longDriveDriveClamp;
//     longDriveDriveClamp.minimumSpeed = 0;
//     longDriveDriveClamp.maximumSpeed = 70;

//     Drivetrain::settleConstants shortDriveSettle;
//     shortDriveSettle.deadband = 2;
//     shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     shortDriveSettle.settleTime = 400;
//     shortDriveSettle.timeout = 1250;

//     Drivetrain::settleConstants shortTurnSettle;
//     shortTurnSettle.deadband = 2;
//     shortTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     shortTurnSettle.settleTime = 500;
//     shortTurnSettle.timeout = 1250;

//     Drivetrain::settleConstants allyStakeTurnSettle;
//     allyStakeTurnSettle.deadband = 2;
//     allyStakeTurnSettle.loopCycleTime = 20;
//     allyStakeTurnSettle.settleTime = 1500;
//     allyStakeTurnSettle.timeout = 2000;

//     Drivetrain::settleConstants noMogoTurnSettle;
//     noMogoTurnSettle.deadband = 2;
//     noMogoTurnSettle.loopCycleTime = 20;
//     noMogoTurnSettle.settleTime = 750;
//     noMogoTurnSettle.timeout = 2000;

//     Drivetrain::settleConstants clampMogoDriveSettle;
//     clampMogoDriveSettle.deadband = 1.5;
//     clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     clampMogoDriveSettle.settleTime = 1000;
//     clampMogoDriveSettle.timeout = 1500;

//     Drivetrain::outputConstants noMogoTurnOutput;
//     noMogoTurnOutput.kp = 2;
//     noMogoTurnOutput.ki = 30;
//     noMogoTurnOutput.kd = 0.125;
//     noMogoTurnOutput.startI = 10;
}

void runAutonRedStackAWP(){
    chassis.setCoordinates(-60, 14, 220);
    setDefaultPIDConstants();

    Drivetrain::clampConstants shortDriveClamp;
    shortDriveClamp.minimumSpeed = 0;
    shortDriveClamp.maximumSpeed = 70;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1.5;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 500;
    shortDriveSettle.timeout = 1200;

    Drivetrain::clampConstants shallowCurveTurnClamp;
    shallowCurveTurnClamp.minimumSpeed = 0;
    shallowCurveTurnClamp.maximumSpeed = 40;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 0;
    clampMogoDriveClamp.maximumSpeed = 60;

    Drivetrain::settleConstants clampMogoDriveSettle;
    clampMogoDriveSettle.deadband = 1.5;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 500;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::settleConstants shortTurnSettle;
    shortTurnSettle.deadband = 2;
    shortTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortTurnSettle.settleTime = 400;
    shortTurnSettle.timeout = 1250;

    Drivetrain::settleConstants ramDriveSettle;
    ramDriveSettle.deadband = 6;
    ramDriveSettle.loopCycleTime = 20;
    ramDriveSettle.settleTime = 400;
    ramDriveSettle.timeout = 1500;

    Drivetrain::clampConstants slowerDriveClamp;
    slowerDriveClamp.minimumSpeed = 0;
    slowerDriveClamp.maximumSpeed = 60;

    armPosition = ARM_ALLIANCE_STAKE_POSITION;
    chassis.driveToPoint(-62, 11, shortDriveClamp, shortDriveSettle);
    chassis.stopDrive(coast);
    wait(250, msec);  //One ring scored on alliance stake

    armPosition = 0;
    chassis.driveToPoint(-20, 24, clampMogoDriveClamp, clampMogoDriveSettle, chassis.defaultDriveOutputConstants,
                         shallowCurveTurnClamp, chassis.defaultHeadingSettleConstants, chassis.defaultHeadingOutputConstants); 
    MogoMech.set(true); //Mobile goal clamped

    wait(250, msec);
    intakeVelocity = INTAKE_DEFAULT_SPEED;
    chassis.turnToPoint(false, -6.5, 39, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveToPoint(-6.5, 39); //One ring scored on mogo

    chassis.swingToHeading("Right", 0, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.stopDrive(coast);
    chassis.driveDistance(8, 0, shortDriveClamp, shortDriveSettle); //Two rings scored on mogo

    chassis.driveToPoint(-14, 34, shortDriveClamp, shortDriveSettle);
    chassis.turnToPoint(false, -20, 48, chassis.defaultTurnClampConstants, shortTurnSettle);
    chassis.driveToPoint(-22, 48); //Three rings scored on mogo
    std::cout << chassis.odom.xPosition << " " << chassis.odom.yPosition << std::endl;

    chassis.driveToPoint(-55, 55);
    Intake.spin(reverse, INTAKE_DEFAULT_SPEED, percent);
    chassis.turnToPoint(false, -68, 70);
    chassis.driveToPoint(-68, 70, chassis.defaultDriveClampConstants, ramDriveSettle);
    Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    wait(500, msec);
    chassis.driveDistance(-20, chassis.odom.orientation, chassis.defaultDriveClampConstants, shortDriveSettle);
    chassis.driveDistance(16, chassis.odom.orientation, slowerDriveClamp, shortDriveSettle); //Five rings scored on mogo

    // chassis.driveToPoint(-48, -24, slowerDriveClamp);

    // chassis.stopDrive(coast);
}

void runAutonRedGoalRush(){
//     chassis.setCoordinates(0, 0, 0);
//     setDefaultPIDConstants();

    chassis.setCoordinates(0, 0, 0);
    task runIntake = task(controlIntake);

    intakeVelocity = 95;
    wait(5, sec);
    intakeVelocity = 80;
    wait(5, sec);
    intakeVelocity = 50;
}

void runAutonBlueSoloAWP(){
//     chassis.setCoordinates(60, 12, 135);
//     setDefaultPIDConstants();

//     Drivetrain::clampConstants shortDriveClamp;
//     shortDriveClamp.minimumSpeed = 0;
//     shortDriveClamp.maximumSpeed = 70;

//     Drivetrain::settleConstants shortDriveSettle;
//     shortDriveSettle.deadband = 1.5;
//     shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     shortDriveSettle.settleTime = 500;
//     shortDriveSettle.timeout = 1200;

//     Drivetrain::clampConstants shallowCurveTurnClamp;
//     shallowCurveTurnClamp.minimumSpeed = 0;
//     shallowCurveTurnClamp.maximumSpeed = 25;

//     Drivetrain::clampConstants clampMogoDriveClamp;
//     clampMogoDriveClamp.minimumSpeed = 0;
//     clampMogoDriveClamp.maximumSpeed = 40;

//     Drivetrain::settleConstants clampMogoDriveSettle;
//     clampMogoDriveSettle.deadband = 1.5;
//     clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     clampMogoDriveSettle.settleTime = 1000;
//     clampMogoDriveSettle.timeout = 1500;

//     Drivetrain::settleConstants shortTurnSettle;
//     shortTurnSettle.deadband = 2;
//     shortTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     shortTurnSettle.settleTime = 400;
//     shortTurnSettle.timeout = 1250;

//     Drivetrain::clampConstants steepCurveDriveClamp;
//     steepCurveDriveClamp.minimumSpeed = 0;
//     steepCurveDriveClamp.maximumSpeed = 60;

//     Drivetrain::settleConstants ramDriveSettle;
//     ramDriveSettle.deadband = 6;
//     ramDriveSettle.loopCycleTime = 20;
//     ramDriveSettle.settleTime = 400;
//     ramDriveSettle.timeout = 1500;

//     Drivetrain::clampConstants slowerDriveClamp;
//     slowerDriveClamp.minimumSpeed = 0;
//     slowerDriveClamp.maximumSpeed = 55;

//     task scoreOnAlly = task(armToAllianceStake);
//     chassis.driveToPoint(62, 9, shortDriveClamp, shortDriveSettle);
//     chassis.stopDrive(coast);
//     wait(250, msec);  //One ring scored on alliance stake

//     scoreOnAlly.stop();
//     task armDown = task(armToDown);
//     chassis.driveToPoint(25, 24, clampMogoDriveClamp, clampMogoDriveSettle, chassis.defaultDriveOutputConstants,
//                          shallowCurveTurnClamp, chassis.defaultHeadingSettleConstants, chassis.defaultHeadingOutputConstants); 
//     MogoMech.set(true); 
//     wait(250, msec); //Mobile goal clamped

//     armDown.stop();
//     Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
//     chassis.turnToPoint(false, 6.5, 39, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.driveToPoint(6.5, 39); //One ring scored on mogo

//     chassis.swingToHeading("Left", 0, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.stopDrive(coast);
//     chassis.driveDistance(8, 0, shortDriveClamp, shortDriveSettle); //Two rings scored on mogo

//     chassis.driveToPoint(10, 32, steepCurveDriveClamp, shortDriveSettle);
//     chassis.turnToPoint(false, 20, 45, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.driveToPoint(22, 45, shortDriveClamp, shortDriveSettle); //Three rings scored on mogo

//     chassis.driveToPoint(50, 36, shortDriveClamp);
//     chassis.turnToPoint(false, 48, 0, chassis.defaultTurnClampConstants, shortTurnSettle);
//     Intake.spin(forward, INTAKE_DEFAULT_SPEED * 0.6, percent);
//     chassis.driveToPoint(48, 10, chassis.defaultDriveClampConstants, shortDriveSettle);
//     MogoMech.set(false);
//     chassis.driveToPoint(48, -20, shortDriveClamp, shortDriveSettle); //Ring intaked

//     Intake.stop(brake);
//     chassis.turnToPoint(true, 27, -24, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.driveToPoint(28, -24, clampMogoDriveClamp, clampMogoDriveSettle);
//     MogoMech.set(true); 
//     wait(250, msec); //Mobile goal clamped

//     Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent); //One ring scored on mogo

//     chassis.turnToPoint(false, 24, -48, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.driveToPoint(23, -46, chassis.defaultDriveClampConstants, shortDriveSettle); //Two rings scored on mogo

//     chassis.driveToPoint(16, -16, steepCurveDriveClamp); //Ladder touched

//     chassis.stopDrive(coast);
}

void runAutonBlueRushAWP(){
//     chassis.setCoordinates(57.5, -17, 0);
//     setDefaultPIDConstants();

//     Drivetrain::clampConstants shortDriveClamp;
//     shortDriveClamp.minimumSpeed = 0;
//     shortDriveClamp.maximumSpeed = 70;

//     Drivetrain::clampConstants clampMogoDriveClamp;
//     clampMogoDriveClamp.minimumSpeed = 0;
//     clampMogoDriveClamp.maximumSpeed = 50;

//     Drivetrain::clampConstants longDriveDriveClamp;
//     longDriveDriveClamp.minimumSpeed = 0;
//     longDriveDriveClamp.maximumSpeed = 70;

//     Drivetrain::settleConstants shortDriveSettle;
//     shortDriveSettle.deadband = 1;
//     shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     shortDriveSettle.settleTime = 400;
//     shortDriveSettle.timeout = 1250;

//     Drivetrain::settleConstants shortTurnSettle;
//     shortTurnSettle.deadband = 2;
//     shortTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     shortTurnSettle.settleTime = 500;
//     shortTurnSettle.timeout = 1250;

//     Drivetrain::settleConstants allyStakeTurnSettle;
//     allyStakeTurnSettle.deadband = 2;
//     allyStakeTurnSettle.loopCycleTime = 20;
//     allyStakeTurnSettle.settleTime = 1500;
//     allyStakeTurnSettle.timeout = 2000;

//     Drivetrain::settleConstants noMogoTurnSettle;
//     noMogoTurnSettle.deadband = 2;
//     noMogoTurnSettle.loopCycleTime = 20;
//     noMogoTurnSettle.settleTime = 750;
//     noMogoTurnSettle.timeout = 2000;

//     Drivetrain::settleConstants clampMogoDriveSettle;
//     clampMogoDriveSettle.deadband = 1.5;
//     clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     clampMogoDriveSettle.settleTime = 1000;
//     clampMogoDriveSettle.timeout = 1500;

//     Drivetrain::outputConstants noMogoTurnOutput;
//     noMogoTurnOutput.kp = 2;
//     noMogoTurnOutput.ki = 30;
//     noMogoTurnOutput.kd = 0.125;
//     noMogoTurnOutput.startI = 10;
}

void runAutonBlueStackAWP(){
//     chassis.setCoordinates(60, 12, 135);
//     setDefaultPIDConstants();

//     Drivetrain::clampConstants shortDriveClamp;
//     shortDriveClamp.minimumSpeed = 0;
//     shortDriveClamp.maximumSpeed = 70;

//     Drivetrain::settleConstants shortDriveSettle;
//     shortDriveSettle.deadband = 1.5;
//     shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     shortDriveSettle.settleTime = 500;
//     shortDriveSettle.timeout = 1200;

//     Drivetrain::clampConstants shallowCurveTurnClamp;
//     shallowCurveTurnClamp.minimumSpeed = 0;
//     shallowCurveTurnClamp.maximumSpeed = 25;

//     Drivetrain::clampConstants clampMogoDriveClamp;
//     clampMogoDriveClamp.minimumSpeed = 0;
//     clampMogoDriveClamp.maximumSpeed = 40;

//     Drivetrain::settleConstants clampMogoDriveSettle;
//     clampMogoDriveSettle.deadband = 1.5;
//     clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     clampMogoDriveSettle.settleTime = 1000;
//     clampMogoDriveSettle.timeout = 1500;

//     Drivetrain::settleConstants shortTurnSettle;
//     shortTurnSettle.deadband = 2;
//     shortTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     shortTurnSettle.settleTime = 400;
//     shortTurnSettle.timeout = 1250;

//     Drivetrain::clampConstants steepCurveDriveClamp;
//     steepCurveDriveClamp.minimumSpeed = 0;
//     steepCurveDriveClamp.maximumSpeed = 60;

//     Drivetrain::settleConstants ramDriveSettle;
//     ramDriveSettle.deadband = 6;
//     ramDriveSettle.loopCycleTime = 20;
//     ramDriveSettle.settleTime = 400;
//     ramDriveSettle.timeout = 1500;

//     Drivetrain::clampConstants slowerDriveClamp;
//     slowerDriveClamp.minimumSpeed = 0;
//     slowerDriveClamp.maximumSpeed = 55;

//     task scoreOnAlly = task(armToAllianceStake);
//     chassis.driveToPoint(62, 9, shortDriveClamp, shortDriveSettle);
//     chassis.stopDrive(coast);
//     wait(250, msec);  //One ring scored on alliance stake

//     scoreOnAlly.stop();
//     task armDown = task(armToDown);
//     chassis.driveToPoint(25, 24, clampMogoDriveClamp, clampMogoDriveSettle, chassis.defaultDriveOutputConstants,
//                          shallowCurveTurnClamp, chassis.defaultHeadingSettleConstants, chassis.defaultHeadingOutputConstants); 
//     MogoMech.set(true); //Mobile goal clamped

//     wait(250, msec);
//     armDown.stop();
//     Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
//     chassis.turnToPoint(false, 6.5, 39, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.driveToPoint(6.5, 39); //One ring scored on mogo

//     chassis.swingToHeading("Left", 0, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.stopDrive(coast);
//     chassis.driveDistance(8, 0, shortDriveClamp, shortDriveSettle); //Two rings scored on mogo

//     chassis.driveToPoint(10, 32, steepCurveDriveClamp, shortDriveSettle);
//     chassis.turnToPoint(false, 20, 45, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.driveToPoint(24, 48); //Three rings scored on mogo

//     chassis.driveToPoint(55, 55, steepCurveDriveClamp);
//     Intake.spin(reverse, INTAKE_DEFAULT_SPEED, percent);
//     chassis.turnToPoint(false, 68, 70);
//     chassis.driveToPoint(68, 70, chassis.defaultDriveClampConstants, ramDriveSettle);
//     Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
//     wait(500, msec);
//     chassis.driveDistance(-20, chassis.odom.orientation, chassis.defaultDriveClampConstants, shortDriveSettle);
//     chassis.driveDistance(16, chassis.odom.orientation, shortDriveClamp, shortDriveSettle); //Five rings scored on mogo

//     chassis.driveToPoint(48, -24, slowerDriveClamp);

//     chassis.stopDrive(coast);
}

void runAutonBlueGoalRush(){
//     chassis.setCoordinates(0, 0, 0);
//     setDefaultPIDConstants();
    
}

// /******************** Prog Skills ********************/

void runProgSkills(){

//     chassis.setCoordinates(-57, 0, 270);
//     setDefaultPIDConstants();

//     chassis.defaultDriveClampConstants.maximumSpeed = 65;

//     chassis.defaultDriveSettleConstants.timeout = 5000;

//     Drivetrain::clampConstants shortDriveClamp;
//     shortDriveClamp.minimumSpeed = 0;
//     shortDriveClamp.maximumSpeed = 70;

//     Drivetrain::settleConstants shortDriveSettle;
//     shortDriveSettle.deadband = 1.5;
//     shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     shortDriveSettle.settleTime = 500;
//     shortDriveSettle.timeout = 1200;

//     Drivetrain::clampConstants clampMogoDriveClamp;
//     clampMogoDriveClamp.minimumSpeed = 0;
//     clampMogoDriveClamp.maximumSpeed = 40;

//     Drivetrain::settleConstants clampMogoDriveSettle;
//     clampMogoDriveSettle.deadband = 1.5;
//     clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     clampMogoDriveSettle.settleTime = 500;
//     clampMogoDriveSettle.timeout = 1000;

//     Drivetrain::settleConstants shortTurnSettle;
//     shortTurnSettle.deadband = 2;
//     shortTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     shortTurnSettle.settleTime = 400;
//     shortTurnSettle.timeout = 1250;

//     Drivetrain::clampConstants steepCurveDriveClamp;
//     steepCurveDriveClamp.minimumSpeed = 0;
//     steepCurveDriveClamp.maximumSpeed = 60;

//     Drivetrain::clampConstants slowDriveClamp;
//     slowDriveClamp.minimumSpeed = 0;
//     slowDriveClamp.maximumSpeed = 45;

//     Drivetrain::settleConstants shortTimeoutDriveSettle;
//     shortTimeoutDriveSettle.deadband = 1.5;
//     shortTimeoutDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     shortTimeoutDriveSettle.settleTime = 1000;
//     shortTimeoutDriveSettle.timeout = 1500;

//     Drivetrain::settleConstants scoreWallDriveSettle;
//     scoreWallDriveSettle.deadband = 1.5;
//     scoreWallDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
//     scoreWallDriveSettle.settleTime = 2750;
//     scoreWallDriveSettle.timeout = 2750;

//     task scoreOnAlly = task(armToAllianceStake);
//     wait(1000, msec); //One ring scored on alliance stake

//     scoreOnAlly.stop();
//     task armDown = task(armToDown);
//     chassis.driveToPoint(-48, 0, shortDriveClamp, shortDriveSettle);
//     armDown.stop();
//     chassis.turnToPoint(true, -48, -24, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.driveToPoint(-48, -24, clampMogoDriveClamp, clampMogoDriveSettle);
//     MogoMech.set(true); 
//     wait(100, msec); //Mobile goal clamped

//     chassis.turnToPoint(false, -24, -24, chassis.defaultTurnClampConstants, shortTurnSettle);
//     Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
//     chassis.driveToPoint(-24, -24, shortDriveClamp); //One ring scored on mogo
//     chassis.turnToPoint(false, 0, -48, chassis.defaultTurnClampConstants, shortTurnSettle);
//     chassis.driveToPoint(26, -49);
//     chassis.stopDrive(coast);
//     wait(500, msec); //Two rings scored on mogo

//     chassis.driveToPoint(3, -40, steepCurveDriveClamp);
//     task loadForWallFirst = task(armToLoad);
//     chassis.turnToPoint(false, 0, -65);
//     chassis.driveToPoint(0, -65, slowDriveClamp, scoreWallDriveSettle);
//     loadForWallFirst.stop();
//     ArmMotor.spin(forward, 95, percent);
//     Intake.spin(reverse, ARM_INTAKE_SPEED, percent);
//     wait(1000, msec); //One ring scored on wall stake

//     chassis.driveDistance(-14, 0, shortDriveClamp, shortDriveSettle);
//     ArmMotor.stop(brake);
//     task returnArmFirst = task(armToDown);
//     chassis.turnToPoint(false, -24, -47);
//     Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
//     chassis.driveToPoint(-24, -47); //Three rings scored on mogo

//     returnArmFirst.stop();
//     chassis.turnToPoint(false, -60, -47);
//     chassis.driveToPoint(-65, -47, slowDriveClamp); //Five rings scored on mogo

//     chassis.turnToPoint(false, -48, -60);
//     chassis.driveToPoint(-48, -60, shortDriveClamp); //Six rings scored on mogo

//     chassis.turnToPoint(true, -66, -66);
//     chassis.driveToPoint(-54, -58, slowDriveClamp);
//     Intake.spin(reverse, INTAKE_DEFAULT_SPEED, percent);
//     wait(100, msec);
//     MogoMech.set(false); //Mogo deposited in corner

//     chassis.driveToPoint(-48, 0);
//     chassis.turnToPoint(true, -48, 24);
//     chassis.driveToPoint(-48, 24, clampMogoDriveClamp, clampMogoDriveSettle);
//     MogoMech.set(true);
//     wait(100, msec); //Mobile goal clamped

//     chassis.turnToPoint(false, -24, 24, chassis.defaultTurnClampConstants, shortTurnSettle);
//     Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
//     chassis.driveToPoint(-24, 24, shortDriveClamp); //One ring scored on mogo

//     chassis.turnToPoint(false, 0, 72);
//     chassis.driveToPoint(25, 49); 
//     chassis.stopDrive(coast);
//     wait(500, msec); //Two rings scored on mogo

//     chassis.driveToPoint(3, 40, steepCurveDriveClamp);
//     task loadForWallSecond = task(armToLoad);
//     chassis.turnToPoint(false, 0, 65);
//     chassis.driveToPoint(0, 65, slowDriveClamp, scoreWallDriveSettle);
//     loadForWallSecond.stop();
//     ArmMotor.spin(forward, 95, percent);
//     Intake.spin(reverse, ARM_INTAKE_SPEED, percent);
//     wait(1000, msec); //One ring scored on wall stake

//     chassis.driveDistance(-14, 180, shortDriveClamp, shortDriveSettle);
//     ArmMotor.stop(brake);
//     task returnArmSecond = task(armToDown);
//     chassis.turnToPoint(false, -24, 48);
//     Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
//     chassis.driveToPoint(-24, 48); //Three rings scored on mogo

//     returnArmSecond.stop();
//     chassis.turnToPoint(false, -60, 48);
//     chassis.driveToPoint(-64, 48, slowDriveClamp); //Five rings scored on mogo

//     chassis.turnToPoint(false, -48, 60);
//     chassis.driveToPoint(-48, 60, shortDriveClamp); //Six rings scored on mogo

//     chassis.turnToPoint(true, -66, 66);
//     chassis.driveToPoint(-54, 58, slowDriveClamp);
//     Intake.spin(reverse, INTAKE_DEFAULT_SPEED, percent);
//     wait(100, msec);
//     MogoMech.set(false); //Mogo deposited in corner

//     // Intake.stop(brake);
//     // chassis.driveToPoint(0, 48);
//     // chassis.turnToPoint(false, 48, 48);
//     // FirstIntake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
//     // chassis.driveToPoint(48, 48);
//     // chassis.turnToPoint(true, 48, 0);
//     // chassis.driveToPoint(48, 2, clampMogoDriveClamp, clampMogoDriveSettle); 
//     // task loadForFarAlly = task(armToLoad);
//     // Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
//     // chassis.turnToPoint(false, 72, 0);
//     // // chassis.driveToPoint(72, , slowDriveClamp);
//     // // chassis.odom.xPosition = 67;
}