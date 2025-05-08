#include "vex.h"
#include "autons.h"
#include <iostream>

void setDefaultPIDConstants(){
    chassis.defaultDriveOutputConstants.kp = 12;
    chassis.defaultDriveOutputConstants.ki = 0;
    chassis.defaultDriveOutputConstants.kd = 0.65;
    chassis.defaultDriveOutputConstants.startI = 0;
    chassis.defaultDriveClampConstants.minimumSpeed = 0;
    chassis.defaultDriveClampConstants.maximumSpeed = 90;
    chassis.defaultDriveSettleConstants.deadband = 1.5;
    chassis.defaultDriveSettleConstants.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    chassis.defaultDriveSettleConstants.settleTime = 550;
    chassis.defaultDriveSettleConstants.timeout = 3000;

    chassis.defaultHeadingOutputConstants.kp = 5;
    chassis.defaultHeadingOutputConstants.ki = 0;
    chassis.defaultHeadingOutputConstants.kd = 0.5;
    chassis.defaultHeadingOutputConstants.startI = 0;
    chassis.defaultHeadingClampConstants.minimumSpeed = 0;
    chassis.defaultHeadingClampConstants.maximumSpeed = 100;
    chassis.defaultHeadingSettleConstants.deadband = 4;

    chassis.defaultTurnOutputConstants.kp = 3;
    chassis.defaultTurnOutputConstants.ki = 20;
    chassis.defaultTurnOutputConstants.kd = 0.25;
    chassis.defaultTurnOutputConstants.startI = 10;
    chassis.defaultTurnClampConstants.minimumSpeed = 0;
    chassis.defaultTurnClampConstants.maximumSpeed = 100;
    chassis.defaultTurnSettleConstants.deadband = 2;
    chassis.defaultTurnSettleConstants.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    chassis.defaultTurnSettleConstants.settleTime = 400;
    chassis.defaultTurnSettleConstants.timeout = 1000;

    chassis.defaultSwingOutputConstants.kp = 6;
    chassis.defaultSwingOutputConstants.ki = 10;
    chassis.defaultSwingOutputConstants.kd = 0.4;
    chassis.defaultSwingOutputConstants.startI = 10;
    chassis.defaultSwingClampConstants.minimumSpeed = 0;
    chassis.defaultSwingClampConstants.maximumSpeed = 80;
    chassis.defaultSwingSettleConstants.deadband = 3;
    chassis.defaultSwingSettleConstants.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    chassis.defaultSwingSettleConstants.settleTime = 400;
    chassis.defaultSwingSettleConstants.timeout = 1000;
}


static double intakeVelocity = 0;
int controlIntake(){
    int previousVelocity = 0;
    int sortRGBValue = 0;
    int oppositeRGBValue = 0;

    IntakeOptical.integrationTime(5);
    IntakeOptical.setLightPower(70, percent);

    while (true){
        if (matchStatus != "Auton"){
            break;
        }

        if (intakeVelocity != previousVelocity){
            Intake.spin(forward, intakeVelocity, percent);

            previousVelocity = intakeVelocity;
        }

        if (allianceColor == "Red"){
            sortRGBValue = IntakeOptical.getRgb().blue;
            oppositeRGBValue = IntakeOptical.getRgb().red;
        }
        else {
            sortRGBValue = IntakeOptical.getRgb().red;
            oppositeRGBValue = IntakeOptical.getRgb().blue;
        }

        if (IntakeOptical.isNearObject()){
            std::cout << IntakeOptical.getRgb().red << " "  << IntakeOptical.getRgb().green << " " << IntakeOptical.getRgb().blue << std::endl;
        }
        if (sortRGBValue > oppositeRGBValue && IntakeOptical.isNearObject() && intakeVelocity != 0){ //Color sort
            std::cout << "SORTED" << IntakeOptical.getRgb().red << " "  << IntakeOptical.getRgb().green << " " << IntakeOptical.getRgb().blue << std::endl;

            while (IntakeDistance.objectDistance(inches) > SORT_DETECT_RING_RANGE){
                wait(5, msec);
            }

            wait(SORT_DEFAULT_DETECT_TO_SORT_DELAY /  (intakeVelocity / INTAKE_DEFAULT_SPEED), msec);
            SecondIntake.stop(brake);
            std::cout << "THROWN" << std::endl;
            wait(SORT_SORT_TO_CONTINUE_DELAY, msec);
            SecondIntake.spin(forward, intakeVelocity, percent);
        }

        wait(5, msec);
    }

    return 0;
}

static double armPosition = 0;
static bool armAntiJam = true;
int controlArm(){
    double PIDOutput = 0;
    double FFOutput = 0;
    double output = 0;
    int previousPosition = 0;

    PID armPID(armPosition, ARM_MACRO_KP, 0, 0, 0, 0, DEFAULT_LOOP_CYCLE_TIME, 0, 0);

    while (true){
        if (matchStatus != "Auton"){
            break;
        }

        PIDOutput = armPID.output(armPosition - ArmRotation.position(degrees));
        FFOutput = ARM_MACRO_KCOS * cos(degToRad(ArmRotation.position(degrees)));

        output = PIDOutput + FFOutput;

        if (armAntiJam && armPosition > ARM_LOADING_POSITION){
            intakeVelocity = -ARM_INTAKE_SPEED;
            FirstIntake.stop(brake);
        }

        Arm.spin(forward, percentToVolts(output), volt);

        previousPosition = armPosition;

        wait(armPID.loopCycleTime, msec);
    }

    return 0;
}


void runOdomTest(){
    chassis.setCoordinates(-52, 61.5, 93);

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
    chassis.driveDistance(12, 0);
    chassis.driveDistance(24, 0);
    chassis.driveDistance(-24, 0);
    chassis.driveDistance(-12, 0);
    chassis.driveToPoint(16, 8);
    chassis.driveToPoint(0, 16);

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


void runAutonRedSoloAWP(){
    chassis.setCoordinates(-59, 13, 220);
    setDefaultPIDConstants();

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1.5;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 400;
    shortDriveSettle.timeout = 1200;

    Drivetrain::clampConstants shallowCurveTurnClamp;
    shallowCurveTurnClamp.minimumSpeed = 0;
    shallowCurveTurnClamp.maximumSpeed = 45;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 25;
    clampMogoDriveClamp.maximumSpeed = 75;

    Drivetrain::settleConstants clampMogoDriveSettle;
    clampMogoDriveSettle.deadband = 2;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::outputConstants clampMogoDriveOutput;
    clampMogoDriveOutput.kp = 3;
    clampMogoDriveOutput.ki = 0;
    clampMogoDriveOutput.kd = 0.05;
    clampMogoDriveOutput.startI = 0;
    
    Drivetrain::clampConstants slowerDriveClamp;
    slowerDriveClamp.minimumSpeed = 0;
    slowerDriveClamp.maximumSpeed = 65;

    Drivetrain::clampConstants liftDriveClamp;
    liftDriveClamp.minimumSpeed = 0;
    liftDriveClamp.maximumSpeed = 45;

    Drivetrain::clampConstants ramDriveClamp;
    ramDriveClamp.minimumSpeed = 50;
    ramDriveClamp.maximumSpeed = 95;

    Drivetrain::settleConstants ramDriveSettle;
    ramDriveSettle.deadband = 1.5;
    ramDriveSettle.loopCycleTime = 20;
    ramDriveSettle.settleTime = 600;
    ramDriveSettle.timeout = 600;

    armPosition = ARM_ALLIANCE_STAKE_POSITION;
    chassis.driveToPoint(-61, 10, slowerDriveClamp, shortDriveSettle);
    chassis.stopDrive(coast);
    wait(300, msec);  //One ring scored on alliance stake

    armPosition = 0;
    chassis.driveToPoint(-26, 24, clampMogoDriveClamp, clampMogoDriveSettle, clampMogoDriveOutput,
                         shallowCurveTurnClamp, chassis.defaultHeadingSettleConstants, chassis.defaultHeadingOutputConstants); 
    MogoMech.set(true); //Mobile goal clamped

    intakeVelocity = INTAKE_DEFAULT_SPEED;
    chassis.turnToPoint(false, -9, 39);
    chassis.driveToPoint(-9, 39); //One ring scored on mogo

    chassis.stopDrive(brake);
    wait(200, msec);
    chassis.stopDrive(coast);
    chassis.swingToHeading("Right", 0, chassis.defaultTurnClampConstants, chassis.defaultTurnSettleConstants);
    chassis.stopDrive(coast);
    chassis.driveDistance(11, 0, chassis.defaultDriveClampConstants, shortDriveSettle); //Two rings scored on mogo

    chassis.driveToPoint(-16, 32);
    chassis.turnToPoint(false, -24, 48);
    chassis.driveToPoint(-24, 48, chassis.defaultDriveClampConstants, shortDriveSettle); //Three rings scored on mogo

    chassis.driveToPoint(-48, 40, slowerDriveClamp);
    armPosition = 0;
    chassis.turnToPoint(false, -48, 0);
    IntakeLift.set(true);
    chassis.driveToPoint(-48, 4, slowerDriveClamp, shortDriveSettle);
    IntakeLift.set(false);
    chassis.driveDistance(-10); //Four rings scored on mogo

    intakeVelocity = 0;
    MogoMech.set(false);
    chassis.driveDistance(10);
    chassis.turnToPoint(true, -24, -24);
    chassis.driveToPoint(-25, -23, clampMogoDriveClamp, clampMogoDriveSettle, clampMogoDriveOutput);
    MogoMech.set(true); //Mobile goal clamped

    intakeVelocity = INTAKE_DEFAULT_SPEED;
    chassis.turnToPoint(false, -24, -50);
    chassis.driveToPoint(-24, -50); //One ring scored on mobile goal

    armAntiJam = false;
    armPosition = 70;
    chassis.driveToPoint(-12, -12, slowerDriveClamp); //Ladder Touched

    chassis.stopDrive(coast);
}

void runAutonRedRushAWP(){
    intakeVelocity = 95;
}

void runAutonRedStackAWP(){
    chassis.setCoordinates(-59, 13, 220);
    setDefaultPIDConstants();

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1.5;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 400;
    shortDriveSettle.timeout = 1200;

    Drivetrain::clampConstants shallowCurveTurnClamp;
    shallowCurveTurnClamp.minimumSpeed = 0;
    shallowCurveTurnClamp.maximumSpeed = 45;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 25;
    clampMogoDriveClamp.maximumSpeed = 75;

    Drivetrain::settleConstants clampMogoDriveSettle;
    clampMogoDriveSettle.deadband = 2;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::outputConstants clampMogoDriveOutput;
    clampMogoDriveOutput.kp = 3;
    clampMogoDriveOutput.ki = 0;
    clampMogoDriveOutput.kd = 0.05;
    clampMogoDriveOutput.startI = 0;
    
    Drivetrain::clampConstants slowerDriveClamp;
    slowerDriveClamp.minimumSpeed = 0;
    slowerDriveClamp.maximumSpeed = 65;

    Drivetrain::clampConstants ramDriveClamp;
    ramDriveClamp.minimumSpeed = 50;
    ramDriveClamp.maximumSpeed = 95;

    Drivetrain::settleConstants ramDriveSettle;
    ramDriveSettle.deadband = 1.5;
    ramDriveSettle.loopCycleTime = 20;
    ramDriveSettle.settleTime = 600;
    ramDriveSettle.timeout = 600;

    armPosition = ARM_ALLIANCE_STAKE_POSITION;
    chassis.driveToPoint(-61, 10, slowerDriveClamp, shortDriveSettle);
    chassis.stopDrive(coast);
    wait(300, msec);  //One ring scored on alliance stake

    armPosition = 0;
    chassis.driveToPoint(-26, 24, clampMogoDriveClamp, clampMogoDriveSettle, clampMogoDriveOutput,
                         shallowCurveTurnClamp, chassis.defaultHeadingSettleConstants, chassis.defaultHeadingOutputConstants); 
    MogoMech.set(true); //Mobile goal clamped

    intakeVelocity = INTAKE_DEFAULT_SPEED;
    chassis.turnToPoint(false, -9, 39);
    chassis.driveToPoint(-9, 39); //One ring scored on mogo

    chassis.stopDrive(brake);
    wait(200, msec);
    chassis.stopDrive(coast);
    chassis.swingToHeading("Right", 0, chassis.defaultTurnClampConstants, chassis.defaultTurnSettleConstants);
    chassis.stopDrive(coast);
    chassis.driveDistance(11, 0, chassis.defaultDriveClampConstants, shortDriveSettle); //Two rings scored on mogo

    chassis.driveToPoint(-16, 32);
    chassis.turnToPoint(false, -24, 48);
    chassis.driveToPoint(-22, 48, chassis.defaultDriveClampConstants, shortDriveSettle); //Three rings scored on mogo

    wait(100, msec);
    chassis.driveToPoint(-55, 55);
    chassis.turnToPoint(false, -68, 68);
    wait(100, msec);
    intakeVelocity = -INTAKE_DEFAULT_SPEED;
    armAntiJam = false;
    armPosition = 145;
    chassis.driveDistance(15.5, chassis.odom.orientation, ramDriveClamp, ramDriveSettle);
    intakeVelocity = INTAKE_DEFAULT_SPEED;
    wait(500, msec);
    chassis.driveDistance(-20);
    chassis.driveDistance(15, chassis.odom.orientation, slowerDriveClamp, shortDriveSettle); //Five rings scored on mogo

    chassis.driveToPoint(-48, 40, slowerDriveClamp);
    armPosition = 0;
    chassis.turnToPoint(false, -48, 0);
    IntakeLift.set(true);
    chassis.driveToPoint(-48, 4, slowerDriveClamp, shortDriveSettle);
    IntakeLift.set(false);
    chassis.driveDistance(-10); //Six rings scored on mogo

    chassis.turnToPoint(false, -24, 0);
    armPosition = ARM_DESCORE_POSITION;
    chassis.driveDistance(20, chassis.odom.orientation, slowerDriveClamp); //Ladder touched

    chassis.stopDrive(coast);
}

void runAutonRedGoalRush(){
    chassis.setCoordinates(-51, -62, 87);
    setDefaultPIDConstants();

    //No mogo
    chassis.defaultDriveOutputConstants.kd = 0.7; 
    chassis.defaultTurnOutputConstants.kd = 0.2;

    Drivetrain::clampConstants goalRushDriveClamp;
    goalRushDriveClamp.minimumSpeed = 0;
    goalRushDriveClamp.maximumSpeed = 100;

    Drivetrain::settleConstants pullGoalDriveSettle;
    pullGoalDriveSettle.deadband = 2;
    pullGoalDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    pullGoalDriveSettle.settleTime = 500;
    pullGoalDriveSettle.timeout = 4000;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1.5;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 400;
    shortDriveSettle.timeout = 1200;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 20;
    clampMogoDriveClamp.maximumSpeed = 60;

    Drivetrain::settleConstants clampMogoDriveSettle;
    clampMogoDriveSettle.deadband = 2;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::outputConstants clampMogoDriveOutput;
    clampMogoDriveOutput.kp = 3;
    clampMogoDriveOutput.ki = 0;
    clampMogoDriveOutput.kd = 0.05;
    clampMogoDriveOutput.startI = 0;

    Drivetrain::clampConstants slowerDriveClamp;
    slowerDriveClamp.minimumSpeed = 0;
    slowerDriveClamp.maximumSpeed = 75;

    Drivetrain::settleConstants longTurnSettle;
    longTurnSettle.deadband = 2;
    longTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    longTurnSettle.settleTime = 800;
    longTurnSettle.timeout = 800;

    Drivetrain::settleConstants fastTurnSettle;
    fastTurnSettle.deadband = 4;
    fastTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    fastTurnSettle.settleTime = 300;
    fastTurnSettle.timeout = 600;

    Drivetrain::clampConstants ramDriveClamp;
    ramDriveClamp.minimumSpeed = 50;
    ramDriveClamp.maximumSpeed = 95;

    Drivetrain::settleConstants ramDriveSettle;
    ramDriveSettle.deadband = 1.5;
    ramDriveSettle.loopCycleTime = 20;
    ramDriveSettle.settleTime = 800;
    ramDriveSettle.timeout = 800;

    LeftDoinker.set(true);
    chassis.driveToPoint(-14, -60, goalRushDriveClamp);
    wait(75, msec);
    LeftDoinker.set(false); //Goal rushed

    chassis.driveDistance(-15, chassis.odom.orientation, goalRushDriveClamp, pullGoalDriveSettle);
    chassis.turnToPoint(false, 0, -72);
    LeftDoinker.set(true);
    chassis.driveDistance(-7);
    LeftDoinker.set(false);
    chassis.turnToPoint(false, -24, -48);
    FirstIntake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.driveToPoint(-24, -48); //One ring intaked

    chassis.turnToPoint(true, -24, -24);
    chassis.driveToPoint(-24, -27, clampMogoDriveClamp, clampMogoDriveSettle, clampMogoDriveOutput);
    MogoMech.set(true); //Mobile goal clamped

    chassis.defaultDriveOutputConstants.kd = 0.65;
    chassis.defaultTurnOutputConstants.kd = 0.25;
    intakeVelocity = INTAKE_DEFAULT_SPEED; //Two rings scored on mogo

    chassis.turnToPoint(false, 0, -3, chassis.defaultTurnClampConstants, longTurnSettle);    
    intakeVelocity = 0;
    chassis.driveToPoint(-8, -8);
    RightDoinker.set(true);
    chassis.stopDrive(brake);
    wait(100, msec); //Ring grabbed

    chassis.stopDrive(coast);
    chassis.driveDistance(-18);
    chassis.turnToPoint(false, -48, 0);
    RightDoinker.set(false);
    intakeVelocity = INTAKE_DEFAULT_SPEED;
    chassis.driveToPoint(-30, -12);
    wait(150, msec);
    chassis.turnToPoint(false, -48, 0);
    IntakeLift.set(true);
    chassis.driveDistance(16, chassis.odom.orientation, slowerDriveClamp, shortDriveSettle);
    IntakeLift.set(false); //Four rings scored on mogo

    chassis.turnToPoint(false, -48, -48);
    chassis.driveToPoint(-48, -48);
    chassis.turnToPoint(false, -66, -66);
    wait(250, msec);
    intakeVelocity = -INTAKE_DEFAULT_SPEED;
    armAntiJam = false;
    armPosition = 130;
    chassis.driveDistance(23, chassis.odom.orientation, ramDriveClamp, ramDriveSettle);
    intakeVelocity = INTAKE_DEFAULT_SPEED;
    wait(400, msec);
    chassis.driveDistance(-16);
    chassis.driveDistance(15, chassis.odom.orientation, slowerDriveClamp, shortDriveSettle); //Six rings scored on mogo

    chassis.turnToPoint(false, 0, -36); 
    MogoMech.set(false);
    chassis.stopDrive(brake); //Stops near rushed goal
}

void runAutonBlueSoloAWP(){
    chassis.setCoordinates(59, 13, 140);
    setDefaultPIDConstants();

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1.5;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 400;
    shortDriveSettle.timeout = 1200;

    Drivetrain::clampConstants shallowCurveTurnClamp;
    shallowCurveTurnClamp.minimumSpeed = 0;
    shallowCurveTurnClamp.maximumSpeed = 45;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 25;
    clampMogoDriveClamp.maximumSpeed = 75;

    Drivetrain::settleConstants clampMogoDriveSettle;
    clampMogoDriveSettle.deadband = 2;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::outputConstants clampMogoDriveOutput;
    clampMogoDriveOutput.kp = 3;
    clampMogoDriveOutput.ki = 0;
    clampMogoDriveOutput.kd = 0.05;
    clampMogoDriveOutput.startI = 0;
    
    Drivetrain::clampConstants slowerDriveClamp;
    slowerDriveClamp.minimumSpeed = 0;
    slowerDriveClamp.maximumSpeed = 65;

    Drivetrain::clampConstants liftDriveClamp;
    liftDriveClamp.minimumSpeed = 0;
    liftDriveClamp.maximumSpeed = 45;

    Drivetrain::clampConstants ramDriveClamp;
    ramDriveClamp.minimumSpeed = 50;
    ramDriveClamp.maximumSpeed = 95;

    Drivetrain::settleConstants ramDriveSettle;
    ramDriveSettle.deadband = 1.5;
    ramDriveSettle.loopCycleTime = 20;
    ramDriveSettle.settleTime = 600;
    ramDriveSettle.timeout = 600;

    armPosition = ARM_ALLIANCE_STAKE_POSITION;
    chassis.driveToPoint(61, 10, slowerDriveClamp, shortDriveSettle);
    chassis.stopDrive(coast);
    wait(300, msec);  //One ring scored on alliance stake

    armPosition = 0;
    chassis.driveToPoint(26, 24, clampMogoDriveClamp, clampMogoDriveSettle, clampMogoDriveOutput,
                         shallowCurveTurnClamp, chassis.defaultHeadingSettleConstants, chassis.defaultHeadingOutputConstants); 
    MogoMech.set(true); //Mobile goal clamped

    intakeVelocity = INTAKE_DEFAULT_SPEED;
    chassis.turnToPoint(false, 9, 39);
    chassis.driveToPoint(9, 39); //One ring scored on mogo

    chassis.stopDrive(brake);
    wait(200, msec);
    chassis.stopDrive(coast);
    chassis.swingToHeading("Left", 0, chassis.defaultTurnClampConstants, chassis.defaultTurnSettleConstants);
    chassis.stopDrive(coast);
    chassis.driveDistance(11, 0, chassis.defaultDriveClampConstants, shortDriveSettle); //Two rings scored on mogo

    chassis.driveToPoint(16, 32);
    chassis.turnToPoint(false, 24, 48);
    chassis.driveToPoint(24, 48, chassis.defaultDriveClampConstants, shortDriveSettle); //Three rings scored on mogo

    chassis.driveToPoint(48, 40, slowerDriveClamp);
    armPosition = 0;
    chassis.turnToPoint(false, 48, 0);
    IntakeLift.set(true);
    chassis.driveToPoint(48, 4, slowerDriveClamp, shortDriveSettle);
    IntakeLift.set(false);
    chassis.driveDistance(-10); //Four rings scored on mogo

    intakeVelocity = 0;
    MogoMech.set(false);
    chassis.driveDistance(10);
    chassis.turnToPoint(true, 24, -24);
    chassis.driveToPoint(25, -23, clampMogoDriveClamp, clampMogoDriveSettle, clampMogoDriveOutput);
    MogoMech.set(true); //Mobile goal clamped

    intakeVelocity = INTAKE_DEFAULT_SPEED;
    chassis.turnToPoint(false, 24, -50);
    chassis.driveToPoint(24, -50); //One ring scored on mobile goal

    armAntiJam = false;
    armPosition = 70;
    chassis.driveToPoint(12, -12, slowerDriveClamp); //Ladder Touched

    chassis.stopDrive(coast);
}

void runAutonBlueRushAWP(){
    intakeVelocity = 0;
}

void runAutonBlueStackAWP(){
    chassis.setCoordinates(59, 13, 140);
    setDefaultPIDConstants();

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1.5;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 400;
    shortDriveSettle.timeout = 1200;

    Drivetrain::clampConstants shallowCurveTurnClamp;
    shallowCurveTurnClamp.minimumSpeed = 0;
    shallowCurveTurnClamp.maximumSpeed = 45;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 25;
    clampMogoDriveClamp.maximumSpeed = 75;

    Drivetrain::settleConstants clampMogoDriveSettle;
    clampMogoDriveSettle.deadband = 2;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::outputConstants clampMogoDriveOutput;
    clampMogoDriveOutput.kp = 3;
    clampMogoDriveOutput.ki = 0;
    clampMogoDriveOutput.kd = 0.05;
    clampMogoDriveOutput.startI = 0;
    
    Drivetrain::clampConstants slowerDriveClamp;
    slowerDriveClamp.minimumSpeed = 0;
    slowerDriveClamp.maximumSpeed = 65;

    Drivetrain::clampConstants ramDriveClamp;
    ramDriveClamp.minimumSpeed = 50;
    ramDriveClamp.maximumSpeed = 95;

    Drivetrain::settleConstants ramDriveSettle;
    ramDriveSettle.deadband = 1.5;
    ramDriveSettle.loopCycleTime = 20;
    ramDriveSettle.settleTime = 600;
    ramDriveSettle.timeout = 600;

    armPosition = ARM_ALLIANCE_STAKE_POSITION;
    chassis.driveToPoint(61, 10, slowerDriveClamp, shortDriveSettle);
    chassis.stopDrive(coast);
    wait(300, msec);  //One ring scored on alliance stake

    armPosition = 0;
    chassis.driveToPoint(26, 24, clampMogoDriveClamp, clampMogoDriveSettle, clampMogoDriveOutput,
                         shallowCurveTurnClamp, chassis.defaultHeadingSettleConstants, chassis.defaultHeadingOutputConstants); 
    MogoMech.set(true); //Mobile goal clamped

    intakeVelocity = INTAKE_DEFAULT_SPEED;
    chassis.turnToPoint(false, 9, 39);
    chassis.driveToPoint(9, 39); //One ring scored on mogo

    chassis.stopDrive(brake);
    wait(200, msec);
    chassis.stopDrive(coast);
    chassis.swingToHeading("Left", 0, chassis.defaultTurnClampConstants, chassis.defaultTurnSettleConstants);
    chassis.stopDrive(coast);
    chassis.driveDistance(11, 0, chassis.defaultDriveClampConstants, shortDriveSettle); //Two rings scored on mogo

    chassis.driveToPoint(16, 32);
    chassis.turnToPoint(false, 24, 48);
    chassis.driveToPoint(22, 48, chassis.defaultDriveClampConstants, shortDriveSettle); //Three rings scored on mogo

    wait(100, msec);
    chassis.driveToPoint(55, 55);
    chassis.turnToPoint(false, 68, 68);
    wait(100, msec);
    intakeVelocity = -INTAKE_DEFAULT_SPEED;
    armAntiJam = false;
    armPosition = 145;
    chassis.driveDistance(15.5, chassis.odom.orientation, ramDriveClamp, ramDriveSettle);
    intakeVelocity = INTAKE_DEFAULT_SPEED;
    wait(500, msec);
    chassis.driveDistance(-20);
    chassis.driveDistance(15, chassis.odom.orientation, slowerDriveClamp, shortDriveSettle); //Five rings scored on mogo

    chassis.driveToPoint(48, 40, slowerDriveClamp);
    armPosition = 0;
    chassis.turnToPoint(false, 48, 0);
    IntakeLift.set(true);
    chassis.driveToPoint(48, 4, slowerDriveClamp, shortDriveSettle);
    IntakeLift.set(false);
    chassis.driveDistance(-10); //Six rings scored on mogo

    chassis.turnToPoint(false, 24, 0);
    armPosition = ARM_DESCORE_POSITION;
    chassis.driveDistance(20, chassis.odom.orientation, slowerDriveClamp); //Ladder touched

    chassis.stopDrive(coast);
}

void runAutonBlueGoalRush(){
    chassis.setCoordinates(51, -62, 273);
    setDefaultPIDConstants();

    //No mogo
    chassis.defaultDriveOutputConstants.kd = 0.7; 
    chassis.defaultTurnOutputConstants.kd = 0.2;

    Drivetrain::clampConstants goalRushDriveClamp;
    goalRushDriveClamp.minimumSpeed = 0;
    goalRushDriveClamp.maximumSpeed = 100;

    Drivetrain::settleConstants pullGoalDriveSettle;
    pullGoalDriveSettle.deadband = 2;
    pullGoalDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    pullGoalDriveSettle.settleTime = 500;
    pullGoalDriveSettle.timeout = 4000;

    Drivetrain::settleConstants shortDriveSettle;
    shortDriveSettle.deadband = 1.5;
    shortDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    shortDriveSettle.settleTime = 400;
    shortDriveSettle.timeout = 1200;

    Drivetrain::clampConstants clampMogoDriveClamp;
    clampMogoDriveClamp.minimumSpeed = 20;
    clampMogoDriveClamp.maximumSpeed = 60;

    Drivetrain::settleConstants clampMogoDriveSettle;
    clampMogoDriveSettle.deadband = 2;
    clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    clampMogoDriveSettle.settleTime = 1000;
    clampMogoDriveSettle.timeout = 1500;

    Drivetrain::outputConstants clampMogoDriveOutput;
    clampMogoDriveOutput.kp = 3;
    clampMogoDriveOutput.ki = 0;
    clampMogoDriveOutput.kd = 0.05;
    clampMogoDriveOutput.startI = 0;

    Drivetrain::clampConstants slowerDriveClamp;
    slowerDriveClamp.minimumSpeed = 0;
    slowerDriveClamp.maximumSpeed = 75;

    Drivetrain::settleConstants longTurnSettle;
    longTurnSettle.deadband = 2;
    longTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    longTurnSettle.settleTime = 800;
    longTurnSettle.timeout = 800;

    Drivetrain::settleConstants fastTurnSettle;
    fastTurnSettle.deadband = 4;
    fastTurnSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    fastTurnSettle.settleTime = 300;
    fastTurnSettle.timeout = 600;

    Drivetrain::clampConstants ramDriveClamp;
    ramDriveClamp.minimumSpeed = 50;
    ramDriveClamp.maximumSpeed = 95;

    Drivetrain::settleConstants ramDriveSettle;
    ramDriveSettle.deadband = 1.5;
    ramDriveSettle.loopCycleTime = 20;
    ramDriveSettle.settleTime = 800;
    ramDriveSettle.timeout = 800;

    RightDoinker.set(true);
    chassis.driveToPoint(14, -60, goalRushDriveClamp);
    wait(75, msec);
    RightDoinker.set(false); //Goal rushed

    chassis.driveDistance(-15, chassis.odom.orientation, goalRushDriveClamp, pullGoalDriveSettle);
    chassis.turnToPoint(false, 0, -72);
    RightDoinker.set(true);
    chassis.driveDistance(-7);
    RightDoinker.set(false);
    chassis.turnToPoint(false, 24, -48);
    FirstIntake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    chassis.driveToPoint(24, -48); //One ring intaked

    chassis.turnToPoint(true, 24, -24);
    chassis.driveToPoint(24, -27, clampMogoDriveClamp, clampMogoDriveSettle, clampMogoDriveOutput);
    MogoMech.set(true); //Mobile goal clamped

    chassis.defaultDriveOutputConstants.kd = 0.65;
    chassis.defaultTurnOutputConstants.kd = 0.25;
    intakeVelocity = INTAKE_DEFAULT_SPEED; //Two rings scored on mogo

    chassis.turnToPoint(false, 0, -3, chassis.defaultTurnClampConstants, longTurnSettle);    
    intakeVelocity = 0;
    chassis.driveToPoint(8, -8);
    LeftDoinker.set(true);
    chassis.stopDrive(brake);
    wait(100, msec); //Ring grabbed

    chassis.stopDrive(coast);
    chassis.driveDistance(-18);
    chassis.turnToPoint(false, 48, 0);
    LeftDoinker.set(false);
    intakeVelocity = INTAKE_DEFAULT_SPEED;
    chassis.driveToPoint(30, -12);
    wait(150, msec);
    chassis.turnToPoint(false, 48, 0);
    IntakeLift.set(true);
    chassis.driveDistance(16, chassis.odom.orientation, slowerDriveClamp, shortDriveSettle);
    IntakeLift.set(false); //Four rings scored on mogo

    chassis.turnToPoint(false, 48, -48);
    chassis.driveToPoint(48, -48);
    chassis.turnToPoint(false, 66, -66);
    wait(250, msec);
    intakeVelocity = -INTAKE_DEFAULT_SPEED;
    armAntiJam = false;
    armPosition = 130;
    chassis.driveDistance(23, chassis.odom.orientation, ramDriveClamp, ramDriveSettle);
    intakeVelocity = INTAKE_DEFAULT_SPEED;
    wait(400, msec);
    chassis.driveDistance(-16);
    chassis.driveDistance(15, chassis.odom.orientation, slowerDriveClamp, shortDriveSettle); //Six rings scored on mogo

    chassis.turnToPoint(false, 0, -36);
    MogoMech.set(false);
    chassis.stopDrive(brake); //Stops near rushed goal
}


void runProgSkills(){
    // chassis.setCoordinates(-57, 0, 270);
    // setDefaultPIDConstants();

    // Drivetrain::clampConstants shortDriveClamp;
    // shortDriveClamp.minimumSpeed = 0;
    // shortDriveClamp.maximumSpeed = 70;

    // Drivetrain::clampConstants clampMogoDriveClamp;
    // clampMogoDriveClamp.minimumSpeed = 0;
    // clampMogoDriveClamp.maximumSpeed = 60;

    // Drivetrain::settleConstants clampMogoDriveSettle;
    // clampMogoDriveSettle.deadband = 1.5;
    // clampMogoDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    // clampMogoDriveSettle.settleTime = 500;
    // clampMogoDriveSettle.timeout = 1000;

    // Drivetrain::clampConstants steepCurveTurnClamp;
    // steepCurveTurnClamp.minimumSpeed = 0;
    // steepCurveTurnClamp.maximumSpeed = 50;

    // Drivetrain::clampConstants slowDriveClamp;
    // slowDriveClamp.minimumSpeed = 0;
    // slowDriveClamp.maximumSpeed = 45;

    // Drivetrain::settleConstants scoreWallDriveSettle;
    // scoreWallDriveSettle.deadband = 1.5;
    // scoreWallDriveSettle.loopCycleTime = DEFAULT_LOOP_CYCLE_TIME;
    // scoreWallDriveSettle.settleTime = 2750;
    // scoreWallDriveSettle.timeout = 2750;

    // armPosition = ARM_ALLIANCE_STAKE_POSITION;
    // wait(600, msec); //One ring scored on alliance stake

    // armPosition = 0;
    // chassis.driveToPoint(-48, 0, shortDriveClamp);
    // chassis.turnToPoint(true, -48, -24);
    // chassis.driveToPoint(-48, -21, clampMogoDriveClamp, clampMogoDriveSettle);
    // MogoMech.set(true); 
    // wait(100, msec); //Mobile goal clamped

    // chassis.turnToPoint(false, -24, -24);
    // intakeVelocity = INTAKE_DEFAULT_SPEED;
    // chassis.driveToPoint(-24, -24); //One ring scored on mogo

    // chassis.turnToPoint(false, 0, -48);
    // armPosition = ARM_LOADING_POSITION;
    // chassis.driveToPoint(24, -48, chassis.defaultDriveClampConstants, chassis.defaultDriveSettleConstants, chassis.defaultDriveOutputConstants, steepCurveTurnClamp, 
    //                      chassis.defaultHeadingSettleConstants, chassis.defaultHeadingOutputConstants);
    // chassis.stopDrive(coast);
    // wait(500, msec); //Two rings scored on mogo

    // chassis.driveToPoint(6, -40);
    // armPosition = ARM_UP_POSITION;
    // chassis.turnToHeading(180);
    // chassis.driveDistance(20, chassis.odom.orientation, slowDriveClamp, scoreWallDriveSettle);
    // ArmMotor.spin(forward, 95, percent);
    // intakeVelocity = -ARM_INTAKE_SPEED;
    // wait(750, msec); //One ring scored on wall stake

    // chassis.driveDistance(-14, 0);
    // ArmMotor.stop(brake);
    // armPosition = 0;
    // chassis.turnToPoint(false, -24, -48);
    // Intake.spin(forward, INTAKE_DEFAULT_SPEED, percent);
    // chassis.driveToPoint(-24, -48); //Three rings scored on mogo

    // chassis.turnToPoint(false, -60, -48);
    // chassis.driveToPoint(-65, -48); //Five rings scored on mogo

    // chassis.turnToPoint(false, -48, -60);
    // chassis.driveToPoint(-48, -60); //Six rings scored on mogo

    // chassis.turnToPoint(true, -66, -66);
    // chassis.driveToPoint(-54, -58, slowDriveClamp);
    // intakeVelocity = -INTAKE_DEFAULT_SPEED;
    // wait(100, msec);
    // MogoMech.set(false); //Mogo deposited in corner

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