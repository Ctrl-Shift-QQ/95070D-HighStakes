#pragma once

#include "drivetrain.h"
#include "PID.h"
#include "match-config.h"
#include "mech-config.h"

void setDefaultPIDConstants();

int controlIntake();
void controlArm();
int armToDown();
int armToLoad();
int armToLadder();
int armToUp();
int armToAllianceStake();
int armToWallStake();

void runOdomTest();
void runDriveTest();
void runTurnTest();
void runSwingTest();

void runAutonRedSoloAWP();
void runAutonRedRushAWP();
void runAutonRedStackAWP();
void runAutonRedGoalRush();
void runAutonBlueSoloAWP();
void runAutonBlueRushAWP();
void runAutonBlueStackAWP();
void runAutonBlueGoalRush();

void runProgSkills();