#pragma once

#include "drivetrain.h"
#include "PID.h"
#include "match-config.h"
#include "mech-config.h"

void setDefaultPIDConstants();

int controlIntake();
int controlArm();

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