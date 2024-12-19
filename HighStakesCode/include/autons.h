#include "drivetrain.h"
#include "mech-config.h"

/******************** PID Tunings ********************/

void setDefaultPIDConstants();

/******************** Tasks ********************/

int colorSort();

int armDown();

int armLoad();

/******************** Tests ********************/

void runOdomTest();

void runDriveTest();

void runTurnTest();

void runSwingTest();

/******************** Autons ********************/

void runAutonRedSoloAWP();

void runAutonRedRushAWP();

void runAutonRedStackAWP();

void runAutonRedGoalRush();

void runAutonBlueSoloAWP();

void runAutonBlueRushAWP();

void runAutonBlueStackAWP();

void runAutonBlueGoalRush();

/******************** Prog Skills ********************/

void runProgSkills();