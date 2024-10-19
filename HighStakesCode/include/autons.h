#include "drivetrain.h"

/******************** PID Tunings ********************/

void setDefaultPIDConstants();

/******************** Tasks ********************/

extern bool redAlliance;

int colorSort();

int armDown();

int armLoad();

/******************** Autons ********************/

void runOdomTest();

void runAutonRedSoloAWP();

void runAutonRedRushAWP();

void runAutonRedStackAWP();

void runAutonRedGoalRush();

void runAutonBlueSoloAWP();

void runAutonBlueRushAWP();

void runAutonBlueStackAWP();

void runAutonBlueGoalRush();

/******************** PID Tunings ********************/

void runProgSkills();