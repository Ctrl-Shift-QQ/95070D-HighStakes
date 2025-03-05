#pragma once

#include <iostream>

typedef enum { //Enum for each of the autons
  AutonNone = 0,
  AutonRedSoloAWP,
  AutonRedRushAWP,
  AutonRedStackAWP,
  AutonRedGoalRush,
  AutonBlueSoloAWP,
  AutonBlueRushAWP,
  AutonBlueStackAWP,
  AutonBlueGoalRush,
  AutonCount, //Gives easy access to auton count via static_cast<int> AutonCount
} Auton;

extern std::string allianceColor = "Red";

extern Auton currentAuton = AutonNone;

extern bool runningPreAuton = true; //Prevents driver control from running during pre-auton