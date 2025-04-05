#pragma once

#include <iostream>

enum Auton {
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
};

extern std::string allianceColor;

extern Auton currentAuton;

extern bool runningPreAuton; //Prevents driver control from running during pre-auton