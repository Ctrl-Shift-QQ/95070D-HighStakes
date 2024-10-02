/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       vex                                                       */
/*    Created:      7/8/2024, 4:25:23 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "controls.h"
#include "odom.h"
#include "autons.h"
#include <iostream>

using namespace vex;


// A global instance of competition
competition Competition;


typedef enum { //Enum for each of the autons
  AutonNone = 0,
  AutonRedSolo,
  AutonRedLeft,
  AutonRedRight,
  AutonRedRush,
  AutonBlueSolo,
  AutonBlueLeft,
  AutonBlueRight,
  AutonBlueRush,
  AutonCount, //Gives easy access to auton count via static_cast<int> AutonCount
} Auton;

static Auton currentAuton = AutonNone;

void preAuton(){
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //Inertial Calibration
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(2, 6);
  Inertial.calibrate();
  Controller1.Screen.print("CALIBRATING!!!");
  wait(3, sec);
  Controller1.Screen.clearScreen();

  //Auton Selector
  bool selectingSide = true;
  bool redAlliance; //True = Red, False = Blue
  bool selectingAuton = true;

  while (selectingSide){ //Selects the alliance color
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 6);
    Controller1.Screen.print("Alliance Color");

    if (redAlliance){
      Controller1.Screen.setCursor(3, 2);
      Controller1.Screen.print("Red x");
      Controller1.Screen.setCursor(3, 18);
      Controller1.Screen.print("Blue");
    }
    else {
      Controller1.Screen.setCursor(3, 2);
      Controller1.Screen.print("Red");
      Controller1.Screen.setCursor(3, 18);
      Controller1.Screen.print("Blue x");
    }
    
    if (pressed(Left)){ //Press left button to select red alliance
      redAlliance = true;
    }

    if (pressed(Right)){ //Press right button to select blue alliance
      redAlliance = false;
    }

    if (Controller1.ButtonUp.pressing()){ //Press up button to select
      Controller1.Screen.clearScreen();

      selectingSide = false;
    }

    wait(20, msec);
  }

  wait(50, msec);
  Controller1.rumble("..");

  int columns[] = {8, 5, 5, 8}; //Columns to center the text
  std::string autonNames[] = {"Solo AWP", "Left-Side AWP", "Right-Side AWP", "Goal Rush"};
  Auton redAutons[] = {AutonRedSolo, AutonRedLeft, AutonRedRight, AutonRedRush};
  Auton blueAutons[] = {AutonBlueSolo, AutonBlueLeft, AutonBlueRight, AutonBlueRush};

  while (selectingAuton){ //Selects the auton
    if (currentAuton == AutonNone){
      Controller1.Screen.setCursor(2, 3);
      Controller1.Screen.print("No Auton Selected");
    }
    else {
      Controller1.Screen.setCursor(1, 5);
      Controller1.Screen.print("Auton Selected:");

      if (!redAlliance){ //Prints the corresponding text and the corresponding column
        Controller1.Screen.setCursor(3, columns[static_cast<int> (currentAuton) - ((static_cast<int> (AutonCount) - 1) / 2) /* Offset by half of the auton count (excluding AutonNone) */ - 1]);
        Controller1.Screen.print(autonNames[static_cast<int> (currentAuton) - ((static_cast<int> (AutonCount) - 1) / 2) /* Offset by half of the auton count (excluding AutonNone) */ - 1].c_str());
      }
      //Offset by one to correspond to correct auton because AutonNone takes an extra spot
      else {
        Controller1.Screen.setCursor(3, columns[static_cast<int> (currentAuton) - 1]); 
        Controller1.Screen.print(autonNames[static_cast<int> (currentAuton) - 1].c_str());
      }
    }
  
    if (pressed(Left)){ //Press left button go left on auton list
      Controller1.Screen.clearScreen();

      if (redAlliance){
        if (currentAuton == AutonNone || currentAuton == redAutons[0]){ //Sets auton to last one if press left on first
          currentAuton = redAutons[sizeof(redAutons) - 1];
        }
        else {
          currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) - 1); //Sets auton to the one before it
        }
      }
      else {
        if (currentAuton == AutonNone || currentAuton == blueAutons[0]){ //Sets auton to last one if press left on first
          currentAuton = blueAutons[sizeof(redAutons) - 1];
        }
        else {
          currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) - 1); //Sets auton to the one before it
        }
      }
    }

    if (pressed(Right)){ //Press right button go left on auton list
      Controller1.Screen.clearScreen();

      if (redAlliance){
        if (currentAuton == AutonNone || currentAuton == redAutons[sizeof(redAutons) - 1]){ //Sets auton to first one if press right on last
          currentAuton = redAutons[0];
        }
        else {
          currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) + 1); //Sets auton to the one after it
        }
      }
      else {
        if (currentAuton == AutonNone || currentAuton == blueAutons[sizeof(redAutons) - 1]){ //Sets auton to first one if press right on last
          currentAuton = blueAutons[0];
        }
        else {
          currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) + 1); //Sets auton to the one after it
        }
      }
    }

    if (Controller1.ButtonUp.pressing()){ //Press up button to select
      Controller1.Screen.clearScreen();

      selectingAuton = false; //Exits loop
    }
    
    wait(20, msec);
  }

  wait(50, msec);
  Controller1.rumble("-.-.");
}

void autonomous(){
  double startTime = Brain.Timer.time(); //Records start time

  switch (currentAuton){ //Runs corresponding auton
    case AutonNone: {
      break;
    }
    case AutonRedSolo: {
      runAutonRedSolo();
      break;
    }
    case AutonRedLeft: {
      runAutonRedLeft();
      break;
    }
    case AutonRedRight: {
      runAutonRedRight();
      break;
    }
    case AutonRedRush: {
      runAutonRedRush();
      break;
    }
    case AutonBlueSolo: {
      runAutonBlueSolo();
      break;
    }
    case AutonBlueLeft: {
      runAutonBlueLeft();
      break;
    }
    case AutonBlueRight: {
      runAutonBlueRight();
      break;
    }
    case AutonBlueRush: {
      runAutonBlueRush();
      break;
    }
    default: {
      break;
    }
  }

  Controller1.Screen.setCursor(2, 6);
  Controller1.Screen.print((Brain.Timer.time() - startTime) / 1000); //Records time spent
  Controller1.Screen.print(" Seconds");
}

void usercontrol(){
  while (true){
    runArcadeDrive(100, 50);
    runIntake(80);
    runMogo();
    
    wait(20, msec);
  }
}

int main(){
  Odom odom;
  odom.setPhysicalMeasurements(2.75, 1.5, 4);
  odom.setPosition(0, 0, 0);

  Inertial.calibrate();
  wait(3, sec);

  while (true){
    odom.updatePosition();
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 3);
    Controller1.Screen.print("X: ");
    Controller1.Screen.print(odom.xPosition);
    Controller1.Screen.setCursor(2, 3);
    Controller1.Screen.print("Y: ");
    Controller1.Screen.print(odom.yPosition);
    Controller1.Screen.setCursor(3, 3);
    Controller1.Screen.print("θ: ");
    Controller1.Screen.print(odom.orientation);
  }
  // Competition.drivercontrol(usercontrol);
  // Competition.autonomous(autonomous);

  // preAuton();

  while (true){
    wait(20, msec);
  }
}
