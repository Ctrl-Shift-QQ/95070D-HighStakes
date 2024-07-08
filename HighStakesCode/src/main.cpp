/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       vex                                                       */
/*    Created:      7/8/2024, 4:25:23 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <iostream>

using namespace vex;


// A global instance of competition
competition Competition;


typedef enum {
    AutonNone = 0,

} Auton; //Enum for each of the autons

static Auton currentAuton = AutonNone;

void preAuton(){

    //Inertial Calibration
    Controller1.Screen.setCursor(2, 6);
    Inertial.calibrate();
    Controller1.Screen.print("CALIBRATING!!!");
    wait(3, sec);
    Controller1.Screen.clearScreen();

    //Auton Selector
    bool runningSelector = true;

    int columns[] = {};
    std::string autonNames[] = {};
    Auton autons[] = {};

    bool buttonLeftPressed;
    bool buttonRightPressed;

    while (runningSelector){
    if (currentAuton == AutonNone){
        Controller1.Screen.setCursor(2, 3);
        Controller1.Screen.print("No Auton Selected");
    }
    else{
        Controller1.Screen.setCursor(1, 5);
        Controller1.Screen.print("Auton Selected:");
    }

    for (int i = 0; i < 10; i++){
        if (currentAuton == autons[i]){ //Displays auton label
        Controller1.Screen.setCursor(3, columns[i]);
        Controller1.Screen.print(autonNames[i].c_str());
        }
    }

    if (Controller1.ButtonLeft.pressing() && !buttonLeftPressed){ //Pressing left button go left on auton list
        Controller1.Screen.clearScreen();
        if (currentAuton == AutonNone || currentAuton == AutonNone){
        currentAuton = AutonNone;
        }
        else{
        currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) - 1);
        }

        buttonLeftPressed = true;
    }
    if (!Controller1.ButtonLeft.pressing() && buttonLeftPressed){
        buttonLeftPressed = false;
    }

    if (Controller1.ButtonRight.pressing() && !buttonRightPressed){ //Pressing right button go left on auton list
        Controller1.Screen.clearScreen();
        if (currentAuton == AutonNone){
        currentAuton = AutonNone;
        }
        else{
        currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) + 1);
        }

        buttonRightPressed = true;
    }
    if (!Controller1.ButtonRight.pressing() && buttonRightPressed){
        buttonRightPressed = false; 
    }

    if (Controller1.ButtonUp.pressing()){ //Exits selector when up button is pressed
        Controller1.Screen.clearScreen();
        runningSelector = false;
    }

    wait(50, msec);
    }
    Controller1.rumble("-.-.");
}

void autonomous(){
    double startTime = Brain.Timer.time(); //Records start time
  Controller1.Screen.setCursor(2, 6);

  switch (currentAuton){
    case AutonNone: {
      break;
    }
    default: {
      break;
    }
  }

  Controller1.Screen.print((Brain.Timer.time() - startTime) / 1000); //Records time spent
  Controller1.Screen.print(" Seconds");
}

void driverControl(){
    while (true){
        
    }
}

int main(){
  Competition.drivercontrol(driverControl);
  Competition.autonomous(autonomous);

  preAuton();

  while(true){
    wait(20, msec);
  }
}
