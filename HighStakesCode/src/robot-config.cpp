#include "vex.h"


using namespace vex;
using signature = vision::signature;
using code = vision::code;


// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;


// VEXcode device constructors

controller Controller1 = controller(primary);
motor LeftFront = motor(PORT11, ratio6_1, true);
motor LeftBack = motor(PORT9, ratio6_1, true);
motor LeftStack = motor(PORT10, ratio6_1, true);
motor RightFront = motor(PORT14, ratio6_1, false);
motor RightBack = motor(PORT7, ratio6_1, false);
motor RightStack = motor(PORT2, ratio6_1, false);
motor_group LeftDrive(LeftFront, LeftBack, LeftStack);
motor_group RightDrive(RightFront, RightBack, RightStack);
motor FirstIntake = motor(PORT5, ratio18_1, false);
motor SecondIntake = motor(PORT6, ratio6_1, false);
motor_group Intake(FirstIntake, SecondIntake);
motor ArmMotor = motor(PORT3, ratio18_1, true);
motor_group Arm(ArmMotor);
rotation ArmRotation = rotation(PORT15, false);
optical IntakeOptical = optical(PORT21);
inertial Inertial = inertial(PORT12);
rotation HorizontalTracker = rotation(PORT13, false);
digital_out MogoMech = digital_out(Brain.ThreeWirePort.H);
digital_out Doinker = digital_out(Brain.ThreeWirePort.G);


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
