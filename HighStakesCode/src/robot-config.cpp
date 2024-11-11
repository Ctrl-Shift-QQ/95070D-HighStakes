#include "vex.h"


using namespace vex;
using signature = vision::signature;
using code = vision::code;


// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;


// VEXcode device constructors

controller Controller1 = controller(primary);
motor LeftFront = motor(PORT11, ratio6_1, true);
motor LeftBack = motor(PORT10, ratio6_1, true);
motor LeftStack = motor(PORT14, ratio6_1, true);
motor RightFront = motor(PORT19, ratio6_1, false);
motor RightBack = motor(PORT20, ratio6_1, false);
motor RightStack = motor(PORT3, ratio6_1, false);
motor_group LeftDrive(LeftFront, LeftBack, LeftStack);
motor_group RightDrive(RightFront, RightBack, RightStack);
motor FirstIntake = motor(PORT18, ratio18_1, true);
motor SecondIntake = motor(PORT21, ratio6_1, true);
motor_group Intake(FirstIntake, SecondIntake);
motor ArmMotor = motor(PORT17, ratio18_1, false);
motor_group Arm(ArmMotor);
rotation ArmRotation = rotation(PORT13, false);
optical IntakeOptical = optical(PORT21);
inertial Inertial = inertial(PORT7);
encoder SidewaysTracker = encoder(Brain.ThreeWirePort.G);
encoder ForwardTracker = encoder(Brain.ThreeWirePort.E);
digital_out MogoMech = digital_out(Brain.ThreeWirePort.D);
digital_out Doinker = digital_out(Brain.ThreeWirePort.A);


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
