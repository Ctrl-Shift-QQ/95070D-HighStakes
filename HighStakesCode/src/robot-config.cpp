#include "vex.h"


using namespace vex;
using signature = vision::signature;
using code = vision::code;


// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;


// VEXcode device constructors

controller Controller1 = controller(primary);
motor LeftFront = motor(PORT15, ratio6_1, true);
motor LeftBack = motor(PORT11, ratio6_1, true);
motor LeftStack = motor(PORT17, ratio6_1, true);
motor RightFront = motor(PORT19, ratio6_1, false);
motor RightBack = motor(PORT16, ratio6_1, false);
motor RightStack = motor(PORT9, ratio6_1, false);
motor_group LeftDrive(LeftFront, LeftBack, LeftStack);
motor_group RightDrive(RightFront, RightBack, RightStack);
motor FirstIntake = motor(PORT10, ratio18_1, true);
motor SecondIntake = motor(PORT12, ratio6_1, false);
motor_group Intake(FirstIntake, SecondIntake);
motor ArmMotor = motor(PORT8, ratio18_1, true);
motor_group Arm(ArmMotor);
rotation ArmRotation = rotation(PORT21, true);
inertial Inertial = inertial(PORT7);
rotation SidewaysTracker = rotation(PORT18, true);
// rotation ForwardTracker = rotation(PORT20, true);
digital_out MogoMech = digital_out(Brain.ThreeWirePort.A);
digital_out Doinker = digital_out(Brain.ThreeWirePort.H);


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
