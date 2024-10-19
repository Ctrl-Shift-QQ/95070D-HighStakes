#include "vex.h"


using namespace vex;
using signature = vision::signature;
using code = vision::code;


// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;


// VEXcode device constructors

controller Controller1 = controller(primary);
motor LeftFront = motor(PORT3, ratio6_1, true);
motor LeftBack = motor(PORT4, ratio6_1, true);
motor LeftStack = motor(PORT5, ratio6_1, true);
motor RightFront = motor(PORT1, ratio6_1, false);
motor RightBack = motor(PORT2, ratio6_1, false);
motor RightStack = motor(PORT11, ratio6_1, false);
motor_group LeftDrive(LeftFront, LeftBack, LeftStack);
motor_group RightDrive(RightFront, RightBack, RightStack);
motor FirstIntake = motor(PORT9, ratio18_1, false);
motor SecondIntake = motor(PORT7, ratio6_1, false);
motor_group Intake(FirstIntake, SecondIntake);
motor ArmMotor = motor(PORT10, ratio18_1, true);
motor_group Arm(ArmMotor);
rotation ArmRotation = rotation(PORT12, false);
optical IntakeOptical = optical(PORT21);
inertial Inertial = inertial(PORT8);
encoder SidewaysTracker = encoder(Brain.ThreeWirePort.E);
encoder ForwardTracker = encoder(Brain.ThreeWirePort.A);
digital_out MogoMech = digital_out(Brain.ThreeWirePort.H);
digital_out Doinker = digital_out(Brain.ThreeWirePort.D);

//Bad Ports: 6, 19, 18, 17, 20


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
