using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LeftFront;
extern motor LeftBack;
extern motor LeftStack;
extern motor RightFront;
extern motor RightBack;
extern motor RightStack;
extern motor_group LeftDrive;
extern motor_group RightDrive;
extern motor FirstIntake;
extern motor SecondIntake;
extern motor_group Intake;
extern inertial Inertial;
extern encoder SidewaysTracker;
extern encoder ForwardTracker;
extern digital_out MogoMech;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );