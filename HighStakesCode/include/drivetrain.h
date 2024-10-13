#include "odometry.h"
#include "PID.h"

class Drivetrain {
    private:
        /******************** Position tracking ********************/

        vex::task positionTrackTask;
        static int trackPosition();


    public:
        Drivetrain(double wheelDiameter, double sidewaysToCenterDistance, double forwardToCenterDistance); //Constructor

        /******************** Odometry ********************/

        Odometry odom;

        void setCoordinates(double startPositionX, double startPositionY, double startPositionOrientation); //Sets coordinates and starts position tracking

        /******************** PID constants ********************/

        typedef struct {
            double kp;
            double ki;
            double kd;
            double startI;
            double minimumSpeed;
        } outputConstants;

        outputConstants defaultDriveOutputConstants;
        outputConstants defaultTurnOutputConstants;
        outputConstants defaultSwingOutputConstants;

        typedef struct {
            double deadband;
            double loopCycleTime;
            double settleTime;
            double timeout;
        } settleConstants;
        
        settleConstants defaultDriveSettleConstants;
        settleConstants defaultTurnSettleConstants;
        settleConstants defaultSwingSettleConstants;


        /******************** Motion algorithms ********************/

        void driveToPoint(double targetX, double targetY);
        void driveToPoint(double targetX, double targetY, outputConstants driveOutputConstants, settleConstants settleConstants);
        void driveToPoint(double targetX, double targetY, outputConstants driveOutputConstants, outputConstants turnOutputConstants, settleConstants settleConstants);

        void driveDistance(double targetDistance);
        void driveDistance(double targetDistance, outputConstants driveOutputConstants, settleConstants driveSettleConstants);
        void driveDistance(double targetDistance, outputConstants driveOutputConstants, outputConstants turnOutputConstants, settleConstants driveSettleConstants);

        void turnToHeading(double targetHeading);
        void turnToHeading(double targetHeading, outputConstants turnOutputConstants);
        void turnToHeading(double targetHeading, outputConstants turnOutputConstants, settleConstants turnSettleConstants);

        void turnToPoint(double targetX, double targetY);
        void turnToPoint(double targetX, double targetY, outputConstants turnOutputConstants);
        void turnToPoint(double targetX, double targetY, outputConstants turnOutputConstants, settleConstants turnSettleConstants);

        // void swingToHeading(std::string direction, double target);
        // void swingToHeading(std::string direction, double target, double kp, double ki, double kd, double startI, double minimumSpeed);
        // void swingToHeading(std::string direction, double target, double kp, double ki, double kd, double startI, double minimumSpeed, double deadband, double loopCycleTime, double settleTime, double timeout);

        void stopDrive(vex::brakeType brakeType);
};

extern Drivetrain chassis;
