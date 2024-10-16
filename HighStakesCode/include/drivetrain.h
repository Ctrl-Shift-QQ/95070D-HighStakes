#include "odometry.h"
#include "PID.h"
#include <iostream>

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
            double maximumSpeed;
        } outputConstants;

        outputConstants defaultDriveOutputConstants;
        outputConstants defaultDriveDistanceTurnOutputConstants;
        outputConstants defaultTurnOutputConstants;
        outputConstants defaultSwingOutputConstants;

        typedef struct {
            double deadband;
            double loopCycleTime;
            double settleTime;
            double timeout;
        } settleConstants;
        
        settleConstants defaultDriveSettleConstants;
        settleConstants defaultDriveDistanceTurnSettleConstants;
        settleConstants defaultTurnSettleConstants;
        settleConstants defaultSwingSettleConstants;


        /******************** Motion algorithms ********************/

        void driveToPoint(double targetX, double targetY);
        void driveToPoint(double targetX, double targetY, settleConstants driveSettleConstants, outputConstants driveOutputConstants);
        void driveToPoint(double targetX, double targetY, settleConstants driveSettleConstants, outputConstants driveOutputConstants, outputConstants turnOutputConstants);

        void driveDistance(double targetDistance);
        void driveDistance(double targetDistance, double targetHeading);
        void driveDistance(double targetDistance, double targetHeading, settleConstants driveSettleConstants);
        void driveDistance(double targetDistance, double targetHeading, settleConstants driveSettleConstants, outputConstants driveOutputConstants);

        void turnToHeading(double targetHeading);
        void turnToHeading(double targetHeading, settleConstants turnSettleConstants);
        void turnToHeading(double targetHeading, settleConstants turnSettleConstants, outputConstants turnOutputConstants);

        void turnToPoint(bool reversed, double targetX, double targetY);
        void turnToPoint(bool reversed, double targetX, double targetY, settleConstants turnSettleConstants);
        void turnToPoint(bool reversed, double targetX, double targetY, settleConstants turnSettleConstants, outputConstants turnOutputConstants);

        void swingToHeading(std::string side, double target);
        void swingToHeading(std::string side, double target, outputConstants swingOutputConstants);
        void swingToHeading(std::string side, double target, double kp, double ki, double kd, double startI, double minimumSpeed, double deadband, double loopCycleTime, double settleTime, double timeout);

        void stopDrive(brakeType brakeType);
};

extern Drivetrain chassis;