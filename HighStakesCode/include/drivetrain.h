#include "odometry.h"
#include "PID.h"
#include <iostream>

class Drivetrain {
    private:
        /******************** Position tracking ********************/

        static int trackPosition();
        task positionTrackTask;

    public:
        Drivetrain(double wheelDiameter, double sidewaysToCenterDistance, double forwardToCenterDistance, double inertialScale); //Constructor

        /******************** Odometry ********************/

        Odometry odom;

        void setCoordinates(double startPositionX, double startPositionY, double startPositionOrientation); //Sets coordinates and starts position tracking

        /******************** PID constants ********************/

        typedef struct {
            double kp;
            double ki;
            double kd;
            double startI;
        } outputConstants;

        outputConstants defaultDriveOutputConstants;
        outputConstants defaultDriveDistanceTurnOutputConstants;
        outputConstants defaultTurnOutputConstants;
        outputConstants defaultSwingOutputConstants;

        typedef struct {
            double minimumSpeed;
            double maximumSpeed;
        } clampConstants;

        clampConstants defaultDriveClampConstants;
        clampConstants defaultDriveDistanceTurnClampConstants;
        clampConstants defaultTurnClampConstants;
        clampConstants defaultSwingClampConstants;

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
        void driveToPoint(double targetX, double targetY, clampConstants driveClampConstants);
        void driveToPoint(double targetX, double targetY, clampConstants driveClampConstants, settleConstants driveSettleConstants);
        void driveToPoint(double targetX, double targetY, clampConstants driveClampConstants, settleConstants driveSettleConstants, outputConstants driveOutputConstants);
        void driveToPoint(double targetX, double targetY, clampConstants driveClampConstants, settleConstants driveSettleConstants, outputConstants driveOutputConstants, clampConstants turnClampConstants, outputConstants turnOutputConstants);

        void driveDistance(double targetDistance);
        void driveDistance(double targetDistance, double targetHeading);
        void driveDistance(double targetDistance, double targetHeading, clampConstants driveClampConstants);
        void driveDistance(double targetDistance, double targetHeading, clampConstants driveClampConstants, settleConstants driveSettleConstants);
        void driveDistance(double targetDistance, double targetHeading, clampConstants driveClampConstants, settleConstants driveSettleConstants, outputConstants driveOutputConstants);

        void turnToHeading(double targetHeading);
        void turnToHeading(double targetHeading, clampConstants turnClampConstants);
        void turnToHeading(double targetHeading, clampConstants turnClampConstants, settleConstants turnSettleConstants);
        void turnToHeading(double targetHeading, clampConstants turnClampConstants, settleConstants turnSettleConstants, outputConstants turnOutputConstants);

        void turnToPoint(bool reversed, double targetX, double targetY);
        void turnToPoint(bool reversed, double targetX, double targetY, clampConstants turnClampConstants);
        void turnToPoint(bool reversed, double targetX, double targetY, clampConstants turnClampConstants, settleConstants turnSettleConstants);
        void turnToPoint(bool reversed, double targetX, double targetY, clampConstants turnClampConstants, settleConstants turnSettleConstants, outputConstants turnOutputConstants);

        void swingToHeading(std::string driveSide, double targetHeading);
        void swingToHeading(std::string driveSide, double targetHeading, clampConstants swingClampConstants);
        void swingToHeading(std::string driveSide, double targetHeading, clampConstants swingClampConstants, settleConstants swingSettleConstants);
        void swingToHeading(std::string driveSide, double targetHeading, clampConstants swingClampConstants, settleConstants swingSettleConstants, outputConstants swingOutputConstants);

        void stopDrive(brakeType brakeType);
};

extern Drivetrain chassis;