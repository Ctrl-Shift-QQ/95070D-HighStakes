#pragma once

#include "odometry.h"
#include "PID.h"
#include <iostream>

enum OdomType{
    ZeroTrackerOdom = 0,
    OneTrackerOdom,
    TwoTrackerOdom,
};

class Drivetrain {
    //Units: inches and degrees unless otherwise specified
    private:
        motor_group &LeftDrive; 
        motor_group &RightDrive;
        rotation &HorizontalTracker;
        rotation &VerticalTracker; 
        inertial &Inertial;

        double gearRatio;
        double verticalWheelDiameter; 
        double verticalToCenterDistance; 
        double horizontalWheelDiameter;
        double horizontalToCenterDistance;
        double inertialScale;

        OdomType odomType;

        static int trackPosition();
        task positionTrackTask;
    public:
        Drivetrain(motor_group &LeftDrive, motor_group &RightDrive, double gearRatio, double verticalWheelDiameter, inertial &Inertial, double inertialScale); //Zero Tracker Odom Constructor
        Drivetrain(motor_group &LeftDrive, motor_group &RightDrive, double gearRatio, double verticalWheelDiameter, 
                   rotation &HorizontalTracker, double horizontalWheelDiameter, double horizontalToCenterDistance, inertial &Inertial, double inertialScale); //One Tracker Odom Constructor
        Drivetrain(motor_group &LeftDrive, motor_group &RightDrive, double gearRatio, rotation &VerticalTracker, double verticalWheelDiameter, double verticalToCenterDistance, 
                   rotation &HorizontalTracker, double horizontalWheelDiameter, double horizontalToCenterDistance, inertial &Inertial, double inertialScale); //Two Tracker Odom Constructor

        double getHorizontalTrackerPosition();
        double getVerticalTrackerPosition();
        double getAbsoluteHeading();

        void resetHorizontalTrackerPosition();
        void resetVerticalTrackerPosition();
        void setAbsoluteHeading(double heading);
        
        Odometry odom;

        void setCoordinates(double startPositionX, double startPositionY, double startPositionOrientation); //Sets coordinates and starts position tracking

        typedef struct {
            double kp;
            double ki;
            double kd;
            double startI;
        } outputConstants;

        outputConstants defaultDriveOutputConstants;
        outputConstants defaultHeadingOutputConstants;
        outputConstants defaultTurnOutputConstants;
        outputConstants defaultSwingOutputConstants;

        typedef struct {
            double minimumSpeed;
            double maximumSpeed;
        } clampConstants;

        clampConstants defaultDriveClampConstants;
        clampConstants defaultHeadingClampConstants;
        clampConstants defaultTurnClampConstants;
        clampConstants defaultSwingClampConstants;

        typedef struct {
            double deadband;
            double loopCycleTime;
            double settleTime;
            double timeout;
        } settleConstants;
        
        settleConstants defaultDriveSettleConstants;
        settleConstants defaultHeadingSettleConstants;
        settleConstants defaultTurnSettleConstants;
        settleConstants defaultSwingSettleConstants;

        void driveToPoint(double targetX, double targetY);
        void driveToPoint(double targetX, double targetY, clampConstants driveClampConstants);
        void driveToPoint(double targetX, double targetY, clampConstants driveClampConstants, settleConstants driveSettleConstants);
        void driveToPoint(double targetX, double targetY, clampConstants driveClampConstants, settleConstants driveSettleConstants, outputConstants driveOutputConstants);
        void driveToPoint(double targetX, double targetY, clampConstants driveClampConstants, settleConstants driveSettleConstants, outputConstants driveOutputConstants, clampConstants headingClampConstants, settleConstants headingSettleConstants, outputConstants headingOutputConstants);

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