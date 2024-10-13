#include "vex.h"
#include "util.h"
#include <iostream>

double getSign(double input){
    return (input > 0) - (input < 0);
}

double degToRad(double deg){
    return deg * (M_PI / 180);
}

double radToDeg(double rad){
    return rad * (180 / M_PI);
}

double headingToPolar(double heading){
    double output = 360 - (heading - 90);

    if (output > 360){ //Brings output to 0 - 360
        output -= 360;
    }
    else if (output < 0){
        output += 360;
    }

    return output;
}

double polarToHeading(double polar){
    double output = 360 - (polar - 90);

    if (output > 360){ //Brings output to 0 - 360
        output -= 360;
    }
    else if (output < 0){
        output += 360;
    }

    return output;
}

double headingError(double target, double heading){
    double output;

    if (fabs(target - heading) < 180){ //Calculates error if shortest path doesn't cross 0
        output = target - heading;
    }
    else { //Calculates error if shortest path does cross 0
        output = (360 - fabs(target - heading));
        if (target > heading){
            return -output;
        }
    }

    return output;
}

double driveOutputScale(double minimumSpeed, double leftDriveOutput, double rightDriveOutput){
    if (fabs(leftDriveOutput) < minimumSpeed || fabs(rightDriveOutput) < minimumSpeed){ //Sets speeds to minimumSpeed if calculated output is less
        if (fabs(leftDriveOutput) < fabs(rightDriveOutput)){
            return minimumSpeed / fabs(leftDriveOutput);
        }

        return minimumSpeed / fabs(rightDriveOutput);
    }
    else if (fabs(leftDriveOutput) > 100 || fabs(rightDriveOutput) > 100){ //Sets speeds to 100 if calculated output is more
        if (fabs(leftDriveOutput) > fabs(rightDriveOutput)){
            return 100 / fabs(leftDriveOutput);
        }
    
        return 100 / fabs(rightDriveOutput);
    }
    
    return 1;
}
