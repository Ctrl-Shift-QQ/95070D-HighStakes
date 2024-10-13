class PID {
    private:
        double startError;

        //Tunings
        double kp;
        double ki;
        double kd;

        //For output calculations
        double startI;
        double integral;
        double previousError;
        double derivative;

        //For settling calculations
        double deadband;
        double loopCycleTime;
        double timeSpent;
        double settleTime;
        double timeout;

    public:
        PID(double startError, double kp, double ki, double kd, double startI, double deadband, double loopCycleTime, double settleTime, double timeout); //Constructor

        double output(double error); //Returns the output of the PID loop
        bool isSettled(double error); //Returns whether or not the PID is settled
};