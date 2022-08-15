#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP


class PIDController
{

    private:

        // The P, I, and D term of the PID
        double kP;
        double kI;
        double kD;

        double target;
        double error;
        double errorSum;
        double lastError;

        double maxOutput;
        double minOutput;
        

    public:

        // PIDController constructor
        PIDController(double kP, double kI, double kD);
        
        // Set the PID term constants
        void setConstants(double kP, double kI, double kD);
        
        // Set the target value to the PID to reach
        void setTarget(double target);
        
        // Step the PID every iteration of the loop
        double stepPID(double input);
    

};

#endif