#ifndef POSEPID_HPP
#define POSEPID_HPP

#include "PIDController.hpp"
#include "Pose.hpp"

class PosePID
{

    private:

        // Three PIDs for three seperate components
        PIDController* xPID;
        PIDController* yPID;
        PIDController* thetaPID;

        Pose* targetPose;
        Pose* outputPose;

    public:

        // PosePID constructor
        PosePID();

        void setXConstants(double kP, double kI, double kD);
        void setYConstants(double kP, double kI, double kD);
        void setThetaConstants(double kP, double kI, double kD);
        
        // Set the target value to the PID to reach
        void setTarget(Pose* target);

        double getTarget();
        
        // Step the PID every iteration of the loop
        Pose* stepPID(Pose* input, double deltaTime);

        bool isSettled();
    

};

#endif