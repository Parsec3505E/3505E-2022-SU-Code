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

    public:

        // PosePID constructor
        PosePID();
        
        // Set the target value to the PID to reach
        void setTarget(Pose target);
        
        // Step the PID every iteration of the loop
        void stepPID(Pose input);
    

};

#endif