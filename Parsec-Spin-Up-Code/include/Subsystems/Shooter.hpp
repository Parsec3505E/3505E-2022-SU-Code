#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP


#include "api.h"

#include "Utility/PIDController.hpp"


class Shooter
{

    private:

        double RPM;

        // Shooter States
        enum ShooterStates{};

        PIDController* rpmPID;

        double motorVelLimit;
        double motorAccLimit;

        // Shooter Motor Declarations
        pros::Motor* shooterPwr1;
        pros::Motor* shooterPwr2;
        pros::Motor* shooterInd;

        // Shooter Sensors

        // Shooter Encoder Declarations
        pros::ADIEncoder* flywheelEncoder;

    public:

        // Shooter Constructor
        Shooter();

        // Update the state of the Shooter
        void updateShooter();

        enum ShooterStates getState();

    private:

        // Set the state of the Shooter
        void setState(enum ShooterStates);

        void setTargetRPM(double RPM);

        double slewRPM(double request);

        bool isSettled(double epsilon);

        double calcShotRPM(double distance);


};

#endif