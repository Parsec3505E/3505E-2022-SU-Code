#ifndef SHOOTER_HPP
#define SHOOTER_HPP


#include "api.h"

#include "Utility/PIDController.hpp"


class Shooter
{

    public:

        // Shooter States
        enum ShooterStates{CLOSED_LOOP, OPERATOR_CONTROL, DISABLED};

    private:

        ShooterStates mShooterState;

        double RPM;

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
        void updateShooter(pros::Controller driver);

        enum ShooterStates getState();

    //private:

        // Set the state of the Shooter
        void setState(enum ShooterStates);

        void setTargetRPM(double RPM);

        double slewRPM(double request);

        bool isSettled(double epsilon);

        double calcShotRPM(double distance);
};

#endif