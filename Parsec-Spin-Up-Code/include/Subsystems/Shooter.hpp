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

        int targetVel;
        bool beenSettled;
        int timeSettled;
        int minSettledTime;
        double epsilon;

        // Shooter Motor Declarations
        pros::Motor* shooterPwr1;
        pros::Motor* shooterPwr2;
        pros::Motor* shooterInd;

        // Shooter Sensors

        // Shooter Encoder Declarations
        pros::ADIEncoder* flywheelEncoder;

        bool indexerTrigger = false;

        std::uint32_t currTime;
        std::uint32_t prevTime;

    public:

        // Shooter Constructor
        Shooter();

        // Update the state of the Shooter
        void updateShooter(pros::Controller driver);

        void getRPM();

        void setMotorSpeed(int vel);
        void indexAll();
        

        enum ShooterStates getState();

    //private:

        // Set the state of the Shooter
        void setState(enum ShooterStates);

        void setTargetRPM(double RPM);

        double slewRPM(double request);

        bool isSettled();

        double calcShotRPM(double distance);

        double stepPID(double deltaTime);
};



typedef struct{
Shooter shooter;
} shooter_arg;


#endif