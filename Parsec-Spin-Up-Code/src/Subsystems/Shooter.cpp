#include "Subsystems/Shooter.hpp"

Shooter::Shooter()
{

    shooterPwr1 = new pros::Motor(6, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    shooterPwr2 = new pros::Motor(7, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    shooterInd = new pros::Motor(8, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

    this->motorVelLimit = 0;
    this->motorAccLimit = 0;

    flywheelEncoder = new pros::ADIEncoder('G', 'H');

    rpmPID = new PIDController(0, 0, 0);

}

enum ShooterStates
{

};


void Shooter::updateShooter()
{

}

void Shooter::setState(enum ShooterStates)
{

}

enum Shooter::ShooterStates Shooter::getState()
{

}

void Shooter::setTargetRPM(double RPM)
{

}

double Shooter::slewRPM(double request)
{

}

bool Shooter::isSettled(double epsilon)
{

}

double Shooter::calcShotRPM(double distance)
{

}








