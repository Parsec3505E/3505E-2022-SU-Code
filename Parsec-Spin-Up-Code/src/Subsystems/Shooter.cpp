#include "Subsystems/Shooter.hpp"

Shooter::Shooter()
{

    shooterPwr1 = new pros::Motor(1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    shooterPwr2 = new pros::Motor(5, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    shooterInd = new pros::Motor(10, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

    motorVelLimit = 0;
    motorAccLimit = 0;

    flywheelEncoder = new pros::ADIEncoder('G', 'H');

    rpmPID = new PIDController(0, 0, 0);

}

void Shooter::updateShooter(pros::Controller driver)
{

    switch (mShooterState)
    {

    case CLOSED_LOOP:

        // Put closed loop code for the shooter here
        
        break;

    case OPERATOR_CONTROL:
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            shooterPwr1->move_velocity(1000);
            shooterPwr2->move_velocity(1000);

        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_A))
        {
            shooterInd->move_velocity(70);

        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_X))
        {
            shooterInd->move_velocity(-70);

        }
        else
        {
            shooterPwr1->move_velocity(0);
            shooterPwr2->move_velocity(0);
            shooterInd->move_velocity(0);
        }

        break;
    
    case DISABLED:

        shooterPwr1->move_velocity(0);
        shooterPwr2->move_velocity(0);

        break;

    }

}

void Shooter::setState(ShooterStates shooterState)
{
    mShooterState = shooterState;
}

enum Shooter::ShooterStates Shooter::getState()
{
    return mShooterState;
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








