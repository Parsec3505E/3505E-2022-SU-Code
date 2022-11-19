#include "Subsystems/Shooter.hpp"

Shooter::Shooter()
{

    shooterPwr1 = new pros::Motor(1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    shooterPwr2 = new pros::Motor(7, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
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
        rpmPID->setTarget(RPM);

        shooterPwr1->move_velocity(shooterPID(RPM));
        shooterPwr2->move_velocity(shooterPID(RPM));
        // Put closed loop code for the shooter here
        
        break;


    case OPERATOR_CONTROL:
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            shooterPwr1->move_velocity(1000);
            shooterPwr2->move_velocity(1000);

        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            shooterPwr1->move_velocity(700);
            shooterPwr2->move_velocity(700);

        }
        else
        {
            shooterPwr1->move_velocity(0);
            shooterPwr2->move_velocity(0);
        }


        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_A))
        {
            if(indexerTrigger == false)
            {
                shooterInd->move_absolute(-200, 60);
            }
            indexerTrigger = true;

        }
        else
        {
            shooterInd->move_absolute(0, 60);
            indexerTrigger = false;
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

double Shooter::shooterPID(double targetRPM)
{
 
    double currentRPM = rpmPID->stepPID(targetRPM, 0.0);
    rpmPID->isSettled();
    return currentRPM;
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








