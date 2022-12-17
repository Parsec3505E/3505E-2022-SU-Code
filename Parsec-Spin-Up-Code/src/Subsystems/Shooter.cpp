#include "Subsystems/Shooter.hpp"

Shooter::Shooter()
{

    shooterPwr1 = new pros::Motor(1, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    shooterPwr2 = new pros::Motor(5, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    shooterInd = new pros::Motor(19, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

    motorVelLimit = 0;
    motorAccLimit = 0;

    RPMTarget = 0;

    flywheelEncoder = new pros::ADIEncoder('G', 'H');

    rpmPID = new PIDController(0, 0, 0);

}

void Shooter::updateShooter(pros::Controller driver)
{

    switch (mShooterState)
    {

    case CLOSED_LOOP:

        // Put closed loop code for the shooter here

        shooterPwr1->move_velocity(this->RPMTarget);
        shooterPwr2->move_velocity(this->RPMTarget);
        
        break;

    case OPERATOR_CONTROL:
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            shooterPwr1->move_velocity(600);
            shooterPwr2->move_velocity(600);

        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            // shooterPwr1->move_velocity(122);
            // shooterPwr2->move_velocity(122);
            shooterPwr1->move_voltage(9000);
            shooterPwr2->move_voltage(9000);

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
            shooterInd->move_absolute(0, 85);
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
    this->RPMTarget = RPM;
}

void Shooter::shoot(double RPM)
{
    // shooterPwr1->move_velocity(RPM);
    // shooterPwr2->move_velocity(RPM);
    pros::delay(3000);
    shooterInd->move_absolute(-100, 100);
    pros::delay(1000);
    shooterInd->move_absolute(0, 100);
    pros::delay(1000);

    // shooterPwr1->move_velocity(0);
    // shooterPwr2->move_velocity(0);


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








