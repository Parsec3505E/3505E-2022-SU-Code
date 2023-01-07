#include "Subsystems/Shooter.hpp"

Shooter::Shooter()
{
    //PORT 17 IS BROKEN FOR SOME REASON!!!
    shooterPwr1 = new pros::Motor(5, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
    shooterPwr2 = new pros::Motor(14, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
    shooterInd = new pros::Motor(13, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

    motorVelLimit = 0;
    motorAccLimit = 0;

    flywheelEncoder = new pros::ADIEncoder('G', 'H');

    rpmPID = new PIDController(0, 0, 0);
    //targetRPM = 0;
    
    targetVel = 0;
    beenSettled = false;
    timeSettled = pros::millis();

    minSettledTime = 100;
    epsilon = 0;

}

void Shooter::updateShooter(pros::Controller driver)
{

    switch (mShooterState)
    {

    case CLOSED_LOOP:

        // Put closed loop code for the shooter here
        //driver.print(2, 2, "Hello2");
        
        break;

    case OPERATOR_CONTROL:
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            targetVel += 10;
            epsilon += 1;
            

        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            //targetVel = 300;
            targetVel -= 10;
            epsilon -= 1;
        }
        // else
        // {
        //     targetVel = 0;
        // }

        shooterPwr1->move_velocity(targetVel);
        shooterPwr2->move_velocity(targetVel);
        driver.print(2, 2, "%.1f  %d    ", shooterPwr1->get_actual_velocity(), targetVel);

        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_A) && isSettled()) 
        {

            if(!indexerTrigger)
            {
                shooterInd->move_absolute(-165, 95);
                if(shooterInd->get_position() <= -160){
                    indexerTrigger = true;
                }
                
            }
            else{
                shooterInd->move_absolute(0, 95);
                if(shooterInd->get_position() >= -5){
                    indexerTrigger = false;
                }
                
            }
            


        }
        // else
        // {
        //     shooterInd->move_absolute(0, 95);
        //     indexerTrigger = false;
        // }

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

void Shooter::getRPM()
{

}

void Shooter::setMotorSpeed(int vel)
{
    shooterPwr1->move_velocity(vel);
    shooterPwr2->move_velocity(vel);
    targetVel = vel;

}

void Shooter::indexAll()
{

    shooterInd->move_absolute(-165, 95);
    epsilon = 150;
    while(!isSettled()){}
    shooterInd->move_absolute(0, -95);
    while(shooterInd->get_position() >= -100){}
    //shooterInd->move_velocity(0);



}

double Shooter::slewRPM(double request)
{

}

bool Shooter::isSettled()
{
    
    if(shooterPwr1->get_actual_velocity() >= targetVel*0.8){
        if(!beenSettled){
            beenSettled = true;
            timeSettled = pros::millis();
        }
        return (pros::millis()-timeSettled)>minSettledTime;
    }
    else{
        beenSettled = false;
        return false;
    }
    
    
    

}

double Shooter::calcShotRPM(double distance)
{

}








