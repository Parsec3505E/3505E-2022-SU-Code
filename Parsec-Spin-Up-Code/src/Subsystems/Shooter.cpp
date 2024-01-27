#include "Subsystems/Shooter.hpp"

Shooter::Shooter()
{
    //PORT 17 IS BROKEN FOR SOME REASON!!!
    shooterPwr1 = new pros::Motor(5, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
    shooterPwr2 = new pros::Motor(14, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
    shooterInd = new pros::Motor(11, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

    motorVelLimit = 0;
    motorAccLimit = 0;

    flywheelEncoder = new pros::ADIEncoder('G', 'H');

    rpmPID = new PIDController(0, 0, 0);
    //targetRPM = 0;
    
    targetVel = 0;
    beenSettled = false;
    timeSettled = pros::millis();

    minSettledTime = 200;
    epsilon = 0.8;

}

void Shooter::updateShooter(pros::Controller driver)
{

    switch (mShooterState)
    {

    case CLOSED_LOOP:
        //driver.print(2, 2, "%.1f  %d    ", shooterPwr1->get_actual_velocity(), targetVel);
        // Put closed loop code for the shooter here
        //driver.print(2, 2, "Hello2");
        
        break;

    case OPERATOR_CONTROL:
        
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            targetVel = 290;
            epsilon = 0.85;
        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            targetVel = 300;
            epsilon = 0.8;
        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            targetVel = 380;
            epsilon = 0.7;
        }
        else if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            // shooterPwr1->move_velocity(0);
            // shooterPwr2->move_velocity(0);
            targetVel += 10;
            epsilon = 0.7;
        }
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            shooterPwr1->move_velocity(targetVel);
            shooterPwr2->move_velocity(targetVel);
            

        }
        else if(!driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            shooterPwr1->move_velocity(0);
            shooterPwr2->move_velocity(0);
            
        }
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && isSettled() ) 
        {
                        
            shooterInd->move_absolute(360, 600);
            pros::delay(500);
            shooterInd->set_zero_position(0.0);
            while(!isSettled()){}
            shooterInd->move_absolute(360, 600);
            pros::delay(500);
            shooterInd->set_zero_position(0.0);
            while(!isSettled()){}
            shooterInd->move_absolute(360, 600);
            pros::delay(500);
            shooterInd->set_zero_position(0.0);
            

            // pros::delay(500);
            // shooterInd->move_absolute(355, 600);
            // shooterInd->move_relative(-355, 600);
            // while(shooterInd->get_position() <= -360){}
            // shooterInd->move_relative(-355, 600);
        //    if(!indexerTrigger)
        //     {

        //         if(shooterInd->get_position() <= -360){
        //             indexerTrigger = true;
        //         }
                
        //     }
            // else{
            //     shooterInd->move_absolute(0, 95);
            //     if(shooterInd->get_position() >= -5){
            //         indexerTrigger = false;
            //     }
                
            // }

        }

        driver.print(2, 2, "%.1f  %d    ", shooterPwr1->get_actual_velocity(), targetVel);




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
    targetVel = RPM;

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

    epsilon = 0.8;
    shooterInd->move_absolute(360, 600);
    pros::delay(500);
    shooterInd->set_zero_position(0.0);
    while(!isSettled()){}
    shooterInd->move_absolute(360, 600);
    pros::delay(500);
    shooterInd->set_zero_position(0.0);
  
    //shooterInd->move_velocity(0);



}

double Shooter::slewRPM(double request)
{

}

bool Shooter::isSettled()
{
    //pros::screen::print(pros::E_TEXT_MEDIUM, 7, "SHOOTER VEL    %f", shooterPwr1->get_actual_velocity());
    pros::screen::print(pros::E_TEXT_MEDIUM, 9, "TIME SETTLED: %f", timeSettled);
    if(shooterPwr1->get_actual_velocity() >= targetVel*epsilon){
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