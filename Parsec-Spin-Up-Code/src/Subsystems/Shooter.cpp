#include "Subsystems/Shooter.hpp"

Shooter::Shooter()
{
    //PORT 17 IS BROKEN FOR SOME REASON!!!
    shooterPwr = new pros::Motor(1, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
   
    shooterInd = new pros::ADIDigitalOut('B');
   

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
    indFlag = false;
    reloadDelay = 0;
    delayConst = 500;

}

void Shooter::updateShooter(pros::Controller driver)
{

    switch (mShooterState)
    {

    case CLOSED_LOOP:
        //driver.print(2, 2, "%.1f  %d    ", shooterPwr->get_actual_velocity(), targetVel);
        // Put closed loop code for the shooter here
        //driver.print(2, 2, "Hello2");
        
        break;

    case OPERATOR_CONTROL:
        if(indFlag && pros::millis()-reloadDelay>delayConst){
            reloadDelay = pros::millis();
            shooterInd->set_value(true);
            indFlag = false;
        }

        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            targetVel = 340;
            epsilon = 0.85;
        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            targetVel = 310;
            epsilon = 0.8;
        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            targetVel = 380;
            epsilon = 0.7;
        }
        else if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            // shooterPwr->move_velocity(0);
            // shooterPwr2->move_velocity(0);
            targetVel += 10;
            epsilon = 0.7;
        }
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            shooterPwr->move_velocity(targetVel);
            
            

        }
        else if(!driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            shooterPwr->move_velocity(0);
           
        }
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2)&&!indFlag && pros::millis()-reloadDelay>delayConst) 
        {
            reloadDelay = pros::millis();
            shooterInd->set_value(false);
            indFlag = true;
            //shooterInd->set_value(false);

            

            

        }

        driver.print(2, 2, "%.1f  %d    ", shooterPwr->get_actual_velocity(), targetVel);




       break;
    case DISABLED:
    
        shooterPwr->move_velocity(0);
        
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

void Shooter::setIndexerState(bool state)
{
    
    shooterInd->set_value(state);
}


void Shooter::getRPM()
{

}

void Shooter::setMotorSpeed(int vel)
{
    shooterPwr->move_velocity(vel);
    targetVel = vel;

}

void Shooter::indexAll()
{
    
    
  shooterInd->set_value(false);
  pros::delay(500);
  shooterInd->set_value(true);
  pros::delay(500);
 setMotorSpeed(445);
    pros::delay(3000);
  shooterInd->set_value(false);
  pros::delay(500);
  shooterInd->set_value(true);
  pros::delay(500);

   



}
void Shooter::indexAll2()
{
    
    
  shooterInd->set_value(false);
  pros::delay(500);
  shooterInd->set_value(true);
  pros::delay(600);
 
  shooterInd->set_value(false);
  pros::delay(500);
  shooterInd->set_value(true);
  pros::delay(500);

   



}
double Shooter::slewRPM(double request)
{

}

bool Shooter::isSettled()
{
    //pros::screen::print(pros::E_TEXT_MEDIUM, 7, "SHOOTER VEL    %f", shooterPwr->get_actual_velocity());
    pros::screen::print(pros::E_TEXT_MEDIUM, 9, "TIME SETTLED: %f", timeSettled);
    if(shooterPwr->get_actual_velocity() >= targetVel*epsilon){
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