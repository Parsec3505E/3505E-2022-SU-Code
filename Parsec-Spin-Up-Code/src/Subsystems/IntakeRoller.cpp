#include "Subsystems/IntakeRoller.hpp"
#include "pros/misc.h"


IntakeRoller::IntakeRoller()
{
    //PORT 17 IS BROKEN FOR SOME REASON!!!
    intakeMotor = new pros::Motor(16, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    colourSensor = new pros::Optical(11);

    rollerPID = new PIDController(0, 0, 0);

}


void IntakeRoller::updateIntake(pros::Controller driver)
{

switch (mIntakeState)
{


    case OPERATOR_CONTROL:
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intakeMotor->move_velocity(600);

        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intakeMotor->move_velocity(-600);
        }
        else
        {
            intakeMotor->move_velocity(0);
            intakeMotor->move_velocity(0);
        }
        readColour();

        break;
    

    }

}

// void IntakeRoller::updateRoller()
// {

// }


// void IntakeRoller::setRollerState(enum RollerStates)
// {

// }

void IntakeRoller::setIntakeState(IntakeStates intakeState)
{
    mIntakeState = intakeState;
}
void IntakeRoller::rollToColour(bool colour){
    //TRUE IS RED
    if(colour==true){
        while(colourSensor->get_hue()>200.0){
            intakeMotor->move_velocity(400);
        }
        while(colourSensor->get_hue()<10.0){
            intakeMotor->move_velocity(400);
        }
        
        intakeMotor->move_velocity(0);
        //intakeMotor->move_relative(20, -200);

    }
    else{
        while(colourSensor->get_hue()<10.0){
            intakeMotor->move_velocity(400);
        }
        while(colourSensor->get_hue()>200.0){
            intakeMotor->move_velocity(400);
        }
        intakeMotor->move_velocity(0);
        //intakeMotor->move_relative(20, -200);
    }
    

}
double IntakeRoller::readColour()
{
    // if(colourSensor->get_hue()<10.0){
    //     pros::screen::print(pros::E_TEXT_MEDIUM, 5, "RED %f      ", colourSensor->get_hue());
    // }
    // else if(colourSensor->get_hue()>200.0){
    //     pros::screen::print(pros::E_TEXT_MEDIUM, 5, "BLUE %f      ", colourSensor->get_hue());
    // }

    // pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Hue value: %f      ", colourSensor->get_hue());
    colourSensor->set_led_pwm(50);
    return colourSensor->get_hue();
}

enum IntakeRoller::IntakeStates IntakeRoller::getIntakeState()
{
    return mIntakeState;
}

// enum IntakeRoller::RollerStates IntakeRoller::getRollerState()
// {


// }

bool IntakeRoller::isSettled(double epsilon)
{

}







