#include "Subsystems/IntakeRoller.hpp"
#include "pros/misc.h"


IntakeRoller::IntakeRoller()
{

    intakeMotor = new pros::Motor(16, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    //colourSensor = new pros::Optical(9);

    rollerPID = new PIDController(0, 0, 0);

}


void IntakeRoller::updateIntake(pros::Controller driver)
{

switch (mIntakeState)
{


    case OPERATOR_CONTROL:
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intakeMotor->move_velocity(600);
            intakeMotor->move_velocity(600);

        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intakeMotor->move_velocity(-600);
            intakeMotor->move_velocity(-600);
        }
        else
        {
            intakeMotor->move_velocity(0);
            intakeMotor->move_velocity(0);
        }

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







