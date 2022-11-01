#include "Subsystems/IntakeRoller.hpp"
#include "pros/misc.h"


IntakeRoller::IntakeRoller()
{

    intakeRollerMotor = new pros::Motor(16, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    //colourSensor = new pros::Optical(9);

    rollerPID = new PIDController(0, 0, 0);

    this->intake_state = false;

}

enum IntakeStates
{

};

enum RollerStates
{

};

void IntakeRoller::updateIntake(pros::Controller* driver)
{

    switch(mIntakeState)
    {
        // The Operator Control state that allows the driver to have open loop control over the drivetrain

        case OPERATOR_CONTROL:

            if(driver->get_digital(pros::E_CONTROLLER_DIGITAL_L1))
            {
                intakeRollerMotor->move_velocity(600);
            }
            else if(driver->get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            {
                intakeRollerMotor->move_velocity(-600);
            }
            else
            {
                intakeRollerMotor->move_voltage(0);
            }

            break;
    }

}

void IntakeRoller::updateRoller()
{

}


void IntakeRoller::setIntakeState(enum IntakeStates)
{

}

void IntakeRoller::setRollerState(enum RollerStates)
{

}

enum IntakeRoller::IntakeStates IntakeRoller::getIntakeState()
{

}

enum IntakeRoller::RollerStates IntakeRoller::getRollerState()
{

}

bool IntakeRoller::isSettled(double epsilon)
{

}







