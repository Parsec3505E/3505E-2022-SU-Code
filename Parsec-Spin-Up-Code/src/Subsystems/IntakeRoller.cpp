#include "Subsystems/IntakeRoller.hpp"
#include "pros/misc.h"


IntakeRoller::IntakeRoller()
{

    intakeRollerMotor = new pros::Motor(5, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    colourSensor = new pros::Optical(12);

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

            if(driver->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) && this->intake_state)
            {
                intakeRollerMotor->move_voltage(90);
                this->intake_state = true;
            }
            else if(driver->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && this->intake_state)
            {
                intakeRollerMotor->move_voltage(127);
                this->intake_state = true;
            }
            else if(driver->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
            {
                intakeRollerMotor->move_voltage(0);
                this->intake_state = true;
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







