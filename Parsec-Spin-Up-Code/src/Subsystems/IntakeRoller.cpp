#include "Subsystems/IntakeRoller.hpp"


IntakeRoller::IntakeRoller()
{

    intakeRollerMotor = new pros::Motor(5, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    colourSensor = new pros::Optical(12);

    rollerPID = new PIDController(0, 0, 0);

}

enum IntakeStates
{

};

enum RollerStates
{

};

void IntakeRoller::updateIntake()
{

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







