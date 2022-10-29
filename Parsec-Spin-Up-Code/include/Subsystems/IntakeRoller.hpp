#ifndef INTAKE_HPP
#define INTAKE_HPP


#include "api.h"
#include "Utility/PIDController.hpp"


class IntakeRoller
{

    public:

        // IntakeRoller States
        enum IntakeStates{OPERATOR_CONTROL};
        enum RollerStates{};

        PIDController* rollerPID;

        // IntakeRoller Motor Declarations
        pros::Motor* intakeRollerMotor;

        // Roller Sensors

        // Roller Colour Sensor Declarations
        pros::Optical* colourSensor;

        bool intake_state;

        IntakeStates mIntakeState;


        // IntakeRoller Constructor
        IntakeRoller();

        // Update the state of the Intake
        void updateIntake(pros::Controller* driver);

        // Update the state of the Roller
        void updateRoller();

        enum IntakeStates getIntakeState();
        enum RollerStates getRollerState();

        // Set the state of the intake
        void setIntakeState(enum IntakeStates);

        // Set the state of the roller
        void setRollerState(enum RollerStates);

        bool isSettled(double epsilon);
};
#endif