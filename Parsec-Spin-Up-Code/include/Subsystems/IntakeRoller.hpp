#ifndef INTAKE_HPP
#define INTAKE_HPP


#include "api.h"
#include "Utility/PIDController.hpp"


class IntakeRoller
{

    public:

        // IntakeRoller States
        enum IntakeStates{OPERATOR_CONTROL};
        // enum RollerStates{};

        PIDController* rollerPID;

        // IntakeRoller Motor Declarations
        pros::Motor* intakeMotor;


        // Roller Sensors

        // Roller Colour Sensor Declarations
        pros::Optical* colourSensor;

        IntakeStates mIntakeState;


        // IntakeRoller Constructor
        IntakeRoller();

        // Update the state of the Intake
        void updateIntake(pros::Controller driver);

        //Optical Methods
        void rollToColourAUTO();

        void rollToColourMANUAL(bool colour);

        double readColour();

        // Update the state of the Roller
        // void updateRoller();

        enum IntakeStates getIntakeState();
        // enum RollerStates getRollerState();

        // Set the state of the intake
        void setIntakeState(enum IntakeStates intakeState);

        // Set the state of the roller
        // void setRollerState(enum RollerStates);

        bool isSettled(double epsilon);
};

typedef struct{
IntakeRoller intake;
} intake_arg;

#endif