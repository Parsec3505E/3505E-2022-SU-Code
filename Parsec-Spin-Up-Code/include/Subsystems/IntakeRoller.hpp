#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP


#include "pros/motors.hpp"

#include "pros/optical.hpp"
#include "Utility/PIDController.hpp"


class IntakeRoller
{

    private:

        // IntakeRoller States
        enum IntakeStates{};
        enum RollerStates{};

        PIDController* rollerPID;

        // IntakeRoller Motor Declarations
        pros::Motor* intakeRollerMotor;

        // Roller Sensors

        // Roller Colour Sensor Declarations
        pros::Optical* colourSensor;

    public:

        // IntakeRoller Constructor
        IntakeRoller();

        // Update the state of the Intake
        void updateIntake();

        // Update the state of the Roller
        void updateRoller();

        enum IntakeStates getIntakeState();
        enum RollerStates getRollerState();

    private:

        // Set the state of the intake
        void setIntakeState(enum IntakeStates);

        // Set the state of the roller
        void setRollerState(enum RollerStates);

        bool isSettled(double epsilon);




};

#endif