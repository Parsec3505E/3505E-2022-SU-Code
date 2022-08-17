#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP


#include "api.h"



class Expansion
{

    private:

        // Expansion States
        enum ExpansionStates{};

        ExpansionStates mExpansionState;

        // Expansion Motor Declarations
        pros::ADIDigitalOut* expansionPiston;

    public:
        // Expansion Constructor
        Expansion();

        // Update the state of the Expansion
        void updateExpansion();

        enum ExpansionStates getState();

    private:

        void setState(enum ExpansionStates);




};

#endif