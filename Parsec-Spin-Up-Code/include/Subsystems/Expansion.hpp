#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP


#include "api.h"



class Expansion
{

    private:

        // Expansion States
        enum ExpansionStates{OPERATOR_CONTROL};

        ExpansionStates mExpansionState;

        // Expansion Piston Declarations
        pros::ADIDigitalOut* expansionPistonR;
        pros::ADIDigitalOut* expansionPistonL;

    public:
    
        // Expansion Constructor
        Expansion();

        // Update the state of the Expansion
        void updateExpansion(pros::Controller driver);

        enum ExpansionStates getState();

    private:

        void setState(enum ExpansionStates);




};

#endif