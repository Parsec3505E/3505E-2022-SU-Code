#ifndef EXPANSION_HPP
#define EXPANSION_HPP


#include "api.h"



class Expansion
{

    public:

        // Expansion States
        enum ExpansionStates{OPERATOR_CONTROL};

        ExpansionStates mExpansionState;

        // Expansion Piston Declarations
        pros::ADIDigitalOut* expansionPistonR;
        pros::ADIDigitalOut* expansionPistonL;

    
    
        // Expansion Constructor
        Expansion();

        // Update the state of the Expansion
        void updateExpansion(pros::Controller driver);

        enum ExpansionStates getState();


        void setState(enum ExpansionStates);




};

#endif