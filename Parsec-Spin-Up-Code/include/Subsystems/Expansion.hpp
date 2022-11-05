#ifndef EXPANSION_HPP
#define EXPANSION_HPP


#include "api.h"



class Expansion
{
    public:
        enum ExpansionStates{CLOSED_LOOP, OPERATOR_CONTROL, DISABLED};
    private:

        // Expansion States
        

        ExpansionStates mExpansionState;

        // Expansion Motor Declarations
        pros::ADIDigitalOut* expansionPiston;

    public:
        // Expansion Constructor
        Expansion();

        // Update the state of the Expansion
        void updateExpansion(pros::Controller driver);

        enum ExpansionStates getState();

    //private:

        void setExpansion(enum ExpansionStates);




};

#endif