#include "Subsystems/Expansion.hpp"

Expansion::Expansion()
{
    expansionPiston = new pros::ADIDigitalOut('A');
}

enum ExpansionStates{


};

void Expansion::updateExpansion()
{

}

Expansion::ExpansionStates Expansion::getState()
{

    return mExpansionState;

}

void Expansion::setState(ExpansionStates state)
{

}




