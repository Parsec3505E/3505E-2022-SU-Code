#include "Subsystems/Expansion.hpp"

Expansion::Expansion()
{
    //!!!!!!!UPDATE THESE PORTS!!!!!!!!
    expansionPistonR = new pros::ADIDigitalOut('E');
}

enum ExpansionStates{


};

void Expansion::updateExpansion(pros::Controller driver)
{
    switch (mExpansionState)
{


    case OPERATOR_CONTROL:
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            expansionPistonR->set_value(true);

        }
        
        break;
    

    }
}

Expansion::ExpansionStates Expansion::getState()
{

    return mExpansionState;

}

void Expansion::expand()
{

    expansionPistonR->set_value(true);

}

void Expansion::setState(ExpansionStates state)
{
    mExpansionState = state;
}




