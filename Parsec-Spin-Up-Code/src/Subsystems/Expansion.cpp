#include "Subsystems/Expansion.hpp"

Expansion::Expansion()
{
    expansionPiston = new pros::ADIDigitalOut('A');
}

enum ExpansionStates{


};

void Expansion::updateExpansion(pros::Controller driver)
{
     switch (mExpansionState)
    {

    case CLOSED_LOOP:

        // Put closed loop code for the shooter here
        
        break;

    case OPERATOR_CONTROL:
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            

        }
        
        break;
    
    case DISABLED:

       

        break;

    }

}

void Expansion::setExpansion(ExpansionStates expansionState)
{
    mExpansionState = expansionState;
}

enum Expansion::ExpansionStates Expansion::getState()
{

    return mExpansionState;

}






