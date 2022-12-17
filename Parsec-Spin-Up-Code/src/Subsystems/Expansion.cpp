#include "Subsystems/Expansion.hpp"

Expansion::Expansion()
{
    //!!!!!!!UPDATE THESE PORTS!!!!!!!!
    expansionPistonR = new pros::ADIDigitalOut('G');
   // expansionPistonL = new pros::ADIDigitalOut('B');
}

enum ExpansionStates{


};

void Expansion::updateExpansion(pros::Controller driver)
{
    switch (mExpansionState)
{


    case OPERATOR_CONTROL:

        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            expansionPistonR->set_value(true);
        }
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            expansionPistonR->set_value(false);
        }


        // else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
        // {
        //     expansionPistonL->set_value(true);
        // }
        
        break;
    

    }
}

Expansion::ExpansionStates Expansion::getState()
{

    return mExpansionState;

}

void Expansion::setState(ExpansionStates state)
{

    mExpansionState = state;

}




