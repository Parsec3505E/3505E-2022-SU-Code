#include "Subsystems/Expansion.hpp"

Expansion::Expansion()
{
    //!!!!!!!UPDATE THESE PORTS!!!!!!!!
    expansionPistonR = new pros::ADIDigitalOut('H');
    expansionPistonL = new pros::ADIDigitalOut('B');
}

enum ExpansionStates{


};

void Expansion::updateExpansion(pros::Controller driver)
{
    switch (mExpansionState)
{


    case OPERATOR_CONTROL:
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
        {
            //according to amnesiac toad there
            //needs to be delay between both sides expanding ! ! ! !
            expansionPistonR->set_value(true);
            pros::delay(1000);
            expansionPistonL->set_value(true);


        }
        
        break;
    

    }
}

Expansion::ExpansionStates Expansion::getState()
{

    return mExpansionState;

}

void Expansion::setState(ExpansionStates state)
{

}




