#include "utility/PIDController.hpp"

PIDController::PIDController(double kP, double kI, double kD)
{

    this->kP = kP;
    this->kI = kI;
    this->kD = kD;

    this->target = 0;
    this->error = 0;
    this->errorSum = 0;
    this->lastError = 0;

    this->maxOutput = 0;
    this->minOutput = 0;


}

void PIDController::setConstants(double kP, double kI, double kD)
{

    this->kP = kP;
    this->kI = kI;
    this->kD = kD;

}

void PIDController::setTarget(double target)
{
    this->target = target;
}

double PIDController::stepPID(double input)
{
    
}


