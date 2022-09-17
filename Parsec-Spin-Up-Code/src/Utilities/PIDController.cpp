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

    this->integralWindUp = 10;

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

void PIDController::setEpsilon(double epsilon)
{
    this->epsilon = epsilon;
}

double PIDController::stepPID(double input, double deltaTime)
{
    this->error = this->target - input;

    if (fabs(error) > this->integralWindUp)
    {
        this->integral = 0;
    }
    else
    {
        this->integral = (this->integral + this->error) * deltaTime;
    }

    this->deriative = (this->error - this->prevError) / deltaTime;

    double output = (this->kP * this->error) + (this->kI * this->integral) + (this->kD * this->deriative);

    return output;
}

bool PIDController::isSettled()
{
    bool isSettled;

    if (this->error > -this->epsilon && this->error < this->epsilon)
    {
        isSettled = true;
    }
    else
    {
        isSettled = false;
    }

    return isSettled;
}
