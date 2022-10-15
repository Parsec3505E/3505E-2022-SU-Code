#include "utility/PIDController.hpp"

PIDController::PIDController(double kP, double kI, double kD)
{

    this->kP = kP;
    this->kI = kI;
    this->kD = kD;

    this->target = 0.0;
    this->error = 0.0;
    this->prevError = 0.0;
    this->errorSum = 0.0;
    this->lastError = 0.0;

    this->integralWindUp = 10.0;

    this->maxOutput = 0.0;
    this->minOutput = 0.0;
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
        this->integral = 0.0;
    }
    else
    {
        this->integral = (this->integral + this->error); // *
    }

    this->deriative = (this->error - this->prevError); // / 

    //pros::screen::print(pros::E_TEXT_MEDIUM, 7, "d term: %f", this->deriative);

    double output = (this->kP * this->error) + (this->kI * this->integral) + (this->kD * this->deriative);

    this->prevError = this->error;

    return output;
}

bool PIDController::isSettled()
{
    bool isSettled;

    //pros::screen::print(pros::E_TEXT_MEDIUM, 7, "epsilon: %f", this->epsilon);

    if (fabs(this->error) < this->epsilon)
    {
        isSettled = true;
    }
    else
    {
        isSettled = false;
    }

    return isSettled;
}
