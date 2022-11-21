#include "utility/PosePID.hpp"

PosePID::PosePID()
{
    this->xPID = new PIDController(0.9, 0.0, 0.0);
    this->yPID = new PIDController(0.9, 0.0, 0.0);
    this->thetaPID = new PIDController(-0.1, 0.0, 0.0);

    this->xPID->setEpsilon(1.5);
    this->yPID->setEpsilon(1.5);
    this->thetaPID->setEpsilon(0.14);

    this->targetPose = new Pose(Vector(0.0, 0.0), 0.0);
    this->outputPose = new Pose(Vector(0.0, 0.0), 0.0);

}

void PosePID::setXConstants(double kP, double kI, double kD)
{
    xPID->setConstants(kP, kI, kD);
}

void PosePID::setYConstants(double kP, double kI, double kD)
{
    yPID->setConstants(kP, kI, kD);
}

void PosePID::setThetaConstants(double kP, double kI, double kD)
{
    thetaPID->setConstants(kP, kI, kD);
}


void PosePID::setTarget(Pose* target)
{

    this->targetPose->setXComponent(target->getXComponent());
    this->targetPose->setYComponent(target->getYComponent());
    this->targetPose->setThetaComponent(target->getThetaComponent());

}

double PosePID::getTarget()
{

    return this->targetPose->getThetaComponent();

}


Pose* PosePID::stepPID(Pose* input, double deltaTime)
{
    this->xPID->setTarget(this->targetPose->getXComponent());
    double xOutput = this->xPID->stepPID(input->getXComponent(), deltaTime);
    this->outputPose->setXComponent(xOutput);
    this->xPID->isSettled();

    this->yPID->setTarget(this->targetPose->getYComponent());
    double yOutput = this->yPID->stepPID(input->getYComponent(), deltaTime);
    this->outputPose->setYComponent(yOutput);
    this->yPID->isSettled();

    this->thetaPID->setTarget(this->targetPose->getThetaComponent());
    double thetaOutput = this->thetaPID->stepPID(input->getThetaComponent(), deltaTime);
    this->outputPose->setThetaComponent(thetaOutput);
    this->thetaPID->isSettled();

    // pros::screen::print(pros::E_TEXT_MEDIUM, 4, "x: %f", xOutput);
    // pros::screen::print(pros::E_TEXT_MEDIUM, 6, "y: %f", yOutput);
    // pros::screen::print(pros::E_TEXT_MEDIUM, 8, "theta: %f", thetaOutput);

    return outputPose;
    
}

bool PosePID::isSettled()
{
    bool settled;

    if(this->thetaPID->isSettled() && this->xPID->isSettled() && this->yPID->isSettled())
    {
        settled = true;
    }
    else
    {
        settled = false;
    }

    return settled;
}
