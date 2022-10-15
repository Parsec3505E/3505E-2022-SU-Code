#include "utility/PosePID.hpp"

PosePID::PosePID()
{
    this->xPID = new PIDController(0.1, 0.0, 0.0);
    this->yPID = new PIDController(0.1, 0.0, 0.0);
    this->thetaPID = new PIDController(-0.2, 0.0, -0.1);

    xPID->setEpsilon(2.0);
    yPID->setEpsilon(2.0);
    thetaPID->setEpsilon(0.0873);

    targetPose = new Pose(Vector(0.0, 0.0), 0.0);
    outputPose = new Pose(Vector(0.0, 0.0), 0.0);

}


void PosePID::setTarget(Pose* target)
{

    targetPose->setXComponent(target->getXComponent());
    targetPose->setYComponent(target->getYComponent());
    targetPose->setThetaComponent(target->getThetaComponent());

}

double PosePID::getTarget()
{

    return this->targetPose->getThetaComponent();

}


Pose* PosePID::stepPID(Pose* input, double deltaTime)
{

    xPID->setTarget(this->targetPose->getXComponent());
    double xOutput = xPID->stepPID(input->getXComponent(), deltaTime) * 2;
    outputPose->setXComponent(xOutput);
    xPID->isSettled();

    yPID->setTarget(this->targetPose->getYComponent());
    double yOutput = yPID->stepPID(input->getYComponent(), deltaTime) * 2;
    outputPose->setYComponent(yOutput);
    yPID->isSettled();

    thetaPID->setTarget(targetPose->getThetaComponent());
    double thetaOutput = thetaPID->stepPID(input->getThetaComponent(), deltaTime);
    outputPose->setThetaComponent(thetaOutput);
    thetaPID->isSettled();

    // pros::screen::print(pros::E_TEXT_MEDIUM, 4, "x: %f", xOutput);
    // pros::screen::print(pros::E_TEXT_MEDIUM, 6, "y: %f", yOutput);
    // pros::screen::print(pros::E_TEXT_MEDIUM, 8, "theta: %f", thetaOutput);

    return outputPose;
    
}

bool PosePID::isSettled()
{
    bool settled;

    if(thetaPID->isSettled() && xPID->isSettled() && yPID->isSettled())
    {
        settled = true;
    }
    else
    {
        settled = false;
    }

    return settled;
}
