#include "utility/PosePID.hpp"

PosePID::PosePID()
{
    xPID = new PIDController(0, 0, 0);
    yPID = new PIDController(0, 0, 0);
    thetaPID = new PIDController(0, 0, 0);

    targetPose = new Pose(Vector(0, 0), 0);
    outputPose = new Pose(Vector(0, 0), 0);
}


void PosePID::setTarget(Pose* target)
{

    this->targetPose->setXComponent(target->getXComponent());
    this->targetPose->setYComponent(target->getYComponent());
    this->targetPose->setThetaComponent(target->getThetaComponent());

}

Pose* PosePID::stepPID(Pose* input, double deltaTime)
{
    xPID->setTarget(this->targetPose->getXComponent());
    double xOutput = xPID->stepPID(input->getXComponent(), deltaTime);
    yPID->setTarget(this->targetPose->getYComponent());
    double yOutput = yPID->stepPID(input->getYComponent(), deltaTime);
    thetaPID->setTarget(this->targetPose->getThetaComponent());
    double thetaOutput = thetaPID->stepPID(input->getThetaComponent(), deltaTime);

    outputPose->setXComponent(xOutput);
    outputPose->setYComponent(yOutput);
    outputPose->setThetaComponent(thetaOutput);
    
}