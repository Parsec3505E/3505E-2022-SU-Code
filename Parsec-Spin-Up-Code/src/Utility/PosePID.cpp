#include "utility/PosePID.hpp"

PosePID::PosePID()
{
    xPID = new PIDController(0, 0, 0);
    yPID = new PIDController(0, 0, 0);
    thetaPID = new PIDController(0, 0, 0);

    targetPose = new Pose(Vector(0, 0), 0);
}


void PosePID::setTarget(Pose target)
{

    this->targetPose->setXComponent(target.getXComponent());
    this->targetPose->setYComponent(target.getYComponent());
    this->targetPose->setThetaComponent(target.getThetaComponent());

}

void PosePID::stepPID(Pose input)
{
    
}