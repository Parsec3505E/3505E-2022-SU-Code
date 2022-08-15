#include "utility/Pose.hpp"

Pose::Pose(Vector vector, double posTheta)
{

    this->x = vector.getXComponent();
    this->y = vector.getYcomponent();
    this->theta = posTheta;

}

void Pose::setXComponent(double x)
{
    this->x = x;
}

void Pose::setYComponent(double y)
{
    this->y = y;
}

void Pose::setThetaComponent(double theta)
{
    this->theta = theta;
}

double Pose::getXComponent()
{
    return this->x;
}

double Pose::getYComponent()
{
    return this->y;
}

double Pose::getThetaComponent()
{
    return this->theta;
}