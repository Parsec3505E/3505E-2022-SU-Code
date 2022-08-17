#include "Subsystems/Drivetrain.hpp"


Drivetrain::Drivetrain()
{
    // Construct the Pose/PosePID objects
    robotPose = new Pose(Vector(0, 0), 0);
    velocityPose = new Pose(Vector(0, 0), 0);

    posePID = new PosePID();

    // Construct the Motor objects
    rightFront = new pros::Motor(1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    rightFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    rightBack = new pros::Motor(2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    rightBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	leftFront = new pros::Motor(3, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    leftFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	leftBack = new pros::Motor(4, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    leftBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // Construct Odometry Encoder objects
    forwardEncoder = new pros::ADIEncoder('a', 'b');
    sideEncoder = new pros::ADIEncoder('C', 'D');

    // Construct Gyro object
    gyro = new pros::Imu(10);

    // Initialize all last motor vels to 0
    currentMotorRPM.insert({"rightFront", 0});
    currentMotorRPM.insert({"rightBack", 0});
    currentMotorRPM.insert({"leftFront", 0});
    currentMotorRPM.insert({"leftBack", 0});

    motorReq.insert({"rightFront", 0});
    motorReq.insert({"rightBack", 0});
    motorReq.insert({"leftFront", 0});
    motorReq.insert({"leftBack", 0});

    speedSlew.insert({"rightFront", 0});
    speedSlew.insert({"rightBack", 0});
    speedSlew.insert({"leftFront", 0});
    speedSlew.insert({"leftBack", 0});

    accReq.insert({"rightFront", 0});
    accReq.insert({"rightBack", 0});
    accReq.insert({"leftFront", 0});
    accReq.insert({"leftBack", 0});

    accSlew.insert({"rightFront", 0});
    accSlew.insert({"rightBack", 0});
    accSlew.insert({"leftFront", 0});
    accSlew.insert({"leftBack", 0});


}

enum DrivetrainStates{


};

void Drivetrain::updateDrivetrain()
{

}

Drivetrain::DrivetrainStates Drivetrain::getState()
{

    return mDriveState;

}

void Drivetrain::setState(DrivetrainStates state)
{

}

Pose Drivetrain::getRobotPose()
{

}

void Drivetrain::setRobotPose(Pose pose)
{

}

void Drivetrain::moveRobot(Pose velocityPose)
{

}

void Drivetrain::setTargetPose(Pose targetPose)
{

}


double * Drivetrain::getAcceleration(double prevRPM, double requestedRPM)
{

    static double returnVals[2];

    double deltaRPM = requestedRPM - prevRPM;
    double deltaTime = this->curTime - this->prevTime;

    double rate = deltaRPM / deltaTime;

    returnVals[0] = rate;
    returnVals[1] = deltaTime;

    return returnVals;

}

Pose Drivetrain::slewPose(Pose request)
{

    double greatestMotorRPM = 0;
    double motorRPMRatio = 0;

    double greatestMotorAcc = 0;
    double motorAccRatio = 0;

    double deltaTime = 0.0;


    this->velocityPose->setXComponent((request.getXComponent() * sin(M_PI_4)) + (request.getYComponent() * cos(M_PI_4)));
    this->velocityPose->setYComponent(((-request.getXComponent()) * cos(M_PI_4)) + (request.getYComponent() * sin(M_PI_4)));
    this->velocityPose->setThetaComponent(Drivetrain::DRIVE_RADIUS * request.getThetaComponent());

    // Speed Slew

    // Append the requested RPM to the map

    this->motorReq["rightFront"] = (this->velocityPose->getXComponent() - this->velocityPose->getThetaComponent()) * 60.0 / (Drivetrain::WHEEL_RADIUS * 2.0 * M_PI);
    this->motorReq["leftFront"] = (this->velocityPose->getYComponent() + this->velocityPose->getThetaComponent()) * 60.0 / (Drivetrain::WHEEL_RADIUS * 2 * M_PI);
    this->motorReq["rightBack"] = (this->velocityPose->getYComponent() - this->velocityPose->getThetaComponent()) * 60.0 / (Drivetrain::WHEEL_RADIUS * 2 * M_PI);
    this->motorReq["leftBack"] = (this->velocityPose->getXComponent() + this->velocityPose->getThetaComponent()) * 60.0 / (Drivetrain::WHEEL_RADIUS * 2 * M_PI);


    for(const auto &value : motorReq)
    {
        if(fabs(value.second) > greatestMotorRPM)
        {
            greatestMotorRPM = fabs(value.second);
        }
    }

    motorRPMRatio =  Drivetrain::MOTOR_MAX_RPM / greatestMotorRPM;

    this->speedSlew["rightFront"] = this->motorReq["rightFront"] * motorRPMRatio;
    this->speedSlew["leftFront"] = this->motorReq["leftFront"] * motorRPMRatio;
    this->speedSlew["rightBack"] = this->motorReq["rightBack"] * motorRPMRatio;
    this->speedSlew["leftBack"] = this->motorReq["leftBack"] * motorRPMRatio;

    // Acceleration Slew


    this->curTime = time(NULL);

    this->accReq["rightFront"] = getAcceleration(rightFront->get_actual_velocity(), this->speedSlew["rightFront"])[0];
    this->accReq["leftFront"] = getAcceleration(leftFront->get_actual_velocity(), this->speedSlew["leftFront"])[0];
    this->accReq["rightBack"] = getAcceleration(rightBack->get_actual_velocity(), this->speedSlew["rightBack"])[0];
    this->accReq["leftBack"] = getAcceleration(leftBack->get_actual_velocity(), this->speedSlew["leftBack"])[0];

    deltaTime = getAcceleration(rightFront->get_actual_velocity(), this->speedSlew["rightFront"])[1];

    this->prevTime = this->curTime;

    for(const auto &value : accReq)
    {
        if(fabs(value.second) > greatestMotorAcc)
        {
            greatestMotorAcc = fabs(value.second);
        }
    }

    motorAccRatio =  Drivetrain::MOTOR_MAX_ACC / greatestMotorAcc;

    this->accSlew["rightFront"] = this->accReq["rightFront"] * motorAccRatio;
    this->accSlew["leftFront"] = this->accReq["leftFront"] * motorAccRatio;
    this->accSlew["rightBack"] = this->accReq["rightBack"] * motorAccRatio;
    this->accSlew["leftBack"] = this->accReq["leftBack"] * motorAccRatio;


    // Set the motors

    rightFront->move_velocity((this->accSlew["rightFront"] * deltaTime) + rightFront->get_actual_velocity());
    leftFront->move_velocity((this->accSlew["leftFront"] * deltaTime) + leftFront->get_actual_velocity());
    rightBack->move_velocity((this->accSlew["rightBack"] * deltaTime) + rightBack->get_actual_velocity());
    leftBack->move_velocity((this->accSlew["leftBack"] * deltaTime) + leftBack->get_actual_velocity());


}



void Drivetrain::odometryStep()
{

}

bool Drivetrain::isSettled(double epsilon)
{

}

Pose Drivetrain::calcPoseToGoal()
{

}











