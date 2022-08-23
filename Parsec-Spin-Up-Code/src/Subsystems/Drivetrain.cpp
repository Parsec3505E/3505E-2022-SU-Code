#include "Subsystems/Drivetrain.hpp"
#include "Utility/Pose.hpp"


Drivetrain::Drivetrain()
{
    // Construct the Pose/PosePID objects
    robotPose = new Pose(Vector(0, 0), 0);
    velocityPose = new Pose(Vector(0, 0), 0);

    posePID = new PosePID();

    // Construct the Motor objects
    rightFront = new pros::Motor(5, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    rightFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    rightBack = new pros::Motor(1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    rightBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	leftFront = new pros::Motor(6, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    leftFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	leftBack = new pros::Motor(3, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    leftBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // Construct Odometry Encoder objects
    forwardEncoder = new pros::ADIEncoder('a', 'b');
    sideEncoder = new pros::ADIEncoder('C', 'D');

    // Construct Gyro object
    gyro = new pros::Imu(10);

    // Initialize all last motor vels to 0

    requestedAcc.insert({"rightFront", 0});
    requestedAcc.insert({"rightBack", 0});
    requestedAcc.insert({"leftFront", 0});
    requestedAcc.insert({"leftBack", 0});

    proposedDeltaVelocities.insert({"rightFront", 0});
    proposedDeltaVelocities.insert({"rightBack", 0});
    proposedDeltaVelocities.insert({"leftFront", 0});
    proposedDeltaVelocities.insert({"leftBack", 0});

    proposedMotorVelocities.insert({"rightFront", 0});
    proposedMotorVelocities.insert({"rightBack", 0});
    proposedMotorVelocities.insert({"leftFront", 0});
    proposedMotorVelocities.insert({"leftBack", 0});

    finalVelocities.insert({"rightFront", 0});
    finalVelocities.insert({"rightBack", 0});
    finalVelocities.insert({"leftFront", 0});
    finalVelocities.insert({"leftBack", 0});

    rotationVels.insert({"rightFront", 0});
    rotationVels.insert({"rightBack", 0});
    rotationVels.insert({"leftFront", 0});
    rotationVels.insert({"leftBack", 0});

    lastVels.insert({"rightFront", 0});
    lastVels.insert({"rightBack", 0});
    lastVels.insert({"leftFront", 0});
    lastVels.insert({"leftBack", 0});



}

enum DrivetrainStates{


};

void Drivetrain::updateDrivetrain()
{

    Pose* pose;
    pose = new Pose(Vector(10, 20), 45);
    
    this->currTime = pros::micros();

    moveRobot(pose);

    this->prevTime = this->currTime;


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

void Drivetrain::moveRobot(Pose* velocityPose)
{


    double a = (velocityPose->getXComponent() * cos(M_PI_4)) + (velocityPose->getYComponent() * sin(M_PI_4)) * 60.0 / (Drivetrain::WHEEL_RADIUS * 2 * M_PI);
    double b = (velocityPose->getXComponent() - sin(M_PI_4)) + (velocityPose->getYComponent() * cos(M_PI_4)) * 60.0 / (Drivetrain::WHEEL_RADIUS * 2 * M_PI);
    double c = (Drivetrain::DRIVE_RADIUS * velocityPose->getThetaComponent()) * 60.0 / (Drivetrain::WHEEL_RADIUS * 2 * M_PI);

    this->rotationVels["rightFront"] = b + c;
    this->rotationVels["leftront"] = a - c;
    this->rotationVels["rightBack"] = a + c;
    this->rotationVels["leftBack"] = b - c;

    rightFront->move_velocity(slewPose(rotationVels)["rightFront"]);
    leftFront->move_velocity(slewPose(rotationVels)["leftFront"]);
    rightBack->move_velocity(slewPose(rotationVels)["rightBack"]);
    leftBack->move_velocity(slewPose(rotationVels)["leftBack"]);

}

void Drivetrain::setTargetPose(Pose targetPose)
{

}


double Drivetrain::getAcceleration(double prevRPM, double requestedRPM)
{

    double deltaRPM = requestedRPM - prevRPM;
    double deltaTime = this->currTime - this->prevTime;

    double rate = deltaRPM / deltaTime;

    return rate;

}

std::map<std::string, double> Drivetrain::slewPose(std::map<std::string, double> requestedRPM)
{

    double greatestDeltaVelocity = 0;
    double motorVelocityRatio = 0;

    double greatestVelocityMagnitude = 0;

    double velCap = 0;

    // Get current acceleration from all four motors

    this->requestedAcc["rightFront"] = getAcceleration(this->lastVels["rightFront"], requestedRPM["rightFront"]);
    this->requestedAcc["leftront"] = getAcceleration(this->lastVels["leftFront"], requestedRPM["leftFront"]);
    this->requestedAcc["rightBack"] = getAcceleration(this->lastVels["rightBack"], requestedRPM["rightBack"]);
    this->requestedAcc["leftBack"] = getAcceleration(this->lastVels["leftBack"], requestedRPM["leftBack"]);

    greatestDeltaVelocity = this->MOTOR_MAX_ACC * (this->currTime - this->prevTime);


    // Get the delta velocities for each motor

    this->proposedDeltaVelocities["rightFront"] = this->requestedAcc["rightFront"] * (this->currTime - this->prevTime);
    this->proposedDeltaVelocities["leftFront"] = this->requestedAcc["leftFront"] * (this->currTime - this->prevTime);
    this->proposedDeltaVelocities["rightBack"] = this->requestedAcc["rightBack"] * (this->currTime - this->prevTime);
    this->proposedDeltaVelocities["leftBack"] = this->requestedAcc["leftBack"] * (this->currTime - this->prevTime);

    // Find the greatest delta velocity in the map

    this->proposedDeltaVelocities["rightFront"] = std::min(greatestDeltaVelocity, this->proposedDeltaVelocities["rightFront"]);
    this->proposedDeltaVelocities["leftFront"] = std::min(greatestDeltaVelocity, this->proposedDeltaVelocities["leftFront"]);
    this->proposedDeltaVelocities["rightBack"] =std::min(greatestDeltaVelocity, this->proposedDeltaVelocities["rightBack"]);
    this->proposedDeltaVelocities["leftBack"] = std::min(greatestDeltaVelocity, this->proposedDeltaVelocities["leftBack"]);
    

    // Get the velCap value used in the ratio


    motorVelocityRatio = velCap / greatestDeltaVelocity;


    // Assign the corresponding final velocity capped motor vals

    this->proposedMotorVelocities["rightFront"] = this->lastVels["rightFront"] + this->proposedDeltaVelocities["rightFront"];
    this->proposedMotorVelocities["leftFront"] = this->lastVels["leftFront"] + this->proposedDeltaVelocities["leftFront"];
    this->proposedMotorVelocities["rightBack"] = this->lastVels["rightBack"] + this->proposedDeltaVelocities["rightBack"];
    this->proposedMotorVelocities["leftBack"] = this->lastVels["leftBack"] + this->proposedDeltaVelocities["leftBack"];


    for(const auto &value : proposedMotorVelocities)
    {
        if(fabs(value.second) > greatestVelocityMagnitude)
        {
            greatestVelocityMagnitude = fabs(value.second);
        }
    }


    velCap = std::min(greatestVelocityMagnitude, this->MOTOR_MAX_RPM);


    motorVelocityRatio = velCap / greatestVelocityMagnitude;


    this->finalVelocities["rightFront"] = requestedRPM["rightFront"] * motorVelocityRatio;
    this->finalVelocities["leftFront"] = requestedRPM["leftFront"] * motorVelocityRatio;
    this->finalVelocities["rightBack"] = requestedRPM["rightBack"] * motorVelocityRatio;
    this->finalVelocities["leftBack"] = requestedRPM["leftBack"] * motorVelocityRatio;


     this->lastVels["rightFront"] = finalVelocities["rightFront"];
    this->lastVels["leftFront"] = finalVelocities["leftFront"];
    this->lastVels["rightBack"] = finalVelocities["rightBack"];
    this->lastVels["leftBack"] = finalVelocities["leftBack"];

    return finalVelocities;



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











