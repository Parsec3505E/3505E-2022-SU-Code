#include "Subsystems/Drivetrain.hpp"
#include "Utility/Pose.hpp"
#include "pros/misc.h"
#include "pros/screen.h"
#include <iostream>



Drivetrain::Drivetrain()
{
    // Construct the Pose/PosePID objects
    robotPose = new Pose(Vector(0, 0), 0);
    velocityPose = new Pose(Vector(0, 0), 0);
    targetPose = new Pose(Vector(0, 0), 0);

    posePID = new PosePID();

    // Construct the Motor objects
    rightFront = new pros::Motor(5, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    rightFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    rightBack = new pros::Motor(1, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
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

    driver = new pros::Controller(pros::E_CONTROLLER_MASTER);

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

    prevTime = pros::millis();
    
}

void Drivetrain::updateDrivetrain()
{

    // Finite State Machine (FSM)

    switch(mDriveState)
    {
        // The Operator Control state that allows the driver to have open loop control over the drivetrain

        case OPERATOR_CONTROL:

            // Setting the x, y and theta components to the joystick values

            this->targetPose->setXComponent(driver->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
            this->targetPose->setYComponent(driver->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
            this->targetPose->setThetaComponent(driver->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * -1);

            this->currTime = pros::millis();

            moveRobot(this->targetPose);

            this->prevTime = this->currTime;

    }

    // Pose* pose;
    // pose = new Pose(Vector(50, 30), 0);
    
    // this->currTime = pros::millis();
    //     pros::screen::print(pros::E_TEXT_MEDIUM, 10, "currTime: %d", this->currTime);

    //     pros::screen::print(pros::E_TEXT_MEDIUM, 1, "prevTime: %d", this->prevTime);


    // moveRobot(pose);

    // this->prevTime = this->currTime;


}

Drivetrain::DrivetrainStates Drivetrain::getState()
{
    return mDriveState;
}

void Drivetrain::setState(DrivetrainStates state)
{
    mDriveState = state;
}

Pose Drivetrain::getRobotPose()
{

}

void Drivetrain::setRobotPose(Pose pose)
{

}

void Drivetrain::moveRobot(Pose* velocityPose)
{

    // X Drive rotation matrix/math

    double a = ((velocityPose->getXComponent() * cos(M_PI_4)) + (velocityPose->getYComponent() * sin(M_PI_4))) * 60.0 / (Drivetrain::WHEEL_RADIUS * 2 * M_PI);
    double b = ((-velocityPose->getXComponent() * sin(M_PI_4)) + (velocityPose->getYComponent() * cos(M_PI_4))) * 60.0 / (Drivetrain::WHEEL_RADIUS * 2 * M_PI);
    double c = (Drivetrain::DRIVE_RADIUS * velocityPose->getThetaComponent()) * 60.0 / (Drivetrain::WHEEL_RADIUS * 2 * M_PI);


    this->rotationVels["rightFront"] = b + c;
    this->rotationVels["leftFront"] = a - c;
    this->rotationVels["rightBack"] = a + c;
    this->rotationVels["leftBack"] = b - c;

    // Slewing the rotationVels

    std::map<std::string, double> motorVels = slewPose(this->rotationVels);

    // Setting the motor slewed values to the physical motors

    rightFront->move_velocity(motorVels["rightFront"]);
    leftFront->move_velocity(motorVels["leftFront"]);
    rightBack->move_velocity(motorVels["rightBack"]);
    leftBack->move_velocity(motorVels["leftBack"]);
}

void Drivetrain::setTargetPose(Pose targetPose)
{

}


double Drivetrain::getAcceleration(double prevRPM, double requestedRPM)
{

    double deltaRPM = requestedRPM - prevRPM;
    double deltaTime = (this->currTime - this->prevTime) / 1000.0;

    double rate = deltaRPM / deltaTime;

    return rate;

}

std::map<std::string, double> Drivetrain::slewPose(std::map<std::string, double> requestedRPM)
{

    double greatestDeltaVelocity = 0;
    double motorVelocityRatio = 0;

    double greatestVelocityMagnitude = 0;

    double greatestRequestedRPM = 0;

    double velCap = 0;

    double deltaSec = (this->currTime - this->prevTime) / 1000.0;


    // Get current acceleration from all four motors

    this->requestedAcc["rightFront"] = getAcceleration(this->lastVels["rightFront"], requestedRPM["rightFront"]);
    this->requestedAcc["leftront"] = getAcceleration(this->lastVels["leftFront"], requestedRPM["leftFront"]);
    this->requestedAcc["rightBack"] = getAcceleration(this->lastVels["rightBack"], requestedRPM["rightBack"]);
    this->requestedAcc["leftBack"] = getAcceleration(this->lastVels["leftBack"], requestedRPM["leftBack"]);

    // Finding the greatest allowable delta velocity

    greatestDeltaVelocity = this->MOTOR_MAX_ACC * deltaSec;

    // Calculating the proposed delta velocities for each motor based on the requested acceleration and elapsed time

    this->proposedDeltaVelocities["rightFront"] = this->requestedAcc["rightFront"] * deltaSec;
    this->proposedDeltaVelocities["leftFront"] = this->requestedAcc["leftFront"] * deltaSec;
    this->proposedDeltaVelocities["rightBack"] = this->requestedAcc["rightBack"] * deltaSec;
    this->proposedDeltaVelocities["leftBack"] = this->requestedAcc["leftBack"] * deltaSec;

    // Setting the proposed delta velocities to the minimum of (greatest allowable delta velocity & current proposed delta velocities)

    this->proposedDeltaVelocities["rightFront"] = std::copysign(std::min(greatestDeltaVelocity, fabs(this->proposedDeltaVelocities["rightFront"])), this->proposedDeltaVelocities["rightFront"]);
    this->proposedDeltaVelocities["leftFront"] = std::copysign(std::min(greatestDeltaVelocity, fabs(this->proposedDeltaVelocities["leftFront"])), this->proposedDeltaVelocities["leftFront"]);
    this->proposedDeltaVelocities["rightBack"] = std::copysign(std::min(greatestDeltaVelocity, fabs(this->proposedDeltaVelocities["rightBack"])), this->proposedDeltaVelocities["rightBack"]);
    this->proposedDeltaVelocities["leftBack"] = std::copysign(std::min(greatestDeltaVelocity, fabs(this->proposedDeltaVelocities["leftBack"])), this->proposedDeltaVelocities["leftBack"]);
    

    // Calculating the velocitiy magnitudes based on the previous velocities from the last iteration of the loop and the proposed delta velocities

    this->proposedMotorVelocities["rightFront"] = this->lastVels["rightFront"] + this->proposedDeltaVelocities["rightFront"];
    this->proposedMotorVelocities["leftFront"] = this->lastVels["leftFront"] + this->proposedDeltaVelocities["leftFront"];
    this->proposedMotorVelocities["rightBack"] = this->lastVels["rightBack"] + this->proposedDeltaVelocities["rightBack"];
    this->proposedMotorVelocities["leftBack"] = this->lastVels["leftBack"] + this->proposedDeltaVelocities["leftBack"];


    // Finding the greatest velocity vector magnitude

    for(const auto &value : proposedMotorVelocities)
    {
        if(fabs(value.second) > greatestVelocityMagnitude)
        {
            greatestVelocityMagnitude = fabs(value.second);
        }
    }

    // Get the velCap value used in the ratio

    velCap = std::min(greatestVelocityMagnitude, this->MOTOR_MAX_RPM);


    // Find the greatest requested motor RPM

    for(const auto &it : requestedRPM)
    {
        if(fabs(it.second) > greatestRequestedRPM)
        {
            greatestRequestedRPM = fabs(it.second);
        }
    }

    // Calculate the velocity ratio to apply to each motor

    motorVelocityRatio = velCap / greatestRequestedRPM;

    // Apply the velocity ratio to the motors and get the final slewed velocities that we send to the motors

    this->finalVelocities["rightFront"] = requestedRPM["rightFront"] * motorVelocityRatio;
    this->finalVelocities["leftFront"] = requestedRPM["leftFront"] * motorVelocityRatio;
    this->finalVelocities["rightBack"] = requestedRPM["rightBack"] * motorVelocityRatio;
    this->finalVelocities["leftBack"] = requestedRPM["leftBack"] * motorVelocityRatio;

    // Continue to set the previous velocities to the final velocities each iteration of the loop

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
