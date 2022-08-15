#include "Subsystems/Drivetrain.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"


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
    lastMotorVels.insert({"rightFront", 0});
    lastMotorVels.insert({"rightBack", 0});
    lastMotorVels.insert({"leftFront", 0});
    lastMotorVels.insert({"leftBack", 0});

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

Pose Drivetrain::slewPose(Pose required)
{

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







