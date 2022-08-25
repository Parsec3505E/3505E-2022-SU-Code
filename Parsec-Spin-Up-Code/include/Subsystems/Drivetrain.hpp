#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"

#include <map>
#include "iostream"

#include <ctime>

#include "utility/Pose.hpp"
#include "utility/PosePID.hpp"

#include <chrono>

    
class Drivetrain
{

    public:
        //Drivetrain States
        enum DrivetrainStates{OPERATOR_CONTROL};

    private:

        const double DRIVE_RADIUS = 6.453;
        const double WHEEL_RADIUS = 2.0;

        Pose* robotPose;
        Pose* velocityPose;
        Pose* targetPose;

        DrivetrainStates mDriveState;

        PosePID* posePID;

        double motorVelLimit;
        double motorAccLimit;

        // Drivetrain Motor Declarations
        pros::Motor* rightFront;
        pros::Motor* leftFront;
        pros::Motor* rightBack;
        pros::Motor* leftBack;

        // Drivetrain Sensors

        // Odometry Encoder Declarations
        pros::ADIEncoder* forwardEncoder;
        pros::ADIEncoder* sideEncoder;

        // Drivetrain Gyro Declaration
        pros::IMU* gyro;

        pros::Controller* driver;

        std::map<std::string, double> requestedAcc;
        std::map<std::string, double> proposedMotorVelocities;
        std::map<std::string, double> proposedDeltaVelocities;
        std::map<std::string, double> finalVelocities;
        std::map<std::string, double> rotationVels;
        std::map<std::string, double> lastVels;

        const double MOTOR_MAX_RPM = 200.0;

        const double MOTOR_MAX_ACC = 200.0;

        std::uint32_t currTime;
        std::uint32_t prevTime;

    public:

        // Drivetrain Constructor
        Drivetrain();

        // Update the state of the Drivetrain
        void updateDrivetrain();

        // Set the state of the Drivetrain
        void setState(enum DrivetrainStates);

        // Get the state of the Drivetrain
        enum DrivetrainStates getState();

    private:

        Pose getRobotPose();

        void setRobotPose(Pose pose);

        void moveRobot(Pose* velocityPose);

        void setTargetPose(Pose targetPose);

        double getAcceleration(double prevRPM, double requestedRPM);

        std::map<std::string, double> slewPose(std::map<std::string, double> requestedRPM);

        void odometryStep();

        bool isSettled(double epsilon);

        Pose calcPoseToGoal();


};

#endif