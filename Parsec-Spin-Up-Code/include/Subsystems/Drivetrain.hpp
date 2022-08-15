#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"

#include <map>
#include "iostream"

#include "utility/Pose.hpp"
#include "utility/PosePID.hpp"

class Drivetrain
{

    private:

        Pose* robotPose;
        Pose* velocityPose;

        //Drivetrain States
        enum DrivetrainStates{};

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

        std::map<std::string, double> lastMotorVels;

    public:

        // Drivetrain Constructor
        Drivetrain();

        // Update the state of the Drivetrain
        void updateDrivetrain();

        // Get the state of the Drivetrain
        enum DrivetrainStates getState();

    private:

        // Set the state of the Drivetrain
        void setState(enum DrivetrainStates);

        Pose getRobotPose();

        void setRobotPose(Pose pose);

        void moveRobot(Pose velocityPose);

        void setTargetPose(Pose targetPose);

        Pose slewPose(Pose request);

        void odometryStep();

        bool isSettled(double epsilon);

        Pose calcPoseToGoal();


};

#endif