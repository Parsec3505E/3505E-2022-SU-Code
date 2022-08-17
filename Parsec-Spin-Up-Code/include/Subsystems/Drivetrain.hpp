#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"

#include <map>
#include "iostream"

#include <ctime>

#include "utility/Pose.hpp"
#include "utility/PosePID.hpp"

    
class Drivetrain
{

    private:

        const double DRIVE_RADIUS = 6.453;
        const double WHEEL_RADIUS = 2.0;


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

        std::map<std::string, double> currentMotorRPM;
        std::map<std::string, double> motorReq;
        std::map<std::string, double> speedSlew;;
        std::map<std::string, double> accReq;;
        std::map<std::string, double> accSlew;;

        const double MOTOR_MAX_RPM = 280.0;

        const double MOTOR_MAX_ACC = 10.0;

        time_t curTime;
        time_t prevTime = time(NULL);

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

        double * getAcceleration(double prevRPM, double requestedRPM);

        Pose slewPose(Pose request);

        void odometryStep();

        bool isSettled(double epsilon);

        Pose calcPoseToGoal();


};

#endif