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
        enum DrivetrainStates{OPERATOR_CONTROL, PID, BLANK, DEAD};

    private:

        const double DRIVE_RADIUS = 6.453;
        const double WHEEL_RADIUS = 2.0;

        Pose* robotPose;
        Pose* velocityPose;
        Pose* targetPose;

        Pose* previousPose; // Previous operator control pose
        Pose* outputPose; // Output pose of the PID



        DrivetrainStates mDriveState;

        PosePID* posePID;

        PIDController* driverXPID;
        PIDController* driverYPID;
        PIDController* driverThetaPID;

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

        std::map<std::string, double> requestedAcc;
        std::map<std::string, double> proposedMotorVelocities;
        std::map<std::string, double> proposedDeltaVelocities;
        std::map<std::string, double> finalVelocities;
        std::map<std::string, double> rotationVels;
        std::map<std::string, double> lastVels;

        const double MOTOR_MAX_RPM = 200.0;

        const double MOTOR_MAX_ACC = 120.0;

        std::uint32_t currTime;
        std::uint32_t prevTime;

        // ------------------------------- ODOMETRY VARS ------------------------------- 

        const double WHEEL_DIAMETER = 2.75;

        //Distances of tracking wheels from tracking center (INCHES)
        const double FORWARD_ENCODER_TRACK_RADIUS = 5.5225;
        //Might need to be negative
        const double SIDE_ENCODER_TRACK_RADIUS = 5.5225;

        //Calculated Values (every loop)
        //Angles (DEGREES) *NEEDS TO BE CONVERTED TO RADIANS FOR MATH*
        double forwardEncoderRaw = 0.0;
        double sideEncoderRaw = 0.0;

        double forwardEncoderPrevRaw = 0.0;
        double sideEncoderPrevRaw = 0.0;

        //Distances traveled by tracking wheels each loop (INCHES)
        double deltaDistForward = 0.0;
        double deltaDistSide = 0.0;

        //The current angle of the bot (RADIANS)
        double headingRaw = 0.0;
        //The previous angle of the bot (RADIANS)
        double prevHeadingRaw = 0.0;

        //The change in heading each loop (RADIANS)
        double deltaHeading = 0.0;

        //The changes in the X and Y positions (INCHES)
        /*These are calculated on a local basis each loop,
        then converted to global position changes */
        double deltaXLocal = 0.0;
        double deltaYLocal = 0.0;

        //The X and Y offsets converted from their local frame to global frame (INCHES)
        double deltaXGlobal = 0.0;
        double deltaYGlobal = 0.0;

        //The global position of the bot (INCHES)
        double xPoseGlobal = 0.0;
        double yPoseGlobal = 0.0;


    public:

        // Drivetrain Constructor
        Drivetrain();

        // Update the state of the Drivetrain
        void updateDrivetrain(pros::Controller &driver);

        // Set the state of the Drivetrain
        void setState(enum DrivetrainStates);

        // Get the state of the Drivetrain
        enum DrivetrainStates getState();

        void resetGyro();

        void drivePID(double x, double y, double heading);

        void driveToPoint(double x, double y, double heading, double xyepsilon, double thetaepsilon, double XP, double YP, double thetaP);
        
        void turnToPoint(double x, double y, double xyepsilon, double thetaepsilon, double XP, double YP, double thetaP);

        void turnToHeading(double heading, double xyepsilon, double thetaepsilon, double XP, double YP, double thetaP);

        void setPower(int rFront, int lFront, int rBack, int lBack);
        
        void setTicks(int vel, int rFront, int lFront, int rBack, int lBack);

        void driveSeconds(int ms, int rFront, int lFront, int rBack, int lBack);

        Pose* getRobotPose();

        void setRobotPose(Pose pose);

        bool isSettled();

        ~Drivetrain();

        void moveRobot(Pose* velocityPose);

    private:

        

        void setTargetPose(Pose targetPose);

        double getAcceleration(double prevRPM, double requestedRPM);

        std::map<std::string, double> slewPose(std::map<std::string, double> requestedRPM);

        void odometryStep(pros::Controller driver);


        Pose calcPoseToGoal();

       

        void stop();


};

typedef struct{
Drivetrain drive;
} drive_arg;

#endif