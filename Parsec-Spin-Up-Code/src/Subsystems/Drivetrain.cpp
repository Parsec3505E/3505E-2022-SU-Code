#include "Subsystems/Drivetrain.hpp"
#include "Utility/Pose.hpp"
#include "pros/misc.h"
#include "pros/screen.h"
#include <iostream>



Drivetrain::Drivetrain()
{
    // Construct the Pose/PosePID objects
    // from rollers 30.0, 13.0
    robotPose = new Pose(Vector(30.0, 13.0), 0.0);
    velocityPose = new Pose(Vector(0.0, 0.0), 0.0);
    targetPose = new Pose(Vector(0.0, 0.0), 0.0);

    previousPose = new Pose(Vector(0.0, 0.0), 0.0);

    xPoseGlobal = robotPose->getXComponent();
    yPoseGlobal = robotPose->getYComponent();

    posePID = new PosePID();

    driverXPID = new PIDController(-10.0, 0.0, 0.0);
    driverYPID = new PIDController(-10.0, 0.0, 0.0);
    driverThetaPID = new PIDController(-30.0, 0.0, 0.0);



    // Construct the Motor objects
    //PORT 17 IS BROKEN FOR SOME REASON!!!
    rightFront = new pros::Motor(4, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    rightFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
//20
    rightBack = new pros::Motor(20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    rightBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	leftFront = new pros::Motor(3, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    leftFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	leftBack = new pros::Motor(15, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    leftBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // Construct Odometry Encoder objects
    forwardEncoder = new pros::ADIEncoder('A', 'B', false);
    sideEncoder = new pros::ADIEncoder('C', 'D', false);

    // Construct Gyro object
    gyro = new pros::Imu(17);

    forwardEncoderPrevRaw = 0.0;
    sideEncoderPrevRaw = 0.0;

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

Drivetrain::~Drivetrain()
{}

void Drivetrain::updateDrivetrain(pros::Controller &driver)
{

    // Finite State Machine (FSM)

    odometryStep(driver);

    switch(mDriveState)
    {
        // The Operator Control state that allows the driver to have open loop control over the drivetrain

        case OPERATOR_CONTROL:
            {

                // PID CODE FOR CORRECTING DRIVING

                //driver.print(2, 2, "in OP%d    ", pros::millis());
                // Setting the x, y and theta components to the joystick values

                //pros::screen::print(pros::E_TEXT_MEDIUM, 8, "target 2: %f", this->targetPose->getThetaComponent());
                
                // Finding the delta distance travelled from last iteration

                // double deltaDistX = this->robotPose->getXComponent() - this->previousPose->getXComponent();
                // double deltaDistY = this->robotPose->getYComponent() - this->previousPose->getYComponent();
                // double deltaDistTheta = this->robotPose->getThetaComponent() - this->previousPose->getThetaComponent();

                // // Start iteration timer

                this->currTime = pros::millis();

                // // Calculating delta time
                
                double deltaTimeMs = this->currTime - this->prevTime;

                // // Get the commanded remote control values - local

                double x_val = (abs(driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) >= 12) ? double(driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) : 0.0;
                double y_val = (abs(driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) >= 12) ? double(driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) : 0.0;
                double theta_val = double(driver.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) * 0.05;

                // Rotate these values from global to local

                Pose* globalPoseVel = (this->robotPose->subtractPose(this->previousPose))->DividePose(deltaTimeMs);
                Pose* localPoseVel = globalPoseVel->rotatePose(this->robotPose->getThetaComponent());
                            
                // // Get commanded remote control velocities

                double controllerVelocityX = x_val / deltaTimeMs;
                double controllerVelocityY = y_val / deltaTimeMs;
                double controllerVelocityTheta = theta_val / deltaTimeMs;

                // driver.print(2, 2, "%.1f           ",controllerVelocityX);



                Pose* controllerPose = new Pose(Vector(controllerVelocityX, controllerVelocityY), controllerVelocityTheta);

                driverXPID->setTarget(controllerVelocityX);
                //driverXPID->stepPID(localPoseVel->getXComponent(), deltaTimeMs);

                driverYPID->setTarget(controllerVelocityY);
                //driverYPID->stepPID(localPoseVel->getYComponent(), deltaTimeMs);

                driverThetaPID->setTarget(controllerVelocityTheta);
                //driverThetaPID->stepPID(localPoseVel->getThetaComponent(), deltaTimeMs);


                //Subtract previous pose from current pose and divide by delta time

                this->targetPose->setXComponent(x_val + driverXPID->stepPID(1000.0 * localPoseVel->getXComponent(), deltaTimeMs));
                this->targetPose->setYComponent(y_val + driverYPID->stepPID(1000.0 * localPoseVel->getYComponent(), deltaTimeMs));
                this->targetPose->setThetaComponent(theta_val + driverThetaPID->stepPID(100.0 * localPoseVel->getThetaComponent(), deltaTimeMs));
                 driver.print(2, 2, "%.1f Y: %.1f T: %.1f   ", 100.0 * localPoseVel->getXComponent(), 100.0 * localPoseVel->getYComponent(), 10.0 * localPoseVel->getThetaComponent());
                //driver.print(2, 2, "%.1f           ",localPoseVel->getXComponent());

                
                


                moveRobot(this->targetPose);

                this->prevTime = this->currTime;

                this->previousPose->setXComponent(this->robotPose->getXComponent());
                this->previousPose->setYComponent(this->robotPose->getYComponent());
                this->previousPose->setThetaComponent(this->robotPose->getThetaComponent());

                // break;


                // int x_val = (abs(driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) >= 12) ? driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) : 0;
                // int y_val = (abs(driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) >= 12) ? driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) : 0;
                // //pros::screen::print(pros::E_TEXT_MEDIUM, 7, "x value %d      ", x_val);
                // //pros::screen::print(pros::E_TEXT_MEDIUM, 8, "y value %d       ", y_val);

                // this->targetPose->setXComponent(-x_val);
                // this->targetPose->setYComponent(-y_val);
                // this->targetPose->setThetaComponent(driver.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) * 0.1);

                // this->currTime = pros::millis();

                //moveRobot(this->targetPose);

                //this->prevTime = this->currTime;

                break;

                
            }

        case OPEN_LOOP_OPERATOR:
        {
            int x_val = (abs(driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) >= 12) ? driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) : 0;
            int y_val = (abs(driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) >= 12) ? driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) : 0;
            //pros::screen::print(pros::E_TEXT_MEDIUM, 7, "x value %d      ", x_val);
            //pros::screen::print(pros::E_TEXT_MEDIUM, 8, "y value %d       ", y_val);

            this->targetPose->setXComponent(-x_val);
            this->targetPose->setYComponent(-y_val);
            this->targetPose->setThetaComponent(driver.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) * 0.1);

            this->currTime = pros::millis();

            moveRobot(this->targetPose);

            this->prevTime = this->currTime;

            break;
        }
        case PID:
            {
                //driver.print(2, 2, "in PID%d    ", pros::millis());
                posePID->setTarget(this->targetPose);

                //pros::screen::print(pros::E_TEXT_MEDIUM, 8, "target: %f", this->targetPose->getThetaComponent());
                // pros::screen::print(pros::E_TEXT_MEDIUM, 8, "target: %f", thetaTarget);
                    
                this->currTime = pros::millis();
                
                Pose* globalPoseVel = posePID->stepPID(this->robotPose, this->currTime - this->prevTime);
                Pose* localPoseVel = globalPoseVel->rotatePose(this->robotPose->getThetaComponent());

                moveRobot(localPoseVel);

                this->prevTime = this->currTime;

                break;
            }
        case BLANK:

            break;
        case DEAD:
            stop();

            break;
    }

    // ADD CODE FOR STOP CASE



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

void Drivetrain::setRobotPose(Pose pose)
{
    this->robotPose->setXComponent(pose.getXComponent());
    this->robotPose->setYComponent(pose.getYComponent());
    this->robotPose->setThetaComponent(pose.getThetaComponent());

}

void Drivetrain::moveRobot(Pose* velocityPose)
{

    // X Drive rotation matrix/math

    double a = ((velocityPose->getXComponent() * cos(M_PI_4)) + (velocityPose->getYComponent() * sin(M_PI_4))) * 60.0 / (Drivetrain::WHEEL_RADIUS * 2 * M_PI);
    double b = ((-velocityPose->getXComponent() * sin(M_PI_4)) + (velocityPose->getYComponent() * cos(M_PI_4))) * 60.0 / (Drivetrain::WHEEL_RADIUS * 2 * M_PI);
    double c = (DRIVE_RADIUS * velocityPose->getThetaComponent()) * 60.0 / (WHEEL_RADIUS * 2 * M_PI);

    this->rotationVels["rightFront"] = b + c;
    this->rotationVels["leftFront"] = a - c;
    this->rotationVels["rightBack"] = a + c;
    this->rotationVels["leftBack"] = b - c;

    // Slewing the rotationVels

    //std::map<std::string, double> motorVels = slewPose(this->rotationVels);
    std::map<std::string, double> motorVels = this->rotationVels;

    // Setting the motor slewed values to the physical motors

    rightFront->move_velocity(motorVels["rightFront"]);
    leftFront->move_velocity(motorVels["leftFront"]);
    rightBack->move_velocity(motorVels["rightBack"]);
    leftBack->move_velocity(motorVels["leftBack"]);
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

    greatestDeltaVelocity = MOTOR_MAX_ACC * deltaSec;

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

    velCap = std::min(greatestVelocityMagnitude, MOTOR_MAX_RPM);


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

void Drivetrain::resetGyro()
{
    this->gyro->tare_rotation();
    pros::delay(50);
}


void Drivetrain::odometryStep(pros::Controller driver)
{
    
    // ------------------------------- CALCULATIONS ------------------------------- 

    forwardEncoderRaw = (double)this->forwardEncoder->get_value();
    sideEncoderRaw = (double)this->sideEncoder->get_value();

    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "forwardEEncoderRaw: : %f", forwardEncoderRaw);
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "sideEncoderRaw: : %f", sideEncoderRaw);
    //pros::screen::print(pros::E_TEXT_MEDIUM, 2, "gyro : %f", gyro->get_yaw());

    deltaDistForward = ((forwardEncoderRaw - this->forwardEncoderPrevRaw)/360.0) * M_PI * WHEEL_DIAMETER;
    deltaDistSide = ((sideEncoderRaw - this->sideEncoderPrevRaw)/360.0) * M_PI * WHEEL_DIAMETER;
    headingRaw = (gyro->get_yaw() * M_PI) / 180.0;

    deltaHeading = headingRaw - this->prevHeadingRaw;

    if(deltaHeading == 0.0 ){
        deltaXLocal = deltaDistSide;
        deltaYLocal = deltaDistForward;
    }else{
        deltaYLocal = 2.0*sin(deltaHeading/2.0) * ((deltaDistForward / deltaHeading) - FORWARD_ENCODER_TRACK_RADIUS);
        deltaXLocal = 2.0*sin(deltaHeading/2.0) * ((deltaDistSide / deltaHeading) - SIDE_ENCODER_TRACK_RADIUS);
    }
    // pros::screen::print(pros::E_TEXT_MEDIUM, 1, "deltaXLocal : %f", deltaXLocal);
    // pros::screen::print(pros::E_TEXT_MEDIUM, 3, "deltaYLocal : %f", deltaYLocal);
    //When encoder moves left it should be negative

    deltaXGlobal = (deltaXLocal * cos(headingRaw + (deltaHeading/2.0))) + (deltaYLocal * sin(headingRaw + (deltaHeading/2.0)));
    deltaYGlobal = (-deltaXLocal * sin(headingRaw + (deltaHeading/2.0))) + (deltaYLocal * cos(headingRaw + (deltaHeading/2.0)));

    // pros::screen::print(pros::E_TEXT_MEDIUM, 5, "deltaXGlobal  : %f", deltaXGlobal);
    // pros::screen::print(pros::E_TEXT_MEDIUM, 7, "deltaYGlobal  : %f", deltaYGlobal);


    //Update global positions
    xPoseGlobal += deltaXGlobal;
    yPoseGlobal += deltaYGlobal;

    this->robotPose->setXComponent(xPoseGlobal);
    this->robotPose->setYComponent(yPoseGlobal);
    this->robotPose->setThetaComponent(headingRaw);

    //driver.print(2, 2, "%.1f, %.1f, %.4f", this->deltaXGlobal, xPoseGlobal, headingRaw);

    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "X Global: %f", this->xPoseGlobal);
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Y Global: %f", this->yPoseGlobal);
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Heading: %f", this->headingRaw);


    // driver.print(2, 2, "%.1f, %.1f, %.4f", xPoseGlobal, yPoseGlobal, headingRaw);
    // driver.print(2, 2, "%.1f, %.1f", forwardEncoderRaw, sideEncoderRaw);

    this->forwardEncoderPrevRaw = forwardEncoderRaw;
    this->sideEncoderPrevRaw = sideEncoderRaw;

    this->prevHeadingRaw = headingRaw;


}


bool Drivetrain::isSettled()
{
    return this->posePID->isSettled() && this->targetPose->comparePoses(this->posePID->getTarget());
}

void Drivetrain::drivePID(double x, double y, double heading){
    this->targetPose->setXComponent(x);
    this->targetPose->setYComponent(y);
    this->targetPose->setThetaComponent(heading);
}

void Drivetrain::driveToPoint(double x, double y, double heading, double xyepsilon=0.75, double thetaepsilon=0.04, double XP = -2.25, double YP = -2.25, double thetaP = -0.5)
{
    posePID->setXConstants(XP, 0.0, 0.0);
    posePID->setYConstants(YP, 0.0, 0.0);
    posePID->setThetaConstants(thetaP, 0.0, 0.1);

    posePID->setXEpsilon(xyepsilon);
    posePID->setYEpsilon(xyepsilon);
    posePID->setThetaEpsilon(thetaepsilon);

    drivePID(x, y, heading);
}

void Drivetrain::turnToPoint(double x, double y, double xyepsilon=1.5, double thetaepsilon=0.04, double XP = 0.0, double YP = 0.0, double thetaP = -10.0)
{
    posePID->setXConstants(XP, 0.0, 0.0);
    posePID->setYConstants(YP, 0.0, 0.0);
    posePID->setThetaConstants(thetaP, 0.0, 0.1);

    posePID->setXEpsilon(xyepsilon);
    posePID->setYEpsilon(xyepsilon);
    posePID->setThetaEpsilon(thetaepsilon);


    
    double heading = atan2(x - this->robotPose->getXComponent(), y - this->robotPose->getYComponent());
    drivePID( this->robotPose->getXComponent(),  this->robotPose->getYComponent(),heading);


    //pros::screen::print(pros::E_TEXT_MEDIUM, 9, "HEADING %f      ", heading);
   
}
void Drivetrain::turnToHeading(double heading, double xyepsilon=10.0, double thetaepsilon=0.04, double XP = 0.0, double YP = 0.0, double thetaP = -4.0)
{
    posePID->setXConstants(XP, 0.0, 0.0);
    posePID->setYConstants(YP, 0.0, 0.0);
    posePID->setThetaConstants(thetaP, 0.0, 0.1);

    posePID->setXEpsilon(xyepsilon);
    posePID->setYEpsilon(xyepsilon);
    posePID->setThetaEpsilon(thetaepsilon);
    
    
    drivePID( this->robotPose->getXComponent(),  this->robotPose->getYComponent(),heading);


   
    
}
Pose Drivetrain::calcPoseToGoal()
{

}

Pose* Drivetrain::getRobotPose()
{
    return this->robotPose;
}
void Drivetrain::setTicks(int vel, int rFront, int lFront, int rBack, int lBack){
    rightFront->move_relative(rFront, vel);
    leftFront->move_relative(lFront, vel);
    rightBack->move_relative(rBack, vel);
    leftBack->move_relative(lBack, vel);
}
void Drivetrain::setPower(int rFront, int lFront, int rBack, int lBack){
    rightFront->move_velocity(rFront);
    leftFront->move_velocity(lFront);
    rightBack->move_velocity(rBack);
    leftBack->move_velocity(lBack);
}
void Drivetrain::driveSeconds(int ms, int rFront, int lFront, int rBack, int lBack)
{
    rightFront->move_velocity(rFront);
    leftFront->move_velocity(lFront);
    rightBack->move_velocity(rBack);
    leftBack->move_velocity(lBack);
    pros::delay(ms);
    stop();

  //drivetrain.stop();
}

void Drivetrain::stop()
{
    this->rightFront->move_velocity(0);
    this->rightBack->move_velocity(0);
    this->leftFront->move_velocity(0);
    this->leftBack->move_velocity(0);
}