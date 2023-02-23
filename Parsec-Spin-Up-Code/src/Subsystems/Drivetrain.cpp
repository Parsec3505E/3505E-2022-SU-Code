#include "Subsystems/Drivetrain.hpp"
#include "Utility/Pose.hpp"
#include "pros/misc.h"
#include "pros/screen.h"
#include <iostream>



Drivetrain::Drivetrain()
{

    // Construct the Motor objects
    //PORT 17 IS BROKEN FOR SOME REASON!!!
    rightFront = new pros::Motor(6, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
    rightFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    rightMiddle = new pros::Motor(19, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
    rightMiddle->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    rightBack = new pros::Motor(4, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
    rightBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	leftFront = new pros::Motor(5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
    leftFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    leftMiddle = new pros::Motor(11, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
    leftMiddle->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	leftBack = new pros::Motor(20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
    leftBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // // Construct Odometry Encoder objects
    // forwardEncoder = new pros::ADIEncoder('A', 'B', false);
    // sideEncoder = new pros::ADIEncoder('C', 'D', false);

    // Construct Gyro object
    gyro = new pros::Imu(17);

    
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

                

                break;

                
            }

        case OPEN_LOOP_OPERATOR:
        {
            
            int fwd_val = (abs(driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) >= 90) ? driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) : 0;
            int turn_val = (abs(driver.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) >= 90) ? driver.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) : 0;
            // int fwd_val = joystickControl(driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
            // int turn_val = joystickControl(driver.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));

            int fwdBlueCart = (pow(fwd_val,3.0)/127)*428;
            int turnBlueCart = (pow(turn_val,3.0)/127)*428;

            


            rightFront->move_velocity(fwdBlueCart-turnBlueCart);
            rightMiddle->move_velocity((abs(((fwdBlueCart-turnBlueCart)/428)*600) > 428 ? ((fwdBlueCart-turnBlueCart)/428)*600 : (428 * ((fwdBlueCart-turnBlueCart)/abs((fwdBlueCart-turnBlueCart))))));
            rightBack->move_velocity(fwdBlueCart-turnBlueCart);

            leftFront->move_velocity(fwdBlueCart+turnBlueCart);
            leftMiddle->move_velocity((abs(((fwdBlueCart+turnBlueCart)/428)*600) > 428 ? ((fwdBlueCart+turnBlueCart)/428)*600 : (428 * ((fwdBlueCart+turnBlueCart)/abs((fwdBlueCart+turnBlueCart))))));
            leftBack->move_velocity(fwdBlueCart+turnBlueCart);




            

            break;
        }
        case PID:
            {
               

                break;
            }
        case BLANK:

            break;
        case DEAD:
            stop();

            break;
    }

   

}

Drivetrain::DrivetrainStates Drivetrain::getState()
{
    return mDriveState;
}

void Drivetrain::setState(DrivetrainStates state)
{

    mDriveState = state;

}
void Drivetrain::resetEnc()
{
    rightFront->tare_position(); 
    rightMiddle->tare_position(); 
    rightBack->tare_position();  

    leftFront->tare_position(); 
    leftMiddle->tare_position(); 
    leftBack->tare_position(); 

}
double Drivetrain::joystickControl(double rawSpeed){
    double logSpeed;

    if(rawSpeed >= 0){
        logSpeed = ((rawSpeed)/abs(rawSpeed))*(pow(rawSpeed, 3));
        if(logSpeed < 50.0){
            logSpeed = 0;
        }

        return logSpeed;
    }
}


void Drivetrain::moveEncoder(double inches, int vel){
    resetEnc();
    double inchPerDeg = (M_PI*4.0)/360.0;
    double degToMove = inches/inchPerDeg;
    rightFront->move_relative(degToMove, vel); 
    rightMiddle->move_relative(degToMove, (vel/600)*428); 
    rightBack->move_relative(degToMove, vel); 

    leftFront->move_relative(degToMove, vel); 
    leftMiddle->move_relative(degToMove, (vel/600)*428); 
    leftBack->move_relative(degToMove, vel); 
   
  while (!((rightFront->get_position() < degToMove+5) && (rightFront->get_position() > degToMove-5))) {
    
    pros::delay(2);
  }
}

void Drivetrain::moveSeconds(int seconds, int vel){

    
    rightFront->move_velocity(vel); 
    rightMiddle->move_velocity(vel); 
    rightBack->move_velocity(vel); 

    leftFront->move_velocity(vel);  
    leftMiddle->move_velocity(vel);  
    leftBack->move_velocity(vel); 
    
    pros::delay(seconds);
    stop();

  
}
void Drivetrain::setVel(int vel){

    
    rightFront->move_velocity(vel); 
    rightMiddle->move_velocity(vel); 
    rightBack->move_velocity(vel); 

    leftFront->move_velocity(vel);  
    leftMiddle->move_velocity(vel);  
    leftBack->move_velocity(vel); 
    
   

  
}
void Drivetrain::turnEncoder(double deg, int vel){
    // double inchPerDeg = (M_PI*13.4)/360.0;
    // double turnInches = inchPerDeg*deg;
    // rightFront->move_relative(degToMove, vel); 
    // rightMiddle->move_relative(degToMove, (vel/600)*428); 
    // rightBack->move_relative(degToMove, vel); 

    // leftFront->move_relative(degToMove, vel); 
    // leftMiddle->move_relative(degToMove, (vel/600)*428); 
    // leftBack->move_relative(degToMove, vel); 

}

void Drivetrain::resetGyro()
{
    this->gyro->tare_rotation();
    pros::delay(50);
}

void Drivetrain::turnGyro(double deg, int vel){
    //TURN LEFT
    resetGyro();
    if(deg<0){
      while(gyro->get_yaw()>deg)  {
        rightFront->move_velocity(vel); 
    rightMiddle->move_velocity(vel); 
    rightBack->move_velocity(vel); 

    leftFront->move_velocity(-vel);  
    leftMiddle->move_velocity(-vel);  
    leftBack->move_velocity(-vel); 
      }
      stop();

    
    
    }

    else{
        while(gyro->get_yaw()<deg)  {
        rightFront->move_velocity(-vel); 
    rightMiddle->move_velocity(-vel); 
    rightBack->move_velocity(-vel); 

    leftFront->move_velocity(vel);  
    leftMiddle->move_velocity(vel);  
    leftBack->move_velocity(vel); 
      }
      stop();

    }
    
}


void Drivetrain::stop()
{
    this->rightFront->move_velocity(0);
    this->rightMiddle->move_velocity(0);
    this->rightBack->move_velocity(0);
    this->leftFront->move_velocity(0);
    this->leftMiddle->move_velocity(0);
    this->leftBack->move_velocity(0);
}

void Drivetrain::odometryStep(pros::Controller driver)
{
    
    // ------------------------------- CALCULATIONS ------------------------------- 

    double rightDriveEncoderRaw = -1.0 * double(this->rightFront->get_position() + this->rightBack->get_position()) / 2.0;
    double leftDriveEncoderRaw = -1.0 * double(this->leftFront->get_position() + this->leftBack->get_position()) / 2.0;


    double deltaRightSideEncoderInches = (rightDriveEncoderRaw - this->righDriveEncoderPrev) * (2.0 * M_PI * WHEEL_RADIUS) / 900.0;
    double deltaLeftSideEncoderInches = (leftDriveEncoderRaw - this->leftDriveEncoderPrev) * (2.0 * M_PI * WHEEL_RADIUS) / 900.0;

    double heading = (this->gyro->get_yaw()) * M_PI /180.0;

    double deltaHeading = (heading - prevHeading);

    double totalDistance = ((deltaRightSideEncoderInches + (deltaHeading * DRIVE_RADIUS) + deltaLeftSideEncoderInches - (deltaHeading * DRIVE_RADIUS)) / 2);

    double deltaYLocal = totalDistance * cos(deltaHeading);
    double deltaXLocal = totalDistance * sin(deltaHeading);

    double deltaYGlobal = (deltaXLocal * sin(heading)) + (deltaYLocal * cos(heading));
    double deltaXGlobal = (deltaXLocal * cos(heading)) - (deltaYLocal * sin(heading));

    xPoseGlobal += deltaXGlobal;
    yPoseGlobal += deltaYGlobal;


    // this->robotPose->setXComponent(xPoseGlobal);
    // this->robotPose->setYComponent(yPoseGlobal);
    // this->robotPose->setThetaComponent(heading);

    //driver.print(2, 2, "%.1f, %.1f, %.4f", this->deltaXGlobal, xPoseGlobal, headingRaw);

    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "X Global: %f", this->xPoseGlobal);
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Y Global: %f", this->yPoseGlobal);
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Heading: %f", heading);

    this->prevHeading = heading;
    this->righDriveEncoderPrev = rightDriveEncoderRaw;
    this->leftDriveEncoderPrev = leftDriveEncoderRaw;


}