
// #include "main.h"
#include "autonomous.hpp"
// #include "main.h"

intake_arg* intake_task_arg = new intake_arg;
//pros::Task intakeTask(moveIntakeFor, intake_task_arg, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_MIN);


drive_arg* track_task_arg = new drive_arg;
//pros::Task odomTracking(poseTracking, track_task_arg, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_MIN);
//pros::Task chassisControl(odomChassisControl, track_task_arg, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_MIN);
pros::Controller driver(pros::E_CONTROLLER_MASTER);
void controlFunction(void* controlArg)
{
    
    Drivetrain* drive = ((control_arg*)controlArg)->drive;
    IntakeRoller* intake = ((control_arg*)controlArg)->intake;
    Shooter* shooter  = ((control_arg*)controlArg)->shooter;

	while(true)
	{
		drive->updateDrivetrain(driver);
		intake->updateIntake(driver);
		shooter->updateShooter(driver);

		pros::delay(50);
	}

}
void auton1(){
    //pros::Controller driver(pros::E_CONTROLLER_MASTER);
	std::uint32_t autoStartTime = pros::millis();
	
	control_arg* control_task_arg = new control_arg;


	Drivetrain* drivetrainObj = new Drivetrain();
	control_task_arg->drive = drivetrainObj;

	IntakeRoller* intakeObj = new IntakeRoller();
	control_task_arg->intake = intakeObj;

	Shooter* shooterObj = new Shooter();
	control_task_arg->shooter = shooterObj;


	drivetrainObj->resetGyro();
	pros::delay(3000);

	pros::Task controlTask(controlFunction, control_task_arg, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);

	

//====== START =========
	// ROLLERS
	shooterObj->setMotorSpeed(600);
    drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(-50,-50, -50, -50);
    pros::delay(400);
    drivetrainObj->setPower(0,0, 0,0);
    intakeObj->rollToColourAUTO();

	//MOVE FORWARD 

	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(50,50, 50, 50);
    pros::delay(300);
    drivetrainObj->setPower(0,0, 0,0);
	
	//TURN TO GOAL

	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	drivetrainObj->turnToPoint(17.78, 122.63);
    while(!drivetrainObj->isSettled()){}
	driver.print(2, 2, "Hello222222");
    drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	
	//SHOOTING
	
    shooterObj->indexAll();
	shooterObj->setMotorSpeed(0);
	
	


    
    //TOWARDS SECOND ROLLER
	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(50, 50, -50, -50);
    pros::delay(900);
	 driver.print(2, 2, "Hello");
    drivetrainObj->setPower(0,0, 0,0);

	// drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	// drivetrainObj->driveToPoint(40.0, 16.0, 0);
    // while(!drivetrainObj->isSettled()){}
    // drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);


	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	drivetrainObj->driveToPoint(117.0, 105.35, -(M_PI/2.0));
    while(!drivetrainObj->isSettled()){}
    drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	// drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    // drivetrainObj->setPower(50, 50, 50, 50);
    // pros::delay(500);
    // drivetrainObj->setPower(0,0, 0,0);


	// drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	// drivetrainObj->turnToPoint(0.0, 105.35);
    // while(!drivetrainObj->isSettled()){}
	// driver.print(2, 2, "%f     ", drivetrainObj->getRobotPose()->getThetaComponent() );
    // drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	// ROLLERS
	 drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(100,100, -100, -100);
    pros::delay(500);
    drivetrainObj->setPower(0,0, 0,0);

    drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(-50,-50, -50, -50);
    pros::delay(500);
    drivetrainObj->setPower(0,0, 0,0);
    intakeObj->rollToColourAUTO();


   
    

    
	
	
	
	pros::delay(1000);
	


	while(pros::millis() - autoStartTime < 14500)
	{}

	//wait till timer hits 14.9 seconds
	//do end of auton stuff
	// persistPose.setXComponent(drivetrainObj->getRobotPose()->getXComponent());
	// persistPose.setYComponent(drivetrainObj->getRobotPose()->getYComponent());
	// persistPose.setThetaComponent(drivetrainObj->getRobotPose()->getThetaComponent());
	// drivetrainObj->~Drivetrain();
	controlTask.remove();
}
void skills()
{
   
}