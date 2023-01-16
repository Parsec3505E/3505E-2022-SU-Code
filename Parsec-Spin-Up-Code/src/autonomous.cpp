
// #include "main.h"
#include "autonomous.hpp"
// #include "main.h"
pros::Controller driver(pros::E_CONTROLLER_MASTER);

void controlFunction(void* controlArg)
{
    
    Drivetrain* drive = ((control_arg*)controlArg)->drive;
    IntakeRoller* intake = ((control_arg*)controlArg)->intake;
    Shooter* shooter  = ((control_arg*)controlArg)->shooter;
	int curTime = pros::millis();
	while(pros::millis() - curTime < 14990 + 1000000)
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
	pros::delay(2500);

	pros::Task controlTask(controlFunction, control_task_arg, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT);

	

//====== START =========
	// ROLLERS
	shooterObj->setState(Shooter::ShooterStates::CLOSED_LOOP);
	shooterObj->setMotorSpeed(570);
    drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(-50,-50, -50, -50);
    pros::delay(400);
    drivetrainObj->setPower(0,0, 0,0);
    intakeObj->rollToColourAUTO();

	// // //MOVE FORWARD 

	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(50,50, 50, 50);
    pros::delay(300);
    drivetrainObj->setPower(0,0, 0,0);

	
	// //TURN TO GOAL
	drivetrainObj->turnToPoint(17.78, 122.63, 10, 0.06, 0.0, 0.0, -5.0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
    while(!drivetrainObj->isSettled()){}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	driver.print(2, 2, "Hello11111");
	// //SHOOTING
	
    shooterObj->indexAll();
	shooterObj->setMotorSpeed(0);
	
	


    
    //TOWARDS SECOND ROLLER

	//WITH MOVE TI POINT
	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
	Pose tempPose(Vector(50.0, 40.0), 0.0);
    drivetrainObj->moveRobot(&tempPose);
    pros::delay(1900);
	 //driver.print(2, 2, "Hello");
    drivetrainObj->setPower(0,0, 0,0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	pros::delay(350);

//TURNING
	drivetrainObj->turnToHeading(-(M_PI/2.0), 10.0, 0.1, 0.0, 0.0, -3.0);
	pros::delay(100);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	pros::delay(100);
    while(!drivetrainObj->isSettled()){}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	driver.print(2, 2, "Hello222222");

	pros::delay(100);

	// // drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	// // drivetrainObj->driveToPoint(40.0, 16.0, 0);
    // // while(!drivetrainObj->isSettled()){}
    // // drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

//TOWARDS ROLLER MANUALLY
// drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
// 	Pose tempPose2(Vector(115.0, 70.0), -(M_PI/2.0));
//     drivetrainObj->moveRobot(&tempPose2);
//     pros::delay(2100);
// 	 //driver.print(2, 2, "Hello");
//     drivetrainObj->setPower(0,0, 0,0);
// 	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

//TOWARDS ROLLER DRIVETOPOINT
	drivetrainObj->driveToPoint(90.0, 70.0, -(M_PI/2.0), 2.0, 0.5, -2.25, -2.25, -1.5);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	//while(drivetrainObj->getRobotPose()->getXComponent() < 115.0 && drivetrainObj->getRobotPose()->getYComponent() < 70.0)
	while(!drivetrainObj->isSettled()){}
    drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	//driver.print(2, 2, "Hello333333");

	// // drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    // // drivetrainObj->setPower(50, 50, 50, 50);
    // // pros::delay(500);
    // // drivetrainObj->setPower(0,0, 0,0);


	// // drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	// // drivetrainObj->turnToPoint(0.0, 105.35);
    // // while(!drivetrainObj->isSettled()){}
	// // driver.print(2, 2, "%f     ", drivetrainObj->getRobotPose()->getThetaComponent() );
    // // drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	// // SECOND ROLLER


	
	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
	Pose tempPose2(Vector(90.0, 80.0), -(M_PI/2.0));
    drivetrainObj->moveRobot(&tempPose2);
    pros::delay(1000);
	 //driver.print(2, 2, "Hello");
    drivetrainObj->setPower(0,0, 0,0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);


   
    

    
	
	
	
	// pros::delay(1000);
	


	while(pros::millis() - autoStartTime < 14500)
	{}

	//wait till timer hits 14.9 seconds
	//do end of auton stuff
	// persistPose.setXComponent(drivetrainObj->getRobotPose()->getXComponent());
	// persistPose.setYComponent(drivetrainObj->getRobotPose()->getYComponent());
	// persistPose.setThetaComponent(drivetrainObj->getRobotPose()->getThetaComponent());
	// drivetrainObj->~Drivetrain();
}
void auton2(){
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
	pros::delay(2500);

	pros::Task controlTask(controlFunction, control_task_arg, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT);

	

//====== START =========
	// ROLLERS
	shooterObj->setState(Shooter::ShooterStates::CLOSED_LOOP);
	shooterObj->setMotorSpeed(425);
    drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(-50,-50, -50, -50);
    pros::delay(400);
    drivetrainObj->setPower(0,0, 0,0);
    intakeObj->rollToColourAUTO();

	// // //MOVE FORWARD 

	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(50,50, 50, 50);
    pros::delay(500);
    drivetrainObj->setPower(0,0, 0,0);
 pros::delay(350);

	//MOVE TO THE MIDDLE
	// drivetrainObj->driveToPoint(77.0, 37.0, 0.0, 20.0, 0.5, -2.25, -2.25, -1.5);
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	// //while(drivetrainObj->getRobotPose()->getXComponent() < 115.0 && drivetrainObj->getRobotPose()->getYComponent() < 70.0)
	// while(!drivetrainObj->isSettled()){}
    // drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);


drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
	Pose tempPose2(Vector(75.0, 65.0), 0.0);
    drivetrainObj->moveRobot(&tempPose2);
    pros::delay(2000);
    drivetrainObj->setPower(0,0, 0,0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	
	
	pros::delay(350);
	// //TURN TO GOAL
	//was 17.78
	//was 122.63
	drivetrainObj->turnToPoint(17.78, 90.63, 10, 0.02, 0.0, 0.0, -5.0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
    while(!drivetrainObj->isSettled()){}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	driver.print(2, 2, "Hello11111");
	// //SHOOTING
	
    shooterObj->indexAll();
	shooterObj->setMotorSpeed(0);
	
	


    
    //TOWARDS SECOND ROLLER

	//WITH MOVE TI POINT
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
	// Pose tempPose(Vector(50.0, 30.0), 0.0);
    // drivetrainObj->moveRobot(&tempPose);
    // pros::delay(1900);
	//  //driver.print(2, 2, "Hello");
    // drivetrainObj->setPower(0,0, 0,0);
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	// pros::delay(350);

//TURNING
	drivetrainObj->turnToHeading(-(M_PI/2.0), 10.0, 0.1, 0.0, 0.0, -3.0);
	pros::delay(100);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	pros::delay(100);
    while(!drivetrainObj->isSettled()){}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	driver.print(2, 2, "Hello222222");

	pros::delay(100);

	//TOWARDS ROLLER DRIVETOPOINT
	drivetrainObj->driveToPoint(85.0, 63.0, -(M_PI/2.0), 2.0, 0.5, -2.25, -2.25, -1.5);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	//while(drivetrainObj->getRobotPose()->getXComponent() < 115.0 && drivetrainObj->getRobotPose()->getYComponent() < 70.0)
	while(!drivetrainObj->isSettled()){}
    drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);



}
    


void skills()
{
   
}