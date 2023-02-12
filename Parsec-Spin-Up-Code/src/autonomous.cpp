
// #include "main.h"
#include "autonomous.hpp"
// #include "main.h"
pros::Controller driver(pros::E_CONTROLLER_MASTER);

void controlFunction(void* controlArg)
{
    
    Drivetrain* drive = ((control_arg*)controlArg)->drive;
    IntakeRoller* intake = ((control_arg*)controlArg)->intake;
    Shooter* shooter  = ((control_arg*)controlArg)->shooter;
	Expansion* expansion = ((control_arg*)controlArg)->expansion;
	int curTime = pros::millis();
	while(pros::millis() - curTime < 14990 + 1000000)
	{
		drive->updateDrivetrain(driver);
		intake->updateIntake(driver);
		shooter->updateShooter(driver);

		pros::delay(50);
	}

}
void odomAuton(){
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

	intakeObj->setIntakeState(IntakeRoller::IntakeStates::BLANK);
    drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(-50,-50, -50, -50);
    pros::delay(400);
    drivetrainObj->setPower(0,0, 0,0);
    intakeObj->rollToColourAUTO();

	// // //MOVE FORWARD 
	shooterObj->setMotorSpeed(400);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(50,50, 50, 50);
    pros::delay(300);
    drivetrainObj->setPower(0,0, 0,0);

 
	// drivetrainObj->driveToPoint(70.0, 70.0, 0.0, 2.0, 0.5, -2.25, -2.25, -1.5);
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	// //while(drivetrainObj->getRobotPose()->getXComponent() < 115.0 && drivetrainObj->getRobotPose()->getYComponent() < 70.0)
	// while(!drivetrainObj->isSettled()){}
    // drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	drivetrainObj->turnToPoint(17.78, 122.63, 0.08, 0.06, 0.0, 0.0, -4.0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
    while(!drivetrainObj->isSettled()){}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	shooterObj->indexAll();
	// shooterObj->setMotorSpeed(0);
	

	while(pros::millis() - autoStartTime < 14500)
	{}
	drivetrainObj->~Drivetrain();

	controlTask.remove();

	//wait till timer hits 14.9 seconds
	//do end of auton stuff
	// persistPose.setXComponent(drivetrainObj->getRobotPose()->getXComponent());
	// persistPose.setYComponent(drivetrainObj->getRobotPose()->getYComponent());
	// persistPose.setThetaComponent(drivetrainObj->getRobotPose()->getThetaComponent());

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
	shooterObj->setMotorSpeed(495);
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

 

	//DRIVE TO MIDDLE
//added 2 to y
	Pose tempPose2(Vector(77.0, 72.0), 0.0);
    drivetrainObj->moveRobot(&tempPose2);
    pros::delay(3000);
    drivetrainObj->setPower(0,0, 0,0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	
	// //TURN TO GOAL
	drivetrainObj->turnToHeading(-0.667, 10.0, 0.1, 0.0, 0.0, -3.0);
	//drivetrainObj->turnToPoint(17.78, 122.63, 10, 0.06, 0.0, 0.0, -5.0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
    while(!drivetrainObj->isSettled()){}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	driver.print(2, 2, "Hello11111");
	// //SHOOTING
	pros::delay(500);
    shooterObj->indexAll();
	shooterObj->setMotorSpeed(0);

	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
	
	


    
    //TOWARDS SECOND ROLLER

	//WITH MOVE TI POINT
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
	// Pose tempPose(Vector(50.0, 40.0), 0.0);
    // drivetrainObj->moveRobot(&tempPose);
    // pros::delay(1900);
	//  //driver.print(2, 2, "Hello");
    // drivetrainObj->setPower(0,0, 0,0);
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	// pros::delay(350);

//TURNING
	// drivetrainObj->turnToHeading(-(M_PI/2.0), 10.0, 0.1, 0.0, 0.0, -3.0);
	// pros::delay(100);
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	// pros::delay(100);
    // while(!drivetrainObj->isSettled()){}
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	// driver.print(2, 2, "Hello222222");

	// pros::delay(100);

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
	// drivetrainObj->driveToPoint(90.0, 70.0, -(M_PI/2.0), 2.0, 0.5, -2.25, -2.25, -1.5);
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	// //while(drivetrainObj->getRobotPose()->getXComponent() < 115.0 && drivetrainObj->getRobotPose()->getYComponent() < 70.0)
	// while(!drivetrainObj->isSettled()){}
    // drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
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


	
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
	// Pose tempPose2(Vector(90.0, 80.0), -(M_PI/2.0));
    // drivetrainObj->moveRobot(&tempPose2);
    // pros::delay(1000);
	//  //driver.print(2, 2, "Hello");
    // drivetrainObj->setPower(0,0, 0,0);
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);


   
    

    
	
	
	
	// pros::delay(1000);
	


	while(pros::millis() - autoStartTime < 14500)
	{}

	//wait till timer hits 14.9 seconds
	//do end of auton stuff
	// persistPose.setXComponent(drivetrainObj->getRobotPose()->getXComponent());
	// persistPose.setYComponent(drivetrainObj->getRobotPose()->getYComponent());
	// persistPose.setThetaComponent(drivetrainObj->getRobotPose()->getThetaComponent());
	drivetrainObj->~Drivetrain();

	controlTask.remove();
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
	shooterObj->setMotorSpeed(500);
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

	

//DRIVE TO MIDDLE
drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
//added 2 to x and y
	Pose tempPose2(Vector(77.0, 67.0), 0.0);
    drivetrainObj->moveRobot(&tempPose2);
    pros::delay(2750);
    drivetrainObj->setPower(0,0, 0,0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	
	
	pros::delay(500);
	// //TURN TO GOAL
	//was 17.78
	//was 122.63
	drivetrainObj->turnToPoint(17.78, 100.63, 10, 0.02, 0.0, 0.0, -5.0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
    while(!drivetrainObj->isSettled()){}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	driver.print(2, 2, "Hello11111");
	

	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
	drivetrainObj->driveSeconds(800, 50, 50, 50, 50);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	pros::delay(500);
	drivetrainObj->turnToPoint(17.78, 89.63, 10, 0.01, 0.0, 0.0, -5.0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
    while(!drivetrainObj->isSettled()){}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	//==== SHOOT === 
	pros::delay(350);
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
	drivetrainObj->turnToHeading((0.0), 10.0, 0.1, 0.0, 0.0, -3.0);
	pros::delay(100);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	pros::delay(100);
    while(!drivetrainObj->isSettled()){}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
	driver.print(2, 2, "Hello222222");

	pros::delay(100);

	//TOWARDS ROLLER DRIVETOPOINT
	// drivetrainObj->driveToPoint(85.0, 63.0, -(M_PI/2.0), 2.0, 0.5, -2.25, -2.25, -1.5);
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
	// //while(drivetrainObj->getRobotPose()->getXComponent() < 115.0 && drivetrainObj->getRobotPose()->getYComponent() < 70.0)
	// while(!drivetrainObj->isSettled()){}
    // drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

while(pros::millis() - autoStartTime < 14500)
	{}


}

void farSideRollerAuton(){
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

//+++++++ START ++++++++++

	//shooterObj->setState(Shooter::ShooterStates::CLOSED_LOOP);
	//shooterObj->setMotorSpeed(500);

	//RIGHT ODOM
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
	// Pose tempPose2(Vector(55.0, 13.0), 0.0);
    // drivetrainObj->moveRobot(&tempPose2);
    // pros::delay(2750);
    // drivetrainObj->setPower(0,0, 0,0);
	// drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	//RIGHT
	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(50,-50,50,-50);
    pros::delay(700);
    drivetrainObj->setPower(0,0, 0,0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	//BACKWARDS & ROLL

    drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(-50,-50,-50,-50);
    pros::delay(400);
    drivetrainObj->setPower(0,0, 0,0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);
    intakeObj->rollToColourAUTO();

	while(pros::millis() - autoStartTime < 14500)
	{}

	
}

void rollerAuton(){
	
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

	intakeObj->setIntakeState(IntakeRoller::IntakeStates::BLANK);
    drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(-50,-50, -50, -50);
    pros::delay(400);
    drivetrainObj->setPower(0,0, 0,0);
    intakeObj->rollToColourAUTO();

	// // //MOVE FORWARD 
	// shooterObj->setMotorSpeed(400);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(50,50, 50, 50);
    pros::delay(300);
    drivetrainObj->setPower(0,0, 0,0);

	
	

	
	

	while(pros::millis() - autoStartTime < 14500)
	{}
	drivetrainObj->~Drivetrain();

	controlTask.remove();

}

void skillsNoDisk()
{
   std::uint32_t autoStartTime = pros::millis();
	
	control_arg* control_task_arg = new control_arg;


	Drivetrain* drivetrainObj = new Drivetrain();
	control_task_arg->drive = drivetrainObj;

	IntakeRoller* intakeObj = new IntakeRoller();
	control_task_arg->intake = intakeObj;

	Shooter* shooterObj = new Shooter();
	control_task_arg->shooter = shooterObj;

	Expansion* expansionObj = new Expansion();
	control_task_arg->expansion = expansionObj;


	drivetrainObj->resetGyro();
	pros::delay(2500);

	pros::Task controlTask(controlFunction, control_task_arg, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT);


//====== START =========
	// ROLLERS
	shooterObj->setState(Shooter::ShooterStates::CLOSED_LOOP);

	intakeObj->setIntakeState(IntakeRoller::IntakeStates::BLANK);
    drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(-50,-50, -50, -50);
    pros::delay(400);
    drivetrainObj->setPower(0,0, 0,0);
    intakeObj->rollToColourMANUAL(true);

	

	//MOVE FORWARD
	drivetrainObj->setPower(40,50, 40, 50);
    pros::delay(1500);
    drivetrainObj->setPower(0,0, 0,0);

	//TURN TOWARDS ROLLER 2
    drivetrainObj->setPower(100,-100, 100, -100);
    pros::delay(300);
    drivetrainObj->setPower(0,0, 0,0);

	//BACK UP & ROLL
	drivetrainObj->setPower(-50,-80, -50, -80);
    pros::delay(600);
    drivetrainObj->setPower(0,0, 0,0);
    intakeObj->rollToColourMANUAL(true);

	//GET INTO POS TO EXPAND
	drivetrainObj->setPower(50,50, 50, 50);
    pros::delay(500);
    drivetrainObj->setPower(0,0, 0,0);

	drivetrainObj->setPower(-50,50, -50, 50);
    pros::delay(300);
    drivetrainObj->setPower(0,0, 0,0);

	//EXPAND

	// expansionObj->expand();
	

	
	

	while(pros::millis() - autoStartTime < 14500)
	{}
	drivetrainObj->~Drivetrain();

	controlTask.remove();

	//wait till timer hits 14.9 seconds
	//do end of auton stuff
	// persistPose.setXComponent(drivetrainObj->getRobotPose()->getXComponent());
	// persistPose.setYComponent(drivetrainObj->getRobotPose()->getYComponent());
	// persistPose.setThetaComponent(drivetrainObj->getRobotPose()->getThetaComponent());
}

void skills()
{
   std::uint32_t autoStartTime = pros::millis();
	
	control_arg* control_task_arg = new control_arg;


	Drivetrain* drivetrainObj = new Drivetrain();
	control_task_arg->drive = drivetrainObj;

	IntakeRoller* intakeObj = new IntakeRoller();
	control_task_arg->intake = intakeObj;

	Shooter* shooterObj = new Shooter();
	control_task_arg->shooter = shooterObj;

	Expansion* expansionObj = new Expansion();
	control_task_arg->shooter = shooterObj;


	drivetrainObj->resetGyro();
	pros::delay(2500);

	pros::Task controlTask(controlFunction, control_task_arg, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT);


//====== START =========
	// ROLLERS
	shooterObj->setState(Shooter::ShooterStates::CLOSED_LOOP);

	intakeObj->setIntakeState(IntakeRoller::IntakeStates::BLANK);
    drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);
    drivetrainObj->setPower(-60,-60, -60, -60);
    pros::delay(400);
	shooterObj->setMotorSpeed(400);
    drivetrainObj->setPower(0,0, 0,0);
    intakeObj->rollToColourMANUAL(true);

	// // //MOVE FORWARD 

	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);	
	
	
    drivetrainObj->setPower(50,50, 50, 50);
    pros::delay(300);
    drivetrainObj->setPower(0,0, 0,0);

	drivetrainObj->turnToPoint(8.78, 122.63, 10.0, 0.08, 0.0, 0.0, -4.0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
    while(!drivetrainObj->isSettled()){}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	
	shooterObj->indexAll();
	shooterObj->setMotorSpeed(0);

	drivetrainObj->turnToPoint(15.78, 122.63, 10.0, 0.08, 0.0, 0.0, -4.0);
	drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
    while(!drivetrainObj->isSettled()){}
	drivetrainObj->setState(Drivetrain::DrivetrainStates::DEAD);

	drivetrainObj->setState(Drivetrain::DrivetrainStates::BLANK);

	drivetrainObj->setPower(50,-50, 50, -50);
    pros::delay(100);
    drivetrainObj->setPower(0,0, 0,0);

	drivetrainObj->setPower(100,100, 100, 100);
    pros::delay(1000);
    drivetrainObj->setPower(0,0, 0,0);


    drivetrainObj->setPower(100,-100, 100, -100);
    pros::delay(300);
    drivetrainObj->setPower(0,0, 0,0);

	drivetrainObj->setPower(-50,-50, -50, -80);
    pros::delay(1500);
    drivetrainObj->setPower(0,0, 0,0);

	// drivetrainObj->setPower(-100,100, -100, 100);
    // pros::delay(300);
    // drivetrainObj->setPower(0,0, 0,0);

	// drivetrainObj->setPower(-50,-50, -50, -50);
    // pros::delay(1000);
    // drivetrainObj->setPower(0,0, 0,0);
    intakeObj->rollToColourMANUAL(true);

	drivetrainObj->setPower(50,50, 50, 50);
    pros::delay(500);
    drivetrainObj->setPower(0,0, 0,0);

	drivetrainObj->setPower(-50,50, -50, 50);
    pros::delay(300);
    drivetrainObj->setPower(0,0, 0,0);
	
	

	
	//EXPAND

	// expansionObj->expand();

	while(pros::millis() - autoStartTime < 14500)
	{}
	drivetrainObj->~Drivetrain();

	controlTask.remove();

	//wait till timer hits 14.9 seconds
	//do end of auton stuff
	// persistPose.setXComponent(drivetrainObj->getRobotPose()->getXComponent());
	// persistPose.setYComponent(drivetrainObj->getRobotPose()->getYComponent());
	// persistPose.setThetaComponent(drivetrainObj->getRobotPose()->getThetaComponent());
}