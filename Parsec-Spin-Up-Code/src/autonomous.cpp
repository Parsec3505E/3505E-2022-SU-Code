
// #include "main.h"
#include "autonomous.hpp"
// #include "main.h"
pros::Controller driver(pros::E_CONTROLLER_MASTER);


void HCRoller(){
	Drivetrain drivetrainObj = Drivetrain();
	IntakeRoller intakeObj =  IntakeRoller();
	Shooter shooterObj =  Shooter();
	Expansion expansionObj = Expansion();
	// expansionObj.expansionPistonR->set_value(false);
	drivetrainObj.resetGyro();
	shooterObj.setIndexerState(true);
	pros::delay(3000);

	//===== START =====
	
	//ROLLERS
	drivetrainObj.moveSeconds(700, 40);
	drivetrainObj.setVel(40);
	intakeObj.rollToColourSEC(800);
	drivetrainObj.moveEncoder(-5, 200);
}
void HCRollerDisc()
{
	Drivetrain drivetrainObj = Drivetrain();
	IntakeRoller intakeObj =  IntakeRoller();
	Shooter shooterObj =  Shooter();
	Expansion expansionObj = Expansion();
	// expansionObj.expansionPistonR->set_value(false);
	drivetrainObj.resetGyro();
	shooterObj.setIndexerState(true);
	shooterObj.setMotorSpeed(450);
	pros::delay(3000);

	

	//===== START =====
	
	//ROLLERS
	

	
	shooterObj.indexAll();
	
	shooterObj.setMotorSpeed(0);

	drivetrainObj.moveSeconds(800, 40);
	drivetrainObj.setVel(40);
	intakeObj.rollToColourSEC(800);
	drivetrainObj.moveEncoder(-5, 200);
	// pros::delay(500);
	
	//Turn & Drive to middle
	// drivetrainObj.turnGyro(-64.0, 100);

	// drivetrainObj.moveEncoder(120, 100);
	// pros::delay(500);
	
	

	// //Shoot into goal
	// shooterObj.indexAll();
	// shooterObj.setMotorSpeed(0);

	// //turn & drive to 2nd roller
	// drivetrainObj.turnGyro(-22.0, 100);
	// // drivetrainObj.moveEncoder(65, 100);



	
}
void HCRollerTwo()
{
	Drivetrain drivetrainObj = Drivetrain();
	IntakeRoller intakeObj =  IntakeRoller();
	Shooter shooterObj =  Shooter();

	drivetrainObj.resetGyro();
	shooterObj.setIndexerState(true);
	pros::delay(3000);

	//===== START =====
	
	//ROLLERS
	
	drivetrainObj.moveEncoder(20, 100);
	pros::delay(500);
	drivetrainObj.turnGyro(22.0, 100);
	drivetrainObj.moveSeconds(700, 40);
	drivetrainObj.setVel(40);
	intakeObj.rollToColourSEC(800);
	
	
}
void HCRollerTwoDisc(){
	Drivetrain drivetrainObj = Drivetrain();
	IntakeRoller intakeObj =  IntakeRoller();
	Shooter shooterObj =  Shooter();

	drivetrainObj.resetGyro();
	shooterObj.setIndexerState(true);
	shooterObj.setMotorSpeed(275);
	pros::delay(4000);

	//===== START =====
	
	//ROLLERS
	shooterObj.indexAll2();
	
	shooterObj.setMotorSpeed(0);

	drivetrainObj.moveEncoder(70, 200);
	pros::delay(500);
	drivetrainObj.turnGyro(70.0, 100);
	drivetrainObj.moveSeconds(850, 50);
	drivetrainObj.setVel(45);
	intakeObj.rollToColourSEC(250);
	drivetrainObj.setVel(0);
}
