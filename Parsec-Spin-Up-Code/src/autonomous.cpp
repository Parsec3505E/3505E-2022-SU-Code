
// #include "main.h"
#include "autonomous.hpp"
// #include "main.h"


intake_arg* intake_task_arg = new intake_arg;
//pros::Task intakeTask(moveIntakeFor, intake_task_arg, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_MIN);


drive_arg* track_task_arg = new drive_arg;
//pros::Task odomTracking(poseTracking, track_task_arg, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_MIN);
//pros::Task chassisControl(odomChassisControl, track_task_arg, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_MIN);

void controlFunction(void* controlArg)
{
    pros::Controller driver(pros::E_CONTROLLER_MASTER);
    Drivetrain* drive = ((control_arg*)controlArg)->drive;
    IntakeRoller* intake = ((control_arg*)controlArg)->intake;
    Shooter* shooter  = ((control_arg*)controlArg)->shooter;

	while(true)
	{
		drive->updateDrivetrain(driver);
		intake->updateIntake(driver);
		shooter->updateShooter(driver);
	}

}

void autonwithSec(){
    control_arg* control_task_arg = new control_arg;

	
	Drivetrain drive = Drivetrain();
    IntakeRoller intake = IntakeRoller();
    Shooter shooter  = Shooter();

    drive.resetEncoders();

    pros::delay(500);
    intake.spinSeconds(600, 3000);

    drive.gyroTurn(45.0, true, 100);

    drive.driveSeconds(1000, -100, -100, -100, -100);

    // pros::delay(3000);

    drive.gyroTurn(45.0, true, 100);
    shooter.shoot(700);

    // pros::delay(1000);

    // drive.driveForwardEncoder(100, 5);
    // intake.spinSeconds(600, 3000);

    // pros::delay(1000);

    // drive.driveBackwardEncoder(100, 5);

    // pros::delay(1000);

  




    // TASK STUFF
	// Drivetrain* drivetrainObj = new Drivetrain();
	// control_task_arg->drive = drivetrainObj;

	// IntakeRoller* intakeObj = new IntakeRoller();
	// control_task_arg->intake = intakeObj;

	// Shooter* shooterObj = new Shooter();
	// control_task_arg->shooter = shooterObj;


	// pros::Task controlTask(controlFunction, control_task_arg, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);

	// drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);
}
void autonwithEnc(){
    Drivetrain drive = Drivetrain();
    IntakeRoller intake = IntakeRoller();
    Shooter shooter  = Shooter();

    drive.resetEncoders();

   
    intake.spinSeconds(600, 3000);

    drive.gyroTurn(45.0, true, 100);
    
    drive.driveBackwardEncoder(100, 10);

}


void skills()
{
   
    //runChassisControl = true;
    //runOdomTracking = true;
    /*
    odomDriveTo(70.3, 70.3, 100.0, 10.0);
    while(runChassisControl)
    {
        pros::delay(1);
    }
    drive.stop();
    */
}