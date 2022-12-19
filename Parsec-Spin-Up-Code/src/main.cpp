#include "main.h"
#include "Subsystems/Drivetrain.hpp"
#include "Subsystems/IntakeRoller.hpp"
#include "Subsystems/Shooter.hpp"
#include "Subsystems/Expansion.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"




/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


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

Pose persistPose = Pose(Vector(10.0, 10.0), 0.1);

void autonomous() {
	
	Shooter shooter = Shooter();
	IntakeRoller intake = IntakeRoller();
	Drivetrain drive = Drivetrain();
	Expansion expansion = Expansion();

	expansion.expansionPistonR->set_value(false);
	expansion.expansionPistonL->set_value(false);

	shooter.shooterPwr1->move_voltage(11000);
    shooter.shooterPwr2->move_voltage(11000);

	drive.driveSeconds(250, 50, 50, 50, 50);
	
	//drive.gyroTurn(10, true, 50);
	intake.spinSeconds(150, 700);
	drive.driveSeconds(115, 50, -50, 50, -50);

	pros::delay(500);
	shooter.shoot(600); 
	shooter.shooterPwr1->move_voltage(11100);
    shooter.shooterPwr2->move_voltage(11100);
	drive.driveSeconds(100, 50, -50, 50, -50);
	// //IF DISK IS STUCK LIL WIGGLE
	// shooter.shooterInd->move_absolute(-50, 100);
    // pros::delay(500);
    // shooter.shooterInd->move_absolute(0, 100);

	// drive.driveSeconds(1000, -100, -100, -100, -100);
	pros::delay(2900);
	shooter.shoot(600);
	// std::uint32_t autoStartTime = pros::millis();
	
	// control_arg* control_task_arg = new control_arg;


	// Drivetrain* drivetrainObj = new Drivetrain();
	// control_task_arg->drive = drivetrainObj;

	// IntakeRoller* intakeObj = new IntakeRoller();
	// control_task_arg->intake = intakeObj;

	// Shooter* shooterObj = new Shooter();
	// control_task_arg->shooter = shooterObj;


	// drivetrainObj->resetGyro();
	// pros::delay(3000);

	// pros::Task controlTask(controlFunction, control_task_arg, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);

	// drivetrainObj->setState(Drivetrain::DrivetrainStates::PID);

	

	// // DRIVE COMMANDS

	// drivetrainObj->driveToPoint(65.0, 40.0, 90);
	// while(drivetrainObj->isSettled()){}
	// pros::delay(1000);



	// while(pros::millis() - autoStartTime < 14500)
	// {}

	// //wait till timer hits 14.9 seconds
	// //do end of auton stuff
	// persistPose.setXComponent(drivetrainObj->getRobotPose()->getXComponent());
	// persistPose.setYComponent(drivetrainObj->getRobotPose()->getYComponent());
	// persistPose.setThetaComponent(drivetrainObj->getRobotPose()->getThetaComponent());
	// drivetrainObj->~Drivetrain();
	// controlTask.remove();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



void opcontrol() {
	
	Drivetrain drive;
	IntakeRoller intake;
	Shooter shooter;
	Expansion expansion;

	drive.setRobotPose(persistPose);

    //driver.print(2, 2, "%.1f, %.1f, %.4f", persistPose.getXComponent(), persistPose.getYComponent(), persistPose.getThetaComponent());
	
	//drive.resetGyro();
	//driver.rumble("...");
	drive.setState(Drivetrain::DrivetrainStates::OPERATOR_CONTROL);
	intake.setIntakeState(IntakeRoller::IntakeStates::OPERATOR_CONTROL);
	expansion.setState(Expansion::ExpansionStates::OPERATOR_CONTROL);
	
	//driver.rumble("...");
	// intake.setIntakeState(IntakeRoller::IntakeStates::OPERATOR_CONTROL);
	shooter.setState(Shooter::ShooterStates::OPERATOR_CONTROL);

	std::uint32_t oppStartTime = pros::millis();
	while (true) {

		drive.updateDrivetrain(driver);
		intake.updateIntake(driver);
		shooter.updateShooter(driver);
		
		if((pros::millis() - oppStartTime) > 95000){
			expansion.updateExpansion(driver);
		}
		
		
		pros::delay(50);
	}
}
