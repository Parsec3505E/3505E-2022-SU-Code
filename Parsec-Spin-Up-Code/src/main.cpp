#include "main.h"
#include "Subsystems/Drivetrain.hpp"
#include "Subsystems/IntakeRoller.hpp"
#include "Subsystems/Shooter.hpp"
#include "autonomous.hpp"
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


Pose persistPose = Pose(Vector(10.0, 10.0), 0.1);

void autonomous() {
	auton1();
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
	//auton1();
	// auton1();

	//Uncomment below
	pros::Controller driver(pros::E_CONTROLLER_MASTER);
	Drivetrain drive;
	IntakeRoller intake;
	Shooter shooter;

	// drive.setRobotPose(persistPose);

    //driver.print(2, 2, "%.1f, %.1f, %.4f", persistPose.getXComponent(), persistPose.getYComponent(), persistPose.getThetaComponent());
	
	//drive.resetGyro();
	//driver.rumble("...");
	drive.setState(Drivetrain::DrivetrainStates::OPERATOR_CONTROL);
	intake.setIntakeState(IntakeRoller::IntakeStates::OPERATOR_CONTROL);
	

	shooter.setState(Shooter::ShooterStates::OPERATOR_CONTROL);
	shooter.setTargetRPM(380);
	while (true) {

		drive.updateDrivetrain(driver);
		intake.updateIntake(driver);
		shooter.updateShooter(driver);
		
		pros::delay(50);
	}
}
