#include "main.h"
#include "Subsystems/Drivetrain.hpp"
#include "Subsystems/IntakeRoller.hpp"
#include "Subsystems/Shooter.hpp"
#include "Subsystems/Expansion.hpp"
#include "autonomous.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
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
void competition_initialize()
{
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

void autonomous()
{
	// farSideRollerAuton();
	// auton1();
	// odomAuton();
	// skills();
	// auton2();
	// rollerAuton();
	// hardCodedAuton();
	// HCRollerDisc();
	// HCRollerTwoDisc();
	odomAuton();
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

void opcontrol()
{

	pros::Controller driver(pros::E_CONTROLLER_MASTER);
	Drivetrain drive;
	IntakeRoller intake;
	Shooter shooter;
	Expansion expansion;

	// drive.resetGyro();
	// pros::delay(4000);

	// drive.setRobotPose(persistPose);

	// driver.print(2, 2, "%.1f, %.1f, %.4f", persistPose.getXComponent(), persistPose.getYComponent(), persistPose.getThetaComponent());

	drive.setState(Drivetrain::DrivetrainStates::OPEN_LOOP);
	intake.setIntakeState(IntakeRoller::IntakeStates::OPERATOR_CONTROL);
	expansion.setState(Expansion::ExpansionStates::OPERATOR_CONTROL);

	shooter.setState(Shooter::ShooterStates::OPERATOR_CONTROL);

	shooter.setTargetRPM(340);
	shooter.setIndexerState(true);

	std::uint32_t oppStartTime = pros::millis();
	while (true)
	{

		drive.updateDrivetrain(driver);
		intake.updateIntake(driver);
		shooter.updateShooter(driver);

		if ((pros::millis() - oppStartTime) > 95000)
		{
			expansion.updateExpansion(driver);
		}

		pros::delay(50);
	}
}
