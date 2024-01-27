#include "October-Comp/Robot.hpp"

using namespace okapi;

Robot::Robot()
{
    gyro = new IMU(1, IMUAxes::z);

    std::shared_ptr<ChassisController> drive =
        ChassisControllerBuilder()

            .withMotors(
                7,  // Top left
                -5, // Top right (reversed)
                -3, // Bottom right (reversed)
                4   // Bottom left
            )
            // Green gearset, 4 in wheel diam, 11.5 in wheel track
            .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
            .withGains(
                {0.001, 0, 0.0001}, // Distance controller gains
                {0.001, 0, 0.0001}, // Turn controller gains
                {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
            )
            .build();

    auto XDriveTrain { std::dynamic_pointer_cast<okapi::XDriveModel>(drive->getModel()) };

}

void Robot::operatorControl(double x, double y, double turn)
{
    xDriveTrain->xArcade(x, y, turn);
}