#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP

#include "main.h"
// #include "Subsystems/Drivetrain.hpp"
// #include "Subsystems/Expansion.hpp"
// #include "Subsystems/IntakeRoller.hpp"
// #include "Subsystems/Shooter.hpp"


typedef struct{
Drivetrain* drive;
IntakeRoller* intake;
Shooter* shooter;
} control_arg;

void auton1();
void skills();

void endAllTasks();



#endif