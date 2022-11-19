#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP

#include "main.h"


typedef struct{
Drivetrain* drive;
IntakeRoller* intake;
Shooter* shooter;
} control_arg;

void auton1();
void skills();

void endAllTasks();



#endif