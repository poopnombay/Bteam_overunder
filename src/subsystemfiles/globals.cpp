#include "main.h"

pros::Controller Master(pros::E_CONTROLLER_MASTER);

// Drivetrain motors
pros::Motor DLB(8, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor DLF(7, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor DRB(6, pros::E_MOTOR_GEAR_BLUE, false);
pros::Motor DRF(5, pros::E_MOTOR_GEAR_BLUE, false);

// Drivetrain motor groups
pros::Motor_Group DL ({DLB, DLF});
pros::Motor_Group DR ({DRB, DRF});

// Catapult motor
pros::Motor cata(20, pros::E_MOTOR_GEAR_RED, false);

// Intake motor
pros::Motor intake(19, pros::E_MOTOR_GEAR_GREEN, true);