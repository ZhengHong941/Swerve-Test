#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"


void initialize() {
	pros::Motor luA(Left_UpperA_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor luB(Left_UpperB_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor llA(Left_LowerA_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor llB(Left_LowerB_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor ruA(Right_UpperA_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor ruB(Right_UpperB_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rlA(Right_LowerA_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rlB(Right_LowerB_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
}

void disabled() {}
void competition_initialize() {}

void autonomous() {}

bool tankdrive;
double left, right;
double upperL, lowerL;
double upperR, lowerR;
double powerL, turnL;
double powerR, turnR;
double turn;

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor luA(Left_UpperA_motor);
	pros::Motor luB(Left_UpperB_motor);
	pros::Motor llA(Left_LowerA_motor);
	pros::Motor llB(Left_LowerB_motor);
	pros::Motor ruA(Right_UpperA_motor);
	pros::Motor ruB(Right_UpperB_motor);
	pros::Motor rlA(Right_LowerA_motor);
	pros::Motor rlB(Right_LowerB_motor);
	while (true) {
		turn = master.get_analog(ANALOG_RIGHT_X);
		powerL = master.get_analog(ANALOG_LEFT_Y);
		turnL = master.get_analog(ANALOG_LEFT_X);
		upperL = turnL - powerL - turn;
		lowerL = turnL + powerL + turn;
		powerR = master.get_analog(ANALOG_LEFT_Y);
		turnR = master.get_analog(ANALOG_LEFT_X);
		upperR = turnR - powerR + turn;
		lowerR = turnR + powerR - turn;
		luA.move(upperL);
		luB.move(upperL);
		llA.move(lowerL);
		llB.move(lowerL);
		ruA.move(upperR);
		ruB.move(upperR);
		rlA.move(lowerR);
		rlB.move(lowerR);
		pros::delay(2);
	}
}