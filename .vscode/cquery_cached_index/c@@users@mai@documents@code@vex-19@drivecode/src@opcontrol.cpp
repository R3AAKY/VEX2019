#include "main.h"

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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr1(1);
	pros::Motor left_mtr2(2);
	pros::Motor right_mtr1(3);
	pros::Motor right_mtr2(4);

	left_mtr1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	left_mtr2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_mtr1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_mtr2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	while (true) {
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		if(abs(left)<5){
			left_mtr1.move_velocity(0);
			left_mtr2.move_velocity(0);
		}else{
			left_mtr1 = left;
			left_mtr2 = left;
		}

		if(abs(right)<5){
			right_mtr1.move_velocity(0);
			right_mtr2.move_velocity(0);
		}else{
			right_mtr1 = right;
			right_mtr2 = right;
		}
		pros::delay(20);
	}
}
