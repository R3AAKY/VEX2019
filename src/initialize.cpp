#include "main.h"
#include "base.h"

#define ENCODER_LEFT_TOP 1
#define ENCODER_LEFT_BOTTOM 2
#define ENCODER_RIGHT_TOP 3
#define ENCODER_RIGHT_BOTTOM 4

#define LEFT_MOTOR_FRONT 3
#define LEFT_MOTOR_BACK 4
#define RIGHT_MOTOR_FRONT 1
#define RIGHT_MOTOR_BACK 2

#define R_FLY 8
#define L_FLY 10
#define INTAKE 19
#define RIGHT_ARM 7
#define LEFT_ARM 9
#define CLAW 20

#define LIMIT_SWITCH_1 'E'
#define LIMIT_SWITCH_2 'F'
pros::ADIDigitalIn limit_switch1 (LIMIT_SWITCH_1);
pros::ADIDigitalIn limit_switch2 (LIMIT_SWITCH_2);
uint32_t now = pros::millis();

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::ADIEncoder left_sensor (ENCODER_LEFT_TOP,ENCODER_LEFT_BOTTOM, true);
pros::ADIEncoder right_sensor (ENCODER_RIGHT_TOP,ENCODER_RIGHT_BOTTOM, false);
pros::Motor left_mtr1(LEFT_MOTOR_FRONT, false);
pros::Motor left_mtr2(LEFT_MOTOR_BACK, false);
pros::Motor right_mtr1(RIGHT_MOTOR_FRONT, true);
pros::Motor right_mtr2(RIGHT_MOTOR_BACK, true);

pros::Motor right_flywheel (R_FLY,true);
pros::Motor left_flywheel (L_FLY);

pros::Motor intake(INTAKE,true);

pros::Motor right_arm(RIGHT_ARM, true);
pros::Motor left_arm(LEFT_ARM, false);
pros::Motor claw(CLAW, true);

pros::ADIGyro gyro('H');

void initDrive(){
	left_mtr1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	left_mtr2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_mtr1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_mtr2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	right_flywheel.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	left_flywheel.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	right_arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	left_arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_arm.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	left_arm.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	claw.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}

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
	pros::lcd::register_btn1_cb(on_center_button);
	initDrive();
	resetEncoders();
	gyro.reset();
	pros::delay(1000);
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
void competition_initialize() {}
