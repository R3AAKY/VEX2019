#include "main.h"
#include "base.h"
#include <fstream>

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
 /*
 typedef struct {
   double target;
   pros::ADIEncoder left_sensor;
   pros::ADIEncoder right_sensor;
   double Kp, Kd, Ki;
 } drive_arg;

// CONTROLLER
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
*/

void opcontrol() {
/*
  std::ofstream f;
  std::cout << "text" << '\n';
  f.open("shooter.csv",std::ios::in | std::ios::out | std::ios::binary);
  int count =0;
  while(count !=20)
  {

  for (int i= 0; i <20; i++)
    f << i << "," << i+i << '\n';
    count++;
  }
  f.close();
  */
//TUNED 0-40
//  flyWheelPID(40, &L_FLY_10, &R_FLY_8, 0.0035, 0, 1.1);
//flyWheelPID(50, &L_FLY_10, &R_FLY_8, 0.0035, 0.04, 0);
//TUNED 90-120
INTAKE_19 = 80;
flyWheelPID(100,&L_FLY_10, &R_FLY_8, 0.122, 0, 1.1);

/*
L_FLY_10 = 40;
R_FLY_8 = 40;
while(true){
  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)==1)
      break;
std::cout << "	L Pos: " << L_FLY_10.get_position() << "	R Pos: " << R_FLY_8.get_position() << "	L Vel/R Vel: " << L_FLY_10.get_actual_velocity() <<'/' << R_FLY_8.get_actual_velocity() << "\n";
pros::delay(10);
}
L_FLY_10 = 0;
R_FLY_8 = 0;
*/
 /*
 while (true) {
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		if(abs(left)<5){
      L_FLY_10 = 0;
      R_FLY_8 = 0;
		}else{
			L_FLY_10 = left;
			R_FLY_8 = left;
		}

		if(abs(right)<5){
			INTAKE_19 = 0;

		}else{
		  INTAKE_19 = right;
		}

		pros::delay(20);
	}
  */

 //L_CLAW20 = 100;
//pros::Task drive_task(drivePID,(10,left_sensor,right_sensor,KP,KD,KI));
//armControl();
//armControl();
//armControl();
//flyWheelPID(40);
/*
L_CLAW20.tare_position();
L_CLAW20.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
while(true){
  std::cout<< "L:" << L_CLAW20.get_position() << "\n";
  pros::delay(1000);
}

/*
while (true){
std::cout<<limit_switch2.get_value() << '\n';
pros::delay(1000);
}
*/
}



//prosv5 upload --slot 1
//prosv5 v5 rm-file slot_4.bin --erase-all
