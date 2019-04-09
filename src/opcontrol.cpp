#include "main.h"
#include "base.h"
#include <stdio.h>

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
*/

void controller(){
  bool shooterOn = false;
  bool intakeOn = false;
	double shooterSpeed = 100;
  int intakeSpeed = 127;
  while (true) {
 		int left = master.get_analog(ANALOG_LEFT_Y);
 		int right = -master.get_analog(ANALOG_RIGHT_Y);

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
 			right_mtr1 = -right;
 			right_mtr2 = -right;
 		}

 		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
    {
 			//shooterOn = true;
      runFlywheel(127);

    }

 		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
 		//shooterOn = false;
    stopFlywheel();
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
 			intakeOn = true;

 		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
 			intakeOn = false;

    //Raise Arm
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
      left_arm = 100;
      right_arm = 100;
      while (limit_switch1.get_value()!= 1){
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
          break;
      pros::delay(20);
      }
      left_arm = 0;
      right_arm = 0;
    }

    //Lower Arm
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
      left_arm = -65;
      right_arm = -65;
      while (limit_switch2.get_value()!= 1){
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
          break;
      pros::delay(20);
      }
      left_arm = 0;
      right_arm = 0;
    }

    //Raise Claw
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
      claw.move_relative(375,100);
    }

    //Lower Claw
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {

        claw.move_relative(-375,-70);
    }


    if(intakeOn)
   		intake = intakeSpeed;
    else
 		  intake = 0;
  }
}


void debugArm(void* param){
  resetArm();
  while(true){
    //std::cout<< "   L_CLAW: " << claw.get_position() << " L_ARM: " << left_arm.get_position() << "  R_ARM: " << right_arm.get_position() <<  "\n";
    std::cout<< "   L_CLAW: " << claw.get_position() <<"\n";
    pros::delay(500);
  }
}

void debugDrive(void* param){
  while(true){
  	std::cout <<"	Left pos: " << left_sensor.get_value() << "	Right pos: " << right_sensor.get_value() << "	Angle: " << gyro.get_value() << "\n";
    pros::delay(500);
  }
}

void multitask_test(){
  // example of multitask
  pros::task_t my_task = pros::c::task_create(debugArm,NULL,TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Arm");
//  pros::task_t my_task_2 = pros::c::task_create(debugDrive,NULL,TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Drive");
  pros::Task debug_arm_task(my_task);
//s  pros::Task debug_drive_task(my_task_2);
}

void opcontrol() {
  controller();

//  multitask_test();
// TUNED 0-40
//  flywheelPID(40, &left_flywheel, &right_flywheel, 0.0035, 0, 1.1);
//flywheelPID(50, &left_flywheel, &right_flywheel, 0.0035, 0.04, 0);
//TUNED 90-120
//INTAKE_19 = 80;
//flywheelPID(127,&left_flywheel, &right_flywheel, 0.122, 0, 1.1);
//left_flywheel.move_velocity(127);
//right_flywheel=127;
/*
left_flywheel = 40;
right_flywheel = 40;
while(true){
  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)==1)
      break;
std::cout << "	L Pos: " << left_flywheel.get_position() << "	R Pos: " << right_flywheel.get_position() << "	L Vel/R Vel: " << left_flywheel.get_actual_velocity() <<'/' << right_flywheel.get_actual_velocity() << "\n";
pros::delay(10);
}
left_flywheel = 0;
right_flywheel = 0;
*/
//moveStraight(5);
//turn(TURN_90);
//turn(-TURN_90);
// while(true)
// {
//   std::cout << left_sensor.get_value() << '\n';
//   if (left_mtr1.is_reversed()==false)
//   {
//     std::cout << left_mtr1.is_reversed() <<"  FALSE!" <<'\n';
//   left_mtr1.set_reversed(true);
//   left_mtr1 = 30;
//   }
//   else{
//     std::cout << left_mtr1.is_reversed() <<"  TRUE!" <<'\n';
//     left_mtr1.set_reversed(false);
//     left_mtr1 = 30;
//   }
//
//   pros::delay(1500);
//
//
// }

 /*
 while (true) {
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		if(abs(left)<5){
      left_flywheel = 0;
      right_flywheel = 0;
		}else{
			left_flywheel = left;
			right_flywheel = left;
		}

		if(abs(right)<5){
			INTAKE_19 = 0;

		}else{
		  INTAKE_19 = right;
		}

		pros::delay(20);
	}
  */

 //claw = 100;
 //claw = 127;
//pros::Task drive_task(drivePID,(10,left_sensor,right_sensor,KP,KD,KI));
//armControl();
//armControl();
//armControl();

	//claw.move_relative(370,50);


/*
while (true){
std::cout<<limit_switch2.get_value() << '\n';
pros::delay(1000);
}
*/

}



//prosv5 upload --slot 1
//prosv5 v5 rm-file slot_4.bin --erase-all
