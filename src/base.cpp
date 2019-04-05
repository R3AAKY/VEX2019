#include "main.h"
#include "base.h"
#include "stdio.h"
#include <deque>
int speed = 0; //initial motor speed
double error = 0;
double lastError = 0;
double totalError = 0;
double circ = pi*wheelDiameter;

int stage=1; //arm default stage

//Inputs inches, outputs proper tick value needed
double inchesToDegrees(double inches){
	return (inches/circ)*degreesPerRotation;
}

double degreesToInches(double degrees)
{
	return (degrees/degreesPerRotation)*circ;
}


void resetEncoders(){
  left_sensor.reset(); // The encoder is now zero again
	right_sensor.reset(); // The encoder is now zero again
	pros::delay(1500);
}

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setSpeed(int left_speed,int right_speed){
	left_mtr1= left_speed;
	left_mtr2= left_speed;
	right_mtr1 = right_speed;
	right_mtr2 = right_speed;
}

// Found 90 degree turn to be 0.6875 rotations.
// (false) for left turn, isRight (true) for right turn
void turn (double angle_in_inches){
	// multiply by 2 because only one set of wheels is turning
	drivePID(inchesToDegrees(angle_in_inches),true,&left_sensor,&right_sensor,1.1,0.2,2.75);

}

//The actual move straight function that runs the motors
void moveStraight(double distance_in_inches){
	resetEncoders();
	drivePID(inchesToDegrees(distance_in_inches),false,&left_sensor,&right_sensor,1.1,0.2,2.75);

	/*
  double	setpoint = inchesToDegrees(distance_in_inches);

	//movePID(setpoint,left_mtr2)
	while(true){
		error = setpoint - left_pos; // = setpoint - right_pos1 (in degrees)
		printEncoders();
		if (fabs(error)<=threshold)
			break;

		speed = map (error*2 + (error-lastError)*KD + totalError*KI, -setpoint,setpoint,-MAX_SPEED,MAX_SPEED);
		//speed = error*KP + (error-lastError)*KD + totalError*KI;
		setSpeed(speed,speed);

		lastError = error;

		if (fabs(error) < integralThreshold)
			totalError += error;
		pros::delay(50);
		printEncoders();
		readEncoders();
	}
	setSpeed(0,0);

	resetEncoders();
	pros::delay(10);
	*/
}

void resetArm(){
	right_arm.tare_position();
	left_arm.tare_position();
	claw.tare_position();
	right_arm.set_zero_position(right_arm.get_position());
	left_arm.set_zero_position(left_arm.get_position());
	claw.set_zero_position(claw.get_position());
	pros::delay(1000);
}

void armControl(){
	int rotationDegrees = 0;
	int armSpeed = 0;
	int arm_rotationDegrees = 0;
	resetArm();

	switch (stage){
		case 1:{
			//go to stage 2
			//rotationTicks = (36/12)*270; //gear ratio driven/driver = 3, 270 degree rotation
			rotationDegrees = -765;
			armPID(rotationDegrees, &claw,1,0,0);


			/* WITH LIMIT SWITCH
				claw = 30;
				while (limit_switch1.get_value()!= 1){
				pros::delay(2);
				}
				claw = 0;
				*/
			stage = 2;
			break;

		}
		case 2:{
			//	rotationDegrees = -(36/12)*120; // required encoder degrees from stage 1 position to stage 2;
				//claw.move_relative(405, 100); // Moves rotationTicks forward
				//pros::delay(500);
				//armPID(rotationTicks, &claw, 1, 0, 0);
				/*arm_rotationTicks = (84/12)*90/2 ;//divided by 2 because two motors
				left_arm.move_relative(arm_rotationTicks, 100);
				right_arm.move_relative(arm_rotationTicks, 100);
				while (!((fabs(claw.get_position()) < rotationTicks+2)
				&& (fabs(right_arm.get_position()) < arm_rotationTicks+2)
				&& (fabs(left_arm.get_position()) < arm_rotationTicks+2))){
					// Continue running this loop as long as the motor is not within +-2 units of its goal
					std::cout << "Claw: " << claw.get_position() << "		R_Arm: " << right_arm.get_position() << "		L_Arm: " << left_arm.get_position() << '\n';
					pros::delay(2);
				}
				*/


				left_arm = 110;
				right_arm = 110;
				while (limit_switch1.get_value()!= 1){
				pros::delay(2);
				}
				std::cout <<"L: " << left_arm.get_position() << "		R: " << right_arm.get_position() << "\n";
				left_arm = 0;
				right_arm = 0;

			stage = 3;
				std::cout <<"L: " << left_arm.get_position() << "		R: " << right_arm.get_position() << "\n";
			break;
		}
		case 3:{
					//	rotationDegrees = (36/12)*170;
					//	armPID(rotationDegrees, &claw, 1.5,0,0);

						 // Moves rotationTicks forward
			/*
						arm_rotationTicks = -(84/12)*90/2 ;//divided by 2 because two motors
						left_arm.move_relative(arm_rotationTicks, 100);
						right_arm.move_relative(arm_rotationTicks, 100);
						while (!((fabs(right_arm.get_position()) < arm_rotationTicks+2) && (fabs(left_arm.get_position()) < arm_rotationTicks+2))){
							// Continue running this loop as long as the motor is not within +-2 units of its goal
							std::cout << "Claw: " << claw.get_position() << "		R_Arm: " << right_arm.get_position() << "		L_Arm: " << left_arm.get_position() << '\n';
							pros::delay(2);
						}

						std::cout <<"L: " << left_arm.get_position() << "		R: " << right_arm.get_position() << "\n";
						left_arm = 0;
						right_arm = 0;
			*/

			left_arm = -80;
			right_arm = -80;
				while (limit_switch2.get_value()!= 1){
				pros::delay(2);
				}
				std::cout <<"L: " << left_arm.get_position() << "		R: " << right_arm.get_position() << "\n";
				left_arm = 0;
				right_arm = 0;

			stage = 4;
				std::cout <<"L: " << left_arm.get_position() << "		R: " << right_arm.get_position() << "\n";
			break;
		}
		case 4:{

			break;
		}
	}
}


	//Using Integrated Encoders
	void armPID(double target,pros::Motor *mtr, double Kp, double Ki, double Kd){
		bool rev = false;
		if (target <0) //negative
		{
		 rev = true;
			target = -target;
			if (mtr->is_reversed())
				mtr->set_reversed(false);
			else
				mtr->set_reversed(true);
		}
		double error = target - mtr->get_position();
		double lastError = 0;
		double totalError = 0;
		double speed = 50;
		double average = 0;
		double integralLimit = 10;
		double sum = error;
		int counter = 0;
		std::deque <double> que;
		que.push_front(error); // one element in stack.
		mtr->move(speed);
		error = target - mtr->get_position();
		while(!(fabs(error)< 10)){
			error = target - mtr->get_position();


			if (fabs(error) < integralThreshold){
				if (totalError > integralLimit)
					totalError = integralLimit;
				else if (totalError < -integralLimit)
					totalError = -integralLimit;
			}
			else
				totalError = 0;
			//LIMITED STACK TO 5 ELEMENTS (error readings) for averaging
			if (que.size()<3){
			//stack not full yet, keep pushing elements in and summing individually.
			que.push_front(error);
			sum += error;
			}
			else{
			//stack is full now, pop last element out, then push new one in. Update sum accordingly.
			sum += error - que.back();
			que.push_front(error);
			}
			if ((error-lastError) == 0)
			{
				counter++;
				if (counter == 15)
					break;
			}
			speed =map(error*Kp + (error-lastError)*Kd + totalError*Ki,-target,target, -MAX_SPEED, MAX_SPEED);
			if (speed >= 127)
				speed = 127;


			mtr->move(speed);

			lastError = error;

			//std::cout << "E: " << error << "	Average: " << average << "	Pos: " << mtr->get_position() << '\n';
			std::cout << "E: " << error << "	Pos: " << mtr->get_position() << '\n';
			pros::delay(100);
		}

		mtr->move(0);
		if (rev==true)
		{//unreverse
		 	if (mtr->is_reversed())
				mtr->set_reversed(false);
			else
				mtr->set_reversed(true);
			rev = false;
		}
		mtr->tare_position();
		mtr->set_zero_position(mtr->get_position());

		pros::delay(1000);
		std::cout << "FINAL READINGS" << '\n';
		std::cout <<"E: " << error << "	Average: " << average << "	Pos: " << mtr->get_position() << '\n';
	}

	//Using Quad Encoders
	void drivePID(double target, bool isTurn, pros::ADIEncoder *sensorL, pros::ADIEncoder *sensorR, double Kp, double Ki, double Kd){
			double errorL, errorR, speedL, speedR, sum;
			int adjustmentL =0;
			int adjustmentR = 0;
			double average = 0;
			double lastErrorL = 0;
			double lastErrorR = 0;
			double totalErrorL = 0;
			double totalErrorR = 0;
 			double integralLimit = 10;
			std::deque <double> que;
			que.push_front(error); // one element
			sensorL->reset();
			sensorR->reset();
			pros::delay(1000);
			if (isTurn)
			{
				if(target>0)//positve target = right turn, negative target = left turn
				{
						//reverse right side motors
						if (right_mtr1.is_reversed() && right_mtr2.is_reversed()){
							right_mtr1.set_reversed(false);
							right_mtr2.set_reversed(false);
						}
						else{
							right_mtr1.set_reversed(true);
							right_mtr2.set_reversed(true);
						}
				}
				else{
						//reverse left side motors
						if (left_mtr1.is_reversed() && left_mtr2.is_reversed()){
							left_mtr1.set_reversed(false);
							left_mtr2.set_reversed(false);
						}
						else{
							left_mtr1.set_reversed(true);
							left_mtr2.set_reversed(true);
						}
						target = -target;
				}

			}
			errorL = target - sensorL->get_value();
			errorR = target + sensorR->get_value();
			std::cout << "Left_E: " << errorL << "	Right_E: " << errorR << "	LPos: " << sensorL->get_value() << "	RPos: " << sensorR->get_value() << '\n';

			while (fabs(errorL)> 2 | fabs(errorR)>2) 	 {
//		while(!(average < 3 && average > -3)){

			// P
			errorL = target - sensorL->get_value();
			errorR = target + sensorR->get_value();


			// I - starts summing after a certain threshold and ensures total error doesn't become very large
			if (fabs(errorL) < integralThreshold){
				totalErrorL += errorL;
				if (totalErrorL > integralLimit)
					totalErrorL = integralLimit;
				else if (totalErrorL < -integralLimit)
					totalErrorL = -integralLimit;
			}
			else
				totalErrorL = 0;

			if (fabs(errorR) < integralThreshold){
				totalErrorR += errorR;
				if (totalErrorR > integralLimit)
					totalErrorR = integralLimit;
				else if (totalErrorR < -integralLimit)
					totalErrorR = -integralLimit;
			}
			else
				totalErrorR =0;

/*Not sure what to do with the averaging.
			if (que.size()<5){
			que.push_front(error);
			sum += error;
			}
			else{
			sum += error - que.back();
			que.push_front(error);
			}
			average = sum/(size(que));
*/
			speedL = map(errorL*Kp + (errorL-lastErrorL)*Kd + totalErrorL*Ki, -target, target, -MAX_SPEED, MAX_SPEED);
			speedR = map(errorR*Kp + (errorR-lastErrorR)*Kd + totalErrorR*Ki, -target, target, -MAX_SPEED, MAX_SPEED);
			setSpeed(speedL,speedR);

			// D
			lastErrorL = errorL;
			lastErrorR = errorR;
			//debug
			std::cout << "Left_E: " << errorL << "	Right_E: " << errorR << "	LPos: " << sensorL->get_value() << "	RPos: " << sensorR->get_value() << '\n';
			pros::delay(10);
		}
		setSpeed(0,0);
		left_mtr1.set_reversed(false);
		left_mtr2.set_reversed(false);
		right_mtr1.set_reversed(true);
		right_mtr2.set_reversed(true);
		pros::delay(100);
		std::cout << "FINAL READINGS" << '\n';
		std::cout << "E: " << error << "	Average: " << average << "	LPos: " << sensorL->get_value() << "	RPos: " << sensorR->get_value() << '\n';
	}

	void flyWheelPID(int target, pros::Motor *mtrL, pros::Motor *mtrR, double Kp, double Ki, double Kd){
			double errorL, errorR, speedL, speedR, sum;
			double average = 0;
		 	double lastErrorL = 0;
			double lastErrorR = 0;
			double totalErrorL = 0;
			double totalErrorR = 0;
			int integralThresh = 5;
			int integralLimit = 100;
		 	double adjustmentL =0;
			double adjustmentR = 0;
			std::deque <double> que;
			que.push_front(error); // one element
			mtrL->tare_position();
			mtrL->set_zero_position(mtrL->get_position());
			mtrR->tare_position();
			mtrR->set_zero_position(mtrR->get_position());
			pros::delay(550);
			speedL = target;
			speedR = target;

			std::FILE * f = fopen("/usd/data.dat","w");
			int count = 1;
			int delayTime = 5;
			while (true) 	 {
		//		if	(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)!=0)
		 	// 		break;
			// P
			errorL = target - mtrL->get_actual_velocity();
			errorR = target - mtrR->get_actual_velocity();



			// I - starts summing after a certain threshold and ensures total error doesn't become very large
			if (fabs(errorL) < integralThresh){
				totalErrorL += errorL;
				if (totalErrorL > integralLimit)
					totalErrorL = integralLimit;
				else if (totalErrorL < -integralLimit)
					totalErrorL = -integralLimit;
			}
			else
				totalErrorL = 0;

			if (fabs(errorR) < integralThresh){
				totalErrorR += errorR;
				if (totalErrorR > integralLimit)
					totalErrorR = integralLimit;
				else if (totalErrorR < -integralLimit)
					totalErrorR = -integralLimit;
			}
			else
				totalErrorR =0;


	/* Not sure what to do with the averaging.
			if (que.size()<5){
			que.push_front(error);
			sum += error;
			}
			else{
			sum += error - que.back();
			que.push_front(error);
			}
			average = sum/(size(que));
	*/
			//speedL = map(errorL*Kp + (errorL-lastErrorL)*Kd + totalErrorL*Ki, -target, target, -MAX_SPEED, MAX_SPEED);
			//speedR = map(errorR*Kp + (errorR-lastErrorR)*Kd + totalErrorR*Ki, -target, target, -MAX_SPEED, MAX_SPEED);
			adjustmentL = errorL*Kp + (errorL-lastErrorL)*Kd + totalErrorL*Ki;
			adjustmentR = errorR*Kp + (errorR-lastErrorR)*Kd + totalErrorR*Ki;
			speedL += adjustmentL;
			speedR += adjustmentR;
			if (speedL >= 127)
				speedL = 127;
			if (speedR >=127)
				speedR = 127;

			mtrL->move(speedL);
			mtrR->move(speedR);

			//debug
			//std::cout << (*mtrL).get_position() << "	Left_E: " << errorL << "	Right_E: " << errorR <<  "	LPos: " << mtrL->get_position() << "	RPos: " << mtrR->get_position() << "	L Vel/R Vel: " << mtrL->get_actual_velocity() <<'/' << mtrR->get_actual_velocity() << '\n';
			//std::cout << "	Left_S: " << speedL << "	Right_S: " << speedR  << "	L_Deriv: "<< (errorL-lastErrorL) << "	R_Deriv: " << (errorR - lastErrorR) << "	L_Adjustment: " << adjustmentL << "	R_Adjustment: " << adjustmentR << "	L Vel/R Vel: "<< mtrL->get_actual_velocity() <<'/' << mtrR->get_actual_velocity() << '\n';
			std::cout << "	Left_S: " << speedL << "	Right_S: " << speedR  << "	L_Int: "<< totalErrorL << "	R_Int: " << totalErrorR << "	L_Adjustment: " << adjustmentL << "	R_Adjustment: " << adjustmentR << "	L Vel/R Vel: "<< mtrL->get_actual_velocity() <<'/' << mtrR->get_actual_velocity() << '\n';


			// D
			lastErrorL = errorL;
			lastErrorR = errorR;

			pros::delay(250);
			if (count==120)
				break;
			else
			{
				fprintf(f,"%d   %f    %f\n",count*250, mtrL->get_actual_velocity(), mtrR->get_actual_velocity());
				count++;
			}
		}
		fclose(f);
		mtrL->move (0);
		mtrR->move(0);
		pros::delay(100);

	}

	void debugSwitch (pros::Motor *mtr, pros::ADIDigitalIn *swtch){
		mtr->tare_position();
		mtr->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
		while(true){
		  std::cout<< "Switch:" << swtch->get_value() << "	Pos: " << mtr->get_position() << "\n";
		  pros::delay(1000);
		}
	}

/*
	typedef struct {
  int speed;
  bool reverse;
} my_task_arg;

void my_task_fn(void* argument) {
  // Following two lines get the members of the my_task_arg struct
  int speed = ((my_task_arg*)argument)->speed;
  bool reverse = ((my_task_arg*)argument)->reverse;
  std::cout << "Speed is " << speed << "; reverse is: " << reverse << std::endl;
}

void opcontrol() {
  my_task_arg* argument = new my_task_arg();
  argument->speed = 100;
  argument->reverse = false;
  pros::Task my_task(my_task_fn, argument);
}
*/

//driven gears  -> driving : 12 -> 60    12 -> 60
// 1 motor rotation = 25 rotations wheel
// 25 *360 = 90000 degrees
