#include "main.h"
#include "base.h"
#include "stdio.h"
#include <deque>
int speed = 0; //initial motor speed
double error = 0;
double lastError = 0;
double totalError = 0;
double circ = pi*wheelDiameter;

//pros::Mutex mutex = mutex();
pros::task_t raise_claw;
pros::task_t fly;
pros::Task do_fly = fly;
int stage=2; //arm default stage

double inchesToDegrees(double inches){
	return (inches/circ)*degreesPerRotation;
}
double degreesToInches(double degrees){
	return (degrees/degreesPerRotation)*circ;
}

double map(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void setSpeed(int left_speed,int right_speed){
	left_mtr1= left_speed;
	left_mtr2= left_speed;
	right_mtr1 = right_speed;
	right_mtr2 = right_speed;
}
void resetEncoders(){
  left_sensor.reset(); // The encoder is now zero again
	right_sensor.reset(); // The encoder is now zero again
	pros::delay(1500);
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
void debugSwitch (pros::Motor *mtr, pros::ADIDigitalIn *swtch){
mtr->tare_position();
mtr->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
while(true){
	std::cout<< "Switch:" << swtch->get_value() << "	Pos: " << mtr->get_position() << "\n";
	pros::delay(1000);
}
}

// These use multiple params

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

	//mtr->move(0);
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

void drivePID(double target, bool isTurn, pros::ADIEncoder *sensorL, pros::ADIEncoder *sensorR, double Kp, double Ki, double Kd){
		double errorL, errorR, speedL, speedR, sum,targetL,targetR;
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
					targetL = target;
					targetR = -target;
			}
			else{
					targetL = -target;
					targetR = target;
			}
		}
		else{
			targetL = target;
			targetR = target;
		}
		errorL = targetL - sensorL->get_value();
		errorR = targetR - sensorR->get_value();
		std::cout << "Left_E: " << errorL << "	Right_E: " << errorR << "	LPos: " << sensorL->get_value() << "	RPos: " << sensorR->get_value() << '\n';

		while (fabs(errorL)> 2 | fabs(errorR)>2) 	 {
		//		while(!(average < 3 && average > -3)){

		// P
		errorL = targetL - sensorL->get_value();
		errorR = targetR - sensorR->get_value();


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
		speedL = map(errorL*Kp + (errorL-lastErrorL)*Kd + totalErrorL*Ki, -targetL, targetL, -MAX_SPEED, MAX_SPEED);
		speedR = map(errorR*Kp + (errorR-lastErrorR)*Kd + totalErrorR*Ki, -targetR, targetR, -MAX_SPEED, MAX_SPEED);
		if (isTurn){
			if (target>0) //right turn
				setSpeed(speedL,-speedR);
			else //left turn
				setSpeed(-speedL,speedR);
		}
		else
			setSpeed(speedL,speedR);

		// D
		lastErrorL = errorL;
		lastErrorR = errorR;
		//debug
		std::cout << "Left_E: " << errorL << "	Right_E: " << errorR << "	LPos: " << sensorL->get_value() << "	RPos: " << sensorR->get_value() << '\n';
		pros::delay(10);
	}
	setSpeed(0,0);
	pros::delay(100);
	std::cout << "FINAL READINGS" << '\n';
	std::cout << "E: " << error << "	Average: " << average << "	LPos: " << sensorL->get_value() << "	RPos: " << sensorR->get_value() << '\n';
}
void flywheelPID(double target, pros::Motor *mtrL, pros::Motor *mtrR, double Kp, double Ki, double Kd){
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
		int maxSpeed = 127;
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
		if (speedL >= maxSpeed)
			speedL = maxSpeed;
		if (speedR >=maxSpeed)
			speedR = maxSpeed;

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

void armPID(void* a_arg){
	double target = ((arm_arg*)a_arg) -> target;
	pros::Motor *mtr = ((arm_arg*)a_arg) -> mtr;
	double Kp = ((arm_arg*)a_arg) -> Kp;
	double Ki = ((arm_arg*)a_arg) -> Ki;
	double Kd = ((arm_arg*)a_arg) -> Kd;
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
	double integralLimit = 30;
	double sum = error;
	int counter = 0;
	double totalAdjustment = 0;
	std::deque <double> que;
	que.push_front(error); // one element in stack.
	mtr->move(speed);
	error = target - mtr->get_position();
	//while(true){
	while(true){
		error = target - mtr->get_position();


		if (fabs(error) < integralThreshold){
			if (totalError > integralLimit)
				totalError = integralLimit;
			else if (totalError < -integralLimit)
				totalError = -integralLimit;
		}
		else
			totalError = 0;

		totalAdjustment = error*Kp + (error-lastError)*Kd + totalError*Ki;

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

		if ((error-lastError) == 0) //error has not changed
		{
			counter++;
			//if (counter == 10 & fabs(error)>5)
			//	break;
		}
	//	speed =map(totalAdjustment,-target,target, -MAX_SPEED, MAX_SPEED);
		speed = totalAdjustment;
		if (speed >= 127)
			speed = 127;


		mtr->move(127);

		lastError = error;

		//std::cout << "E: " << error << "	Average: " << average << "	Pos: " << mtr->get_position() << '\n';
		std::cout << "E: " << error << "	Pos: " << mtr->get_position() << "	Adj: " << totalAdjustment<< "	Speed: " << speed << "	Actual Speed: " << mtr->get_actual_velocity()<< '\n';
		pros::delay(100);
	}

	//	mtr->move(0);
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
void drivePID(void* d_arg){
		double target = ((drive_arg*)d_arg) ->target;
		bool isTurn = ((drive_arg*)d_arg) -> isTurn;
		pros::ADIEncoder *sensorL = ((drive_arg*)d_arg) ->sensorL;
		pros::ADIEncoder *sensorR = ((drive_arg*)d_arg) ->sensorR;
		double Kp = ((drive_arg*)d_arg) -> Kp;
		double Ki = ((drive_arg*)d_arg) -> Ki;
		double Kd = ((drive_arg*)d_arg) -> Kd;

		double errorL, errorR, speedL, speedR, sum,targetL,targetR;
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
		std::cout<< "TURN: " << isTurn <<"\n";
		if (isTurn)
		{
			if(target>0)//positve target = right turn, negative target = left turn
			{
					targetL = target;
					targetR = -target;
			}
			else{
					targetL = -target;
					targetR = target;
			}
		}
		else{
			targetL = target;
			targetR = target;
		}
		errorL = targetL - sensorL->get_value();
		errorR = targetR - sensorR->get_value();
		std::cout << "TargetL: " << targetL << "	TargetR: "<< targetR << "	Left_E: " << errorL << "	Right_E: " << errorR << "	LPos: " << sensorL->get_value() << "	RPos: " << sensorR->get_value() << '\n';

		while (fabs(errorL)> 2 | fabs(errorR)>2) 	 {
		//		while(!(average < 3 && average > -3)){

		// P
		errorL = targetL - sensorL->get_value();
		errorR = targetR - sensorR->get_value();


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
		speedL = map(errorL*Kp + (errorL-lastErrorL)*Kd + totalErrorL*Ki, -targetL, targetL, -MAX_SPEED, MAX_SPEED);
		speedR = map(errorR*Kp + (errorR-lastErrorR)*Kd + totalErrorR*Ki, -targetR, targetR, -MAX_SPEED, MAX_SPEED);
		if (isTurn){
			if (target>0) //right turn
				setSpeed(speedL,-speedR);
			else //left turn
				setSpeed(-speedL,speedR);
		}
		else
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
void flywheelPID(void* f_arg){
		double target = ((flywheel_arg*)f_arg) ->target;
		pros::Motor *mtrL = ((flywheel_arg*)f_arg) ->mtrL;
		pros::Motor *mtrR = ((flywheel_arg*)f_arg) ->mtrR;
		double Kp = ((flywheel_arg*)f_arg) -> Kp;
		double Ki = ((flywheel_arg*)f_arg) -> Ki;
		double Kd = ((flywheel_arg*)f_arg) -> Kd;

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
		//	fprintf(f,"%d   %f    %f\n",count*250, mtrL->get_actual_velocity(), mtrR->get_actual_velocity());
			count++;
		}
	}
	fclose(f);
	mtrL->move(0);
	mtrR->move(0);
	pros::delay(100);

}

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
void turn (double angle_in_inches){
	// multiply by 2 because only one set of wheels is turning

	//running the turn command as a Task
	drive_arg* arg = new drive_arg();
	arg->target = inchesToDegrees(angle_in_inches);
	arg->isTurn = true;
  arg->sensorL = &left_sensor;
	arg->sensorR = &right_sensor;
  arg->Kp = 1.1;
	arg->Ki = 0.2;
	arg->Kd = 2.75;
	pros::task_t turn = pros::c::task_create(drivePID,arg,TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Turn");
	//running the turn command normally
	//drivePID(inchesToDegrees(angle_in_inches),true,&left_sensor,&right_sensor,1.1,0.2,2.75);
}
void armControl(){
	int rotationDegrees = 0;

	resetArm();

	switch (stage){
		case 1:{
			//go to stage 2
			//rotationTicks = (36/12)*270; //gear ratio driven/driver = 3, 270 degree rotation
			rotationDegrees = -765;

			// default method
			armPID(rotationDegrees, &claw,1,0,0);

			stage = 2;
			break;

		}
		case 2:{
			//rotationDegrees = (36/12)*120; // required encoder degrees from stage 1 position to stage 2;
			//claw.move_relative(405, 100); // Moves rotationTicks forward
			//pros::delay(500);

			//Raise claw using Task
			arm_arg* arg = new arm_arg();
			arg->target = 360;
			arg->mtr = &claw;
			arg->Kp = 1;
			arg->Ki = 0;
			arg->Kd = 0;
			raise_claw = pros::c::task_create(armPID,arg,TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Raise");
			// pros::Task do_raise(raise_claw);
			// do_raise.suspend();
		//	pros::c::task_notify(raise_claw);
			pros::delay(20);

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
			pros::c::task_notify_clear(raise_claw);
			// lower claw here

			stage = 2;
			break;
		}
	}
}

//driven gears  -> driving : 12 -> 60    12 -> 60
// 1 motor rotation = 25 rotations wheel
// 25 *360 = 90000 degrees

bool created = false;
void runFlywheel(double target){

	if (created == false){
		flywheel_arg* arg = new flywheel_arg();
		arg->target = target;
		arg->mtrL = &left_flywheel;
		arg->mtrR = &right_flywheel;
		arg->Kp = 1;
		arg->Ki = 0;
		arg->Kd = 0;
		created=true;
		fly = pros::c::task_create(flywheelPID,arg,TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel");
	}
	// else
//		pros::c::task_resume(fly);
}

void stopFlywheel(){
	if(created){
		pros::c::task_delete(fly);
		left_flywheel=0;
		right_flywheel=0;
		created=false;
	}
}
