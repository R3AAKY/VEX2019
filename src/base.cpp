#include "main.h"
#include "base.h"
#include <deque>
int speed = 0; //initial motor speed
double left_pos = 0;
double right_pos = 0;
double error = 0;
double lastError = 0;
double totalError = 0;
double circ = pi*wheelDiameter;
double robotCirc = 10.75*pi;

int stage=2; //arm default stage

//Inputs inches, outputs proper tick value needed
double inchesToDegrees(double inches){
	return (inches/circ)*degreesPerRotation;
}

double degreesToInches(double degrees)
{
	return (degrees/degreesPerRotation)*circ;
}

void readEncoders(){
	left_pos = left_sensor.get_value();
	right_pos = right_sensor.get_value();
}

void printEncoders(){
	std::cout << "error: " << error <<"	left pos: " << left_pos << "	right pos: " << right_pos << "	angle: " << gyro.get_value() << "\n";
}

void resetEncoders(){
	left_pos = 0;
	right_pos = 0;
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
void turn90degrees (bool direction){
	// multiply by 2 because only one set of wheels is turning
  double setpoint = inchesToDegrees(robotCirc/4); //quarter turn in degrees.
  readEncoders();
	if (direction == isRight){
		left_mtr1.set_reversed(false);
		left_mtr2.set_reversed(false);
		right_mtr1.set_reversed(false);
		right_mtr2.set_reversed(false);
		error = setpoint - left_pos;
	}
	else if (direction != isRight)
	{
		left_mtr1.set_reversed(true);
		left_mtr2.set_reversed(true);
		right_mtr1.set_reversed(true);
		right_mtr2.set_reversed(true);
		error = setpoint - right_pos;
	}

	while(fabs(error)>turn_threshold){
		if (direction == isRight)
			error = setpoint - left_pos; // = setpoint - right_pos1 (in degrees)
		else if (direction == !isRight)
			error = setpoint - right_pos;

		printEncoders();

		if (fabs(error)<=turn_threshold)
			break;

			//speed = error*KP + (error-lastError)*KD + totalError*KI;
		//rspeed = rerror*KP + (rerror-rlastError)*KD + rtotalError*KI;
		if (direction == isRight)
				speed = map(error*KP + (error-lastError)*KD + totalError*KI, -setpoint, setpoint, -MAX_SPEED,MAX_SPEED);
		else if (direction != isRight)
				speed = map(error*KPl + (error-lastError)*KDl + totalError*KIl, -setpoint, setpoint, -MAX_SPEED,MAX_SPEED);

		setSpeed(speed,speed);

		lastError = error;

		if (fabs(error) < integralThreshold)
			totalError += error;
		pros::delay(10);
	}
	setSpeed(0,0);
	lastError = 0;
	totalError = 0;
	readEncoders();
	printEncoders();
	pros::delay(100);
	std::cout << "out of loop!" << "\n\n\n";
	resetEncoders();
	printEncoders();
	left_mtr1.set_reversed(false);
	left_mtr2.set_reversed(false);
	right_mtr1.set_reversed(true);
	right_mtr2.set_reversed(true);
	pros::delay(10);
}

//The actual move straight function that runs the motors
void moveStraight(double distance_in_inches){
	resetEncoders();
 	readEncoders();
  double	setpoint = inchesToDegrees(distance_in_inches);

/* Setpoint, Error, Reference values are position values (in degrees for encoder units).
	 Proporional Controller gain is used for modifying speed/velocity depending on how much error remains (larger error = lower speed required, vice-versa)
*/
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
}

void resetArm(){
	R_ARM_7.tare_position();
	L_ARM_9.tare_position();
	L_CLAW20.tare_position();
	R_ARM_7.set_zero_position(R_ARM_7.get_position());
	L_ARM_9.set_zero_position(L_ARM_9.get_position());
	L_CLAW20.set_zero_position(L_CLAW20.get_position());
	pros::delay(1000);
}

void armControl(){
	int rotationTicks = 0;
	int armSpeed = 0;
	int arm_rotationTicks = 0;
	resetArm();

	switch (stage){
		case 1:{
			//go to stage 2
			//rotationTicks = (36/12)*270; //gear ratio driven/driver = 3, 270 degree rotation
			rotationTicks = 650;
			double setpoint = rotationTicks;
			error = setpoint - L_CLAW20.get_position();
			while(fabs(error)>5){
				error = setpoint - L_CLAW20.get_position(); // = setpoint - right_pos1 (in degrees)
				if (fabs(error)<=5)
					break;
				std::cout << "Error: " << error << "	Position: " << L_CLAW20.get_position() << "		Motor Power: " << L_CLAW20.get_power() <<'\n';

				armSpeed = map(error, -setpoint, setpoint, -MAX_SPEED,MAX_SPEED);
				L_CLAW20 = armSpeed;
			//	L_CLAW20.move_absolute(rotationTicks, armSpeed); // Moves rotationTicks forward
				lastError = error;

				if (fabs(error) < integralThreshold)
					totalError += error;
				pros::delay(10);
			}

		/*	L_CLAW20.move_absolute(rotationTicks, armSpeed); // Moves rotationTicks forward
		  while (!(fabs(L_CLAW20.get_position())< rotationTicks+2)) {
		    // Continue running this loop as long as the motor is not within +-2 units of its goal
				std::cout << "Position: " << L_CLAW20.get_position() << '\n';
				pros::delay(2);

		  }
			*/
			/*
				L_CLAW20 = 30;
				while (limit_switch1.get_value()!= 1){
				pros::delay(2);
				}
				L_CLAW20 = 0;
*/
			stage = 2;
			break;

		}
		case 2:{
			rotationTicks = -(36/12)*120; // required encoder degrees from stage 1 position to stage 2;
			L_CLAW20.move_relative(rotationTicks, 100); // Moves rotationTicks forward
			//armPID(rotationTicks, &L_CLAW20, 1, 0, 0);
			/*arm_rotationTicks = (84/12)*90/2 ;//divided by 2 because two motors
			L_ARM_9.move_relative(arm_rotationTicks, 100);
			R_ARM_7.move_relative(arm_rotationTicks, 100);
			while (!((fabs(L_CLAW20.get_position()) < rotationTicks+2)
			&& (fabs(R_ARM_7.get_position()) < arm_rotationTicks+2)
			&& (fabs(L_ARM_9.get_position()) < arm_rotationTicks+2))){
				// Continue running this loop as long as the motor is not within +-2 units of its goal
				std::cout << "Claw: " << L_CLAW20.get_position() << "		R_Arm: " << R_ARM_7.get_position() << "		L_Arm: " << L_ARM_9.get_position() << '\n';
				pros::delay(2);
			}
			*/
/*
				L_ARM_9 = 100;
				R_ARM_7 = 100;
				while (limit_switch1.get_value()!= 1){
				pros::delay(2);
				}
				std::cout <<"L: " << L_ARM_9.get_position() << "		R: " << R_ARM_7.get_position() << "\n";
				L_ARM_9 = 0;
				R_ARM_7 = 0;

			stage = 3;
			*/
				std::cout <<"L: " << L_ARM_9.get_position() << "		R: " << R_ARM_7.get_position() << "\n";
			break;
		}
		case 3:{
			/*rotationTicks = +(36/12)*90;
			L_CLAW20.move_relative(rotationTicks, 100); // Moves rotationTicks forward

			arm_rotationTicks = -(84/12)*90/2 ;//divided by 2 because two motors
			L_ARM_9.move_relative(arm_rotationTicks, 100);
			R_ARM_7.move_relative(arm_rotationTicks, 100);
			while (!((fabs(R_ARM_7.get_position()) < arm_rotationTicks+2) && (fabs(L_ARM_9.get_position()) < arm_rotationTicks+2))){
				// Continue running this loop as long as the motor is not within +-2 units of its goal
				std::cout << "Claw: " << L_CLAW20.get_position() << "		R_Arm: " << R_ARM_7.get_position() << "		L_Arm: " << L_ARM_9.get_position() << '\n';
				pros::delay(2);
			}

			std::cout <<"L: " << L_ARM_9.get_position() << "		R: " << R_ARM_7.get_position() << "\n";
			L_ARM_9 = 0;
			R_ARM_7 = 0;
*/
			break;
		}
	}
}


	//Using Integrated Encoders
	void armPID(double target,pros::Motor *mtr, double Kp, double Ki, double Kd){
		double error = target - mtr->get_position();
		double lastError = 0;
		double totalError = 0;
		double average = 0;
		double integralLimit = 10;
		double sum = error;
		std::deque <double> que;
		que.push_front(error); // one element in stack.

		while(!(fabs(error)< 5)){
			error = target - mtr->get_position();
			speed = map(error*Kp + (error-lastError)*Kd + totalError*Ki, -target, target, -MAX_SPEED, MAX_SPEED);
			*mtr = speed;

			lastError = error;

			if (fabs(error) < integralThreshold){
				if (totalError > integralLimit)
					totalError = integralLimit;
				else if (totalError < -integralLimit)
					totalError = -integralLimit;
			}
			else
				totalError = 0;
			//LIMITED STACK TO 5 ELEMENTS (error readings) for averaging
			if (que.size()<5){
			//stack not full yet, keep pushing elements in and summing individually.
			que.push_front(error);
			sum += error;
			}
			else{
			//stack is full now, pop last element out, then push new one in. Update sum accordingly.
			sum += error - que.back();
			que.push_front(error);
			}
			average = sum/(size(que));

			std::cout << "E: " << error << "	Average: " << average << "	Pos: " << mtr->get_position() << '\n';
			pros::delay(10);
		}
		*mtr = 0;
		pros::delay(100);
		std::cout << "FINAL READINGS" << '\n';
		std::cout <<"E: " << error << "	Average: " << average << "	Pos: " << mtr->get_position() << '\n';
	}

	//Using Quad Encoders
	void drivePID(double target, pros::ADIEncoder *sensorL, pros::ADIEncoder *sensorR, double Kp, double Ki, double Kd){
			double errorL, errorR, speedL, speedR, sum;
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

			while (!((fabs(errorL)< 1) && (fabs(errorR) < 1))) 	 {
//		while(!(average < 3 && average > -3)){

			// P
			errorL = target - sensorL->get_value();
			errorR = target - sensorR->get_value();

			// D
			lastErrorL = errorL;
			lastErrorR = errorR;

			// I - starts summing after a certain threshold and ensures total error doesn't become very large
			if (fabs(error) < integralThreshold){
				if (totalErrorL > integralLimit)
					totalError = integralLimit;
				else if (totalErrorL < -integralLimit)
					totalErrorL = -integralLimit;
			}
			else
				totalError = 0;

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
			speedL = map(errorL*Kp + (errorL-lastErrorL)*Kd + totalErrorL*Ki, -target, target, -MAX_SPEED, MAX_SPEED);
			speedR = map(errorR*Kp + (errorR-lastErrorR)*Kd + totalErrorR*Ki, -target, target, -MAX_SPEED, MAX_SPEED);
			setSpeed(speedL,speedR);

			//debug
			std::cout << "E: " << error << "	Average: " << average << "	LPos: " << sensorL->get_value() << "	RPos: " << sensorR->get_value() << '\n';
			pros::delay(10);
		}
		setSpeed(0,0);
		pros::delay(100);
		std::cout << "FINAL READINGS" << '\n';
		std::cout << "E: " << error << "	Average: " << average << "	LPos: " << sensorL->get_value() << "	RPos: " << sensorR->get_value() << '\n';
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

void flyWheel()
{
	R_FLY_8.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	L_FLY_10.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

	R_FLY_8.tare_position();
	L_FLY_10.tare_position();
	R_FLY_8.set_zero_position(R_FLY_8.get_position());
	L_FLY_10.set_zero_position(L_FLY_10.get_position());

	pros::delay(1000);
//	R_FLY_8.move_absolute(360,50);
//	L_FLY_10.move_absolute(360,50);

	R_FLY_8 = 40;
	L_FLY_10 = 40;
while (true){
	std::cout << "L Pos: " << L_FLY_10.get_position() << "		R Pos: " << R_FLY_8.get_position() << "		L Vel/R Vel: " << L_FLY_10.get_actual_velocity() <<'/' << R_FLY_8.get_actual_velocity() << "\n";
	pros::delay(1000);
	}

//driven gears  -> driving : 12 -> 60    12 -> 60
// 1 motor rotation = 25 rotations wheel
// 25 *360 = 90000 degrees


}

void flyWheelPID(int targetFlyWheelSpeed){
	//Constants//
	//R_FLY_8.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	//L_FLY_10.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

	R_FLY_8.tare_position();
	L_FLY_10.tare_position();
	R_FLY_8.set_zero_position(R_FLY_8.get_position());
	L_FLY_10.set_zero_position(L_FLY_10.get_position());
pros::delay(1000);
double kp = 0.1;
double ki = 0.05;
double kd = 0.07;

int currentFlyWheelVoltage;
L_FLY_10 = targetFlyWheelSpeed;
R_FLY_8 = targetFlyWheelSpeed;
//PID Variables Here//
int currentVelocity = L_FLY_10.get_actual_velocity();
int error = targetFlyWheelSpeed - currentVelocity;
int lastError = 0;
int totalError = 0;
int integralActiveZone = 5;
int integralLimit = 10;
//int onTargetCount = 0;
double finalAdjustment = error * kp; //add the rest of PID to this calculation

//Temp Variable//
int deltaTime = 0;

while (true)
{

		currentVelocity = L_FLY_10.get_actual_velocity();

		error = targetFlyWheelSpeed - currentVelocity;

		if (error == 0)
		{
			lastError = 0;
		}

		if (abs(error) < integralActiveZone && error != 0)
		{
			totalError += error;
			if (totalError > integralLimit)
				totalError = integralLimit;
			else if (totalError < -integralLimit)
				totalError = -integralLimit;
		}
		else
		{
			totalError = 0;
		}

		finalAdjustment = ((error * kp) + (totalError * ki) + ((error - lastError) * kd));
		currentFlyWheelVoltage += finalAdjustment;

		if (currentFlyWheelVoltage > 127)
		{
			currentFlyWheelVoltage = 127;
		}
		else if (currentFlyWheelVoltage < 0)
		{
			currentFlyWheelVoltage = 0;
		}
		std::cout << "Error: "<<error << "	IntegralError: " << totalError << "	Adjustment+: " << finalAdjustment << " CurrentVoltage: " << currentFlyWheelVoltage <<"	L Pos: " << L_FLY_10.get_position() << "	R Pos: " << R_FLY_8.get_position() << "	L Vel/R Vel: " << L_FLY_10.get_actual_velocity() <<'/' << R_FLY_8.get_actual_velocity() << "\n";

		L_FLY_10 = currentFlyWheelVoltage;
    lastError = error;
		pros::delay(10);
}
}
