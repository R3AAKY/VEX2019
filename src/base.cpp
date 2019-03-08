#include "main.h"
#include "base.h"

int speed = 0; //initial motor speed
double left_pos = 0;
double right_pos = 0;
double error = 0;
double last_error = 0;
double total_error = 0;
double circ = pi*wheelDiameter;
double robotCirc = 11*pi;


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
	std::cout << "error: " << error << "			left position: " << left_pos << "			right position: " << right_pos << "\n";
}

void resetEncoders(){
	left_pos = 0;
	right_pos = 0;
  left_sensor.reset(); // The encoder is now zero again
	right_sensor.reset(); // The encoder is now zero again
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
  double setpoint = 2*inchesToDegrees(robotCirc/4); //quarter turn in degrees.
	resetEncoders();
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
	while(true){
		if (direction == isRight)
			error = setpoint - left_pos; // = setpoint - right_pos1 (in degrees)
		else if (direction != isRight)
			error = setpoint - right_pos;
		printEncoders();
		if (fabs(error)<=turn_threshold)
			break;
		//speed = map(error*KP + (error-last_error)*KD + total_error*KI, -setpoint, setpoint, -MAX_SPEED,MAX_SPEED);
		speed = error*KP + (error-last_error)*KD + total_error*KI;
		if (direction == isRight)
			setSpeed(speed,0);
		else if (direction != isRight)
			setSpeed(0,speed);

		last_error = error;
		if (fabs(error) < integral_threshold)
			total_error += error;
		pros::delay(50);
		readEncoders();
	}
	setSpeed(0,0);

//	printEncoders();
	pros::delay(100);
	//std::cout << "out of loop!" << "\n" << "\n" << "\n";
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
	while(true){
		error = setpoint - left_pos; // = setpoint - right_pos1 (in degrees)
		printEncoders();
		if (fabs(error)<=threshold)
			break;

		speed = error*KP + (error-last_error)*KD + total_error*KI;
		setSpeed(speed,speed);

		last_error = error;

		if (fabs(error) < integral_threshold)
			total_error += error;
		pros::delay(50);
		printEncoders();
		readEncoders();
	}
	setSpeed(0,0);

	resetEncoders();
	pros::delay(10);
}
