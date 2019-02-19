#include "main.h"
#include "base.h"

//Inputs inches, outputs proper tick value needed
double inchesToDegrees(double inches){
	return (inches/circ)*degreesPerRotation;
}

double degreesToInches(double degrees)
{
	return (degrees/degreesPerRotation)*pi*wheelDiameter;
}

// Used to relate position value (meters) to speed/velocity (meters per second) since they are two different units.
double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readEncoders(){
	left_pos = left_sensor.get_value();
	right_pos = right_sensor.get_value();
}

void setSpeed(int speed){
	left_mtr1= speed;
	left_mtr2= speed;
	right_mtr1 = speed;
	right_mtr2 = speed;
}

// Found 90 degree turn to be 0.6875 rotations.
// 1 for left turn, 2 for right turn
void turn90degrees(int direction){
  double setpoint = 0.6875 * degreesPerRotation; //in degrees
  readEncoders();

	speed = 37; //max speed

	if (isLeft == true){
		left_mtr1.set_reversed(false);
		right_mtr1.set_reversed(true);
	}
	else if (isLeft == false)
	{
		right_mtr1.set_reversed(false);
		left_mtr1.set_reversed(true);
	}

	while(true){
		if (direction ==1)
			error = setpoint - left_pos; // = setpoint - right_pos1 (in degrees)
		else if (direction==2)
			error = setpoint - right_pos;
		if ((error)<=turn_threshold)
			break;
		speed = map(error * KP, -setpoint, setpoint, -MAX_SPEED,MAX_SPEED);

		setSpeed(speed);
		readEncoders();
	}
	setSpeed(0);
	pros::delay(10);
}

//The actual move straight function that runs the motors
void moveStraight(double distance_in_inches){
 	readEncoders();
	setpoint = inchesToDegrees(distance_in_inches);
	speed = 37; //max speed

/* Setpoint, Error, Reference values are position values (in degrees for encoder units).
	 Proporional Controller gain is used for modifying speed/velocity depending on how much error remains (larger error = lower speed required, vice-versa)
*/
	while(true){
		error = setpoint - left_pos; // = setpoint - right_pos1 (in degrees)
		if ((error)<=threshold)
			break;
		setSpeed(speed);
		readEncoders();
	}
	setSpeed(0);
	pros::delay(10);
}
