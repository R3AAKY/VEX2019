#include "main.h"
#include "base.h"

int speed = 63; //max speed
double left_pos = 0;
double right_pos = 0;
double error = 0;
double circ = pi*wheelDiameter;

//Inputs inches, outputs proper tick value needed
double inchesToDegrees(double inches){
	return (inches/circ)*degreesPerRotation;
}

double degreesToInches(double degrees)
{
	return (degrees/degreesPerRotation)*circ;
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

void setSpeed(int s){
	left_mtr1= s;
	left_mtr2= s;
	right_mtr1 = s;
	right_mtr2 = s;
}

// Found 90 degree turn to be 0.6875 rotations.
// 1 for left turn, 2 for right turn
void turn90degrees(int direction){
  double setpoint = 0.6875 * degreesPerRotation; //in degrees
  readEncoders();

	if (direction == isLeft){
		left_mtr1.set_reversed(false);
		right_mtr1.set_reversed(true);
	}
	else if (direction != isLeft)
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
  double	setpoint = inchesToDegrees(distance_in_inches);

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
