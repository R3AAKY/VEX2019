#include "main.h"

#ifndef _BASE_H_
#define _BASE_H_

extern pros::ADIEncoder left_sensor;
extern pros::ADIEncoder right_sensor;
extern pros::Motor left_mtr1;
extern pros::Motor left_mtr2;
extern pros::Motor right_mtr1;
extern pros::Motor right_mtr2;
extern double left_pos;
extern double right_pos;
extern double error;

#define MAX_SPEED 127
#define pi 3.14
#define KP 0.8
#define KI 0.001f
#define KD 0.1f
#define threshold 20
#define turn_threshold 20
#define integral_threshold 100
#define degreesPerRotation 360
#define wheelDiameter 4 //4.0" wheels
#define isLeft true

double inchesToDegrees(double inches);
double degreesToInches(double degrees);
double map(double x, double in_min, double in_max, double out_min, double out_max);
void turn90degrees(int direction);
void moveStraight(double distance_in_inches);
void setSpeed(int s);
void readEncoders();

#endif //base.h
