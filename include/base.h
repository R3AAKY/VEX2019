  #include "main.h"

#ifndef _BASE_H_
#define _BASE_H_

extern pros::Controller master;
extern pros::ADIDigitalIn limit_switch1;
extern pros::ADIDigitalIn limit_switch2;
extern pros::ADIEncoder left_sensor;
extern pros::ADIEncoder right_sensor;
extern pros::Motor left_mtr1;
extern pros::Motor left_mtr2;
extern pros::Motor right_mtr1;
extern pros::Motor right_mtr2;

extern pros::Motor right_flywheel;
extern pros::Motor left_flywheel;
extern pros::Motor intake;

extern pros::Motor left_arm;
extern pros::Motor right_arm;
extern pros::Motor claw;

extern pros::ADIGyro gyro;
extern double left_pos;
extern double right_pos;
extern double error;

#define MAX_SPEED 127
#define pi 3.14
#define KP 1.1
#define KI 0.2
#define KD 2.75

#define KPl 0.9
#define KIl 0
#define KDl 2.5

#define threshold 20
#define turn_threshold 1
#define integralThreshold 40
#define degreesPerRotation 360
#define wheelDiameter 3.125
#define TURN_90 10.75*pi/4

void printEncoders();
double inchesToDegrees(double inches);
double degreesToInches(double degrees);
void turn(double angle_in_inches);
void moveStraight(double distance_in_inches);
void setSpeed(int left_speed,int right_speed);
double map(double x, double in_min, double in_max, double out_min, double out_max);
void resetEncoders();
void resetArm();
void armControl();

// These use multiple parameters.
void drivePID(double target, bool isTurn, pros::ADIEncoder *sensorL, pros::ADIEncoder *sensorR, double Kp, double Ki, double Kd);
void armPID(double target, pros::Motor *mtr, double Kp, double Ki, double Kd);
void flywheelPID(double target, pros::Motor *mtrL, pros::Motor *mtrR, double Kp, double Ki, double Kd);

typedef struct arm_arg{
  double target;
  pros::Motor *mtr;
  double Kp, Kd, Ki;
} arm_arg;

typedef struct drive_arg{
  double target;
  bool isTurn;
  pros::ADIEncoder *sensorL;
  pros::ADIEncoder *sensorR;
  double Kp, Kd, Ki;
} drive_arg;

typedef struct flywheel_arg{
  double target;
  pros::Motor *mtrL;
  pros::Motor *mtrR;
  double Kp, Kd, Ki;
} flywheel_arg;

// These use struct arguments to represent multiple parameters
void drivePID(void* d_arg);
void armPID(void* a_arg);
void flywheelPID(void* f_arg);

void runFlywheel(double target);
void stopFlywheel();

#endif //base.h
