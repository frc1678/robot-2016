#include "WPILib.h"
#include <math.h>

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H



#define kDeadzoneThreshold 0.04 //kDeadzoneThreshold = 0.02;
//#define kJoystickChangeThreshold 0.2

void driveTrainValues(float stickLeftInput, float stickRightInput, float kJoystickChangeThreshold);

void deadzone();

void runDrivetrain(float stickLeftInput, float stickRightInput, RobotDrive *drivetrain, float thresh);

void runDrivetrain(float stickLeftInput, float stickRightInput, RobotDrive *drivetrain);

void runDrivetrainShift(float stickLeftInput, float stickRightInput, RobotDrive *drivetrain, 
		float thresh, Solenoid *gearUp, Solenoid *gearDown, Encoder *leftEncoder, Encoder *rightEncoder);

#endif
