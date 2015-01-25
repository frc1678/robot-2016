/*
 * BunchLogger.cpp
 *
 *  Created on: Jan 22, 2015
 *      Author: Developer
 */

#include <logs/CSVLogger.h>
#include <logs/SystemLogger.h>
#include <logs/TextLogger.h>
#include <string>

SystemLogger::SystemLogger() {

	driveLogger = new TextLogger("DriveBase");
	CSVdriveLogger = new CSVLogger("DriveBase","Left_Motor_Output, Right_Motor_Output, LeftEncoderRate, RightEncoderRate");
	ElevLogger = new CSVLogger("ElevatorLog","Encoder, Motor_Output, PID1, PID2, PID3, PID4, PID5, PID6, PID7, PID8, PID9");

	// TODO Auto-generated constructor stub
}

SystemLogger::~SystemLogger() {
	// TODO Auto-generated destructor stub
}

void SystemLogger::logDrive(float leftEncoderVal, float rightEncoderVal, float REncoderRate, float LEncoderRate, double joy1, double joy2, float LeftMotorOutput, float RightMotorOutput)
{
	driveLogger->TextLog(std::to_string(leftEncoderVal), "LEFT_ENCODER_VAL", CODE_STAMP);
	driveLogger->TextLog(std::to_string(rightEncoderVal), "RIGHT_ENCODER_VAL", CODE_STAMP);
	driveLogger->TextLog(std::to_string(LEncoderRate), "LEFT_ENCODER_RATE", CODE_STAMP);
	driveLogger->TextLog(std::to_string(REncoderRate), "RIGHT_ENCODER_RATE", CODE_STAMP);
	driveLogger->TextLog(std::to_string(joy1), "JOYSTICK 1", CODE_STAMP);
	driveLogger->TextLog(std::to_string(joy2), "JOYSTICK 2", CODE_STAMP);
	driveLogger->TextLog(std::to_string(LeftMotorOutput), "LEFT_MOTOR_OUTPUT", CODE_STAMP);
	driveLogger->TextLog(std::to_string(RightMotorOutput), "RIGHT_MOTOR_OUTPUT", CODE_STAMP);
}

void SystemLogger::CSVlogDrive(float LMotorOutput, float RMotorOutput, float LEncoderRate, float REncoderRate)
{
	CSVdriveLogger->LogValue(std::to_string(LMotorOutput));
	CSVdriveLogger->LogValue(std::to_string(RMotorOutput));
	CSVdriveLogger->LogValue(std::to_string(LEncoderRate));
	CSVdriveLogger->LogValue(std::to_string(REncoderRate));
	CSVdriveLogger->StartNewCycle();
}

void SystemLogger::ElevatorLogger(float Encoder, float MotorOutput, float pid1, float pid2, float pid3, float pid4, float pid5, float pid6, float pid7, float pid8, float pid9)
{
	ElevLogger->LogValue(std::to_string(Encoder));
	ElevLogger->LogValue(std::to_string(MotorOutput));
	ElevLogger->LogValue(std::to_string(pid1));
	ElevLogger->LogValue(std::to_string(pid2));
	ElevLogger->LogValue(std::to_string(pid3));
	ElevLogger->LogValue(std::to_string(pid4));
	ElevLogger->LogValue(std::to_string(pid5));
	ElevLogger->LogValue(std::to_string(pid6));
	ElevLogger->LogValue(std::to_string(pid7));
	ElevLogger->LogValue(std::to_string(pid8));
	ElevLogger->LogValue(std::to_string(pid9));
	ElevLogger->StartNewCycle();
}

