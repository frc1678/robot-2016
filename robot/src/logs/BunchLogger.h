/*
 * BunchLogger.h
 *
 *  Created on: Jan 22, 2015
 *      Author: Developer
 */

#ifndef ROBOT_2015_SRC_LOGS_BUNCHLOGGER_H_
#define ROBOT_2015_SRC_LOGS_BUNCHLOGGER_H_

class BunchLogger {
public:
	BunchLogger();
	virtual ~BunchLogger();

	CSVLogger* CSVdriveLogger;
	TextLogger* driveLogger;
	CSVLogger* ElevLogger;

	void logDrive(float leftEncoderVal, float rightEncoderVal, float REncoderRate, float LEncoderRate, double joy1, double joy2, float LeftMotorOutput, float RightMotorOutput);

	void CSVlogDrive(float LMotorOutput, float RMotorOutput, float LEncoderRate, float REncoderRate);

	void ElevatorLogger(float Encoder, float MotorOutput, float pid1, float pid2, float pid3, float pid4, float pid5, float pid6, float pid7, float pid8, float pid9);

};

#endif /* ROBOT_2015_SRC_LOGS_BUNCHLOGGER_H_ */
