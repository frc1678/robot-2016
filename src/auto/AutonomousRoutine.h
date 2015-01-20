/*
 * AutonomousRoutine.h
 *
 *  Created on: Jan 19, 2015
 *      Author: Citrus Circuits
 */

#ifndef SRC_AUTO_AUTONOMOUSROUTINE_H_
#define SRC_AUTO_AUTONOMOUSROUTINE_H_
#include "WPILib.h"
#include <logs/ConstantsLoader.h>

class AutonomousRoutine {
	RobotDrive* drive;
	Solenoid* pin, *tension;
	double stingerTime, moveBeforeTensionDropTime, moveAfterTensionDropTime, foldTime;
	Timer* autoTimer;
	Victor* v;
public:
	AutonomousRoutine(Solenoid* stingerPin, Solenoid* releaseTension, RobotDrive* drive,
			ConstantsLoader* autoKLoad, Victor* winch);
	virtual ~AutonomousRoutine();
	void start();
};

#endif /* SRC_AUTO_AUTONOMOUSROUTINE_H_ */
