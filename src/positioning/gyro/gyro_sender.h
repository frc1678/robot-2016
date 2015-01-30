#ifndef GYRO_H_
#define GYRO_H_

#include <stdint.h>

#include <atomic>

#include "gyro_interface.h"
#include "logs/CSVLogger.h"
#include "logs/TextLogger.h"

// Handles reading the gyro over SPI and sending out angles on a queue.
//
// This is designed to be passed into ::std::thread's constructor so it will run
// as a separate thread.
class GyroSender {
public:
	GyroSender();

	// For ::std::thread to call.
	//
	// Initializes the gyro and then loops forever taking readings.
	void operator()();

	void Quit() {
		run_ = false;
	}

private:

	// Readings per second.
	static const int kReadingRate = 200;

	GyroInterface gyro_;

	std::atomic<bool> run_ { true };
protected:
	CSVLogger *CSVLog;
	TextLogger *TextLog;
};

#endif  // GYRO_H_
