#include "teleop/gyro/gyro_sender.h"

#include <inttypes.h>

#include "WPILib.h"
#include <thread>
#include <unistd.h>

GyroSender::GyroSender() {
	TextLog=new TextLogger("GyroSender.log");
	CSVLog=new CSVLogger("GyroSender.csv", "");
}

void GyroSender::operator()() {

	// Try to initialize repeatedly as long as we're supposed to be running.
	while (run_ && !gyro_.InitializeGyro()) {
		usleep(50000);
	}
	TextLog->TextLog("gyro initialized successfully", "INFO", CODE_STAMP);

	// In radians, ready to send out.
	double angle = 0;

	int startup_cycles_left = 2 * kReadingRate;

	double zeroing_data[6 * kReadingRate];
	size_t zeroing_index = 0;
	bool zeroed = false;
	bool have_zeroing_data = false;
	double zero_offset = 0;

	while (run_) {

		const uint32_t result = gyro_.GetReading();
		if (result == 0) {
			TextLog->TextLog("normal gyro read failed", "WARNING", CODE_STAMP);
			continue;
		}
		switch (gyro_.ExtractStatus(result)) {
		case 0:
			TextLog->TextLog("gyro says data is bad", "WARNING", CODE_STAMP);
			continue;
		case 1:
			break;
		default:
			TextLog->TextLog("gyro gave weird status 0x" + std::to_string(gyro_.ExtractStatus(result)) , "WARNING", CODE_STAMP);
			continue;
		}
		if (gyro_.ExtractErrors(result) != 0) {
			const uint8_t errors = gyro_.ExtractErrors(result);
			if (errors & (1 << 6)) {
				TextLog->TextLog("gyro gave PLL error", "WARNING", CODE_STAMP);
			}
			if (errors & (1 << 5)) {
				TextLog->TextLog("gyro gave quadrature error", "WARNING", CODE_STAMP);
			}
			if (errors & (1 << 4)) {
				TextLog->TextLog("gyro gave non-volatile memory error", "WARNING", CODE_STAMP);
			}
			if (errors & (1 << 3)) {
				TextLog->TextLog("gyro gave volatile memory error", "WARNING", CODE_STAMP);
			}
			if (errors & (1 << 2)) {
				TextLog->TextLog("gyro gave power error", "WARNING", CODE_STAMP);
			}
			if (errors & (1 << 1)) {
				TextLog->TextLog("gyro gave continuous self-test error", "WARNING", CODE_STAMP);
			}
			if (errors & 1) {
				TextLog->TextLog("gyro gave unexpected self-test mode", "WARNING", CODE_STAMP);
			}
			continue;
		}

		if (startup_cycles_left > 0) {
			--startup_cycles_left;
			continue;
		}

		const double new_angle = gyro_.ExtractAngle(result)
						/ static_cast<double>(kReadingRate);
		if (zeroed) {
			angle += new_angle;
			angle += zero_offset;
			SmartDashboard::PutNumber("Angle", angle);
			//            LOG_STRUCT(DEBUG, "sending", *message);
		} else {
			// TODO(brian): Don't break without 6 seconds of standing still before
			// enabling. Ideas:
			//   Don't allow driving until we have at least some data?
			//   Some kind of indicator light?
			{
				//          LOG_STRUCT(DEBUG, "collected", *message);
			}
			zeroing_data[zeroing_index] = new_angle;
			++zeroing_index;
			if (zeroing_index
					>= sizeof(zeroing_data) / sizeof(zeroing_data[0])) {
				zeroing_index = 0;
				have_zeroing_data = true;
			}

		}
	}
}
