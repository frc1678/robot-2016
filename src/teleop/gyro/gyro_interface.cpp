#include "teleop/gyro/gyro_interface.h"

#include <inttypes.h>
#include <cstring>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GyroInterface::GyroInterface() :
		gyro_(new SPI(SPI::kOnboardCS0)) {
	// The gyro goes up to 8.08MHz.
	// The myRIO goes up to 4MHz, so the roboRIO probably does too.
	gyro_->SetClockRate(4e6);
	gyro_->SetChipSelectActiveLow();
	gyro_->SetClockActiveHigh();
	gyro_->SetSampleDataOnRising();
	gyro_->SetMSBFirst();
	CSVLog=new CSVLogger("GryoInterface.csv", "");
	TextLog=new TextLogger("GryoInterface.log");
}

bool GyroInterface::InitializeGyro() {
	uint32_t result;
	if (!DoTransaction(0x20000003, &result)) {
		TextLog->TextLog("failed to start a self-check", "WARNING", CODE_STAMP);
		return false;
	}
	if (result != 1) {
		// We might have hit a parity error or something and are now retrying, so
		// this isn't a very big deal.
		TextLog->TextLog("gyro unexpected initial response 0x" + result, "WARNING", CODE_STAMP);

	}

	// Wait for it to assert the fault conditions before reading them.
	usleep(50000);

	if (!DoTransaction(0x20000000, &result)) {
		TextLog->TextLog("failed to clear latched non-fault data", "WARNING", CODE_STAMP);
		return false;
	}
	TextLog->TextLog("gyro dummy response is 0x" + result, "DEBUG", CODE_STAMP);

	if (!DoTransaction(0x20000000, &result)) {
		TextLog->TextLog("failed to start a self-check", "WARNING", CODE_STAMP);
		return false;
	}
	if (ExtractStatus(result) != 2) {
		TextLog->TextLog("gyro first value 0x" + std::to_string(result) + " not self-test data", "WARNING", CODE_STAMP);
		return false;
	}
	if (ExtractErrors(result) != 0x7F) {
		TextLog->TextLog("gyro first value 0x" + std::to_string(result) + " does not have all errors", "WARNING", CODE_STAMP);
		return false;
	}

	if (!DoTransaction(0x20000000, &result)) {
		TextLog->TextLog("failed to clear latched self-test data", "WARNING", CODE_STAMP);
		return false;
	}
	if (ExtractStatus(result) != 2) {
		TextLog->TextLog("gyro second value 0x" + std::to_string(result) + " not self-test data", "WARNING", CODE_STAMP);
		return false;
	}

	return true;
}

bool GyroInterface::DoTransaction(uint32_t to_write, uint32_t *result) {
	static const uint8_t kBytes = 4;
	static_assert(kBytes == sizeof(to_write),
			"need the same number of bytes as sizeof(the data)");

	if (__builtin_parity(to_write & ~1) == 0)
		to_write |= 1;

	uint8_t to_send[kBytes], to_receive[kBytes];
	const uint32_t to_write_flipped = __builtin_bswap32(to_write);
	memcpy(to_send, &to_write_flipped, kBytes);

	switch (gyro_->Transaction(to_send, to_receive, kBytes)) {
	case -1:
		TextLog->TextLog("SPI::Transaction failed", "INFO", CODE_STAMP);
		return false;
	case kBytes:
		break;
	default:
		TextLog->TextLog("SPI::Transaction returned something weird", "FATAL", CODE_STAMP);
		break;
	}

	memcpy(result, to_receive, kBytes);
	if (__builtin_parity(*result & 0xFFFF) != 1) {
		TextLog->TextLog("high byte parity failure", "INFO", CODE_STAMP);
		return false;
	}
	if (__builtin_parity(*result) != 1) {
		TextLog->TextLog("whole value parity failure", "INFO", CODE_STAMP);
		return false;
	}

	*result = __builtin_bswap32(*result);
	return true;
}

uint16_t GyroInterface::DoRead(uint8_t address) {
	const uint32_t command = (0x8 << 28) | (address << 17);
	uint32_t response;
	while (true) {
		if (!DoTransaction(command, &response)) {
			TextLog->TextLog("reading 0x" + std::to_string(address) + "failed", "WARNING", CODE_STAMP);
			continue;
		}
		if ((response & 0xEFE00000) != 0x4E000000) {
			TextLog->TextLog("gyro read from 0x" + std::to_string(address) + "gave unexpected response 0x" + std::to_string(response), "WARNING", CODE_STAMP);
			continue;
		}
		return (response >> 5) & 0xFFFF;
	}
}

double GyroInterface::ExtractAngle(uint32_t value) {
	const int16_t reading = -(int16_t) (value >> 10 & 0xFFFF);
	return static_cast<double>(reading) * 2.0 * M_PI / 360.0 / 80.0;
}

uint32_t GyroInterface::ReadPartID() {
	return (DoRead(0x0E) << 16) | DoRead(0x10);
}

uint32_t GyroInterface::GetReading() {
	uint32_t result;
	if (!DoTransaction(0x20000000, &result)) {
		return 0;
	}
	return result;
}
