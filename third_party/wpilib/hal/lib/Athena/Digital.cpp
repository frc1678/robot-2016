/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "HAL/Digital.hpp"

#include "HAL/Port.h"
#include "HAL/HAL.hpp"
#include "ChipObject.h"
#include "HAL/cpp/Resource.hpp"
#include "HAL/cpp/priority_mutex.h"
#include "FRC_NetworkCommunication/LoadOut.h"
#include <stdio.h>
#include <math.h>
#include <mutex>
#include "i2clib/i2c-lib.h"
#include "spilib/spi-lib.h"

static_assert(sizeof(uint32_t) <= sizeof(void *), "This file shoves uint32_ts into pointers.");

static const uint32_t kExpectedLoopTiming = 40;
static const uint32_t kDigitalPins = 26;
static const uint32_t kPwmPins = 20;
static const uint32_t kRelayPins = 8;
static const uint32_t kNumHeaders = 10; // Number of non-MXP pins

/**
 * kDefaultPwmPeriod is in ms
 *
 * - 20ms periods (50 Hz) are the "safest" setting in that this works for all devices
 * - 20ms periods seem to be desirable for Vex Motors
 * - 20ms periods are the specified period for HS-322HD servos, but work reliably down
 *      to 10.0 ms; starting at about 8.5ms, the servo sometimes hums and get hot;
 *      by 5.0ms the hum is nearly continuous
 * - 10ms periods work well for Victor 884
 * - 5ms periods allows higher update rates for Luminary Micro Jaguar speed controllers.
 *      Due to the shipping firmware on the Jaguar, we can't run the update period less
 *      than 5.05 ms.
 *
 * kDefaultPwmPeriod is the 1x period (5.05 ms).  In hardware, the period scaling is implemented as an
 * output squelch to get longer periods for old devices.
 */
static const float kDefaultPwmPeriod = 5.05;
/**
 * kDefaultPwmCenter is the PWM range center in ms
 */
static const float kDefaultPwmCenter = 1.5;
/**
 * kDefaultPWMStepsDown is the number of PWM steps below the centerpoint
 */
static const int32_t kDefaultPwmStepsDown = 1000;
static const int32_t kPwmDisabled = 0;

struct DigitalPort {
  Port port;
  uint32_t PWMGeneratorID;
};

// Create a mutex to protect changes to the digital output values
static priority_recursive_mutex digitalDIOMutex;
// Create a mutex to protect changes to the relay values
static priority_recursive_mutex digitalRelayMutex;
// Create a mutex to protect changes to the DO PWM config
static priority_recursive_mutex digitalPwmMutex;
static priority_recursive_mutex digitalI2COnBoardMutex;
static priority_recursive_mutex digitalI2CMXPMutex;

static tDIO* digitalSystem = NULL;
static tRelay* relaySystem = NULL;
static tPWM* pwmSystem = NULL;
static hal::Resource *DIOChannels = NULL;
static hal::Resource *DO_PWMGenerators = NULL;
static hal::Resource *PWMChannels = NULL;

static bool digitalSystemsInitialized = false;

static uint8_t i2COnboardObjCount = 0;
static uint8_t i2CMXPObjCount = 0;
static uint8_t i2COnBoardHandle = 0;
static uint8_t i2CMXPHandle = 0;

static int32_t m_spiCS0Handle = 0;
static int32_t m_spiCS1Handle = 0;
static int32_t m_spiCS2Handle = 0;
static int32_t m_spiCS3Handle = 0;
static int32_t m_spiMXPHandle = 0;
static priority_recursive_mutex spiOnboardSemaphore;
static priority_recursive_mutex spiMXPSemaphore;
static tSPI *spiSystem;

extern "C" {

struct SPIAccumulator {
	void* notifier = nullptr;
	uint64_t triggerTime;
	uint32_t period;

	int64_t value = 0;
	uint32_t count = 0;
	int32_t last_value = 0;

	int32_t center = 0;
	int32_t deadband = 0;

	uint8_t cmd[4];		// command to send (up to 4 bytes)
	uint32_t valid_mask;
	uint32_t valid_value;
	int32_t data_max;	// one more than max data value
	int32_t data_msb_mask;	// data field MSB mask (for signed)
	uint8_t data_shift;	// data field shift right amount, in bits
	uint8_t xfer_size;	// SPI transfer size, in bytes (up to 4)
	uint8_t port;
	bool is_signed;		// is data field signed?
	bool big_endian;	// is response big endian?
};
SPIAccumulator* spiAccumulators[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};

/**
 * Initialize the digital system.
 */
void initializeDigital(int32_t *status) {
  if (digitalSystemsInitialized) return;

  hal::Resource::CreateResourceObject(&DIOChannels, tDIO::kNumSystems * kDigitalPins);
  hal::Resource::CreateResourceObject(&DO_PWMGenerators, tDIO::kNumPWMDutyCycleAElements + tDIO::kNumPWMDutyCycleBElements);
  hal::Resource::CreateResourceObject(&PWMChannels, tPWM::kNumSystems * kPwmPins);
  digitalSystem = tDIO::create(status);

  // Relay Setup
  relaySystem = tRelay::create(status);

  // Turn off all relay outputs.
  relaySystem->writeValue_Forward(0, status);
  relaySystem->writeValue_Reverse(0, status);

  // PWM Setup
  pwmSystem = tPWM::create(status);

  // Make sure that the 9403 IONode has had a chance to initialize before continuing.
  while(pwmSystem->readLoopTiming(status) == 0) delayTicks(1);

  if (pwmSystem->readLoopTiming(status) != kExpectedLoopTiming) {
	// TODO: char err[128];
	// TODO: sprintf(err, "DIO LoopTiming: %d, expecting: %lu\n", digitalModules[port->module-1]->readLoopTiming(status), kExpectedLoopTiming);
	*status = LOOP_TIMING_ERROR; // NOTE: Doesn't display the error
  }

  //Calculate the length, in ms, of one DIO loop
  double loopTime = pwmSystem->readLoopTiming(status)/(kSystemClockTicksPerMicrosecond*1e3);

  pwmSystem->writeConfig_Period((uint16_t) (kDefaultPwmPeriod/loopTime + .5), status);
  uint16_t minHigh = (uint16_t) ((kDefaultPwmCenter-kDefaultPwmStepsDown*loopTime)/loopTime + .5);
  pwmSystem->writeConfig_MinHigh(minHigh, status);
//  printf("MinHigh: %d\n", minHigh);
  // Ensure that PWM output values are set to OFF
  for (uint32_t pwm_index = 0; pwm_index < kPwmPins; pwm_index++) {
    // Initialize port structure
    DigitalPort digital_port;
    digital_port.port.pin = pwm_index;

    setPWM(&digital_port, kPwmDisabled, status);
    setPWMPeriodScale(&digital_port, 3, status); // Set all to 4x by default.
  }

  digitalSystemsInitialized = true;
}

/**
 * Create a new instance of a digital port.
 */
void* initializeDigitalPort(void* port_pointer, int32_t *status) {
  initializeDigital(status);
  Port* port = (Port*) port_pointer;

  // Initialize port structure
  DigitalPort* digital_port = new DigitalPort();
  digital_port->port = *port;

  return digital_port;
}

void freeDigitalPort(void* digital_port_pointer) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  delete port;
}

bool checkPWMChannel(void* digital_port_pointer) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  return port->port.pin < kPwmPins;
}

bool checkRelayChannel(void* digital_port_pointer) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  return port->port.pin < kRelayPins;
}

/**
 * Check a port to make sure that it is not NULL and is a valid PWM port.
 *
 * Sets the status to contain the appropriate error.
 *
 * @return true if the port passed validation.
 */
static bool verifyPWMChannel(DigitalPort *port, int32_t *status) {
  if (port == NULL) {
    *status = NULL_PARAMETER;
    return false;
  } else if (!checkPWMChannel(port)) {
    *status = PARAMETER_OUT_OF_RANGE;
    return false;
  } else {
    return true;
  }
}

/**
 * Check a port to make sure that it is not NULL and is a valid Relay port.
 *
 * Sets the status to contain the appropriate error.
 *
 * @return true if the port passed validation.
 */
static bool verifyRelayChannel(DigitalPort *port, int32_t *status) {
  if (port == NULL) {
    *status = NULL_PARAMETER;
    return false;
  } else if (!checkRelayChannel(port)) {
    *status = PARAMETER_OUT_OF_RANGE;
    return false;
  } else {
    return true;
  }
}

/**
 * Map DIO pin numbers from their physical number (10 to 26) to their position
 * in the bit field.
 */
uint32_t remapMXPChannel(uint32_t pin) {
    return pin - 10;
}

uint32_t remapMXPPWMChannel(uint32_t pin) {
	if(pin < 14) {
		return pin - 10;	//first block of 4 pwms (MXP 0-3)
	} else {
		return pin - 6;	//block of PWMs after SPI
	}
}

/**
 * Set a PWM channel to the desired value. The values range from 0 to 255 and the period is controlled
 * by the PWM Period and MinHigh registers.
 *
 * @param channel The PWM channel to set.
 * @param value The PWM value to set.
 */
void setPWM(void* digital_port_pointer, unsigned short value, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  if (!verifyPWMChannel(port, status)) { return; }

  if(port->port.pin < tPWM::kNumHdrRegisters) {
    pwmSystem->writeHdr(port->port.pin, value, status);
  } else {
    pwmSystem->writeMXP(port->port.pin - tPWM::kNumHdrRegisters, value, status);
  }
}

/**
 * Get a value from a PWM channel. The values range from 0 to 255.
 *
 * @param channel The PWM channel to read from.
 * @return The raw PWM value.
 */
unsigned short getPWM(void* digital_port_pointer, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  if (!verifyPWMChannel(port, status)) { return 0; }

  if(port->port.pin < tPWM::kNumHdrRegisters) {
    return pwmSystem->readHdr(port->port.pin, status);
  } else {
    return pwmSystem->readMXP(port->port.pin - tPWM::kNumHdrRegisters, status);
  }
}

void latchPWMZero(void* digital_port_pointer, int32_t *status) {
	DigitalPort* port = (DigitalPort*) digital_port_pointer;
	if (!verifyPWMChannel(port, status)) { return; }

	pwmSystem->writeZeroLatch(port->port.pin, true, status);
	pwmSystem->writeZeroLatch(port->port.pin, false, status);
}

/**
 * Set how how often the PWM signal is squelched, thus scaling the period.
 *
 * @param channel The PWM channel to configure.
 * @param squelchMask The 2-bit mask of outputs to squelch.
 */
void setPWMPeriodScale(void* digital_port_pointer, uint32_t squelchMask, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  if (!verifyPWMChannel(port, status)) { return; }

  if(port->port.pin < tPWM::kNumPeriodScaleHdrElements) {
    pwmSystem->writePeriodScaleHdr(port->port.pin, squelchMask, status);
  } else {
    pwmSystem->writePeriodScaleMXP(port->port.pin - tPWM::kNumPeriodScaleHdrElements, squelchMask, status);
  }
}

/**
 * Allocate a DO PWM Generator.
 * Allocate PWM generators so that they are not accidentally reused.
 *
 * @return PWM Generator refnum
 */
void* allocatePWM(int32_t *status) {
  return (void*)DO_PWMGenerators->Allocate("DO_PWM");
}

/**
 * Free the resource associated with a DO PWM generator.
 *
 * @param pwmGenerator The pwmGen to free that was allocated with AllocateDO_PWM()
 */
void freePWM(void* pwmGenerator, int32_t *status) {
  uint32_t id = (uint32_t) pwmGenerator;
  if (id == ~0ul) return;
  DO_PWMGenerators->Free(id);
}

/**
 * Change the frequency of the DO PWM generator.
 *
 * The valid range is from 0.6 Hz to 19 kHz.  The frequency resolution is logarithmic.
 *
 * @param rate The frequency to output all digital output PWM signals.
 */
void setPWMRate(double rate, int32_t *status) {
  // Currently rounding in the log rate domain... heavy weight toward picking a higher freq.
  // TODO: Round in the linear rate domain.
  uint8_t pwmPeriodPower = (uint8_t)(log(1.0 / (pwmSystem->readLoopTiming(status) * 0.25E-6 * rate))/log(2.0) + 0.5);
  digitalSystem->writePWMPeriodPower(pwmPeriodPower, status);
}


/**
 * Configure the duty-cycle of the PWM generator
 *
 * @param pwmGenerator The generator index reserved by AllocateDO_PWM()
 * @param dutyCycle The percent duty cycle to output [0..1].
 */
void setPWMDutyCycle(void* pwmGenerator, double dutyCycle, int32_t *status) {
  uint32_t id = (uint32_t) pwmGenerator;
  if (id == ~0ul) return;
  if (dutyCycle > 1.0) dutyCycle = 1.0;
  if (dutyCycle < 0.0) dutyCycle = 0.0;
  float rawDutyCycle = 256.0 * dutyCycle;
  if (rawDutyCycle > 255.5) rawDutyCycle = 255.5;
  {
	std::lock_guard<priority_recursive_mutex> sync(digitalPwmMutex);
    uint8_t pwmPeriodPower = digitalSystem->readPWMPeriodPower(status);
    if (pwmPeriodPower < 4) {
	  // The resolution of the duty cycle drops close to the highest frequencies.
	  rawDutyCycle = rawDutyCycle / pow(2.0, 4 - pwmPeriodPower);
	}
	if(id < 4)
		digitalSystem->writePWMDutyCycleA(id, (uint8_t)rawDutyCycle, status);
	else
		digitalSystem->writePWMDutyCycleB(id - 4, (uint8_t)rawDutyCycle, status);
  }
}

/**
 * Configure which DO channel the PWM signal is output on
 *
 * @param pwmGenerator The generator index reserved by AllocateDO_PWM()
 * @param channel The Digital Output channel to output on
 */
void setPWMOutputChannel(void* pwmGenerator, uint32_t pin, int32_t *status) {
  uint32_t id = (uint32_t) pwmGenerator;
  if (id > 5) return;
  digitalSystem->writePWMOutputSelect(id, pin, status);
}

/**
 * Set the state of a relay.
 * Set the state of a relay output to be forward. Relays have two outputs and each is
 * independently set to 0v or 12v.
 */
void setRelayForward(void* digital_port_pointer, bool on, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  if (!verifyRelayChannel(port, status)) {
    return;
  }

  {
    std::lock_guard<priority_recursive_mutex> sync(digitalRelayMutex);
    uint8_t forwardRelays = relaySystem->readValue_Forward(status);
    if (on)
      forwardRelays |= 1 << port->port.pin;
    else
      forwardRelays &= ~(1 << port->port.pin);
    relaySystem->writeValue_Forward(forwardRelays, status);
  }
}

/**
 * Set the state of a relay.
 * Set the state of a relay output to be reverse. Relays have two outputs and each is
 * independently set to 0v or 12v.
 */
void setRelayReverse(void* digital_port_pointer, bool on, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  if (!verifyRelayChannel(port, status)) {
    return;
  }

  {
    std::lock_guard<priority_recursive_mutex> sync(digitalRelayMutex);
    uint8_t reverseRelays = relaySystem->readValue_Reverse(status);
    if (on)
      reverseRelays |= 1 << port->port.pin;
    else
      reverseRelays &= ~(1 << port->port.pin);
    relaySystem->writeValue_Reverse(reverseRelays, status);
  }
}

/**
 * Get the current state of the forward relay channel
 */
bool getRelayForward(void* digital_port_pointer, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  if (!verifyRelayChannel(port, status)) { return false; }

  uint8_t forwardRelays = relaySystem->readValue_Forward(status);
  return (forwardRelays & (1 << port->port.pin)) != 0;
}

/**
 * Get the current state of the reverse relay channel
 */
bool getRelayReverse(void* digital_port_pointer, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  if (!verifyRelayChannel(port, status)) { return false; }

  uint8_t reverseRelays = relaySystem->readValue_Reverse(status);
  return (reverseRelays & (1 << port->port.pin)) != 0;
}

/**
 * Allocate Digital I/O channels.
 * Allocate channels so that they are not accidently reused. Also the direction is set at the
 * time of the allocation.
 *
 * @param channel The Digital I/O channel
 * @param input If true open as input; if false open as output
 * @return Was successfully allocated
 */
bool allocateDIO(void* digital_port_pointer, bool input, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  char buf[64];
  snprintf(buf, 64, "DIO %d", port->port.pin);
  if (DIOChannels->Allocate(port->port.pin, buf) == ~0ul) {
    *status = RESOURCE_IS_ALLOCATED;
    return false;
  }

  {
    std::lock_guard<priority_recursive_mutex> sync(digitalDIOMutex);

    tDIO::tOutputEnable outputEnable = digitalSystem->readOutputEnable(status);

    if(port->port.pin < kNumHeaders) {
      uint32_t bitToSet = 1 << port->port.pin;
      if (input) {
        outputEnable.Headers = outputEnable.Headers & (~bitToSet); // clear the bit for read
      } else {
        outputEnable.Headers = outputEnable.Headers | bitToSet; // set the bit for write
      }
    } else {
      uint32_t bitToSet = 1 << remapMXPChannel(port->port.pin);

      // Disable special functions on this pin
      short specialFunctions = digitalSystem->readEnableMXPSpecialFunction(status);
      digitalSystem->writeEnableMXPSpecialFunction(specialFunctions & ~bitToSet, status);

      if (input) {
        outputEnable.MXP = outputEnable.MXP & (~bitToSet); // clear the bit for read
      } else {
        outputEnable.MXP = outputEnable.MXP | bitToSet; // set the bit for write
      }
    }

    digitalSystem->writeOutputEnable(outputEnable, status);
  }
  return true;
}

bool allocatePWMChannel(void* digital_port_pointer, int32_t *status) {
	DigitalPort* port = (DigitalPort*) digital_port_pointer;
	if (!verifyPWMChannel(port, status)) { return false; }

	char buf[64];
	snprintf(buf, 64, "PWM %d", port->port.pin);
	if (PWMChannels->Allocate(port->port.pin, buf) == ~0ul) {
		*status = RESOURCE_IS_ALLOCATED;
		return false;
    }

	if (port->port.pin > tPWM::kNumHdrRegisters-1) {
		snprintf(buf, 64, "PWM %d and DIO %d", port->port.pin, remapMXPPWMChannel(port->port.pin) + 10);
		if (DIOChannels->Allocate(remapMXPPWMChannel(port->port.pin) + 10, buf) == ~0ul) return false;
		uint32_t bitToSet = 1 << remapMXPPWMChannel(port->port.pin);
		short specialFunctions = digitalSystem->readEnableMXPSpecialFunction(status);
		digitalSystem->writeEnableMXPSpecialFunction(specialFunctions | bitToSet, status);
	}
	return true;
}

void freePWMChannel(void* digital_port_pointer, int32_t *status) {
    DigitalPort* port = (DigitalPort*) digital_port_pointer;
    if (!port) return;
    if (!verifyPWMChannel(port, status)) { return; }

    PWMChannels->Free(port->port.pin);
    if(port->port.pin > tPWM::kNumHdrRegisters-1) {
        DIOChannels->Free(remapMXPPWMChannel(port->port.pin) + 10);
        uint32_t bitToUnset = 1 << remapMXPPWMChannel(port->port.pin);
        short specialFunctions = digitalSystem->readEnableMXPSpecialFunction(status);
        digitalSystem->writeEnableMXPSpecialFunction(specialFunctions & ~bitToUnset, status);
    }
}

/**
 * Free the resource associated with a digital I/O channel.
 *
 * @param channel The Digital I/O channel to free
 */
void freeDIO(void* digital_port_pointer, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  if (!port) return;
  DIOChannels->Free(port->port.pin);
}

/**
 * Write a digital I/O bit to the FPGA.
 * Set a single value on a digital I/O channel.
 *
 * @param channel The Digital I/O channel
 * @param value The state to set the digital channel (if it is configured as an output)
 */
void setDIO(void* digital_port_pointer, short value, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  if (value != 0 && value != 1) {
    if (value != 0)
      value = 1;
  }
  {
    std::lock_guard<priority_recursive_mutex> sync(digitalDIOMutex);
    tDIO::tDO currentDIO = digitalSystem->readDO(status);

    if(port->port.pin < kNumHeaders) {
      if(value == 0) {
        currentDIO.Headers = currentDIO.Headers & ~(1 << port->port.pin);
      } else if (value == 1) {
        currentDIO.Headers = currentDIO.Headers | (1 << port->port.pin);
      }
    } else {
      if(value == 0) {
        currentDIO.MXP = currentDIO.MXP & ~(1 << remapMXPChannel(port->port.pin));
      } else if (value == 1) {
        currentDIO.MXP = currentDIO.MXP | (1 << remapMXPChannel(port->port.pin));
      }

      uint32_t bitToSet = 1 << remapMXPChannel(port->port.pin);
      short specialFunctions = digitalSystem->readEnableMXPSpecialFunction(status);
      digitalSystem->writeEnableMXPSpecialFunction(specialFunctions & ~bitToSet, status);
    }
    digitalSystem->writeDO(currentDIO, status);
  }
}

/**
 * Read a digital I/O bit from the FPGA.
 * Get a single value from a digital I/O channel.
 *
 * @param channel The digital I/O channel
 * @return The state of the specified channel
 */
bool getDIO(void* digital_port_pointer, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  tDIO::tDI currentDIO = digitalSystem->readDI(status);
  //Shift 00000001 over channel-1 places.
  //AND it against the currentDIO
  //if it == 0, then return false
  //else return true

  if(port->port.pin < kNumHeaders) {
    return ((currentDIO.Headers >> port->port.pin) & 1) != 0;
  } else {
    // Disable special functions
    uint32_t bitToSet = 1 << remapMXPChannel(port->port.pin);
    short specialFunctions = digitalSystem->readEnableMXPSpecialFunction(status);
    digitalSystem->writeEnableMXPSpecialFunction(specialFunctions & ~bitToSet, status);

    return ((currentDIO.MXP >> remapMXPChannel(port->port.pin)) & 1) != 0;
  }
}

/**
 * Read the direction of a the Digital I/O lines
 * A 1 bit means output and a 0 bit means input.
 *
 * @param channel The digital I/O channel
 * @return The direction of the specified channel
 */
bool getDIODirection(void* digital_port_pointer, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  tDIO::tOutputEnable currentOutputEnable = digitalSystem->readOutputEnable(status);
  //Shift 00000001 over port->port.pin-1 places.
  //AND it against the currentOutputEnable
  //if it == 0, then return false
  //else return true

  if(port->port.pin < kNumHeaders) {
    return ((currentOutputEnable.Headers >> port->port.pin) & 1) != 0;
  } else {
    return ((currentOutputEnable.MXP >> remapMXPChannel(port->port.pin)) & 1) != 0;
  }
}

/**
 * Generate a single pulse.
 * Write a pulse to the specified digital output channel. There can only be a single pulse going at any time.
 *
 * @param channel The Digital Output channel that the pulse should be output on
 * @param pulseLength The active length of the pulse (in seconds)
 */
void pulse(void* digital_port_pointer, double pulseLength, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  tDIO::tPulse pulse;

  if(port->port.pin < kNumHeaders) {
    pulse.Headers = 1 << port->port.pin;
  } else {
    pulse.MXP = 1 << remapMXPChannel(port->port.pin);
  }

  digitalSystem->writePulseLength((uint8_t)(1.0e9 * pulseLength / (pwmSystem->readLoopTiming(status) * 25)), status);
  digitalSystem->writePulse(pulse, status);
}

/**
 * Check a DIO line to see if it is currently generating a pulse.
 *
 * @return A pulse is in progress
 */
bool isPulsing(void* digital_port_pointer, int32_t *status) {
  DigitalPort* port = (DigitalPort*) digital_port_pointer;
  tDIO::tPulse pulseRegister = digitalSystem->readPulse(status);

  if(port->port.pin < kNumHeaders) {
    return (pulseRegister.Headers & (1 << port->port.pin)) != 0;
  } else {
    return (pulseRegister.MXP & (1 << remapMXPChannel(port->port.pin))) != 0;
  }
}

/**
 * Check if any DIO line is currently generating a pulse.
 *
 * @return A pulse on some line is in progress
 */
bool isAnyPulsing(int32_t *status) {
  tDIO::tPulse pulseRegister = digitalSystem->readPulse(status);
  return pulseRegister.Headers != 0 && pulseRegister.MXP != 0;
}

/**
 * Write the filter index from the FPGA.
 * Set the filter index used to filter out short pulses.
 *
 * @param digital_port_pointer The digital I/O channel
 * @param filter_index The filter index.  Must be in the range 0 - 3,
 * where 0 means "none" and 1 - 3 means filter # filter_index - 1.
 */
void setFilterSelect(void* digital_port_pointer, int filter_index,
                     int32_t* status) {
  DigitalPort* port = (DigitalPort*)digital_port_pointer;

  std::lock_guard<priority_recursive_mutex> sync(digitalDIOMutex);
  if (port->port.pin < kNumHeaders) {
    digitalSystem->writeFilterSelectHdr(port->port.pin, filter_index, status);
  }
  else {
    digitalSystem->writeFilterSelectMXP(remapMXPChannel(port->port.pin),
                                        filter_index, status);
  }
}

/**
 * Read the filter index from the FPGA.
 * Get the filter index used to filter out short pulses.
 *
 * @param digital_port_pointer The digital I/O channel
 * @return filter_index The filter index.  Must be in the range 0 - 3,
 * where 0 means "none" and 1 - 3 means filter # filter_index - 1.
 */
int getFilterSelect(void* digital_port_pointer, int32_t* status) {
  DigitalPort* port = (DigitalPort*)digital_port_pointer;

  std::lock_guard<priority_recursive_mutex> sync(digitalDIOMutex);
  if (port->port.pin < kNumHeaders) {
    return digitalSystem->readFilterSelectHdr(port->port.pin, status);
  }
  else {
    return digitalSystem->readFilterSelectMXP(remapMXPChannel(port->port.pin),
                                              status);
  }
}

/**
 * Set the filter period for the specified filter index.
 *
 * Set the filter period in FPGA cycles.  Even though there are 2 different
 * filter index domains (MXP vs HDR), ignore that distinction for now since it
 * compilicates the interface.  That can be changed later.
 *
 * @param filter_index The filter index, 0 - 2.
 * @param value The number of cycles that the signal must not transition to be
 * counted as a transition.
 */
void setFilterPeriod(int filter_index, uint32_t value, int32_t* status) {
  std::lock_guard<priority_recursive_mutex> sync(digitalDIOMutex);
  digitalSystem->writeFilterPeriodHdr(filter_index, value, status);
  if (*status == 0) {
    digitalSystem->writeFilterPeriodMXP(filter_index, value, status);
  }
}

/**
 * Get the filter period for the specified filter index.
 *
 * Get the filter period in FPGA cycles.  Even though there are 2 different
 * filter index domains (MXP vs HDR), ignore that distinction for now since it
 * compilicates the interface.  Set status to NiFpga_Status_SoftwareFault if the
 * filter values miss-match.
 *
 * @param filter_index The filter index, 0 - 2.
 * @param value The number of cycles that the signal must not transition to be
 * counted as a transition.
 */
uint32_t getFilterPeriod(int filter_index, int32_t* status) {
  uint32_t hdr_period = 0;
  uint32_t mxp_period = 0;
  {
    std::lock_guard<priority_recursive_mutex> sync(digitalDIOMutex);
    hdr_period = digitalSystem->readFilterPeriodHdr(filter_index, status);
    if (*status == 0) {
      mxp_period = digitalSystem->readFilterPeriodMXP(filter_index, status);
    }
  }
  if (hdr_period != mxp_period) {
    *status = NiFpga_Status_SoftwareFault;
    return -1;
  }
  return hdr_period;
}

struct counter_t {
  tCounter* counter;
  uint32_t index;
};
typedef struct counter_t Counter;

static hal::Resource *counters = NULL;

void* initializeCounter(Mode mode, uint32_t *index, int32_t *status) {
	hal::Resource::CreateResourceObject(&counters, tCounter::kNumSystems);
	*index = counters->Allocate("Counter");
	if (*index == ~0ul) {
		*status = NO_AVAILABLE_RESOURCES;
		return NULL;
	}
	Counter* counter = new Counter();
	counter->counter = tCounter::create(*index, status);
	counter->counter->writeConfig_Mode(mode, status);
	counter->counter->writeTimerConfig_AverageSize(1, status);
	counter->index = *index;
	return counter;
}

void freeCounter(void* counter_pointer, int32_t *status) {
	Counter* counter = (Counter*) counter_pointer;
	if (!counter) return;
	delete counter->counter;
	counters->Free(counter->index);
}

void setCounterAverageSize(void* counter_pointer, int32_t size, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  counter->counter->writeTimerConfig_AverageSize(size, status);
}

/**
 * remap the digital source pin and set the module.
 * If it's an analog trigger, determine the module from the high order routing channel
 * else do normal digital input remapping based on pin number (MXP)
 */
extern "C++" void remapDigitalSource(bool analogTrigger, uint32_t &pin, uint8_t &module) {
  if (analogTrigger) {
    module = pin >> 4;
  } else {
    if(pin >= kNumHeaders) {
      pin = remapMXPChannel(pin);
      module = 1;
    } else {
      module = 0;
    }
  }
}

/**
 * Set the source object that causes the counter to count up.
 * Set the up counting DigitalSource.
 */
void setCounterUpSource(void* counter_pointer, uint32_t pin, bool analogTrigger, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;

  uint8_t module;

  remapDigitalSource(analogTrigger, pin, module);

  counter->counter->writeConfig_UpSource_Module(module, status);
  counter->counter->writeConfig_UpSource_Channel(pin, status);
  counter->counter->writeConfig_UpSource_AnalogTrigger(analogTrigger, status);

  if(counter->counter->readConfig_Mode(status) == kTwoPulse ||
	 counter->counter->readConfig_Mode(status) == kExternalDirection) {
	setCounterUpSourceEdge(counter_pointer, true, false, status);
  }
  counter->counter->strobeReset(status);
}

/**
 * Set the edge sensitivity on an up counting source.
 * Set the up source to either detect rising edges or falling edges.
 */
void setCounterUpSourceEdge(void* counter_pointer, bool risingEdge, bool fallingEdge, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  counter->counter->writeConfig_UpRisingEdge(risingEdge, status);
  counter->counter->writeConfig_UpFallingEdge(fallingEdge, status);
}

/**
 * Disable the up counting source to the counter.
 */
void clearCounterUpSource(void* counter_pointer, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  counter->counter->writeConfig_UpFallingEdge(false, status);
  counter->counter->writeConfig_UpRisingEdge(false, status);
  // Index 0 of digital is always 0.
  counter->counter->writeConfig_UpSource_Channel(0, status);
  counter->counter->writeConfig_UpSource_AnalogTrigger(false, status);
}

/**
 * Set the source object that causes the counter to count down.
 * Set the down counting DigitalSource.
 */
void setCounterDownSource(void* counter_pointer, uint32_t pin, bool analogTrigger, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  unsigned char mode = counter->counter->readConfig_Mode(status);
  if (mode != kTwoPulse && mode != kExternalDirection) {
	// TODO: wpi_setWPIErrorWithContext(ParameterOutOfRange, "Counter only supports DownSource in TwoPulse and ExternalDirection modes.");
	*status = PARAMETER_OUT_OF_RANGE;
	return;
  }

  uint8_t module;

  remapDigitalSource(analogTrigger, pin, module);

  counter->counter->writeConfig_DownSource_Module(module, status);
  counter->counter->writeConfig_DownSource_Channel(pin, status);
  counter->counter->writeConfig_DownSource_AnalogTrigger(analogTrigger, status);

  setCounterDownSourceEdge(counter_pointer, true, false, status);
  counter->counter->strobeReset(status);
}

/**
 * Set the edge sensitivity on a down counting source.
 * Set the down source to either detect rising edges or falling edges.
 */
void setCounterDownSourceEdge(void* counter_pointer, bool risingEdge, bool fallingEdge, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  counter->counter->writeConfig_DownRisingEdge(risingEdge, status);
  counter->counter->writeConfig_DownFallingEdge(fallingEdge, status);
}

/**
 * Disable the down counting source to the counter.
 */
void clearCounterDownSource(void* counter_pointer, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  counter->counter->writeConfig_DownFallingEdge(false, status);
  counter->counter->writeConfig_DownRisingEdge(false, status);
  // Index 0 of digital is always 0.
  counter->counter->writeConfig_DownSource_Channel(0, status);
  counter->counter->writeConfig_DownSource_AnalogTrigger(false, status);
}

/**
 * Set standard up / down counting mode on this counter.
 * Up and down counts are sourced independently from two inputs.
 */
void setCounterUpDownMode(void* counter_pointer, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  counter->counter->writeConfig_Mode(kTwoPulse, status);
}

/**
 * Set external direction mode on this counter.
 * Counts are sourced on the Up counter input.
 * The Down counter input represents the direction to count.
 */
void setCounterExternalDirectionMode(void* counter_pointer, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  counter->counter->writeConfig_Mode(kExternalDirection, status);
}

/**
 * Set Semi-period mode on this counter.
 * Counts up on both rising and falling edges.
 */
void setCounterSemiPeriodMode(void* counter_pointer, bool highSemiPeriod, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  counter->counter->writeConfig_Mode(kSemiperiod, status);
  counter->counter->writeConfig_UpRisingEdge(highSemiPeriod, status);
  setCounterUpdateWhenEmpty(counter_pointer, false, status);
}

/**
 * Configure the counter to count in up or down based on the length of the input pulse.
 * This mode is most useful for direction sensitive gear tooth sensors.
 * @param threshold The pulse length beyond which the counter counts the opposite direction.  Units are seconds.
 */
void setCounterPulseLengthMode(void* counter_pointer, double threshold, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  counter->counter->writeConfig_Mode(kPulseLength, status);
  counter->counter->writeConfig_PulseLengthThreshold((uint32_t)(threshold * 1.0e6) * kSystemClockTicksPerMicrosecond, status);
}

/**
 * Get the Samples to Average which specifies the number of samples of the timer to
 * average when calculating the period. Perform averaging to account for
 * mechanical imperfections or as oversampling to increase resolution.
 * @return SamplesToAverage The number of samples being averaged (from 1 to 127)
 */
int32_t getCounterSamplesToAverage(void* counter_pointer, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  return counter->counter->readTimerConfig_AverageSize(status);
}

/**
 * Set the Samples to Average which specifies the number of samples of the timer to
 * average when calculating the period. Perform averaging to account for
 * mechanical imperfections or as oversampling to increase resolution.
 * @param samplesToAverage The number of samples to average from 1 to 127.
 */
void setCounterSamplesToAverage(void* counter_pointer, int samplesToAverage, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  if (samplesToAverage < 1 || samplesToAverage > 127) {
	*status = PARAMETER_OUT_OF_RANGE;
  }
  counter->counter->writeTimerConfig_AverageSize(samplesToAverage, status);
}

/**
 * Reset the Counter to zero.
 * Set the counter value to zero. This doesn't effect the running state of the counter, just sets
 * the current value to zero.
 */
void resetCounter(void* counter_pointer, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  counter->counter->strobeReset(status);
}

/**
 * Read the current counter value.
 * Read the value at this instant. It may still be running, so it reflects the current value. Next
 * time it is read, it might have a different value.
 */
int32_t getCounter(void* counter_pointer, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  int32_t value = counter->counter->readOutput_Value(status);
  return value;
}

/*
 * Get the Period of the most recent count.
 * Returns the time interval of the most recent count. This can be used for velocity calculations
 * to determine shaft speed.
 * @returns The period of the last two pulses in units of seconds.
 */
double getCounterPeriod(void* counter_pointer, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  tCounter::tTimerOutput output = counter->counter->readTimerOutput(status);
  double period;
  if (output.Stalled)	{
	// Return infinity
	double zero = 0.0;
	period = 1.0 / zero;
  } else {
	// output.Period is a fixed point number that counts by 2 (24 bits, 25 integer bits)
	period = (double)(output.Period << 1) / (double)output.Count;
  }
  return period * 2.5e-8;  // result * timebase (currently 40ns)
}

/**
 * Set the maximum period where the device is still considered "moving".
 * Sets the maximum period where the device is considered moving. This value is used to determine
 * the "stopped" state of the counter using the GetStopped method.
 * @param maxPeriod The maximum period where the counted device is considered moving in
 * seconds.
 */
void setCounterMaxPeriod(void* counter_pointer, double maxPeriod, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  counter->counter->writeTimerConfig_StallPeriod((uint32_t)(maxPeriod * 4.0e8), status);
}

/**
 * Select whether you want to continue updating the event timer output when there are no samples captured.
 * The output of the event timer has a buffer of periods that are averaged and posted to
 * a register on the FPGA.  When the timer detects that the event source has stopped
 * (based on the MaxPeriod) the buffer of samples to be averaged is emptied.  If you
 * enable the update when empty, you will be notified of the stopped source and the event
 * time will report 0 samples.  If you disable update when empty, the most recent average
 * will remain on the output until a new sample is acquired.  You will never see 0 samples
 * output (except when there have been no events since an FPGA reset) and you will likely not
 * see the stopped bit become true (since it is updated at the end of an average and there are
 * no samples to average).
 */
void setCounterUpdateWhenEmpty(void* counter_pointer, bool enabled, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  counter->counter->writeTimerConfig_UpdateWhenEmpty(enabled, status);
}

/**
 * Determine if the clock is stopped.
 * Determine if the clocked input is stopped based on the MaxPeriod value set using the
 * SetMaxPeriod method. If the clock exceeds the MaxPeriod, then the device (and counter) are
 * assumed to be stopped and it returns true.
 * @return Returns true if the most recent counter period exceeds the MaxPeriod value set by
 * SetMaxPeriod.
 */
bool getCounterStopped(void* counter_pointer, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  return counter->counter->readTimerOutput_Stalled(status);
}

/**
 * The last direction the counter value changed.
 * @return The last direction the counter value changed.
 */
bool getCounterDirection(void* counter_pointer, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  bool value = counter->counter->readOutput_Direction(status);
  return value;
}

/**
 * Set the Counter to return reversed sensing on the direction.
 * This allows counters to change the direction they are counting in the case of 1X and 2X
 * quadrature encoding only. Any other counter mode isn't supported.
 * @param reverseDirection true if the value counted should be negated.
 */
void setCounterReverseDirection(void* counter_pointer, bool reverseDirection, int32_t *status) {
  Counter* counter = (Counter*) counter_pointer;
  if (counter->counter->readConfig_Mode(status) == kExternalDirection) {
	if (reverseDirection)
	  setCounterDownSourceEdge(counter_pointer, true, true, status);
	else
	  setCounterDownSourceEdge(counter_pointer, false, true, status);
  }
}

struct encoder_t {
  tEncoder* encoder;
  uint32_t index;
};
typedef struct encoder_t Encoder;

static const double DECODING_SCALING_FACTOR = 0.25;
static hal::Resource *quadEncoders = NULL;

void* initializeEncoder(uint8_t port_a_module, uint32_t port_a_pin, bool port_a_analog_trigger,
						uint8_t port_b_module, uint32_t port_b_pin, bool port_b_analog_trigger,
						bool reverseDirection, int32_t *index, int32_t *status) {

  // Initialize encoder structure
  Encoder* encoder = new Encoder();

  remapDigitalSource(port_a_analog_trigger, port_a_pin, port_a_module);
  remapDigitalSource(port_b_analog_trigger, port_b_pin, port_b_module);

  hal::Resource::CreateResourceObject(&quadEncoders, tEncoder::kNumSystems);
  encoder->index = quadEncoders->Allocate("4X Encoder");
  *index = encoder->index;
  // TODO: if (index == ~0ul) { CloneError(quadEncoders); return; }
  encoder->encoder = tEncoder::create(encoder->index, status);
  encoder->encoder->writeConfig_ASource_Module(port_a_module, status);
  encoder->encoder->writeConfig_ASource_Channel(port_a_pin, status);
  encoder->encoder->writeConfig_ASource_AnalogTrigger(port_a_analog_trigger, status);
  encoder->encoder->writeConfig_BSource_Module(port_b_module, status);
  encoder->encoder->writeConfig_BSource_Channel(port_b_pin, status);
  encoder->encoder->writeConfig_BSource_AnalogTrigger(port_b_analog_trigger, status);
  encoder->encoder->strobeReset(status);
  encoder->encoder->writeConfig_Reverse(reverseDirection, status);
  encoder->encoder->writeTimerConfig_AverageSize(4, status);

  return encoder;
}

void freeEncoder(void* encoder_pointer, int32_t *status) {
  Encoder* encoder = (Encoder*) encoder_pointer;
  if (!encoder) return;
  quadEncoders->Free(encoder->index);
  delete encoder->encoder;
}

/**
 * Reset the Encoder distance to zero.
 * Resets the current count to zero on the encoder.
 */
void resetEncoder(void* encoder_pointer, int32_t *status) {
  Encoder* encoder = (Encoder*) encoder_pointer;
  encoder->encoder->strobeReset(status);
}

/**
 * Gets the raw value from the encoder.
 * The raw value is the actual count unscaled by the 1x, 2x, or 4x scale
 * factor.
 * @return Current raw count from the encoder
 */
int32_t getEncoder(void* encoder_pointer, int32_t *status) {
  Encoder* encoder = (Encoder*) encoder_pointer;
  return encoder->encoder->readOutput_Value(status);
}

/**
 * Returns the period of the most recent pulse.
 * Returns the period of the most recent Encoder pulse in seconds.
 * This method compenstates for the decoding type.
 *
 * @deprecated Use GetRate() in favor of this method.  This returns unscaled periods and GetRate() scales using value from SetDistancePerPulse().
 *
 * @return Period in seconds of the most recent pulse.
 */
double getEncoderPeriod(void* encoder_pointer, int32_t *status) {
  Encoder* encoder = (Encoder*) encoder_pointer;
  tEncoder::tTimerOutput output = encoder->encoder->readTimerOutput(status);
  double value;
  if (output.Stalled) {
	// Return infinity
	double zero = 0.0;
	value = 1.0 / zero;
  } else {
	// output.Period is a fixed point number that counts by 2 (24 bits, 25 integer bits)
	value = (double)(output.Period << 1) / (double)output.Count;
  }
  double measuredPeriod = value * 2.5e-8;
  return measuredPeriod / DECODING_SCALING_FACTOR;
}

/**
 * Sets the maximum period for stopped detection.
 * Sets the value that represents the maximum period of the Encoder before it will assume
 * that the attached device is stopped. This timeout allows users to determine if the wheels or
 * other shaft has stopped rotating.
 * This method compensates for the decoding type.
 *
 * @deprecated Use SetMinRate() in favor of this method.  This takes unscaled periods and SetMinRate() scales using value from SetDistancePerPulse().
 *
 * @param maxPeriod The maximum time between rising and falling edges before the FPGA will
 * report the device stopped. This is expressed in seconds.
 */
void setEncoderMaxPeriod(void* encoder_pointer, double maxPeriod, int32_t *status) {
  Encoder* encoder = (Encoder*) encoder_pointer;
  encoder->encoder->writeTimerConfig_StallPeriod((uint32_t)(maxPeriod * 4.0e8 * DECODING_SCALING_FACTOR), status);
}

/**
 * Determine if the encoder is stopped.
 * Using the MaxPeriod value, a boolean is returned that is true if the encoder is considered
 * stopped and false if it is still moving. A stopped encoder is one where the most recent pulse
 * width exceeds the MaxPeriod.
 * @return True if the encoder is considered stopped.
 */
bool getEncoderStopped(void* encoder_pointer, int32_t *status) {
  Encoder* encoder = (Encoder*) encoder_pointer;
  return encoder->encoder->readTimerOutput_Stalled(status) != 0;
}

/**
 * The last direction the encoder value changed.
 * @return The last direction the encoder value changed.
 */
bool getEncoderDirection(void* encoder_pointer, int32_t *status) {
  Encoder* encoder = (Encoder*) encoder_pointer;
  return encoder->encoder->readOutput_Direction(status);
}

/**
 * Set the direction sensing for this encoder.
 * This sets the direction sensing on the encoder so that it could count in the correct
 * software direction regardless of the mounting.
 * @param reverseDirection true if the encoder direction should be reversed
 */
void setEncoderReverseDirection(void* encoder_pointer, bool reverseDirection, int32_t *status) {
  Encoder* encoder = (Encoder*) encoder_pointer;
  encoder->encoder->writeConfig_Reverse(reverseDirection, status);
}

/**
 * Set the Samples to Average which specifies the number of samples of the timer to
 * average when calculating the period. Perform averaging to account for
 * mechanical imperfections or as oversampling to increase resolution.
 * @param samplesToAverage The number of samples to average from 1 to 127.
 */
void setEncoderSamplesToAverage(void* encoder_pointer, uint32_t samplesToAverage, int32_t *status) {
  Encoder* encoder = (Encoder*) encoder_pointer;
  if (samplesToAverage < 1 || samplesToAverage > 127) {
	*status = PARAMETER_OUT_OF_RANGE;
  }
  encoder->encoder->writeTimerConfig_AverageSize(samplesToAverage, status);
}

/**
 * Get the Samples to Average which specifies the number of samples of the timer to
 * average when calculating the period. Perform averaging to account for
 * mechanical imperfections or as oversampling to increase resolution.
 * @return SamplesToAverage The number of samples being averaged (from 1 to 127)
 */
uint32_t getEncoderSamplesToAverage(void* encoder_pointer, int32_t *status) {
  Encoder* encoder = (Encoder*) encoder_pointer;
  return encoder->encoder->readTimerConfig_AverageSize(status);
}

/**
 * Set an index source for an encoder, which is an input that resets the
 * encoder's count.
 */
void setEncoderIndexSource(void *encoder_pointer, uint32_t pin, bool analogTrigger, bool activeHigh,
    bool edgeSensitive, int32_t *status) {
  Encoder* encoder = (Encoder*) encoder_pointer;
  encoder->encoder->writeConfig_IndexSource_Channel((unsigned char)pin, status);
  encoder->encoder->writeConfig_IndexSource_Module((unsigned char)0, status);
  encoder->encoder->writeConfig_IndexSource_AnalogTrigger(analogTrigger, status);
  encoder->encoder->writeConfig_IndexActiveHigh(activeHigh, status);
  encoder->encoder->writeConfig_IndexEdgeSensitive(edgeSensitive, status);
}

/**
 * Get the loop timing of the PWM system
 *
 * @return The loop time
 */
uint16_t getLoopTiming(int32_t *status) {
  return pwmSystem->readLoopTiming(status);
}

/*
 * Initialize the spi port. Opens the port if necessary and saves the handle.
 * If opening the MXP port, also sets up the pin functions appropriately
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 */
void spiInitialize(uint8_t port, int32_t *status) {
	if(spiSystem == NULL)
		spiSystem = tSPI::create(status);
	if(spiGetHandle(port) !=0 ) return;
	switch(port){
	case 0:
		spiSetHandle(0, spilib_open("/dev/spidev0.0"));
		break;
	case 1:
		spiSetHandle(1, spilib_open("/dev/spidev0.1"));
		break;
	case 2:
		spiSetHandle(2, spilib_open("/dev/spidev0.2"));
		break;
	case 3:
		spiSetHandle(3, spilib_open("/dev/spidev0.3"));
		break;
	case 4:
		initializeDigital(status);
		if(!allocateDIO(getPort(14), false, status)){printf("Failed to allocate DIO 14\n"); return;}
		if(!allocateDIO(getPort(15), false, status)) {printf("Failed to allocate DIO 15\n"); return;}
		if(!allocateDIO(getPort(16), true, status)) {printf("Failed to allocate DIO 16\n"); return;}
		if(!allocateDIO(getPort(17), false, status)) {printf("Failed to allocate DIO 17\n"); return;}
		digitalSystem->writeEnableMXPSpecialFunction(digitalSystem->readEnableMXPSpecialFunction(status)|0x00F0, status);
		spiSetHandle(4, spilib_open("/dev/spidev1.0"));
		break;
	default:
		break;
	}
	return;
}

/**
 * Generic transaction.
 *
 * This is a lower-level interface to the spi hardware giving you more control over each transaction.
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 * @param dataToSend Buffer of data to send as part of the transaction.
 * @param dataReceived Buffer to read data into.
 * @param size Number of bytes to transfer. [0..7]
 * @return Number of bytes transferred, -1 for error
 */
int32_t spiTransaction(uint8_t port, uint8_t *dataToSend, uint8_t *dataReceived, uint8_t size)
{
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	return spilib_writeread(spiGetHandle(port), (const char*) dataToSend, (char*) dataReceived, (int32_t) size);
}

/**
 * Execute a write transaction with the device.
 *
 * Write to a device and wait until the transaction is complete.
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 * @param datToSend The data to write to the register on the device.
 * @param sendSize The number of bytes to be written
 * @return The number of bytes written. -1 for an error
 */
int32_t spiWrite(uint8_t port, uint8_t* dataToSend, uint8_t sendSize)
{
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	return spilib_write(spiGetHandle(port), (const char*) dataToSend, (int32_t) sendSize);
}

/**
 * Execute a read from the device.
 *
 *   This methdod does not write any data out to the device
 *   Most spi devices will require a register address to be written before
 *   they begin returning data
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 * @param buffer A pointer to the array of bytes to store the data read from the device.
 * @param count The number of bytes to read in the transaction. [1..7]
 * @return Number of bytes read. -1 for error.
 */
int32_t spiRead(uint8_t port, uint8_t *buffer, uint8_t count)
{
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	return spilib_read(spiGetHandle(port), (char*) buffer, (int32_t) count);
}

/**
 * Close the SPI port
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 */
void spiClose(uint8_t port) {
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	if (spiAccumulators[port]) {
		int32_t status = 0;
		spiFreeAccumulator(port, &status);
	}
	spilib_close(spiGetHandle(port));
	spiSetHandle(port, 0);
	return;
}

/**
 * Set the clock speed for the SPI bus.
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 * @param speed The speed in Hz (0-1MHz)
 */
void spiSetSpeed(uint8_t port, uint32_t speed) {
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	spilib_setspeed(spiGetHandle(port), speed);
}

/**
 * Set the SPI options
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 * @param msb_first True to write the MSB first, False for LSB first
 * @param sample_on_trailing True to sample on the trailing edge, False to sample on the leading edge
 * @param clk_idle_high True to set the clock to active low, False to set the clock active high
 */
void spiSetOpts(uint8_t port, int msb_first, int sample_on_trailing, int clk_idle_high) {
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	spilib_setopts(spiGetHandle(port), msb_first, sample_on_trailing, clk_idle_high);
}

/**
 * Set the CS Active high for a SPI port
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 */
void spiSetChipSelectActiveHigh(uint8_t port, int32_t *status){
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	if(port < 4)
	{
		spiSystem->writeChipSelectActiveHigh_Hdr(spiSystem->readChipSelectActiveHigh_Hdr(status) | (1<<port), status);
	}
	else
	{
		spiSystem->writeChipSelectActiveHigh_MXP(1, status);
	}
}

/**
 * Set the CS Active low for a SPI port
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 */
void spiSetChipSelectActiveLow(uint8_t port, int32_t *status){
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	if(port < 4)
	{
		spiSystem->writeChipSelectActiveHigh_Hdr(spiSystem->readChipSelectActiveHigh_Hdr(status) & ~(1<<port), status);
	}
	else
	{
		spiSystem->writeChipSelectActiveHigh_MXP(0, status);
	}
}

/**
 * Get the stored handle for a SPI port
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 * @return The stored handle for the SPI port. 0 represents no stored handle.
 */
int32_t spiGetHandle(uint8_t port){
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	switch(port){
	case 0:
		return m_spiCS0Handle;
	case 1:
		return m_spiCS1Handle;
	case 2:
		return m_spiCS2Handle;
	case 3:
		return m_spiCS3Handle;
	case 4:
		return m_spiMXPHandle;
	default:
		return 0;
	}
}

/**
 * Set the stored handle for a SPI port
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP.
 * @param handle The value of the handle for the port.
 */
void spiSetHandle(uint8_t port, int32_t handle){
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	switch(port){
	case 0:
		m_spiCS0Handle = handle;
		break;
	case 1:
		m_spiCS1Handle = handle;
		break;
	case 2:
		m_spiCS2Handle = handle;
		break;
	case 3:
		m_spiCS3Handle = handle;
		break;
	case 4:
		m_spiMXPHandle = handle;
		break;
	default:
		break;
	}
}

/**
 * Get the semaphore for a SPI port
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 * @return The semaphore for the SPI port.
 */
extern "C++" priority_recursive_mutex& spiGetSemaphore(uint8_t port) {
	if(port < 4)
		return spiOnboardSemaphore;
	else
		return spiMXPSemaphore;
}

static void spiAccumulatorProcess(uint64_t currentTime, void *param) {
	SPIAccumulator* accum = (SPIAccumulator*)param;

	// perform SPI transaction
	uint8_t resp_b[4];
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(accum->port));
	spilib_writeread(spiGetHandle(accum->port), (const char*) accum->cmd, (char*) resp_b, (int32_t) accum->xfer_size);

	// convert from bytes
	uint32_t resp = 0;
	if (accum->big_endian) {
		for (int i=0; i < accum->xfer_size; ++i) {
			resp <<= 8;
			resp |= resp_b[i] & 0xff;
		}
	} else {
		for (int i = accum->xfer_size - 1; i >= 0; --i) {
			resp <<= 8;
			resp |= resp_b[i] & 0xff;
		}
	}

        // process response
	if ((resp & accum->valid_mask) == accum->valid_value) {
		// valid sensor data; extract data field
		int32_t data = (int32_t)(resp >> accum->data_shift);
		data &= accum->data_max - 1;
		// 2s complement conversion if signed MSB is set
		if (accum->is_signed && (data & accum->data_msb_mask) != 0)
			data -= accum->data_max;
		// center offset
		data -= accum->center;
		// only accumulate if outside deadband
		if (data < -accum->deadband || data > accum->deadband)
			accum->value += data;
		++accum->count;
		accum->last_value = data;
	} else {
		// no data from the sensor; just clear the last value
		accum->last_value = 0;
	}

	// reschedule timer
	accum->triggerTime += accum->period;
	// handle timer slip
	if (accum->triggerTime < currentTime)
		accum->triggerTime = currentTime + accum->period;
	int32_t status = 0;
	updateNotifierAlarm(accum->notifier, accum->triggerTime, &status);
}

/**
 * Initialize a SPI accumulator.
 *
 * @param port SPI port
 * @param period Time between reads, in us
 * @param cmd SPI command to send to request data
 * @param xfer_size SPI transfer size, in bytes
 * @param valid_mask Mask to apply to received data for validity checking
 * @param valid_data After valid_mask is applied, required matching value for
 *                   validity checking
 * @param data_shift Bit shift to apply to received data to get actual data
 *                   value
 * @param data_size Size (in bits) of data field
 * @param is_signed Is data field signed?
 * @param big_endian Is device big endian?
 */
void spiInitAccumulator(uint8_t port, uint32_t period, uint32_t cmd,
		uint8_t xfer_size, uint32_t valid_mask, uint32_t valid_value,
		uint8_t data_shift, uint8_t data_size, bool is_signed,
		bool big_endian, int32_t *status) {
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	if (port > 4) return;
	if (!spiAccumulators[port])
		spiAccumulators[port] = new SPIAccumulator();
	SPIAccumulator* accum = spiAccumulators[port];
	if (big_endian) {
		for (int i = xfer_size - 1; i >= 0; --i) {
			accum->cmd[i] = cmd & 0xff;
			cmd >>= 8;
		}
	} else {
		accum->cmd[0] = cmd & 0xff; cmd >>= 8;
		accum->cmd[1] = cmd & 0xff; cmd >>= 8;
		accum->cmd[2] = cmd & 0xff; cmd >>= 8;
		accum->cmd[3] = cmd & 0xff;
	}
	accum->period = period;
	accum->xfer_size = xfer_size;
	accum->valid_mask = valid_mask;
	accum->valid_value = valid_value;
	accum->data_shift = data_shift;
	accum->data_max = (1 << data_size);
	accum->data_msb_mask = (1 << (data_size - 1));
	accum->is_signed = is_signed;
	accum->big_endian = big_endian;
	if (!accum->notifier) {
		accum->notifier = initializeNotifier(spiAccumulatorProcess, accum, status);
		accum->triggerTime = getFPGATime(status) + period;
		if (*status != 0) return;
		updateNotifierAlarm(accum->notifier, accum->triggerTime, status);
	}
}

/**
 * Frees a SPI accumulator.
 */
void spiFreeAccumulator(uint8_t port, int32_t *status) {
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	SPIAccumulator* accum = spiAccumulators[port];
	if (!accum) {
		*status = NULL_PARAMETER;
		return;
	}
	cleanNotifier(accum->notifier, status);
	delete accum;
	spiAccumulators[port] = nullptr;
}

/**
 * Resets the accumulator to zero.
 */
void spiResetAccumulator(uint8_t port, int32_t *status) {
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	SPIAccumulator* accum = spiAccumulators[port];
	if (!accum) {
		*status = NULL_PARAMETER;
		return;
	}
	accum->value = 0;
	accum->count = 0;
	accum->last_value = 0;
}

/**
 * Set the center value of the accumulator.
 *
 * The center value is subtracted from each value before it is added to the accumulator. This
 * is used for the center value of devices like gyros and accelerometers to make integration work
 * and to take the device offset into account when integrating.
 */
void spiSetAccumulatorCenter(uint8_t port, int32_t center, int32_t *status) {
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	SPIAccumulator* accum = spiAccumulators[port];
	if (!accum) {
		*status = NULL_PARAMETER;
		return;
	}
	accum->center = center;
}

/**
 * Set the accumulator's deadband.
 */
void spiSetAccumulatorDeadband(uint8_t port, int32_t deadband, int32_t *status) {
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	SPIAccumulator* accum = spiAccumulators[port];
	if (!accum) {
		*status = NULL_PARAMETER;
		return;
	}
	accum->deadband = deadband;
}

/**
 * Read the last value read by the accumulator engine.
 */
int32_t spiGetAccumulatorLastValue(uint8_t port, int32_t *status) {
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	SPIAccumulator* accum = spiAccumulators[port];
	if (!accum) {
		*status = NULL_PARAMETER;
		return 0;
	}
	return accum->last_value;
}

/**
 * Read the accumulated value.
 *
 * @return The 64-bit value accumulated since the last Reset().
 */
int64_t spiGetAccumulatorValue(uint8_t port, int32_t *status) {
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	SPIAccumulator* accum = spiAccumulators[port];
	if (!accum) {
		*status = NULL_PARAMETER;
		return 0;
	}
	return accum->value;
}

/**
 * Read the number of accumulated values.
 *
 * Read the count of the accumulated values since the accumulator was last Reset().
 *
 * @return The number of times samples from the channel were accumulated.
 */
uint32_t spiGetAccumulatorCount(uint8_t port, int32_t *status) {
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	SPIAccumulator* accum = spiAccumulators[port];
	if (!accum) {
		*status = NULL_PARAMETER;
		return 0;
	}
	return accum->count;
}

/**
 * Read the average of the accumulated value.
 *
 * @return The accumulated average value (value / count).
 */
double spiGetAccumulatorAverage(uint8_t port, int32_t *status) {
	int64_t value;
	uint32_t count;
	spiGetAccumulatorOutput(port, &value, &count, status);
	if (count == 0) return 0.0;
	return ((double)value) / count;
}

/**
 * Read the accumulated value and the number of accumulated values atomically.
 *
 * This function reads the value and count atomically.
 * This can be used for averaging.
 *
 * @param value Pointer to the 64-bit accumulated output.
 * @param count Pointer to the number of accumulation cycles.
 */
void spiGetAccumulatorOutput(uint8_t port, int64_t *value, uint32_t *count,
		int32_t *status) {
	std::lock_guard<priority_recursive_mutex> sync(spiGetSemaphore(port));
	SPIAccumulator* accum = spiAccumulators[port];
	if (!accum) {
		*status = NULL_PARAMETER;
		*value = 0;
		*count = 0;
		return;
	}
	*value = accum->value;
	*count = accum->count;
}

/*
 * Initialize the I2C port. Opens the port if necessary and saves the handle.
 * If opening the MXP port, also sets up the pin functions appropriately
 * @param port The port to open, 0 for the on-board, 1 for the MXP.
 */
void i2CInitialize(uint8_t port, int32_t *status) {
	initializeDigital(status);

	if(port > 1)
	{
		//Set port out of range error here
		return;
	}

	priority_recursive_mutex &lock = port == 0 ? digitalI2COnBoardMutex : digitalI2CMXPMutex;
	{
		std::lock_guard<priority_recursive_mutex> sync(lock);
		if(port == 0) {
			i2COnboardObjCount++;
			if (i2COnBoardHandle > 0) return;
			i2COnBoardHandle = i2clib_open("/dev/i2c-2");
		} else if(port == 1) {
			i2CMXPObjCount++;
			if (i2CMXPHandle > 0) return;
			if(!allocateDIO(getPort(24), false, status)) return;
			if(!allocateDIO(getPort(25), false, status)) return;
			digitalSystem->writeEnableMXPSpecialFunction(digitalSystem->readEnableMXPSpecialFunction(status)|0xC000, status);
			i2CMXPHandle = i2clib_open("/dev/i2c-1");
		}
	return;
	}
}

/**
 * Generic transaction.
 *
 * This is a lower-level interface to the I2C hardware giving you more control over each transaction.
 *
 * @param dataToSend Buffer of data to send as part of the transaction.
 * @param sendSize Number of bytes to send as part of the transaction.
 * @param dataReceived Buffer to read data into.
 * @param receiveSize Number of bytes to read from the device.
 * @return Transfer Aborted... false for success, true for aborted.
 */
int32_t i2CTransaction(uint8_t port, uint8_t deviceAddress, uint8_t *dataToSend, uint8_t sendSize, uint8_t *dataReceived, uint8_t receiveSize)
{
	if(port > 1) {
		//Set port out of range error here
		return -1;
	}

	int32_t handle = port == 0 ? i2COnBoardHandle:i2CMXPHandle;
	priority_recursive_mutex &lock = port == 0 ? digitalI2COnBoardMutex : digitalI2CMXPMutex;

	{
		std::lock_guard<priority_recursive_mutex> sync(lock);
		return i2clib_writeread(handle, deviceAddress, (const char*) dataToSend, (int32_t) sendSize, (char*) dataReceived, (int32_t) receiveSize);
	}
}

/**
 * Execute a write transaction with the device.
 *
 * Write a single byte to a register on a device and wait until the
 *   transaction is complete.
 *
 * @param registerAddress The address of the register on the device to be written.
 * @param data The byte to write to the register on the device.
 * @return Transfer Aborted... false for success, true for aborted.
 */
int32_t i2CWrite(uint8_t port, uint8_t deviceAddress, uint8_t* dataToSend, uint8_t sendSize)
{
	if(port > 1) {
		//Set port out of range error here
		return -1;
	}

	int32_t handle = port == 0 ? i2COnBoardHandle:i2CMXPHandle;
	priority_recursive_mutex &lock = port == 0 ? digitalI2COnBoardMutex : digitalI2CMXPMutex;
	{
		std::lock_guard<priority_recursive_mutex> sync(lock);
		return i2clib_write(handle, deviceAddress, (const char*) dataToSend, (int32_t) sendSize);
	}
}

/**
 * Execute a read transaction with the device.
 *
 * Read bytes from a device.
 * Most I2C devices will auto-increment the register pointer internally allowing
 *   you to read consecutive registers on a device in a single transaction.
 *
 * @param registerAddress The register to read first in the transaction.
 * @param count The number of bytes to read in the transaction.
 * @param buffer A pointer to the array of bytes to store the data read from the device.
 * @return Transfer Aborted... false for success, true for aborted.
 */
int32_t i2CRead(uint8_t port, uint8_t deviceAddress, uint8_t *buffer, uint8_t count)
{
	if(port > 1) {
		//Set port out of range error here
		return -1;
	}

	int32_t handle = port == 0 ? i2COnBoardHandle:i2CMXPHandle;
	priority_recursive_mutex &lock = port == 0 ? digitalI2COnBoardMutex : digitalI2CMXPMutex;
	{
		std::lock_guard<priority_recursive_mutex> sync(lock);
		return i2clib_read(handle, deviceAddress, (char*) buffer, (int32_t) count);
	}

}

void i2CClose(uint8_t port) {
	if(port > 1) {
		//Set port out of range error here
		return;
	}
	priority_recursive_mutex &lock = port == 0 ? digitalI2COnBoardMutex : digitalI2CMXPMutex;
	{
		std::lock_guard<priority_recursive_mutex> sync(lock);
		if((port == 0 ? i2COnboardObjCount--:i2CMXPObjCount--) == 0) {
			int32_t handle = port == 0 ? i2COnBoardHandle:i2CMXPHandle;
			i2clib_close(handle);
		}
	}
	return;
}

}  // extern "C"
