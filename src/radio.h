#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include "Arduino.h"

class RadioChannel {
public:
	RadioChannel(String name, uint8_t channel, uint16_t zeroPoint, uint16_t failsafe);
	bool FailureCheck();
	uint8_t SwitchPosition();
	float NormalizedValue();
private:
	String name_;
	uint16_t rawValue_;
	uint16_t rawValue_previous_;
	uint16_t failsafeValue_;
	uint8_t channel_;
	uint16_t zeroPointRawValue_;

	const float FILTER_PARAM = 0.7f;
};

#endif
