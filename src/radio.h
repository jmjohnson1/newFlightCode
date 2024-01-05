#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>

class RadioChannel {
public:
	RadioChannel(uint8_t channel, uint16_t zeroPoint, uint16_t failsafe);
	bool FailureCheck();
	uint8_t SwitchPosition();
	float NormalizedValue();
private:
	uint16_t rawValue_;
	uint16_t rawValue_previous_;
	uint16_t failsafeValue_;
	uint8_t channel_;
	uint16_t zeroPointRawValue_;

	const float FILTER_PARAM = 0.7f;
};

#endif
