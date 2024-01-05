#include "radio.h"

RadioChannel::RadioChannel(uint8_t channel, uint16_t zeroPoint, uint16_t failsafe) {
	rawValue_ = failsafe;
	rawValue_previous_ = failsafe;
	failsafeValue_ = failsafe;
	channel_ = channel;
	zeroPointRawValue_ = zeroPoint;
}

bool RadioChannel::FailureCheck() {
	uint16_t maxValue = 2200;
	uint16_t minValue = 800;

	if (rawValue_ > maxValue || rawValue_ < minValue) {
		return true;
	}
	return false;
}

uint8_t RadioChannel::SwitchPosition() {
	if (rawValue_ < 1250) {
		return 0; // LOW
	} else if (rawValue_ > 1250 && rawValue_ < 1750) {
		return 1; // MID
	} else {
		return 2; // HIGH
	}
}

float RadioChannel::NormalizedValue() {
	float normalizedValue = static_cast<float>(rawValue_ - zeroPointRawValue_)/static_cast<float>(2000 - zeroPointRawValue_);
	return normalizedValue;
}
