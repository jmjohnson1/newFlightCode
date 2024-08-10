#include "radio.h"

RadioChannel::RadioChannel(String name, uint8_t channel, uint16_t zeroPoint, 
													 uint16_t failsafe, bool critical, uint16_t minRange, uint16_t maxRange) {
	name_ = name;
	Serial.print("name_ = ");
	Serial.println(name_);
	rawValue_ = failsafe;
	Serial.print("rawValue_ = ");
	Serial.println(rawValue_);
	rawValue_previous_ = failsafe;
	failsafeValue_ = failsafe;
	channel_ = channel;
	zeroPointRawValue_ = zeroPoint;
	isCritical_ = critical;
	minRange_ = minRange;
	maxRange_ = maxRange;
}

/**
 * This checks for a failure in the current channel. If there is one, then the bit corresponding to
 * this channel in the 16 bit failureFlag is updated to reflect it. For example, an issue with
 * channels 2 and 6 should yield a failureFlag that looks like 0000000000100010.
*/
void RadioChannel::FailureCheck(uint16_t *failureFlag) {
	// I may have made this too convoluted.
	uint16_t maxValue = 2200;
	uint16_t minValue = 800;
	uint16_t currentChannelFail = 0; // Current channel set to 0000000000000000
	uint16_t newFailureFlag = 0; 

	// If the rawValue is above the max or below the min, then something's wrong.
	if (rawValue_ > maxValue || rawValue_ < minValue) {
		// Left shift the 1 into the proper position for the current channel to indicate it has failed
		currentChannelFail = 1<<(channel_ - 1);
	}
	// OR the old failure flag with the one for the current channel to update it
	newFailureFlag = *failureFlag || currentChannelFail;
	*failureFlag = newFailureFlag;
}

SwPos RadioChannel::SwitchPosition() {
	if (rawValue_ < 1250) {
		return SwPos::SWITCH_LOW;
	} else if (rawValue_ > 1250 && rawValue_ < 1750) {
		return SwPos::SWITCH_MID;
	} else {
		return SwPos::SWITCH_HIGH;
	}
}

float RadioChannel::NormalizedValue() {
	float normalizedValue = static_cast<float>(rawValue_ - zeroPointRawValue_)/
		static_cast<float>(maxRange_ - zeroPointRawValue_);
	return normalizedValue;
}

void RadioChannel::Update(uint16_t newValue) {
	rawValue_previous_ = rawValue_;
	rawValue_ = newValue;
	if (isCritical_) {
		LowpassCritical();
	}
}

void RadioChannel::LowpassCritical() {
	rawValue_ = (1 - FILTER_PARAM)*rawValue_previous_ + FILTER_PARAM*rawValue_;
}
