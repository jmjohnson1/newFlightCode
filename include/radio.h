#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include "Arduino.h"

enum class SwPos {
	SWITCH_LOW = 0,
	SWITCH_MID = 1,
	SWITCH_HIGH = 2,
};

class RadioChannel {
public:
	RadioChannel(String name, uint8_t channel, uint16_t zeroPoint, uint16_t failsafe, bool critical = false, uint16_t minRange = 1000, uint16_t maxRange = 2000);
	void FailureCheck(uint16_t *failureFlag);
	SwPos SwitchPosition();
	float NormalizedValue();
	void Update(uint16_t newValue);
	void LowpassCritical();
	void TriggerFailsafe() {rawValue_ = failsafeValue_;}

	uint8_t GetChannel() {return channel_;}
	uint16_t GetRawValue() {return rawValue_;}
	String GetName() {return name_;}
	// This has to be public because of the way I do datalogging for now
	uint16_t rawValue_;
private:
	String name_;
	uint16_t rawValue_previous_;
	uint16_t failsafeValue_;
	uint8_t channel_;
	uint16_t zeroPointRawValue_;
	uint16_t minRange_;
	uint16_t maxRange_;
	bool isCritical_;

	const float FILTER_PARAM = 0.7f;
};

#endif
