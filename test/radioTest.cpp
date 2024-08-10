#include "radio.h"
/*#include "SBUS.h"*/
#include "datalogger.h"


RadioChannel *throttleChannel = nullptr;
Datalogger logging;
/*SBUS sbus(Serial5);*/

const uint8_t numChannels = 1;
RadioChannel *radioChannels[numChannels];

void LoggingSetup() {
	// Raw radio PWM values (1000-2000)
	for (int i = 0; i < numChannels; i++) {
		Serial.println(i);
		logging.AddItem(&(radioChannels[i]->rawValue_), radioChannels[i]->GetName(), 10);
	}
}

void Setup() {
  Serial.begin(500000); // USB serial (baud rate doesn't actually matter for Teensy)
  delay(500); // Give Serial some time to initialize
	/*sbus.begin();*/
	Serial.print("Initial address = ");
	Serial.println((uint64_t)throttleChannel);
	throttleChannel = new RadioChannel("throttle", 1, 1000, 1000, true, 1000, 1965);
	Serial.print("Assigned address = ");
	Serial.println((uint64_t)throttleChannel);
	Serial.print("Raw value = ");
	Serial.println(throttleChannel->GetRawValue());
	radioChannels[0] = throttleChannel;
	Serial.print("Radio channel address: ");
	Serial.println((uint64_t)radioChannels[0]);

	LoggingSetup();
}

void Loop() {
	Serial.println("In main loop");
	delay(2000);
}

int main() {
	Setup();
	while (true) {
		Loop();
	}
}
