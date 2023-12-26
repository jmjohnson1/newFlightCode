#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>
#include <Arduino.h>

#include "src/mavlink/common/mavlink.h"

// Define class for a single parameter
class Parameter {
public:
	char name[16];
	float value;
	uint8_t mavParamType;

	Parameter() {
		// Default constructor
		// Initialize with empty values
		memset(name, 0, sizeof(name));
		value = 0.0f;
		mavParamType = 0;
	}

	Parameter(const char paramName[16], float parameterValue, uint8_t parameterType) {
		// Constructor to set parameter values
		strncpy(name, paramName, sizeof(name));
		value = parameterValue;
		mavParamType = parameterType;
	}
};

// Devine a class to manage an array of parameters
class ParameterManager {
public:
	static const int MAX_PARAMETERS = 20; // Maximum number of parameters
	Parameter parameters[MAX_PARAMETERS];
	int numParameters; // Actual number of parameters
	
	ParameterManager() {
		numParameters = 0;
	}

	void addParameter(const char paramName[16], float parameterValue, uint8_t parameterType) {
		if (numParameters < MAX_PARAMETERS) {
			parameters[numParameters] = Parameter(paramName, parameterValue, parameterType);
			numParameters++;
		}
		// Handle error if the array is full
	}

	// Read parameters from EEPROM and store them in the array
	void readParametersFromEEPROM() {
		// TODO: Implement EEPROM reading logic for Teensy/Arduino here
	}

	// Write the parameters to EEPROM
	void writeParametersToEEPROM() {
		// TODO: Implement EEPROM writing logic here for Teensy/Arduino
	}

	int find(const char paramName[16]) {
		// Find a parameter by name and return its index in the array. Will return -1 if not found.
		for (int i = 0; i < numParameters; i++) {
			if (strcmp(parameters[i].name, paramName) == 0) {
				return i;
			}
		}
		return -1; // Parameter not found
	}
	
private:
	const Parameter defaultParams[9] = { {"kp_roll", 0.2, MAV_PARAM_TYPE_REAL32},
                                       {"ki_roll", 0.3, MAV_PARAM_TYPE_REAL32},
                                       {"kd_roll", 0.3, MAV_PARAM_TYPE_REAL32},
                                       {"kp_pitch", 0.3, MAV_PARAM_TYPE_REAL32},
                                       {"ki_pitch", 0.3, MAV_PARAM_TYPE_REAL32},
                                       {"kd_pitch", 0.3, MAV_PARAM_TYPE_REAL32},
                                       {"kp_yaw", 0.3, MAV_PARAM_TYPE_REAL32},
                                       {"ki_yaw", 0.3, MAV_PARAM_TYPE_REAL32},
                                       {"kd_yaw", 0.3, MAV_PARAM_TYPE_REAL32} };
};

#endif //PARAM_H
