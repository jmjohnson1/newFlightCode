#include "flightRunner.h"

// Just calls the functions required to execute the program
int main() {
	Setup();
	for(;;) {
		Loop();
	}
	return 0;
}
