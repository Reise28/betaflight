/* Experimental RECOVER detector for Betaflight 4.5.5. */
#pragma once

#include <stdbool.h>

void recoverUpdate(void);
bool recoverEmergencyArmEligible(void);
bool recoverEmergencyArmRequested(void);
void recoverApplyThrottle(void);
