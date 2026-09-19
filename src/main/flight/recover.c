/*
 * Experimental RECOVER detector.
 *
 * IMPORTANT: this first build does NOT arm motors and does NOT alter flight
 * controls.  It only maintains a rolling pre-button flight-state estimate and
 * exposes it through DEBUG_RECOVER.  This lets us tune the detector before
 * granting RECOVER any motor authority.
 */

#include <math.h>

#include "platform.h"

#include "build/debug.h"
#include "common/maths.h"
#include "drivers/time.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/recover.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#define RECOVER_LOW_G_THRESHOLD        0.55f
#define RECOVER_LOW_G_HOLD_US          120000
#define RECOVER_HISTORY_US             300000
#define RECOVER_GYRO_MOVING_DPS        60.0f
#define RECOVER_BARO_FALL_CMS          80.0f
#define RECOVER_BARO_LPF_ALPHA         0.12f

static timeUs_t lowGStartUs;
static timeUs_t lastConfirmedLowGUs;
static timeUs_t lastMotionUs;
static bool lowGTiming;
static bool haveConfirmedLowG;
static bool haveMotion;
static bool emergencyArmEligible;
static bool emergencyArmRequested;
static float previousBaroCm;
static timeUs_t previousBaroUs;
static float baroVelocityCms;

static float vectorMagnitude3(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

void recoverUpdate(void)
{
    const timeUs_t now = micros();

    float accG = 1.0f;
    if (sensors(SENSOR_ACC) && acc.dev.acc_1G > 0) {
        accG = vectorMagnitude3(acc.accADC[X], acc.accADC[Y], acc.accADC[Z]) * acc.dev.acc_1G_rec;
    }

    const float gyroDps = vectorMagnitude3(gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z]);

    const bool lowG = accG < RECOVER_LOW_G_THRESHOLD;
    const bool moving = gyroDps > RECOVER_GYRO_MOVING_DPS;

    bool lowGConfirmed = false;
    if (lowG) {
        if (!lowGTiming) {
            lowGStartUs = now;
            lowGTiming = true;
        }

        if (cmpTimeUs(now, lowGStartUs) >= RECOVER_LOW_G_HOLD_US) {
            lowGConfirmed = true;
            lastConfirmedLowGUs = now;
            haveConfirmedLowG = true;
        }
    } else {
        lowGTiming = false;
    }

    if (moving) {
        lastMotionUs = now;
        haveMotion = true;
    }

#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        const float baroCm = getBaroAltitude();
        if (previousBaroUs != 0) {
            const timeDelta_t dtUs = cmpTimeUs(now, previousBaroUs);
            if (dtUs > 0 && dtUs < 200000) {
                const float rawVelocity = (baroCm - previousBaroCm) * 1000000.0f / dtUs;
                baroVelocityCms += RECOVER_BARO_LPF_ALPHA * (rawVelocity - baroVelocityCms);
            }
        }
        previousBaroCm = baroCm;
        previousBaroUs = now;
    }
#endif

    const bool recentLowG = haveConfirmedLowG && cmpTimeUs(now, lastConfirmedLowGUs) <= RECOVER_HISTORY_US;
    const bool recentMotion = haveMotion && cmpTimeUs(now, lastMotionUs) <= RECOVER_HISTORY_US;
    const bool baroFalling = baroVelocityCms < -RECOVER_BARO_FALL_CMS;

    // Continuously confirmed low-G is the primary evidence for a tossed/falling disarmed craft.
    // Motion or barometric descent adds confidence.  We intentionally keep
    // the decision based on samples accumulated BEFORE/while the button edge
    // arrives; RECOVER never waits 300 ms after the pilot presses it.
    emergencyArmEligible = recentLowG && (recentMotion || baroFalling || lowGConfirmed);

    const bool recoverActive = IS_RC_MODE_ACTIVE(BOXRECOVER);
    emergencyArmRequested = recoverActive && emergencyArmEligible;

    int flags = 0;
    flags |= lowG ? 1 : 0;
    flags |= recentLowG ? 2 : 0;
    flags |= recentMotion ? 4 : 0;
    flags |= baroFalling ? 8 : 0;
    flags |= recoverActive ? 16 : 0;
    flags |= emergencyArmEligible ? 32 : 0;
    flags |= emergencyArmRequested ? 64 : 0;

    DEBUG_SET(DEBUG_RECOVER, 0, lrintf(accG * 1000.0f));       // milli-g
    DEBUG_SET(DEBUG_RECOVER, 1, lrintf(gyroDps));              // deg/s
    DEBUG_SET(DEBUG_RECOVER, 2, lrintf(baroVelocityCms));      // cm/s
    DEBUG_SET(DEBUG_RECOVER, 3, flags);                        // bit field above
}

bool recoverEmergencyArmEligible(void)
{
    return emergencyArmEligible;
}

bool recoverEmergencyArmRequested(void)
{
    return emergencyArmRequested;
}
