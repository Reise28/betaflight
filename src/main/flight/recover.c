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
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "flight/recover.h"
#include "flight/imu.h"
#include "rx/rx.h"
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
/*
 * RECOVER V0.6 throttle controller.
 *
 * When strongly inverted, collective thrust is deliberately low because
 * normal thrust points toward the ground.  As the thrust axis becomes useful
 * for supporting the craft, collective throttle is progressively increased.
 *
 * Vertical descent braking is enabled mainly near upright attitude.
 */
#define RECOVER_THROTTLE_MIN_PWM          1100.0f
#define RECOVER_THROTTLE_INVERTED_PWM     1150.0f
#define RECOVER_THROTTLE_TUMBLE_PWM       1250.0f
#define RECOVER_THROTTLE_LEVEL_PWM        1350.0f
#define RECOVER_THROTTLE_MAX_PWM          1750.0f

#define RECOVER_GYRO_BOOST_START_DPS        60.0f
#define RECOVER_GYRO_BOOST_FULL_DPS        600.0f
#define RECOVER_GYRO_BOOST_PWM             100.0f

#define RECOVER_DESCENT_START_CMS           80.0f
#define RECOVER_DESCENT_FULL_CMS           700.0f
#define RECOVER_DESCENT_BOOST_PWM          350.0f

/* cos(30 deg) and cos(70 deg) */
#define RECOVER_COS_TILT_30                  0.8660254f
#define RECOVER_COS_TILT_70                  0.3420201f

/* Pilot catches throttle around 40% +/- 10%. */
#define RECOVER_THROTTLE_CATCH_LOW        1300
#define RECOVER_THROTTLE_CATCH_HIGH       1500
#define RECOVER_THROTTLE_BLEND_US       400000


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
static float recoverThrottleCommand = RECOVER_THROTTLE_INVERTED_PWM;
static bool recoverThrottleHandoffPending;
static timeUs_t recoverThrottleBlendStartUs;
static float recoverThrottleBlendStartCommand;

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
    if (recoverActive) {
        /*
         * getCosTiltAngle():
         *
         *   +1 = perfectly upright
         *    0 = thrust axis horizontal
         *   -1 = completely inverted
         */
        const float cosTilt = getCosTiltAngle();

        /*
         * At or beyond 90 degrees, normal collective thrust cannot support
         * the craft vertically.  Between 90 and 70 degrees, gradually allow
         * enough collective power to improve rotational authority.
         */
        const float thrustUsefulBlend = constrainf(
            cosTilt / RECOVER_COS_TILT_70,
            0.0f,
            1.0f);

        /*
         * 0 at 70 degrees or worse,
         * 1 at 30 degrees or better.
         */
        const float uprightBlend = constrainf(
            (cosTilt - RECOVER_COS_TILT_70) /
            (RECOVER_COS_TILT_30 - RECOVER_COS_TILT_70),
            0.0f,
            1.0f);

        /*
         * More angular velocity permits a small collective boost so the
         * mixer has motor authority to arrest a violent tumble.
         */
        const float gyroBlend = constrainf(
            (gyroDps - RECOVER_GYRO_BOOST_START_DPS) /
            (RECOVER_GYRO_BOOST_FULL_DPS - RECOVER_GYRO_BOOST_START_DPS),
            0.0f,
            1.0f);

        /*
         * baroVelocityCms is positive upward, negative downward.
         */
        const float descentCms = fmaxf(0.0f, -baroVelocityCms);

        const float descentBlend = constrainf(
            (descentCms - RECOVER_DESCENT_START_CMS) /
            (RECOVER_DESCENT_FULL_CMS - RECOVER_DESCENT_START_CMS),
            0.0f,
            1.0f);

        /*
         * Base collective:
         *
         * inverted/sideways -> approximately 1150
         * useful thrust     -> approximately 1250
         * near level        -> approximately 1350
         */
        const float baseThrottle =
            RECOVER_THROTTLE_INVERTED_PWM +
            thrustUsefulBlend *
                (RECOVER_THROTTLE_TUMBLE_PWM -
                 RECOVER_THROTTLE_INVERTED_PWM) +
            uprightBlend *
                (RECOVER_THROTTLE_LEVEL_PWM -
                 RECOVER_THROTTLE_TUMBLE_PWM);

        /*
         * Gyro boost matters mainly while attitude recovery is still needed.
         */
        const float gyroBoost =
            (1.0f - uprightBlend) *
            gyroBlend *
            RECOVER_GYRO_BOOST_PWM;

        /*
         * Descent braking becomes strong only when thrust points sufficiently
         * upward.
         */
        const float descentBoost =
            uprightBlend *
            descentBlend *
            RECOVER_DESCENT_BOOST_PWM;

        recoverThrottleCommand = constrainf(
            baseThrottle + gyroBoost + descentBoost,
            RECOVER_THROTTLE_MIN_PWM,
            RECOVER_THROTTLE_MAX_PWM);
    }
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

void recoverApplyThrottle(void)
{
    const bool recoverActive = IS_RC_MODE_ACTIVE(BOXRECOVER);

    /*
     * While RECOVER is active the physical throttle stick has no motor
     * authority.
     */
    if (recoverActive) {
        recoverThrottleHandoffPending = true;
        recoverThrottleBlendStartUs = 0;
        rcCommand[THROTTLE] = recoverThrottleCommand;
        return;
    }

    /*
     * RECOVER was released.
     */
    if (!recoverThrottleHandoffPending) {
        return;
    }

    /*
     * If release of RECOVER caused a disarm, no throttle handoff is needed.
     */
    if (!ARMING_FLAG(ARMED)) {
        recoverThrottleHandoffPending = false;
        recoverThrottleBlendStartUs = 0;
        return;
    }

    /*
     * updateRcCommands() has already calculated the normal pilot throttle.
     * Keep it aside while we decide whether control may be returned.
     */
    const float pilotThrottleCommand = rcCommand[THROTTLE];

    /*
     * Do not reconnect pilot throttle until the physical stick deliberately
     * enters the 30-50 percent catch window.
     */
    if (recoverThrottleBlendStartUs == 0) {
        if (rcData[THROTTLE] < RECOVER_THROTTLE_CATCH_LOW ||
            rcData[THROTTLE] > RECOVER_THROTTLE_CATCH_HIGH) {

            rcCommand[THROTTLE] = recoverThrottleCommand;
            return;
        }

        recoverThrottleBlendStartCommand = recoverThrottleCommand;
        recoverThrottleBlendStartUs = micros();
    }

    /*
     * Smooth throttle handoff over 400 ms.
     */
    const float blend = constrainf(
        (float)cmpTimeUs(micros(), recoverThrottleBlendStartUs) /
        (float)RECOVER_THROTTLE_BLEND_US,
        0.0f,
        1.0f);

    rcCommand[THROTTLE] =
        recoverThrottleBlendStartCommand +
        blend *
        (pilotThrottleCommand - recoverThrottleBlendStartCommand);

    if (blend >= 1.0f) {
        recoverThrottleHandoffPending = false;
        recoverThrottleBlendStartUs = 0;
    }
}
