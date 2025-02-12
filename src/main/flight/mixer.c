/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <math.h>
#include <float.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/alt_hold.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer_init.h"
#include "flight/mixer_tricopter.h"
#include "flight/pid.h"
#include "flight/rpm_filter.h"

#include "io/gps.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "mixer.h"

#define DYN_LPF_THROTTLE_STEPS             100
#define DYN_LPF_THROTTLE_UPDATE_DELAY_US  5000 // minimum of 5ms between updates

#define CRASHFLIP_MOTOR_DEADBAND         0.02f // 2%; send 'disarm' value to motors below this drive value
#define CRASHFLIP_STICK_DEADBAND         0.15f // 15%

static FAST_DATA_ZERO_INIT float motorMixRange;

float FAST_DATA_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

static FAST_DATA_ZERO_INIT int throttleAngleCorrection;

float getMotorMixRange(void)
{
    return motorMixRange;
}

void writeMotors(void)
{
    motorWriteAll(motor);
}

static void writeAllMotors(int16_t mc)
{
    // Sends commands to all motors
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motor[i] = mc;
    }
    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(mixerRuntime.disarmMotorOutput);
    delay(50); // give the timers and ESCs a chance to react.
}

static FAST_DATA_ZERO_INIT float throttle = 0;
static FAST_DATA_ZERO_INIT float rcThrottle = 0;
static FAST_DATA_ZERO_INIT float mixerThrottle = 0;
static FAST_DATA_ZERO_INIT float motorOutputMin;
static FAST_DATA_ZERO_INIT float motorRangeMin;
static FAST_DATA_ZERO_INIT float motorRangeMax;
static FAST_DATA_ZERO_INIT float motorOutputRange;
static FAST_DATA_ZERO_INIT int8_t motorOutputMixSign;
static FAST_DATA_ZERO_INIT bool crashflipSuccess = false;

static void calculateThrottleAndCurrentMotorEndpoints(timeUs_t currentTimeUs)
{
    static uint16_t rcThrottlePrevious = 0;   // Store the last throttle direction for deadband transitions
    static timeUs_t reversalTimeUs = 0; // time when motors last reversed in 3D mode
    static float motorRangeMinIncrease = 0;

    float currentThrottleInputRange = 0;
    if (mixerRuntime.feature3dEnabled) {
        uint16_t rcCommand3dDeadBandLow;
        uint16_t rcCommand3dDeadBandHigh;

        if (!ARMING_FLAG(ARMED)) {
            rcThrottlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.
        }

        if (IS_RC_MODE_ACTIVE(BOX3D) || flight3DConfig()->switched_mode3d) {
            // The min_check range is halved because the output throttle is scaled to 500us.
            // So by using half of min_check we maintain the same low-throttle deadband
            // stick travel as normal non-3D mode.
            const int mincheckOffset = (rxConfig()->mincheck - PWM_RANGE_MIN) / 2;
            rcCommand3dDeadBandLow = rxConfig()->midrc - mincheckOffset;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + mincheckOffset;
        } else {
            rcCommand3dDeadBandLow = rxConfig()->midrc - flight3DConfig()->deadband3d_throttle;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + flight3DConfig()->deadband3d_throttle;
        }

        const float rcCommandThrottleRange3dLow = rcCommand3dDeadBandLow - PWM_RANGE_MIN;
        const float rcCommandThrottleRange3dHigh = PWM_RANGE_MAX - rcCommand3dDeadBandHigh;

        if (rcCommand[THROTTLE] <= rcCommand3dDeadBandLow || isCrashFlipModeActive()) {
            // INVERTED
            motorRangeMin = mixerRuntime.motorOutputLow;
            motorRangeMax = mixerRuntime.deadbandMotor3dLow;
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutputMin = mixerRuntime.motorOutputLow;
                motorOutputRange = mixerRuntime.deadbandMotor3dLow - mixerRuntime.motorOutputLow;
            } else
#endif
            {
                motorOutputMin = mixerRuntime.deadbandMotor3dLow;
                motorOutputRange = mixerRuntime.motorOutputLow - mixerRuntime.deadbandMotor3dLow;
            }

            if (motorOutputMixSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = -1;

            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand3dDeadBandLow - rcCommand[THROTTLE];
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else if (rcCommand[THROTTLE] >= rcCommand3dDeadBandHigh) {
            // NORMAL
            motorRangeMin = mixerRuntime.deadbandMotor3dHigh;
            motorRangeMax = mixerRuntime.motorOutputHigh;
            motorOutputMin = mixerRuntime.deadbandMotor3dHigh;
            motorOutputRange = mixerRuntime.motorOutputHigh - mixerRuntime.deadbandMotor3dHigh;
            if (motorOutputMixSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = 1;
            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand[THROTTLE] - rcCommand3dDeadBandHigh;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        } else if ((rcThrottlePrevious <= rcCommand3dDeadBandLow &&
                !flight3DConfigMutable()->switched_mode3d) ||
                isMotorsReversed()) {
            // INVERTED_TO_DEADBAND
            motorRangeMin = mixerRuntime.motorOutputLow;
            motorRangeMax = mixerRuntime.deadbandMotor3dLow;

#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutputMin = mixerRuntime.motorOutputLow;
                motorOutputRange = mixerRuntime.deadbandMotor3dLow - mixerRuntime.motorOutputLow;
            } else
#endif
            {
                motorOutputMin = mixerRuntime.deadbandMotor3dLow;
                motorOutputRange = mixerRuntime.motorOutputLow - mixerRuntime.deadbandMotor3dLow;
            }

            if (motorOutputMixSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = -1;

            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else {
            // NORMAL_TO_DEADBAND
            motorRangeMin = mixerRuntime.deadbandMotor3dHigh;
            motorRangeMax = mixerRuntime.motorOutputHigh;
            motorOutputMin = mixerRuntime.deadbandMotor3dHigh;
            motorOutputRange = mixerRuntime.motorOutputHigh - mixerRuntime.deadbandMotor3dHigh;
            if (motorOutputMixSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = 1;
            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        }
        if (currentTimeUs - reversalTimeUs < 250000) {
            // keep iterm zero for 250ms after motor reversal
            pidResetIterm();
        }

        throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
        rcThrottle = throttle;
    } else {
        throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN + throttleAngleCorrection;
        currentThrottleInputRange = PWM_RANGE;
#ifdef USE_DYN_IDLE
        if (mixerRuntime.dynIdleMinRps > 0.0f) {
            const float maxIncrease = isAirmodeActivated()
                ? mixerRuntime.dynIdleMaxIncrease : mixerRuntime.dynIdleStartIncrease;
            float minRps = getMinMotorFrequencyHz();
            DEBUG_SET(DEBUG_DYN_IDLE, 3, lrintf(minRps * 10.0f));
            float rpsError = mixerRuntime.dynIdleMinRps - minRps;
            // PT1 type lowpass delay and smoothing for D
            minRps = mixerRuntime.prevMinRps + mixerRuntime.minRpsDelayK * (minRps - mixerRuntime.prevMinRps);
            float dynIdleD = (mixerRuntime.prevMinRps - minRps) * mixerRuntime.dynIdleDGain;
            mixerRuntime.prevMinRps = minRps;
            float dynIdleP = rpsError * mixerRuntime.dynIdlePGain;
            rpsError = MAX(-0.1f, rpsError); //I rises fast, falls slowly
            mixerRuntime.dynIdleI += rpsError * mixerRuntime.dynIdleIGain;
            mixerRuntime.dynIdleI = constrainf(mixerRuntime.dynIdleI, 0.0f, maxIncrease);
            motorRangeMinIncrease = constrainf((dynIdleP + mixerRuntime.dynIdleI + dynIdleD), 0.0f, maxIncrease);
            DEBUG_SET(DEBUG_DYN_IDLE, 0, MAX(-1000, lrintf(dynIdleP * 10000)));
            DEBUG_SET(DEBUG_DYN_IDLE, 1, lrintf(mixerRuntime.dynIdleI * 10000));
            DEBUG_SET(DEBUG_DYN_IDLE, 2, lrintf(dynIdleD * 10000));
        } else {
            motorRangeMinIncrease = 0;
        }
#endif
        throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
        rcThrottle = throttle;

        float soft_arm_percent_inv = getSoftArmPercentInv();
        float motor_output_low = soft_arm_percent_inv * mixerRuntime.motorOutputLow + (1.0f - soft_arm_percent_inv) * mixerRuntime.motorOutputSoftLow; // go between soft arm output low and normal idle

        motorRangeMax = mixerRuntime.motorOutputHigh;
        motorRangeMin = motor_output_low + motorRangeMinIncrease * (mixerRuntime.motorOutputHigh - motor_output_low);
        motorOutputMin = motorRangeMin;
        motorOutputRange = motorRangeMax - motorRangeMin;
        motorOutputMixSign = 1;

        DEBUG_SET(DEBUG_SOFT_ARM, 0, lrintf(100 - soft_arm_percent_inv * 1000));
        DEBUG_SET(DEBUG_SOFT_ARM, 1, lrintf(soft_arm_percent_inv * 1000));
        DEBUG_SET(DEBUG_SOFT_ARM, 2, lrintf(motorRangeMin * 1000));
        DEBUG_SET(DEBUG_SOFT_ARM, 3, lrintf(rcThrottle * 1000));
    }
}

static bool applyCrashFlipModeToMotors(void)
{
#ifdef USE_ACC
    static bool isTiltAngleAtStartSet = false;
    static float tiltAngleAtStart = 1.0f;
#endif

    if (!isCrashFlipModeActive()) {
#ifdef USE_ACC
        // trigger the capture of initial tilt angle on next activation of crashflip mode
        isTiltAngleAtStartSet = false;
        // default the success flag to false, to block quick re-arming unless successful
#endif
        // signal that crashflip mode is off
        return false;
    }

    const float stickDeflectionPitchAbs = getRcDeflectionAbs(FD_PITCH);
    const float stickDeflectionRollAbs = getRcDeflectionAbs(FD_ROLL);
    const float stickDeflectionYawAbs = getRcDeflectionAbs(FD_YAW);

    float signPitch = getRcDeflection(FD_PITCH) < 0 ? 1 : -1;
    float signRoll = getRcDeflection(FD_ROLL) < 0 ? 1 : -1;
    float signYaw = (getRcDeflection(FD_YAW) < 0 ? 1 : -1) * (mixerConfig()->yaw_motors_reversed ? 1 : -1);

    float stickDeflectionLength = sqrtf(sq(stickDeflectionPitchAbs) + sq(stickDeflectionRollAbs));

    if (stickDeflectionYawAbs > MAX(stickDeflectionPitchAbs, stickDeflectionRollAbs)) {
        // If yaw is the dominant, disable pitch and roll
        stickDeflectionLength = stickDeflectionYawAbs;
        signRoll = 0;
        signPitch = 0;
    } else {
        // If pitch/roll dominant, disable yaw
        signYaw = 0;
    }

    const float cosPhi = (stickDeflectionLength > 0) ? (stickDeflectionPitchAbs + stickDeflectionRollAbs) / (sqrtf(2.0f) * stickDeflectionLength) : 0;
    const float cosThreshold = sqrtf(3.0f) / 2.0f; // cos(30 deg)

    if (cosPhi < cosThreshold) {
        // Enforce either roll or pitch exclusively, if not on diagonal
        if (stickDeflectionRollAbs > stickDeflectionPitchAbs) {
            signPitch = 0;
        } else {
            signRoll = 0;
        }
    }

    // Calculate crashflipPower from stick deflection with a reasonable amount of stick deadband
    float crashflipPower = stickDeflectionLength > CRASHFLIP_STICK_DEADBAND ? stickDeflectionLength : 0.0f;

    // calculate flipPower attenuators
    float crashflipRateAttenuator = 1.0f;
    float crashflipAttitudeAttenuator = 1.0f;
    const float crashflipRateLimit = mixerConfig()->crashflip_rate * 10.0f; // eg 35 = no power by 350 deg/s
    const float halfComplete = 0.5f; // attitude or rate changes less that this will be ignored

    // disable both attenuators if the user's crashflip_rate is zero
    if (crashflipRateLimit > 0) {
#ifdef USE_ACC
        // Calculate an attenuator based on change of attitude (requires Acc)
        // with Acc, crashflipAttitudeAttenuator will be zero after approx 90 degree rotation, and
        // motors will stop / not spin while attitude remains more than ~90 degrees from initial attitude
        // without Acc, the user must manually center the stick, or exit crashflip mode, or disarm, to stop the motors
        // re-initialisation of crashFlip mode by arm/disarm is required to reset the initial tilt angle
        if (sensors(SENSOR_ACC)) {
            const float tiltAngle = getCosTiltAngle();  // -1 if flat inverted, 0 when 90° (on edge), +1 when flat and upright
            if (!isTiltAngleAtStartSet) {
                tiltAngleAtStart = tiltAngle;
                isTiltAngleAtStartSet = true;
                crashflipSuccess = false;
            }
            // attitudeChangeNeeded is 1.0 at the start, decreasing to 0 when attitude change exceeds approx 90 degrees
            const float attitudeChangeNeeded = fmaxf(1.0f - fabsf(tiltAngle - tiltAngleAtStart), 0.0f);
            // no attenuation unless a significant attitude change has occurred
            crashflipAttitudeAttenuator = attitudeChangeNeeded > halfComplete ? 1.0f : attitudeChangeNeeded / halfComplete;

            // signal success to enable quick restart, if attitude change implies success when reverting the switch
            crashflipSuccess = attitudeChangeNeeded == 0.0f;
        }
#endif // USE_ACC
        // Calculate an attenuation factor based on rate of rotation... note:
        // if driving roll or pitch, quad usually turns on that axis, but if one motor sticks, could be a diagonal rotation
        // if driving diagonally, the turn could be either roll or pitch
        // if driving yaw, typically one motor sticks, and the quad yaws a little then flips diagonally
        const float gyroRate = fmaxf(fabsf(gyro.gyroADCf[FD_ROLL]), fabsf(gyro.gyroADCf[FD_PITCH]));
        const float gyroRateChange = fminf(gyroRate / crashflipRateLimit, 1.0f);
        // no attenuation unless a significant gyro rate change has occurred
        crashflipRateAttenuator = gyroRateChange < halfComplete ? 1.0f : (1.0f - gyroRateChange) / halfComplete;

        crashflipPower *= crashflipAttitudeAttenuator * crashflipRateAttenuator;
    }

    for (int i = 0; i < mixerRuntime.motorCount; ++i) {
        float motorOutputNormalised =
            signPitch * mixerRuntime.currentMixer[i].pitch +
            signRoll * mixerRuntime.currentMixer[i].roll +
            signYaw * mixerRuntime.currentMixer[i].yaw;

        if (motorOutputNormalised < 0) {
            if (mixerConfig()->crashflip_motor_percent > 0) {
                motorOutputNormalised = -motorOutputNormalised * (float)mixerConfig()->crashflip_motor_percent / 100.0f;
            } else {
                motorOutputNormalised = 0;
            }
        }

        motorOutputNormalised = MIN(1.0f, crashflipPower * motorOutputNormalised);
        float motorOutput = motorOutputMin + motorOutputNormalised * motorOutputRange;

        // set motors to disarm value when intended increase is less than deadband value
        motorOutput = (motorOutputNormalised < CRASHFLIP_MOTOR_DEADBAND) ? mixerRuntime.disarmMotorOutput : motorOutput;
        
        motor[i] = motorOutput;
    }

    // signal that crashflip mode has been applied to motors
    return true;
}

#ifdef USE_RPM_LIMIT
#define STICK_HIGH_DEADBAND 5    // deadband to make sure throttle cap can raise, even with maxcheck set around 2000
static void applyRpmLimiter(mixerRuntime_t *mixer)
{
    static float prevError = 0.0f;
    const float unsmoothedAverageRpm = getDshotRpmAverage();
    const float averageRpm = pt1FilterApply(&mixer->rpmLimiterAverageRpmFilter, unsmoothedAverageRpm);
    const float error = averageRpm - mixer->rpmLimiterRpmLimit;

    // PID
    const float p = error * mixer->rpmLimiterPGain;
    const float d = (error - prevError) * mixer->rpmLimiterDGain; // rpmLimiterDGain already adjusted for looprate (see mixer_init.c)
    mixer->rpmLimiterI += error * mixer->rpmLimiterIGain;         // rpmLimiterIGain already adjusted for looprate (see mixer_init.c)
    mixer->rpmLimiterI = MAX(0.0f, mixer->rpmLimiterI);
    float pidOutput = p + mixer->rpmLimiterI + d;

    // Throttle limit learning
    if (error > 0.0f && rcCommand[THROTTLE] < rxConfig()->maxcheck) {
        mixer->rpmLimiterThrottleScale *= 1.0f - 4.8f * pidGetDT();
    } else if (pidOutput < -400.0f * pidGetDT() && lrintf(rcCommand[THROTTLE]) >= rxConfig()->maxcheck - STICK_HIGH_DEADBAND && !areMotorsSaturated()) { // Throttle accel corresponds with motor accel
        mixer->rpmLimiterThrottleScale *= 1.0f + 3.2f * pidGetDT();
    }
    mixer->rpmLimiterThrottleScale = constrainf(mixer->rpmLimiterThrottleScale, 0.01f, 1.0f);

    float rpmLimiterThrottleScaleOffset = pt1FilterApply(&mixer->rpmLimiterThrottleScaleOffsetFilter, constrainf(mixer->rpmLimiterRpmLimit / motorEstimateMaxRpm(), 0.0f, 1.0f) - mixer->rpmLimiterInitialThrottleScale);
    throttle *= constrainf(mixer->rpmLimiterThrottleScale + rpmLimiterThrottleScaleOffset, 0.0f, 1.0f);

    // Output
    pidOutput = MAX(0.0f, pidOutput);
    throttle = constrainf(throttle - pidOutput, 0.0f, 1.0f);
    prevError = error;

    DEBUG_SET(DEBUG_RPM_LIMIT, 0, lrintf(averageRpm));
    DEBUG_SET(DEBUG_RPM_LIMIT, 1, lrintf(rpmLimiterThrottleScaleOffset * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 2, lrintf(mixer->rpmLimiterThrottleScale * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 3, lrintf(throttle * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 4, lrintf(error));
    DEBUG_SET(DEBUG_RPM_LIMIT, 5, lrintf(p * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 6, lrintf(mixer->rpmLimiterI * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 7, lrintf(d * 100.0f));
}
#endif // USE_RPM_LIMIT

static void applyMixToMotors(RateControls rate_controls, float throttle_final)
{
    float motor_values [4];
    float collision_motor_delta = getCollisionMotorDelta();
    float mixer_thrust = mix_motors(&mixerRuntime.motor_mixer, &motor_values, &rate_controls, throttle_final, getSoftArmPercentInv() * collision_motor_delta, getBatterySagCellVoltage());
    float cg_learning_k = update_cg_compensation(&mixerRuntime.motor_mixer, pidData[FD_ROLL].I, pidData[FD_PITCH].I, mixer_thrust);

    // remove iterm as we learn it in cg compensation this will desaturate iterm
    pidData[FD_ROLL].I -= pidData[FD_ROLL].I * cg_learning_k;
    pidData[FD_PITCH].I -= pidData[FD_PITCH].I * cg_learning_k;

    float min_motor = 10000.0;
    float max_motor = -10000.0;

    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float motorOutput = motor_values[i];
        if (motorOutput > max_motor) {
            max_motor = motorOutput;
        }
        if (motorOutput < min_motor) {
            min_motor = motorOutput;
        }

        motorOutput = motorOutputMin + motorOutputRange * motorOutput;
        DEBUG_SET(DEBUG_SOFT_ARM, 4 + i, lrintf(motorOutput * 1000));

#ifdef USE_SERVOS
        if (mixerIsTricopter()) {
            motorOutput += mixerTricopterMotorCorrection(i);
        }
#endif
        if (failsafeIsActive()) {
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < motorRangeMin) ? mixerRuntime.disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
#endif
            motorOutput = constrainf(motorOutput, mixerRuntime.disarmMotorOutput, motorRangeMax);
        } else {
            motorOutput = constrainf(motorOutput, motorRangeMin, motorRangeMax);
        }
        motor[i] = motorOutput;
    }

    motorMixRange = max_motor - min_motor + 0.05;

    // Disarmed mode
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

static float applyThrottleLimit(float throttle)
{
    if (currentControlRateProfile->throttle_limit_percent < 100 && !RPM_LIMIT_ACTIVE) {
        const float throttleLimitFactor = currentControlRateProfile->throttle_limit_percent / 100.0f;
        switch (currentControlRateProfile->throttle_limit_type) {
            case THROTTLE_LIMIT_TYPE_SCALE:
                return throttle * throttleLimitFactor;
            case THROTTLE_LIMIT_TYPE_CLIP:
                return MIN(throttle, throttleLimitFactor);
        }
    }

    return throttle;
}

static void applyMotorStop(void)
{
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motor[i] = mixerRuntime.disarmMotorOutput;
    }
}

#ifdef USE_DYN_LPF
static void updateDynLpfCutoffs(timeUs_t currentTimeUs, float throttle)
{
    static timeUs_t lastDynLpfUpdateUs = 0;
    static int dynLpfPreviousQuantizedThrottle = -1;  // to allow an initial zero throttle to set the filter cutoff

    if (cmpTimeUs(currentTimeUs, lastDynLpfUpdateUs) >= DYN_LPF_THROTTLE_UPDATE_DELAY_US) {
        const int quantizedThrottle = lrintf(throttle * DYN_LPF_THROTTLE_STEPS); // quantize the throttle reduce the number of filter updates
        if (quantizedThrottle != dynLpfPreviousQuantizedThrottle) {
            // scale the quantized value back to the throttle range so the filter cutoff steps are repeatable
            const float dynLpfThrottle = (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
            dynLpfDTermUpdate(dynLpfThrottle);
            dynLpfPreviousQuantizedThrottle = quantizedThrottle;
            lastDynLpfUpdateUs = currentTimeUs;
        }
    }
}
#endif

FAST_CODE_NOINLINE void mixTable(timeUs_t currentTimeUs)
{
    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);

    if (applyCrashFlipModeToMotors()) {
        return; // if crash flip mode has been applied to the motors, mixing is done
    }

    const bool launchControlActive = isLaunchControlActive();

#ifdef USE_LAUNCH_CONTROL
    if (launchControlActive && (currentPidProfile->launchControlMode == LAUNCH_CONTROL_MODE_PITCHONLY)) {
        activeMixer = &mixerRuntime.launchControlMixer[0];
    }
#endif

    // Calculate and Limit the PID sum
    const float scaledAxisPidRoll =
        constrainf(pidData[FD_ROLL].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    const float scaledAxisPidPitch =
        constrainf(pidData[FD_PITCH].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;

    uint16_t yawPidSumLimit = currentPidProfile->pidSumLimitYaw;

#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinDetected = gyroYawSpinDetected();
    if (yawSpinDetected) {
        yawPidSumLimit = PIDSUM_LIMIT_MAX;   // Set to the maximum limit during yaw spin recovery to prevent limiting motor authority
    }
#endif // USE_YAW_SPIN_RECOVERY

    float scaledAxisPidYaw =
        constrainf(pidData[FD_YAW].Sum, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;

    if (!mixerConfig()->yaw_motors_reversed) {
        scaledAxisPidYaw = -scaledAxisPidYaw;
    }

    RateControls rate_controls = {
        .pidsum_roll = scaledAxisPidRoll,
        .pidsum_pitch = scaledAxisPidPitch,
        .pidsum_yaw = scaledAxisPidYaw,
    };

    // Apply the throttle_limit_percent to scale or limit the throttle based on throttle_limit_type
    if (currentControlRateProfile->throttle_limit_type != THROTTLE_LIMIT_TYPE_OFF) {
        throttle = applyThrottleLimit(throttle);
    }

    // use scaled throttle, without dynamic idle throttle offset, as the input to antigravity
    pidUpdateAntiGravityThrottleFilter(throttle);

    // and for TPA
    pidUpdateTpaFactor(throttle);

#ifdef USE_DYN_LPF
    // keep the changes to dynamic lowpass clean, without unnecessary dynamic changes
    updateDynLpfCutoffs(currentTimeUs, throttle);
#endif

    // apply throttle boost when throttle moves quickly
#if defined(USE_THROTTLE_BOOST)
    if (throttleBoost > 0.0f) {
        const float throttleHpf = throttle - pt1FilterApply(&throttleLpf, throttle);
        throttle = constrainf(throttle + throttleBoost * throttleHpf, 0.0f, 1.0f);
    }
#endif

    // send throttle value to blackbox, including scaling and throttle boost, but not TL compensation, dyn idle or airmode
    mixerThrottle = throttle;

#ifdef USE_DYN_IDLE
    // Set min throttle offset of 1% when stick is at zero and dynamic idle is active
    if (mixerRuntime.dynIdleMinRps > 0.0f) {
        throttle = MAX(throttle, 0.01f);
    }
#endif

#ifdef USE_RPM_LIMIT
    if (RPM_LIMIT_ACTIVE && useDshotTelemetry && ARMING_FLAG(ARMED)) {
        applyRpmLimiter(&mixerRuntime);
    }
#endif

    //  The following fixed throttle values will not be shown in the blackbox log
    // ?? Should they be influenced by airmode?  If not, should go after the apply airmode code.
    const bool airmodeEnabled = airmodeIsEnabled() || launchControlActive;
#ifdef USE_YAW_SPIN_RECOVERY
    // 50% throttle provides the maximum authority for yaw recovery when airmode is not active.
    // When airmode is active the throttle setting doesn't impact recovery authority.
    if (yawSpinDetected && !airmodeEnabled) {
        throttle = 0.5f;
    }
#endif // USE_YAW_SPIN_RECOVERY

#ifdef USE_LAUNCH_CONTROL
    // While launch control is active keep the throttle at minimum.
    // Once the pilot triggers the launch throttle control will be reactivated.
    if (launchControlActive) {
        throttle = 0.0f;
    }
#endif

#ifdef USE_ALT_HOLD_MODE
    // Throttle value to be used during altitude hold mode (and failsafe landing mode)
    if (FLIGHT_MODE(ALT_HOLD_MODE)) {
        throttle = altHoldGetThrottle();
    }
#endif

#ifdef USE_GPS_RESCUE
    // If gps rescue is active then override the throttle. This prevents things
    // like throttle boost or throttle limit from negatively affecting the throttle.
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        throttle = gpsRescueGetThrottle();
    }
#endif

    if (featureIsEnabled(FEATURE_MOTOR_STOP)
        && ARMING_FLAG(ARMED)
        && !mixerRuntime.feature3dEnabled
        && !airmodeEnabled
        && !FLIGHT_MODE(GPS_RESCUE_MODE | ALT_HOLD_MODE)   // disable motor_stop while GPS Rescue / Altitude Hold is active
        && (rcData[THROTTLE] < rxConfig()->mincheck)) {
        // motor_stop handling
        applyMotorStop();
    } else {
        // Apply the mix to motor endpoints
        applyMixToMotors(rate_controls, throttle);
    }
}

void mixerSetThrottleAngleCorrection(int correctionValue)
{
    throttleAngleCorrection = correctionValue;
}

float mixerGetThrottle(void)
{
    return mixerThrottle;
}

float mixerGetRcThrottle(void)
{
    return rcThrottle;
}

bool crashFlipSuccessful(void)
{
    return crashflipSuccess;
}
