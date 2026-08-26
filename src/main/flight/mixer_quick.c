/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software and/or
 * modify this software under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#include <float.h>

#include "platform.h"

#include "common/maths.h"

#include "flight/mixer.h"
#include "flight/mixer_init.h"

#include "sensors/battery.h"

#include "mixer_quick.h"

static float impactAttenuation = 1.0f;

void mixerQuickSetImpactAttenuation(float attenuation)
{
    impactAttenuation = constrainf(attenuation, 0.0f, 1.0f);
}

static void mixerQuickThrottleRange(const float *motorMix, const motorMixer_t *activeMixer,
    unsigned motorCount, float mixSign, float *minimumThrottle, float *maximumThrottle)
{
    *minimumThrottle = -FLT_MAX;
    *maximumThrottle = FLT_MAX;

    for (unsigned i = 0; i < motorCount; i++) {
        const float throttleGain = activeMixer[i].throttle;
        if (throttleGain <= 0.0f) {
            continue;
        }

        const float differential = mixSign * motorMix[i];
        *minimumThrottle = MAX(*minimumThrottle, -differential / throttleGain);
        *maximumThrottle = MIN(*maximumThrottle, (1.0f - differential) / throttleGain);
    }
}

float mixerQuickApply(float *motorMix, const motorMixer_t *activeMixer, unsigned motorCount,
    float throttle, float differentialScale, float mixSign)
{
    differentialScale = constrainf(differentialScale, 0.0f, 1.0f);

    float minimumThrottle;
    float maximumThrottle;
    mixerQuickThrottleRange(motorMix, activeMixer, motorCount, mixSign, &minimumThrottle, &maximumThrottle);

    float normalizationScale = 1.0f;
    if (minimumThrottle > maximumThrottle) {
        float highestMotor = 1.0f;
        for (unsigned i = 0; i < motorCount; i++) {
            const float motor = mixSign * motorMix[i] + minimumThrottle * activeMixer[i].throttle;
            highestMotor = MAX(highestMotor, motor);
        }
        normalizationScale = 1.0f / highestMotor;
    }

    const float motorScale = differentialScale * normalizationScale;
    for (unsigned i = 0; i < motorCount; i++) {
        motorMix[i] *= motorScale;
    }

    mixerQuickThrottleRange(motorMix, activeMixer, motorCount, mixSign, &minimumThrottle, &maximumThrottle);
    throttle = constrainf(throttle, minimumThrottle, maximumThrottle);

    // Keep the normalized result safe for unusual custom throttle coefficients.
    for (unsigned i = 0; i < motorCount; i++) {
        const float throttleContribution = throttle * activeMixer[i].throttle;
        const float constrainedMotor = constrainf(mixSign * motorMix[i] + throttleContribution, 0.0f, 1.0f);
        motorMix[i] = mixSign * (constrainedMotor - throttleContribution);
    }

    return throttle;
}

float mixerQuickVoltageScale(float measuredVoltage, float referenceVoltage, float compensationStrength)
{
    if (measuredVoltage <= 0.0f || referenceVoltage <= 0.0f || measuredVoltage <= referenceVoltage) {
        return 1.0f;
    }

    const float fullCompensationScale = referenceVoltage / measuredVoltage;
    return constrainf(1.0f - compensationStrength * (1.0f - fullCompensationScale), 0.0f, 1.0f);
}

void mixerQuickApplyVoltageCompensation(quickVbatCompensationMode_e mode, float voltageScale,
    float *throttle, float *differentialScale)
{
    if (mode == QUICK_VBAT_COMP_OFF) {
        return;
    }

    *differentialScale *= voltageScale;
    if (mode == QUICK_VBAT_COMP_FULL) {
        *throttle *= voltageScale;
    }
}

float mixerQuickMixWithScale(float *motorMix, const motorMixer_t *activeMixer, unsigned motorCount,
    float throttle, bool airmodeEnabled, float mixSign, quickVbatCompensationMode_e voltageMode,
    float voltageScale, float collisionScale, float softArmScale)
{
    const float authorityThrottle = throttle;
    float differentialScale = collisionScale * softArmScale;
    mixerQuickApplyVoltageCompensation(voltageMode, voltageScale, &throttle, &differentialScale);

    if (!airmodeEnabled && authorityThrottle < 0.5f) {
        differentialScale *= 0.5f + authorityThrottle;
    }

    return mixerQuickApply(motorMix, activeMixer, motorCount, throttle, differentialScale, mixSign);
}

float mixerQuickMix(float *motorMix, const motorMixer_t *activeMixer, unsigned motorCount,
    float throttle, bool airmodeEnabled, float mixSign, float softArmScale)
{
    float voltageScale = 1.0f;
#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
    if (mixerConfig()->quick_vbat_compensation != QUICK_VBAT_COMP_OFF && mixerRuntime.vbatSagCompensationFactor > 0.0f) {
        voltageScale = mixerQuickVoltageScale(
            getBatterySagCellVoltage(),
            mixerRuntime.vbatCompensationReference,
            mixerRuntime.vbatSagCompensationFactor);
    }
#endif

    return mixerQuickMixWithScale(
        motorMix, activeMixer, motorCount, throttle, airmodeEnabled, mixSign,
        mixerConfig()->quick_vbat_compensation, voltageScale, impactAttenuation, softArmScale);
}
