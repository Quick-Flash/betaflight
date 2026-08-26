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

#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "flight/mixer.h"

#include "impact_attenuation.h"

static pt1Filter_t accelerationLowpass;
static bool lowpassInitialized;

void impactAttenuationInit(uint16_t sampleRateHz)
{
    const float dT = sampleRateHz > 0 ? 1.0f / sampleRateHz : 0.0f;
    pt1FilterInit(&accelerationLowpass, pt1FilterGain(mixerConfig()->quick_impact_cutoff, dT));
    lowpassInitialized = false;
}

float impactAttenuationUpdate(float accelerationMagnitude, float maximumSetpoint, bool active)
{
    if (!lowpassInitialized) {
        accelerationLowpass.state = accelerationMagnitude;
        lowpassInitialized = true;
    }

    const float filteredAcceleration = pt1FilterApply(&accelerationLowpass, accelerationMagnitude);
    const float highpassAcceleration = fabsf(accelerationMagnitude - filteredAcceleration);
    float authority = 1.0f;

    const float accelerationThreshold = mixerConfig()->quick_impact_acc_threshold * 0.01f;
    const float setpointThreshold = mixerConfig()->quick_impact_setpoint_threshold;
    if (active && maximumSetpoint < setpointThreshold
        && (accelerationMagnitude > accelerationThreshold || filteredAcceleration > accelerationThreshold)) {
        const float highpassStart = mixerConfig()->quick_impact_highpass_start * 0.01f;
        const float highpassEnd = mixerConfig()->quick_impact_highpass_end * 0.01f;
        if (highpassEnd > highpassStart) {
            authority = constrainf(1.0f - (highpassAcceleration - highpassStart) / (highpassEnd - highpassStart), 0.0f, 1.0f);
        }
    }

    DEBUG_SET(DEBUG_IMPACT_ATTENUATION, 0, lrintf(accelerationMagnitude * 1000.0f));
    DEBUG_SET(DEBUG_IMPACT_ATTENUATION, 1, lrintf(filteredAcceleration * 1000.0f));
    DEBUG_SET(DEBUG_IMPACT_ATTENUATION, 2, lrintf(highpassAcceleration * 1000.0f));
    DEBUG_SET(DEBUG_IMPACT_ATTENUATION, 3, lrintf(maximumSetpoint));
    DEBUG_SET(DEBUG_IMPACT_ATTENUATION, 4, lrintf(authority * 1000.0f));
    DEBUG_SET(DEBUG_IMPACT_ATTENUATION, 5, active);

    return authority;
}
