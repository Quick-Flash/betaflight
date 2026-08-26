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

#pragma once

#include "flight/mixer.h"

float mixerQuickMix(float *motorMix, const motorMixer_t *activeMixer, unsigned motorCount,
    float throttle, bool airmodeEnabled, float mixSign, float softArmScale);
void mixerQuickSetImpactAttenuation(float attenuation);

float mixerQuickMixWithScale(float *motorMix, const motorMixer_t *activeMixer, unsigned motorCount,
    float throttle, bool airmodeEnabled, float mixSign, quickVbatCompensationMode_e voltageMode,
    float voltageScale, float collisionScale, float softArmScale);
float mixerQuickApply(float *motorMix, const motorMixer_t *activeMixer, unsigned motorCount,
    float throttle, float differentialScale, float mixSign);
float mixerQuickVoltageScale(float measuredVoltage, float referenceVoltage, float compensationStrength);
void mixerQuickApplyVoltageCompensation(quickVbatCompensationMode_e mode, float voltageScale,
    float *throttle, float *differentialScale);
