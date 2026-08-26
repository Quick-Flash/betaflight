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

extern "C" {
#include "platform.h"

#include "flight/impact_attenuation.h"
#include "flight/mixer.h"

mixerConfig_t mixerConfig_System;
mixerConfig_t mixerConfig_Copy;
}

#include "gtest/gtest.h"

static void configureImpactAttenuation(void)
{
    mixerConfig_System.quick_impact_cutoff = 5;
    mixerConfig_System.quick_impact_highpass_start = 100;
    mixerConfig_System.quick_impact_highpass_end = 250;
    mixerConfig_System.quick_impact_acc_threshold = 200;
    mixerConfig_System.quick_impact_setpoint_threshold = 300;
    impactAttenuationInit(1000);

    for (unsigned i = 0; i < 1000; i++) {
        impactAttenuationUpdate(1.0f, 0.0f, false);
    }
}

TEST(impactAttenuationTest, attenuatesAnImpactAfterSettlingAtOneG)
{
    configureImpactAttenuation();

    const float authority = impactAttenuationUpdate(3.0f, 0.0f, true);

    EXPECT_GT(authority, 0.0f);
    EXPECT_LT(authority, 1.0f);
}

TEST(impactAttenuationTest, ignoresHighFrequencyNoiseBelowTheAccelerationGate)
{
    configureImpactAttenuation();

    for (unsigned i = 0; i < 1000; i++) {
        const float acceleration = (i & 1) ? 1.05f : 0.95f;
        EXPECT_FLOAT_EQ(1.0f, impactAttenuationUpdate(acceleration, 0.0f, true));
    }
}

TEST(impactAttenuationTest, ignoresIntentionalHighRateCommands)
{
    configureImpactAttenuation();

    EXPECT_FLOAT_EQ(1.0f, impactAttenuationUpdate(3.0f, 300.0f, true));
}

TEST(impactAttenuationTest, primesTheBaselineFromTheFirstSample)
{
    mixerConfig_System.quick_impact_cutoff = 5;
    mixerConfig_System.quick_impact_highpass_start = 100;
    mixerConfig_System.quick_impact_highpass_end = 250;
    mixerConfig_System.quick_impact_acc_threshold = 200;
    mixerConfig_System.quick_impact_setpoint_threshold = 300;
    impactAttenuationInit(1000);

    EXPECT_FLOAT_EQ(1.0f, impactAttenuationUpdate(3.0f, 0.0f, true));
}
