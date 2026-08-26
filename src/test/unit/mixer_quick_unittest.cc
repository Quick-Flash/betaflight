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

#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/mixer_quick.h"

#include "sensors/battery.h"

mixerConfig_t mixerConfig_System;
mixerConfig_t mixerConfig_Copy;
mixerRuntime_t mixerRuntime;

static uint16_t testSagCellVoltage;

uint16_t getBatterySagCellVoltage(void)
{
    return testSagCellVoltage;
}
}

#include "gtest/gtest.h"

static const motorMixer_t quadMixer[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },
    { 1.0f, -1.0f, -1.0f,  1.0f },
    { 1.0f,  1.0f,  1.0f,  1.0f },
    { 1.0f,  1.0f, -1.0f, -1.0f },
};

static void expectMotorsInRange(const float *motorMix, const motorMixer_t *mixer, unsigned motorCount, float throttle, float mixSign)
{
    for (unsigned i = 0; i < motorCount; i++) {
        const float motor = mixSign * motorMix[i] + throttle * mixer[i].throttle;
        EXPECT_GE(motor, 0.0f);
        EXPECT_LE(motor, 1.0f);
    }
}

TEST(mixerQuickTest, preservesUnsaturatedMix)
{
    float motorMix[] = { -0.1f, -0.1f, 0.1f, 0.1f };

    const float throttle = mixerQuickApply(motorMix, quadMixer, 4, 0.5f, 1.0f, 1.0f);

    EXPECT_FLOAT_EQ(0.5f, throttle);
    EXPECT_FLOAT_EQ(-0.1f, motorMix[0]);
    EXPECT_FLOAT_EQ(0.1f, motorMix[2]);
    expectMotorsInRange(motorMix, quadMixer, 4, throttle, 1.0f);
}

TEST(mixerQuickTest, normalizesSaturatedMix)
{
    float motorMix[] = { -1.0f, -1.0f, 1.0f, 1.0f };

    const float throttle = mixerQuickApply(motorMix, quadMixer, 4, 0.5f, 1.0f, 1.0f);

    EXPECT_FLOAT_EQ(0.5f, throttle);
    EXPECT_FLOAT_EQ(-0.5f, motorMix[0]);
    EXPECT_FLOAT_EQ(0.5f, motorMix[2]);
    expectMotorsInRange(motorMix, quadMixer, 4, throttle, 1.0f);
}

TEST(mixerQuickTest, appliesDifferentialAuthorityLimit)
{
    float motorMix[] = { -0.1f, -0.1f, 0.1f, 0.1f };

    const float throttle = mixerQuickApply(motorMix, quadMixer, 4, 0.5f, 0.8f, 1.0f);

    EXPECT_FLOAT_EQ(0.5f, throttle);
    EXPECT_NEAR(-0.08f, motorMix[0], 1e-6f);
    EXPECT_NEAR(0.08f, motorMix[2], 1e-6f);
}

TEST(mixerQuickTest, zeroAuthorityRemovesDifferential)
{
    float motorMix[] = { -0.5f, 0.25f, 0.1f, 0.4f };

    const float throttle = mixerQuickApply(motorMix, quadMixer, 4, 0.4f, 0.0f, 1.0f);

    EXPECT_FLOAT_EQ(0.4f, throttle);
    for (const float motor : motorMix) {
        EXPECT_FLOAT_EQ(0.0f, motor);
    }
}

TEST(mixerQuickTest, supportsReverseMixSign)
{
    float motorMix[] = { -1.0f, -1.0f, 1.0f, 1.0f };

    const float throttle = mixerQuickApply(motorMix, quadMixer, 4, 0.5f, 1.0f, -1.0f);

    expectMotorsInRange(motorMix, quadMixer, 4, throttle, -1.0f);
}

TEST(mixerQuickTest, supportsMoreThanFourMotors)
{
    const motorMixer_t hexMixer[] = {
        { 1.0f, 0.0f, 0.0f, 0.0f },
        { 1.0f, 0.0f, 0.0f, 0.0f },
        { 1.0f, 0.0f, 0.0f, 0.0f },
        { 1.0f, 0.0f, 0.0f, 0.0f },
        { 1.0f, 0.0f, 0.0f, 0.0f },
        { 1.0f, 0.0f, 0.0f, 0.0f },
    };
    float motorMix[] = { -0.6f, -0.3f, 0.0f, 0.2f, 0.5f, 0.8f };

    const float throttle = mixerQuickApply(motorMix, hexMixer, 6, 0.5f, 1.0f, 1.0f);

    expectMotorsInRange(motorMix, hexMixer, 6, throttle, 1.0f);
}

TEST(mixerQuickTest, calculatesVoltageScale)
{
    EXPECT_NEAR(3.5f / 4.2f, mixerQuickVoltageScale(4.2f, 3.5f, 1.0f), 1e-6f);
    EXPECT_NEAR(0.5f * (1.0f + 3.5f / 4.2f), mixerQuickVoltageScale(4.2f, 3.5f, 0.5f), 1e-6f);
    EXPECT_FLOAT_EQ(1.0f, mixerQuickVoltageScale(3.4f, 3.5f, 1.0f));
    EXPECT_FLOAT_EQ(1.0f, mixerQuickVoltageScale(0.0f, 3.5f, 1.0f));
}

TEST(mixerQuickTest, appliesSelectedVoltageMode)
{
    float throttle = 0.5f;
    float differentialScale = 0.75f;
    mixerQuickApplyVoltageCompensation(QUICK_VBAT_COMP_OFF, 0.8f, &throttle, &differentialScale);
    EXPECT_FLOAT_EQ(0.5f, throttle);
    EXPECT_FLOAT_EQ(0.75f, differentialScale);

    mixerQuickApplyVoltageCompensation(QUICK_VBAT_COMP_DIFFERENTIAL, 0.8f, &throttle, &differentialScale);
    EXPECT_FLOAT_EQ(0.5f, throttle);
    EXPECT_NEAR(0.6f, differentialScale, 1e-6f);

    throttle = 0.5f;
    differentialScale = 0.75f;
    mixerQuickApplyVoltageCompensation(QUICK_VBAT_COMP_FULL, 0.8f, &throttle, &differentialScale);
    EXPECT_NEAR(0.4f, throttle, 1e-6f);
    EXPECT_NEAR(0.6f, differentialScale, 1e-6f);
}

TEST(mixerQuickTest, voltageModesAffectTheExpectedMixerComponents)
{
    float differentialMix[] = { -0.1f, -0.1f, 0.1f, 0.1f };
    const float differentialThrottle = mixerQuickMixWithScale(
        differentialMix, quadMixer, 4, 0.25f, false, 1.0f,
        QUICK_VBAT_COMP_DIFFERENTIAL, 0.8f, 1.0f, 1.0f);
    EXPECT_FLOAT_EQ(0.25f, differentialThrottle);
    EXPECT_NEAR(-0.06f, differentialMix[0], 1e-6f);
    EXPECT_NEAR(0.06f, differentialMix[2], 1e-6f);

    float fullMix[] = { -0.1f, -0.1f, 0.1f, 0.1f };
    const float fullThrottle = mixerQuickMixWithScale(
        fullMix, quadMixer, 4, 0.25f, false, 1.0f,
        QUICK_VBAT_COMP_FULL, 0.8f, 1.0f, 1.0f);
    EXPECT_NEAR(0.2f, fullThrottle, 1e-6f);
    EXPECT_NEAR(-0.06f, fullMix[0], 1e-6f);
    EXPECT_NEAR(0.06f, fullMix[2], 1e-6f);
}

TEST(mixerQuickTest, reducesLowThrottleAuthorityWhenAirmodeIsDisabled)
{
    float airmodeMix[] = { -0.1f, -0.1f, 0.1f, 0.1f };
    mixerQuickMixWithScale(
        airmodeMix, quadMixer, 4, 0.0f, true, 1.0f,
        QUICK_VBAT_COMP_OFF, 1.0f, 1.0f, 1.0f);

    float noAirmodeMix[] = { -0.1f, -0.1f, 0.1f, 0.1f };
    mixerQuickMixWithScale(
        noAirmodeMix, quadMixer, 4, 0.0f, false, 1.0f,
        QUICK_VBAT_COMP_OFF, 1.0f, 1.0f, 1.0f);

    EXPECT_FLOAT_EQ(0.1f, airmodeMix[2]);
    EXPECT_FLOAT_EQ(0.05f, noAirmodeMix[2]);
}

TEST(mixerQuickTest, ownsTheConfiguredMixerPipeline)
{
    mixerConfig_System.quick_vbat_compensation = QUICK_VBAT_COMP_FULL;
    mixerRuntime.vbatSagCompensationFactor = 1.0f;
    mixerRuntime.vbatCompensationReference = 350.0f;
    testSagCellVoltage = 420;
    mixerQuickSetImpactAttenuation(0.5f);

    float motorMix[] = { -0.1f, -0.1f, 0.1f, 0.1f };
    const float throttle = mixerQuickMix(motorMix, quadMixer, 4, 0.5f, true, 1.0f, 1.0f);

    const float voltageScale = 350.0f / 420.0f;
    EXPECT_NEAR(0.5f * voltageScale, throttle, 1e-6f);
    EXPECT_NEAR(-0.05f * voltageScale, motorMix[0], 1e-6f);
    EXPECT_NEAR(0.05f * voltageScale, motorMix[2], 1e-6f);

    mixerQuickSetImpactAttenuation(1.0f);
}

TEST(mixerQuickTest, appliesSoftArmToPortempaNormalizedAuthority)
{
    float motorMix[] = { -0.8f, -0.8f, 0.8f, 0.8f };

    const float throttle = mixerQuickMixWithScale(
        motorMix, quadMixer, 4, 0.1f, true, 1.0f,
        QUICK_VBAT_COMP_OFF, 1.0f, 1.0f, 0.25f);

    EXPECT_FLOAT_EQ(0.125f, motorMix[2]);
    EXPECT_FLOAT_EQ(0.125f, throttle);
    expectMotorsInRange(motorMix, quadMixer, 4, throttle, 1.0f);
}

TEST(mixerQuickTest, matchesPortempaPitchVector)
{
    float motorMix[] = { 0.5f, -0.5f, 0.5f, -0.5f };

    const float throttle = mixerQuickApply(motorMix, quadMixer, 4, 0.0f, 1.0f, 1.0f);

    EXPECT_FLOAT_EQ(0.5f, throttle);
    const float expected[] = { 1.0f, 0.0f, 1.0f, 0.0f };
    for (unsigned i = 0; i < 4; i++) {
        EXPECT_FLOAT_EQ(expected[i], motorMix[i] + throttle);
    }
}

TEST(mixerQuickTest, matchesPortempaSaturatedRpyVector)
{
    float motorMix[] = { -1.0f, -1.0f, 3.0f, -1.0f };

    const float throttle = mixerQuickApply(motorMix, quadMixer, 4, 0.0f, 1.0f, 1.0f);

    EXPECT_FLOAT_EQ(0.25f, throttle);
    const float expected[] = { 0.0f, 0.0f, 1.0f, 0.0f };
    for (unsigned i = 0; i < 4; i++) {
        EXPECT_FLOAT_EQ(expected[i], motorMix[i] + throttle);
    }
}
