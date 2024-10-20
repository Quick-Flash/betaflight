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

#pragma once

#include "common/axis.h"
#include "common/filter.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"

#ifdef USE_DYN_NOTCH_FILTER
#include "flight/dyn_notch_filter.h"
#endif

#include "flight/pid.h"

#include "pg/pg.h"

#include <rust.h>

#define LPF_MAX_HZ 1500 // so little filtering above 1000hz that if the user wants less delay, they must disable the filter
#define DYN_LPF_MAX_HZ 1000

#ifdef USE_YAW_SPIN_RECOVERY
#define YAW_SPIN_RECOVERY_THRESHOLD_MIN 500
#define YAW_SPIN_RECOVERY_THRESHOLD_MAX 1950
#endif

#define GYRO_IMU_DOWNSAMPLE_CUTOFF_HZ 200

typedef union gyroLowpassFilter_u {
    Pt1Filter pt1FilterState;
    SecondOrderLowpassFilter biquadFilterState;
    Pt2Filter pt2FilterState;
    Pt3Filter pt3FilterState;
} gyroLowpassFilter_t;

typedef enum gyroDetectionFlags_e {
    GYRO_NONE_MASK = 0,
    GYRO_1_MASK = BIT(0),
#if defined(USE_MULTI_GYRO)
    GYRO_2_MASK = BIT(1),
    GYRO_ALL_MASK = (GYRO_1_MASK | GYRO_2_MASK),
    GYRO_IDENTICAL_MASK = BIT(7), // All gyros are of the same hardware type
#endif
} gyroDetectionFlags_t;

typedef struct gyroCalibration_s {
    float sum[XYZ_AXIS_COUNT];
    stdev_t var[XYZ_AXIS_COUNT];
    int32_t cyclesRemaining;
} gyroCalibration_t;

typedef struct gyroSensor_s {
    gyroDev_t gyroDev;
    gyroCalibration_t calibration;
} gyroSensor_t;

typedef struct gyro_s {
    uint16_t sampleRateHz;
    uint32_t targetLooptime;
    uint32_t sampleLooptime;
    float scale;
    float gyroADC[XYZ_AXIS_COUNT];     // aligned, calibrated, scaled, but unfiltered data from the sensor(s)
    float gyroADCf[XYZ_AXIS_COUNT];    // filtered gyro data
    uint8_t sampleCount;               // gyro sensor sample counter

    gyroSensor_t gyroSensor1;
#ifdef USE_MULTI_GYRO
    gyroSensor_t gyroSensor2;
    SensorFusion gyroFusion;
#endif

    gyroDev_t *rawSensorDev;           // pointer to the sensor providing the raw data for DEBUG_GYRO_RAW

    // non-linear median filter for DownSampling RUST
    NonLinearMedian non_linear_median[XYZ_AXIS_COUNT];

    // gyro filters just lowpass at the moment RUST
    GyroFilters gyro_filtering;
    GyroLowpassVariants gyro_lowpass_variants;

    uint16_t accSampleRateHz;
    uint8_t gyroToUse;
    uint8_t gyroDebugMode;
    bool useDualGyroDebugging;
    flight_dynamics_index_t gyroDebugAxis;

    Pt1Filter imuGyroFilter[XYZ_AXIS_COUNT];
} gyro_t;

extern gyro_t gyro;
extern uint8_t activePidLoopDenom;

enum {
    DYN_LPF_NONE = 0,
    DYN_LPF_PT1,
    DYN_LPF_BIQUAD,
    DYN_LPF_PT2,
    DYN_LPF_PT3,
};

typedef enum {
    YAW_SPIN_RECOVERY_OFF,
    YAW_SPIN_RECOVERY_ON,
    YAW_SPIN_RECOVERY_AUTO
} yawSpinRecoveryMode_e;

#define GYRO_CONFIG_USE_GYRO_1      0
#define GYRO_CONFIG_USE_GYRO_2      1
#define GYRO_CONFIG_USE_GYRO_BOTH   2

typedef struct gyroConfig_s {
    uint8_t gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t gyro_hardware_lpf;                // gyro DLPF setting
    uint8_t gyro_high_fsr;
    uint8_t gyro_to_use;

    uint16_t gyro_lpf1_cutoff;
    uint8_t gyro_lpf1_predictive_cutoff;
    uint16_t gyro_lpf1_cutoff_q;
    uint16_t gyro_lpf1_predictive_q;

    uint16_t gyro_lpf2_cutoff;
    uint8_t gyro_lpf2_predictive_cutoff;
    uint16_t gyro_lpf2_cutoff_q;
    uint16_t gyro_lpf2_predictive_q;

    // Lowpass primary/secondary
    uint8_t gyro_lpf1_variant;
    uint8_t gyro_lpf2_variant;

    int16_t gyro_offset_yaw;
    uint8_t yaw_spin_recovery;
    int16_t yaw_spin_threshold;

    uint16_t gyroCalibrationDuration;   // Gyro calibration duration in 1/100 second

    uint16_t gyro_lpf1_dyn_min_hz;
    uint16_t gyro_lpf1_dyn_max_hz;

    uint8_t gyro_filter_debug_axis;

    uint8_t gyrosDetected; // What gyros should detection be attempted for on startup. Automatically set on first startup.
#ifdef USE_MULTI_GYRO
    uint8_t gyro_noise_est_cut;
#endif
} gyroConfig_t;

PG_DECLARE(gyroConfig_t, gyroConfig);

void gyroUpdate(void);
void gyroFiltering(timeUs_t currentTimeUs);
float gyroGetFilteredDownsampled(int axis);
void gyroStartCalibration(bool isFirstArmingCalibration);
bool isFirstArmingGyroCalibrationRunning(void);
bool gyroIsCalibrationComplete(void);
void gyroReadTemperature(void);
int16_t gyroGetTemperature(void);
bool gyroYawSpinDetected(void);
uint16_t gyroAbsRateDps(int axis);
#ifdef USE_DYN_LPF
float dynThrottle(float throttle);
#endif
#ifdef USE_YAW_SPIN_RECOVERY
void initYawSpinRecovery(int maxYawRate);
#endif
