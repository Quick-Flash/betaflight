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

#include "platform.h"

static inline FAST_CODE void GYRO_FILTER_FUNCTION_NAME(void)
{
    float gyroADCf[XYZ_AXIS_COUNT];

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // DEBUG_GYRO_RAW records the raw value read from the sensor (not zero offset, not scaled)
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_RAW, axis, gyro.rawSensorDev->gyroADCRaw[axis]);

        // DEBUG_GYRO_SAMPLE(0) Record the pre-downsample value for the selected debug axis (same as DEBUG_GYRO_SCALED)
        GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 0, lrintf(gyro.gyroADC[axis]));

        // downsample the individual gyro samples
        if (gyro.downsampleFilterEnabled) {
            // using gyro lowpass 2 filter for downsampling
            gyroADCf[axis] = gyro.sampleSum[axis];
        } else {
            // using simple average for downsampling
            if (gyro.sampleCount) {
                gyroADCf[axis] = gyro.sampleSum[axis] / gyro.sampleCount;
            }
            gyro.sampleSum[axis] = 0;
        }

        // DEBUG_GYRO_SAMPLE(1) Record the post-downsample value for the selected debug axis
        GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 1, lrintf(gyroADCf[axis]));

#ifdef USE_RPM_FILTER
        gyroADCf[axis] = rpmFilterApply(axis, gyroADCf[axis]);
#endif

        // DEBUG_GYRO_SAMPLE(2) Record the post-RPM Filter value for the selected debug axis
        GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 2, lrintf(gyroADCf[axis]));
    }

    // apply static notch filters and software lowpass filters
    if (gyro.applyNotchFilter1) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            gyroADCf[axis] = biquadFilterApply(&gyro.notchFilter1[axis], gyroADCf[axis]);
        }
    }
    if (gyro.applyNotchFilter2) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            gyroADCf[axis] = biquadFilterApply(&gyro.notchFilter2[axis], gyroADCf[axis]);
        }
    }

    switch (gyro.lowpassFilterType) {
    case FILTER_PT1:
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            gyroADCf[axis] = pt1FilterApply(&gyro.lowpassFilter[axis].pt1FilterState, gyro.gyroADC[axis]);
        }
        break;
    case FILTER_BIQUAD:
#ifdef USE_DYN_LPF
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            gyroADCf[axis] = biquadFilterApplyDF1(&gyro.lowpassFilter[axis].biquadFilterState, gyro.gyroADC[axis]);
        }
#else
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            gyroADCf[axis] = biquadFilterApply(&gyro.lowpassFilter[axis].biquadFilterState, gyro.gyroADC[axis]);
        }
#endif
        break;
    case FILTER_PT2:
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            gyroADCf[axis] = pt2FilterApply(&gyro.lowpassFilter[axis].pt2FilterState, gyro.gyroADC[axis]);
        }
        break;
    case FILTER_PT3:
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            gyroADCf[axis] = pt3FilterApply(&gyro.lowpassFilter[axis].pt3FilterState, gyro.gyroADC[axis]);
        }
        break;
    case FILTER_NONE:
    default:
        // No filtering
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // DEBUG_GYRO_SAMPLE(3) Record the post-static notch and lowpass filter value for the selected debug axis
        GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 3, lrintf(gyroADCf[axis]));

#ifdef USE_DYN_NOTCH_FILTER
        if (isDynNotchActive()) {
            if (axis == gyro.gyroDebugAxis) {
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT, 0, lrintf(gyroADCf[axis]));
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT_FREQ, 0, lrintf(gyroADCf[axis]));
                GYRO_FILTER_DEBUG_SET(DEBUG_DYN_LPF, 0, lrintf(gyroADCf[axis]));
            }

            dynNotchPush(axis, gyroADCf[axis]);
            gyroADCf[axis] = dynNotchFilter(axis, gyroADCf[axis]);

            if (axis == gyro.gyroDebugAxis) {
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT, 1, lrintf(gyroADCf[axis]));
                GYRO_FILTER_DEBUG_SET(DEBUG_DYN_LPF, 3, lrintf(gyroADCf[axis]));
            }
        }
#endif

        // DEBUG_GYRO_FILTERED records the scaled, filtered, after all software filtering has been applied.
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_FILTERED, axis, lrintf(gyroADCf[axis]));

        gyro.gyroADCf[axis] = gyroADCf[axis];
    }
    gyro.sampleCount = 0;
}
