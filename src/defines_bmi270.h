/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2023 SlimeVR Contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/
#ifndef BMI270_DEFINES_H
#define BMI270_DEFINES_H

// BMI270 gyro/accel axis remapping.
#define BMI270_REMAP_AXIS_X(x, y, z) (sensorId == 0 ? x : x)
#define BMI270_REMAP_AXIS_Y(x, y, z) (sensorId == 0 ? y : y)
#define BMI270_REMAP_AXIS_Z(x, y, z) (sensorId == 0 ? z : z)
// Additional magnetometer remapping (if present and enabled in <debug.h>).
#define BMI270_MAG_REMAP_AXIS_X(x, y, z) (sensorId == 0 ? x : x)
#define BMI270_MAG_REMAP_AXIS_Y(x, y, z) (sensorId == 0 ? y : y)
#define BMI270_MAG_REMAP_AXIS_Z(x, y, z) (sensorId == 0 ? z : z)

// BMI270 magnetometer type, applies to both main and aux trackers, mixed types are not supported currently.
// If only 1 out of 2 trackers has a mag, tracker without a mag should still function normally.
// NOT USED if USE_6_AXIS == true
// Pick one:
#define BMI270_MAG_TYPE BMI270_MAG_TYPE_HMC
// #define BMI270_MAG_TYPE BMI270_MAG_TYPE_QMC

// Use VQF instead of mahony sensor fusion.
// Features: rest bias estimation, magnetic distortion rejection.
#define BMI270_USE_VQF true

// Use BasicVQF instead of VQF (if BMI270_USE_VQF == true).
// Disables the features above.
#define BMI270_USE_BASIC_VQF false

// Use temperature calibration.
#define BMI270_USE_TEMPCAL true

// post-process X axis for better gyroscope CAS performance 
#define BMI270_APPLY_ZX_CROSS_AXIS_FACTOR true

// How long to run gyro calibration for.
// Disables this calibration step if value is 0.
// Default: 5
#define BMI270_CALIBRATION_GYRO_SECONDS 5

// Calibration method options:
// - Skip: disable this calibration step;
// - Rotation: rotate the device in hand;
// - 6 point: put the device in 6 unique orientations.
// Default: ACCEL_CALIBRATION_METHOD_6POINT
// #define BMI270_ACCEL_CALIBRATION_METHOD ACCEL_CALIBRATION_METHOD_SKIP
// #define BMI270_ACCEL_CALIBRATION_METHOD ACCEL_CALIBRATION_METHOD_ROTATION
#define BMI270_ACCEL_CALIBRATION_METHOD ACCEL_CALIBRATION_METHOD_6POINT

// How long to run magnetometer calibration for, if enabled and you have added a magnetometer.
// Magnetometer not be used until you calibrate it.
// Disables this calibration step if value is 0.
// NOT USED if USE_6_AXIS == true
// Default: 20
#define BMI270_CALIBRATION_MAG_SECONDS 20

// Send temperature to the server as AXXYY,
// where XX is calibration progress from 0 to 60, and YY is temperature,
// A is 1: not in calibration mode or 2: calibration in progress.
#define BMI270_TEMPCAL_DEBUG false

// Print debug info every second.
#define BMI270_DEBUG true

// Use sensitivity calibration.
#define BMI270_USE_SENSCAL true

#endif