/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 SlimeVR Contributors

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

#ifndef SENSORS_BMI270SENSOR_H
#define SENSORS_BMI270SENSOR_H

#include "sensor.h"
#include "mahony.h"
#include "magneto1.4.h"

#include <BMI270.h>
#include <vqf.h>
#include <basicvqf.h>
#include "../motionprocessing/types.h"

#include "../motionprocessing/GyroTemperatureCalibrator.h"
#include "../motionprocessing/RestDetection.h"

#if BMI270_USE_VQF
    #if USE_6_AXIS
        #define BMI270_GYRO_RATE BMI270_GYRO_RATE_400HZ
    #else
        #define BMI270_GYRO_RATE BMI270_GYRO_RATE_200HZ
    #endif
#else
    #if USE_6_AXIS
        #define BMI270_GYRO_RATE BMI270_GYRO_RATE_800HZ
    #else
        #define BMI270_GYRO_RATE BMI270_GYRO_RATE_400HZ
    #endif
#endif
#define BMI270_GYRO_RANGE BMI270_GYRO_RANGE_1000
#define BMI270_GYRO_FILTER_MODE BMI270_DLPF_MODE_NORM

#define BMI270_ACCEL_RATE BMI270_ACCEL_RATE_100HZ
#define BMI270_ACCEL_RANGE BMI270_ACCEL_RANGE_16G
#define BMI270_ACCEL_FILTER_MODE BMI270_DLPF_MODE_NORM

// note: if changing ODR or filter modes - adjust rest detection params and buffer size

#define BMI270_TIMESTAMP_RESOLUTION_MICROS 39.0625f
// #define BMI270_TIMESTAMP_RESOLUTION_MICROS 39.0f
#define BMI270_MAP_ODR_MICROS(micros) ((uint16_t)((micros) / BMI270_TIMESTAMP_RESOLUTION_MICROS) * BMI270_TIMESTAMP_RESOLUTION_MICROS)
constexpr float BMI270_ODR_GYR_HZ = 25.0f * (1 << (BMI270_GYRO_RATE - 6));
constexpr float BMI270_ODR_ACC_HZ = 12.5f * (1 << (BMI270_ACCEL_RATE - 5));
constexpr float BMI270_ODR_GYR_MICROS = BMI270_MAP_ODR_MICROS(1.0f / BMI270_ODR_GYR_HZ * 1e6f);
constexpr float BMI270_ODR_ACC_MICROS = BMI270_MAP_ODR_MICROS(1.0f / BMI270_ODR_ACC_HZ * 1e6f);
#if !USE_6_AXIS
// note: this value only sets polling and fusion update rate - HMC is internally sampled at 75hz, QMC at 200hz
#define BMI270_MAG_RATE BMI270_MAG_RATE_50HZ
constexpr float BMI270_ODR_MAG_HZ = (25.0f/32.0f) * (1 << (BMI270_MAG_RATE - 1));
constexpr float BMI270_ODR_MAG_MICROS = BMI270_MAP_ODR_MICROS(1.0f / BMI270_ODR_MAG_HZ * 1e6f);
#else
constexpr float BMI270_ODR_MAG_HZ = 0;
constexpr float BMI270_ODR_MAG_MICROS = 0;
#endif

constexpr uint16_t BMI270_SETTINGS_MAX_ODR_HZ = max(max(BMI270_ODR_GYR_HZ, BMI270_ODR_ACC_HZ), BMI270_ODR_MAG_HZ);
constexpr uint16_t BMI270_SETTINGS_MAX_ODR_MICROS = BMI270_MAP_ODR_MICROS(1.0f / BMI270_SETTINGS_MAX_ODR_HZ * 1e6f);

constexpr float BMI270_FIFO_AVG_DATA_FRAME_LENGTH = (
    BMI270_SETTINGS_MAX_ODR_HZ * 1 +
    BMI270_ODR_GYR_HZ * BMI270_FIFO_G_LEN +
    BMI270_ODR_ACC_HZ * BMI270_FIFO_A_LEN +
    BMI270_ODR_MAG_HZ * BMI270_FIFO_M_LEN
) / BMI270_SETTINGS_MAX_ODR_HZ;
constexpr float BMI270_FIFO_READ_BUFFER_SIZE_MICROS = 30000;
constexpr float BMI270_FIFO_READ_BUFFER_SIZE_SAMPLES =
    BMI270_SETTINGS_MAX_ODR_HZ * BMI270_FIFO_READ_BUFFER_SIZE_MICROS / 1e6f;
constexpr uint16_t BMI270_FIFO_MAX_LENGTH = 1024;
constexpr uint16_t BMI270_FIFO_READ_BUFFER_SIZE_BYTES = min(
    (float)BMI270_FIFO_MAX_LENGTH - 64,
    BMI270_FIFO_READ_BUFFER_SIZE_SAMPLES * BMI270_FIFO_AVG_DATA_FRAME_LENGTH * 1.25f
);

// Typical sensitivity at 25C
// See p. 9 of https://www.mouser.com/datasheet/2/783/BST-BMI270-DS000-1509569.pdf
// #define BMI270_GYRO_TYPICAL_SENSITIVITY_LSB 16.4f  // 2000 deg  0
// #define BMI270_GYRO_TYPICAL_SENSITIVITY_LSB 32.8f  // 1000 deg  1
// #define BMI270_GYRO_TYPICAL_SENSITIVITY_LSB 65.6f  // 500 deg   2
// #define BMI270_GYRO_TYPICAL_SENSITIVITY_LSB 131.2f // 250 deg   3 
// #define BMI270_GYRO_TYPICAL_SENSITIVITY_LSB 262.4f // 125 deg   4
constexpr double BMI270_GYRO_TYPICAL_SENSITIVITY_LSB = (16.4f * (1 << BMI270_GYRO_RANGE));

constexpr std::pair<uint8_t, float> BMI270_ACCEL_SENSITIVITY_LSB_MAP[] = {
    {BMI270_ACCEL_RANGE_2G, 16384.0f},
    {BMI270_ACCEL_RANGE_4G, 8192.0f},
    {BMI270_ACCEL_RANGE_8G, 4096.0f},
    {BMI270_ACCEL_RANGE_16G, 2048.0f}
};
constexpr double BMI270_ACCEL_TYPICAL_SENSITIVITY_LSB = BMI270_ACCEL_SENSITIVITY_LSB_MAP[BMI270_ACCEL_RANGE].second;
constexpr double BMI270_ASCALE = CONST_EARTH_GRAVITY / BMI270_ACCEL_TYPICAL_SENSITIVITY_LSB;

// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr double BMI270_GSCALE = ((32768. / BMI270_GYRO_TYPICAL_SENSITIVITY_LSB) / 32768.) * (PI / 180.0);

constexpr uint32_t BMI270_TEMP_CALIBRATION_REQUIRED_SAMPLES_PER_STEP =
    TEMP_CALIBRATION_SECONDS_PER_STEP / (BMI270_ODR_GYR_MICROS / 1e6);
static_assert(0x7FFF * BMI270_TEMP_CALIBRATION_REQUIRED_SAMPLES_PER_STEP < 0x7FFFFFFF, "Temperature calibration sum overflow");

#define BMI270_VQF_REST_DETECTION_AVAILABLE (BMI270_USE_VQF && !BMI270_USE_BASIC_VQF)

#if BMI270_USE_VQF
#if !BMI270_USE_BASIC_VQF
struct BMI270VQFParams: VQFParams {
    BMI270VQFParams() : VQFParams() {
        #ifndef VQF_NO_MOTION_BIAS_ESTIMATION
        motionBiasEstEnabled = false;
        #endif
        tauAcc = 2.0f;
        restMinT = 2.0f;
        restThGyr = 0.6f; // 400 norm
        restThAcc = 0.06f; // 100 norm
    }
};
#endif
#endif

struct BMI270RestDetectionParams: RestDetectionParams {
    BMI270RestDetectionParams() : RestDetectionParams() {
        restMinTimeMicros = 2.0f * 1e6;
        restThGyr = 0.6f; // 400 norm
        restThAcc = 0.06f; // 100 norm
    }
};

class BMI270Sensor : public Sensor {
    public:
        BMI270Sensor(uint8_t id, uint8_t address, float rotation) :
            Sensor("BMI270Sensor", IMU_BMI270, id, address, rotation)
#if !BMI270_VQF_REST_DETECTION_AVAILABLE
            , restDetection(restDetectionParams, BMI270_ODR_GYR_MICROS / 1e6f, BMI270_ODR_ACC_MICROS / 1e6f)
#endif

#if BMI270_USE_VQF
#if !BMI270_USE_BASIC_VQF
            , vqf(vqfParams, BMI270_ODR_GYR_MICROS / 1e6f, BMI270_ODR_ACC_MICROS / 1e6f, BMI270_ODR_MAG_MICROS / 1e6f)
#else
            , vqf(BMI270_ODR_GYR_MICROS / 1e6f, BMI270_ODR_ACC_MICROS / 1e6f, BMI270_ODR_MAG_MICROS / 1e6f)
#endif
#endif
        {
        };
        ~BMI270Sensor(){};
        void initHMC(BMI270MagRate magRate);
        void initQMC(BMI270MagRate magRate);

        void motionSetup() override final;
        void motionLoop() override final;
        void startCalibration(int calibrationType) override final;
        void maybeCalibrateGyro();
        void maybeCalibrateAccel();
        void maybeCalibrateMag();
        
        void printTemperatureCalibrationState() override final;
        void printDebugTemperatureCalibrationState() override final;
        void resetTemperatureCalibrationState() override final {
            gyroTempCalibrator->reset();
            m_Logger.info("Temperature calibration state has been reset for sensorId:%i", sensorId);
        };
        void saveTemperatureCalibration() override final;

        void applyAccelCalibrationAndScale(sensor_real_t Axyz[3]);
        void applyMagCalibrationAndScale(sensor_real_t Mxyz[3]);

        bool hasGyroOffsetCalibration();
        bool hasGyroSensCalibration();
        bool hasAccelCalibration();
        bool hasMagCalibration();

        void onGyroRawSample(uint32_t dtMicros, int16_t x, int16_t y, int16_t z);
        void onAccelRawSample(uint32_t dtMicros, int16_t x, int16_t y, int16_t z);
        void onMagRawSample(uint32_t dtMicros, int16_t x, int16_t y, int16_t z);
        void readFIFO();

        void getMagnetometerXYZFromBuffer(uint8_t* data, int16_t* x, int16_t* y, int16_t* z);

        void remapGyroAccel(sensor_real_t* x, sensor_real_t* y, sensor_real_t* z);
        void remapMagnetometer(sensor_real_t* x, sensor_real_t* y, sensor_real_t* z);
        void getRemappedRotation(int16_t* x, int16_t* y, int16_t* z);
        void getRemappedAcceleration(int16_t* x, int16_t* y, int16_t* z);

        bool getTemperature(float* out);
    private:
        BMI270 imu {};

        Mahony<sensor_real_t> mahony;
#if !BMI270_VQF_REST_DETECTION_AVAILABLE
        BMI270RestDetectionParams restDetectionParams {};
        RestDetection restDetection;
#endif
#if BMI270_USE_VQF
#if !BMI270_USE_BASIC_VQF
        BMI270VQFParams vqfParams {};
        VQF vqf;
#else
        BasicVQF vqf;
#endif
#endif

        sensor_real_t qwxyz[4] {1.0f, 0.0f, 0.0f, 0.0f};
        Quaternion quat{};

        // clock sync and sample timestamping
        uint32_t sensorTime0 = 0;
        uint32_t sensorTime1 = 0;
        uint32_t localTime0 = 0;
        uint32_t localTime1 = 0;
        double sensorTimeRatio = 1;
        double sensorTimeRatioEma = 1;
        double sampleDtMicros = BMI270_ODR_GYR_MICROS;
        uint32_t syncLatencyMicros = 0;
        uint32_t samplesSinceClockSync = 0;
        uint32_t timestamp0 = 0;
        uint32_t timestamp1 = 0;
        int8_t zx_cross_factor = 0;

        // scheduling
        uint32_t lastPollTime = micros();
        uint32_t lastClockPollTime = micros();
        #if BMI270_DEBUG
        uint32_t cpuUsageMicros = 0;
        uint32_t lastCpuUsagePrinted = 0;
        uint32_t gyrReads = 0;
        uint32_t accReads = 0;
        uint32_t magReads = 0;
        uint16_t numFIFODropped = 0;
        uint16_t numFIFOFailedReads = 0;
        #endif

        uint32_t lastRotationPacketSent = 0;
        uint32_t lastTemperaturePacketSent = 0;

        struct BMI270FIFO {
            uint8_t data[BMI270_FIFO_READ_BUFFER_SIZE_BYTES];
            uint16_t length;
        } fifo {};
        float temperature = 0;
        GyroTemperatureCalibrator* gyroTempCalibrator = nullptr;
        sensor_real_t Gxyz[3] = {0};
        sensor_real_t Axyz[3] = {0};
        sensor_real_t Mxyz[3] = {0};
        sensor_real_t lastAxyz[3] = {0};
        bool fusionUpdated = false;

        double gscaleX = BMI270_GSCALE;
        double gscaleY = BMI270_GSCALE;
        double gscaleZ = BMI270_GSCALE;

        double GOxyzStaticTempCompensated[3];

        bool isGyroOffsetCalibrated = false;
        bool isGyroSensCalibrated = false;
        bool isAccelCalibrated = false;
        bool isMagCalibrated = false;

        SlimeVR::Configuration::BMI270CalibrationConfig m_Calibration;
};

#endif
