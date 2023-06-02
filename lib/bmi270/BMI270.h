/*
===============================================
BMI270 accelerometer/gyroscope library for Intel(R) Curie(TM) devices.
Copyright (c) 2015 Intel Corporation.  All rights reserved.

Based on MPU6050 Arduino library provided by Jeff Rowberg as part of his
excellent I2Cdev device library: https://github.com/jrowberg/i2cdevlib

===============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

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
===============================================
*/

#ifndef _BMI270_H_
#define _BMI270_H_

#include "Arduino.h"
#include "I2Cdev.h"

// stuff checked with BMI270 datasheet

#define BMI270_SPI_READ_BIT         7

#define BMI270_RA_CHIP_ID           0x00

#define BMI270_RA_INTERNAL_STATUS   0x21
#define BMI270_RA_CMD               0x7E

#define BMI270_CMD_SOFT_RESET       0xB6
#define BMI270_CMD_FIFO_FLUSH       0xB0

#define BMI270_CMD_G_TRIGGER        0x02


#define BMI270_RA_PWR_CONF          0x7C
#define BMI270_RA_INIT_CTRL         0x59
#define BMI_RA_INIT_ADDR_0          0x5B
#define BMI270_RA_INIT_DATA         0x5E

#define BMI270_RA_SENSORTIME        0x18

#define BMI270_RA_PWR_CTRL          0x7D
#define BMI270_PWR_AUX_BIT          0
#define BMI270_PWR_GYR_BIT          1
#define BMI270_PWR_ACC_BIT          2
#define BMI270_PWR_TEMP_BIT         3


#define BMI270_RA_ACCEL_CONF        0x40
#define BMI270_RA_ACCEL_RANGE       0x41


#define BMI270_ACCEL_RATE_SEL_BIT    0
#define BMI270_ACCEL_RATE_SEL_LEN    4

#define BMI270_ACCEL_RANGE_SEL_BIT  0
#define BMI270_ACCEL_RANGE_SEL_LEN  2

#define BMI270_ACCEL_DLPF_SEL_BIT   4
#define BMI270_ACCEL_DLPF_SEL_LEN   3


#define BMI270_RA_GYRO_CONF         0x42
#define BMI270_RA_GYRO_RANGE        0x43

#define BMI270_GYRO_RATE_SEL_BIT    0
#define BMI270_GYRO_RATE_SEL_LEN    4

#define BMI270_GYRO_RANGE_SEL_BIT   0
#define BMI270_GYRO_RANGE_SEL_LEN   3

#define BMI270_GYRO_DLPF_SEL_BIT    4
#define BMI270_GYRO_DLPF_SEL_LEN    2

#define BMI270_GYRO_NOISE_PERF_BIT  6

#define BMI270_RA_MAG_X_L           0x04
#define BMI270_RA_MAG_X_H           0x05
#define BMI270_RA_MAG_Y_L           0x06
#define BMI270_RA_MAG_Y_H           0x07
#define BMI270_RA_MAG_Z_L           0x08
#define BMI270_RA_MAG_Z_H           0x09

#define BMI270_RA_ACCEL_X_L         0x0c
#define BMI270_RA_ACCEL_X_H         0x0d
#define BMI270_RA_ACCEL_Y_L         0x0e
#define BMI270_RA_ACCEL_Y_H         0x0f
#define BMI270_RA_ACCEL_Z_L         0x10
#define BMI270_RA_ACCEL_Z_H         0x11

#define BMI270_RA_GYRO_X_L          0x12
#define BMI270_RA_GYRO_X_H          0x13
#define BMI270_RA_GYRO_Y_L          0x14
#define BMI270_RA_GYRO_Y_H          0x15
#define BMI270_RA_GYRO_Z_L          0x16
#define BMI270_RA_GYRO_Z_H          0x17

#define BMI270_RA_STATUS            0x03

#define BMI270_STATUS_DRDY_MAG      5
#define BMI270_STATUS_DRDY_GYR      6
#define BMI270_STATUS_DRDY_ACC      7

#define BMI270_RA_ERR                        0x02
#define BMI270_ERR_MASK_FIFO_ERR             0b01000000
#define BMI270_ERR_MASK_I2C_ERR              0b10000000
#define BMI270_ERR_MASK_ERROR_CODE           0b00011110
#define BMI270_ERR_MASK_CHIP_NOT_OPERABLE    0b00000001

#define BMI270_RA_NVM_CONF          0x70
#define BMI270_ACC_OFFSET_EN        3

#define BMI270_GYR_OFFSET_EN        6
#define BMI270_GYR_GAIN_EN          7

#define BMI270_RA_OFFSET_0          0x71
#define BMI270_RA_OFFSET_1          0x72
#define BMI270_RA_OFFSET_2          0x73
#define BMI270_RA_OFFSET_3          0x74
#define BMI270_RA_OFFSET_4          0x75
#define BMI270_RA_OFFSET_5          0x76
#define BMI270_RA_OFFSET_6          0x77


#define BMI270_GYR_OFFSET_X_MSB_BIT 0
#define BMI270_GYR_OFFSET_X_MSB_LEN 2
#define BMI270_GYR_OFFSET_Y_MSB_BIT 2
#define BMI270_GYR_OFFSET_Y_MSB_LEN 2
#define BMI270_GYR_OFFSET_Z_MSB_BIT 4
#define BMI270_GYR_OFFSET_Z_MSB_LEN 2

#define BMI270_RA_TEMP_L            0x22
#define BMI270_RA_TEMP_H            0x23

#define BMI270_RA_FIFO_CONFIG_0     0x48
#define BMI270_RA_FIFO_CONFIG_1     0x49

#define BMI270_FIFO_HEADER_EN_BIT   4
#define BMI270_FIFO_MAG_EN_BIT      5
#define BMI270_FIFO_ACC_EN_BIT      6
#define BMI270_FIFO_GYR_EN_BIT      7

#define BMI270_RA_FIFO_LENGTH_0     0x24
#define BMI270_RA_FIFO_LENGTH_1     0x25

#define BMI270_RA_FIFO_DATA         0x26

#define BMI270_RA_GYR_CAS           0x3C

#define BMI270_BURST_PAYLOAD_LENGTH (I2C_BUFFER_LENGTH - 2)

#define BMI270_RA_FEAT_PAGE         0x2F
#define BMI270_RA_GEN_SET_1         0x34
#define BMI270_GYR_SELF_OFFSET_BIT  9

#define BMI270_FIFO_DATA_INVALID    0x80

#define BMI270_RA_GYR_CRT_CONF      0x69 //nice
#define BMI270_CRT_RUNNING_BIT      2

#define BMI270_RA_G_TRIG_1          0x32
#define BMI270_FEATURE_SELECT_BIT   8
#define BMI270_FEATURE_BLOCK_BIT    9
#define BMI270_FEATURE_CRT          1
#define BMI270_RA_GYR_GAIN_STATUS   0x38 // feature page 0
#define BMI270_G_TRIG_STATUS_OFFSET 3

#define BMI270_RA_GYR_USR_GAIN_0    0x78 // not documented, got from bmi270 driver

/**
 * Accelerometer Output Data Rate options
 * @see setAccelRate()
 */
typedef enum {
    BMI270_ACCEL_RATE_0_78HZ = 1,  /**<   25/32 Hz */
    BMI270_ACCEL_RATE_1_5HZ,       /**<   25/16 Hz */
    BMI270_ACCEL_RATE_3_1HZ,       /**<   25/8 Hz  */
    BMI270_ACCEL_RATE_6_25HZ,      /**<   25/4 Hz  */
    BMI270_ACCEL_RATE_12_5HZ,      /**<   25/2  Hz */
    BMI270_ACCEL_RATE_25HZ,        /**<   25    Hz */
    BMI270_ACCEL_RATE_50HZ,        /**<   50    Hz */
    BMI270_ACCEL_RATE_100HZ,       /**<  100    Hz */
    BMI270_ACCEL_RATE_200HZ,       /**<  200    Hz */
    BMI270_ACCEL_RATE_400HZ,       /**<  400    Hz */
    BMI270_ACCEL_RATE_800HZ,       /**<  800    Hz */
    BMI270_ACCEL_RATE_1600HZ,      /**< 1600    Hz */
} BMI270AccelRate;

/**
 * Accelerometer Sensitivity Range options
 * @see setFullScaleAccelRange()
 */
typedef enum {
    BMI270_ACCEL_RANGE_2G  = 0x00, /**<  +/-  2g range */
    BMI270_ACCEL_RANGE_4G  = 0x01, /**<  +/-  4g range */
    BMI270_ACCEL_RANGE_8G  = 0x02, /**<  +/-  8g range */
    BMI270_ACCEL_RANGE_16G = 0x03, /**<  +/- 16g range */
} BMI270AccelRange;


/**
 * Gyroscope Output Data Rate options
 * @see setGyroRate()
 */
typedef enum {
    BMI270_GYRO_RATE_25HZ = 6,     /**<   25    Hz */
    BMI270_GYRO_RATE_50HZ,         /**<   50    Hz */
    BMI270_GYRO_RATE_100HZ,        /**<  100    Hz */
    BMI270_GYRO_RATE_200HZ,        /**<  200    Hz */
    BMI270_GYRO_RATE_400HZ,        /**<  400    Hz */
    BMI270_GYRO_RATE_800HZ,        /**<  800    Hz */
    BMI270_GYRO_RATE_1600HZ,       /**< 1600    Hz */
    BMI270_GYRO_RATE_3200HZ,       /**< 3200    Hz */
} BMI270GyroRate;

/**
 * Gyroscope Sensitivity Range options
 * @see setFullScaleGyroRange()
 */
typedef enum {
    BMI270_GYRO_RANGE_2000 = 0, /**<  +/- 2000 degrees/second */
    BMI270_GYRO_RANGE_1000,     /**<  +/- 1000 degrees/second */
    BMI270_GYRO_RANGE_500,      /**<  +/-  500 degrees/second */
    BMI270_GYRO_RANGE_250,      /**<  +/-  250 degrees/second */
    BMI270_GYRO_RANGE_125,      /**<  +/-  125 degrees/second */
} BMI270GyroRange;


/**
 * Digital Low-Pass Filter Mode options
 * @see setGyroDLPFMode()
 * @see setAccelDLPFMode()
 */
typedef enum {
    BMI270_DLPF_MODE_NORM = 0x2,
    BMI270_DLPF_MODE_OSR2 = 0x1,
    BMI270_DLPF_MODE_OSR4 = 0x0,
} BMI270DLPFMode;

// not checked stuff from BMI160 driver, probably mostly invalid

#define BMI270_MAG_PMU_STATUS_BIT   0
#define BMI270_MAG_PMU_STATUS_LEN   2

#define BMI270_STATUS_DRDY_MAG      5
#define BMI270_STATUS_MAG_MAN_OP    2
#define BMI270_MAG_RATE_SEL_BIT     0
#define BMI270_MAG_RATE_SEL_LEN     4
#define BMI270_FIFO_MAG_EN_BIT      5

#define BMI270_RA_MAG_CONF             0x44
#define BMI270_RA_MAG_IF_0_DEVADDR     0x4B
#define BMI270_RA_MAG_IF_1_MODE        0x4C
#define BMI270_RA_MAG_IF_2_READ_RA    0x4D
#define BMI270_RA_MAG_IF_3_WRITE_RA    0x4E
#define BMI270_RA_MAG_IF_4_WRITE_VALUE 0x4F
#define BMI270_RA_IF_CONF              0x6B

#define BMI270_IF_CONF_MODE_PRI_AUTO_SEC_OFF 0 << 4
#define BMI270_IF_CONF_MODE_PRI_I2C_SEC_OIS  1 << 4
#define BMI270_IF_CONF_MODE_PRI_AUTO_SEC_MAG 2 << 4

#define BMI270_MAG_SETUP_MODE       0x80
#define BMI270_MAG_DATA_MODE_2      0x01
#define BMI270_MAG_DATA_MODE_6      0x02
#define BMI270_MAG_DATA_MODE_8      0x03

typedef enum {
    BMI270_MAG_RATE_25_32thHZ = 1,  /**< 25/32 Hz */
    BMI270_MAG_RATE_25_16thHZ,      /**< 25/16 Hz */
    BMI270_MAG_RATE_25_8thHZ,       /**< 25/8  Hz */
    BMI270_MAG_RATE_25_4thHZ,       /**< 25/4  Hz */
    BMI270_MAG_RATE_25_2thHZ,       /**< 25/2  Hz */
    BMI270_MAG_RATE_25HZ,           /**< 25    Hz */
    BMI270_MAG_RATE_50HZ,           /**< 50    Hz */
    BMI270_MAG_RATE_100HZ,          /**< 100   Hz */
    BMI270_MAG_RATE_200HZ,          /**< 200   Hz */
    BMI270_MAG_RATE_400HZ,          /**< 400   Hz */
    BMI270_MAG_RATE_800HZ,          /**< 800   Hz */
} BMI270MagRate;

#define BMI270_CMD_MAG_MODE_NORMAL  0x19

#define BMI270_ACC_PMU_STATUS_BIT   4
#define BMI270_ACC_PMU_STATUS_LEN   2
#define BMI270_GYR_PMU_STATUS_BIT   2
#define BMI270_GYR_PMU_STATUS_LEN   2


#define BMI270_STATUS_FOC_RDY       3
#define BMI270_STATUS_NVM_RDY       4

#define BMI270_FOC_ACC_Z_BIT        0
#define BMI270_FOC_ACC_Z_LEN        2
#define BMI270_FOC_ACC_Y_BIT        2
#define BMI270_FOC_ACC_Y_LEN        2
#define BMI270_FOC_ACC_X_BIT        4
#define BMI270_FOC_ACC_X_LEN        2
#define BMI270_FOC_GYR_EN           6

#define BMI270_RA_FOC_CONF          0x69

#define BMI270_RA_STEP_CNT_L        0x78
#define BMI270_RA_STEP_CNT_H        0x79

#define BMI270_CMD_START_FOC        0x03
#define BMI270_CMD_ACC_MODE_NORMAL  0x11
#define BMI270_CMD_GYR_MODE_NORMAL  0x15
#define BMI270_CMD_INT_RESET        0xB1
#define BMI270_CMD_STEP_CNT_CLR     0xB2

                                                               // mode   parm ext
#define BMI270_FIFO_HEADER_CTL_SKIP_FRAME               0x40   // 0b01 0 000  00
#define BMI270_FIFO_HEADER_CTL_SENSOR_TIME              0x44   // 0b01 0 001  00
#define BMI270_FIFO_HEADER_CTL_INPUT_CONFIG             0x48   // 0b01 0 010  00
#define BMI270_FIFO_HEADER_DATA_FRAME_BASE              0x80   // 0b10 0 000  00
#define BMI270_FIFO_HEADER_DATA_FRAME_FLAG_M            1 << 4 // 0b00 0 100  00
#define BMI270_FIFO_HEADER_DATA_FRAME_FLAG_G            1 << 3 // 0b00 0 010  00
#define BMI270_FIFO_HEADER_DATA_FRAME_FLAG_A            1 << 2 // 0b00 0 001  00
#define BMI270_FIFO_HEADER_DATA_FRAME_MASK_HAS_DATA \
    (BMI270_FIFO_HEADER_DATA_FRAME_FLAG_M |\
    BMI270_FIFO_HEADER_DATA_FRAME_FLAG_G |\
    BMI270_FIFO_HEADER_DATA_FRAME_FLAG_A)

#define BMI270_FIFO_SKIP_FRAME_LEN 1
#define BMI270_FIFO_INPUT_CONFIG_LEN 1
#define BMI270_FIFO_SENSOR_TIME_LEN 3
#define BMI270_FIFO_M_LEN 8
#define BMI270_FIFO_G_LEN 6
#define BMI270_FIFO_A_LEN 6

class BMI270 {
    public:
        BMI270();
        bool initialize(
            uint8_t addr,
            BMI270GyroRate gyroRate = BMI270_GYRO_RATE_800HZ,
            BMI270GyroRange gyroRange = BMI270_GYRO_RANGE_500,
            BMI270DLPFMode gyroFilterMode = BMI270_DLPF_MODE_NORM,
            BMI270AccelRate accelRate = BMI270_ACCEL_RATE_800HZ,
            BMI270AccelRange accelRange = BMI270_ACCEL_RANGE_4G,
            BMI270DLPFMode accelFilterMode = BMI270_DLPF_MODE_OSR4
        );

        void powerUp(uint8_t gyroscope, uint8_t accelerometer, uint8_t temperature);


        uint8_t getInternalStatus();
        bool uploadFW();

        bool testConnection();

        void setGyroRate(uint8_t rate);
        void setAccelRate(uint8_t rate);
        void setGyroDLPFMode(uint8_t bandwidth);
        void setGyroFilterPerfMode(bool highPerf);

        uint8_t getAccelDLPFMode();
        void setAccelDLPFMode(uint8_t bandwidth);

        uint8_t getFullScaleGyroRange();
        void setFullScaleGyroRange(uint8_t range);
        uint8_t getFullScaleAccelRange();
        void setFullScaleAccelRange(uint8_t range);

        void autoCalibrateGyroOffset();
        bool getGyroOffsetEnabled();
        void setGyroOffsetEnabled(bool enabled);
        void setGyroIOC(bool enable);

        int16_t getXGyroOffset();
        void setXGyroOffset(int16_t offset);
        int16_t getYGyroOffset();
        void setYGyroOffset(int16_t offset);
        int16_t getZGyroOffset();
        void setZGyroOffset(int16_t offset);

        void autoCalibrateXAccelOffset(int target);
        void autoCalibrateYAccelOffset(int target);
        void autoCalibrateZAccelOffset(int target);
        bool getAccelOffsetEnabled();
        void setAccelOffsetEnabled(bool enabled);

        int8_t getXAccelOffset();
        void setXAccelOffset(int8_t offset);
        int8_t getYAccelOffset();
        void setYAccelOffset(int8_t offset);
        int8_t getZAccelOffset();
        void setZAccelOffset(int8_t offset);

        bool getGyroFIFOEnabled();
        void setGyroFIFOEnabled(bool enabled);
        bool getAccelFIFOEnabled();
        void setAccelFIFOEnabled(bool enabled);
        bool getMagFIFOEnabled();
        void setMagFIFOEnabled(bool enabled);

        void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
        void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
        int16_t getAccelerationX();
        int16_t getAccelerationY();
        int16_t getAccelerationZ();

        void getMagnetometer(int16_t* mx, int16_t* my, int16_t* mz);
        void getMagnetometerXYZBuffer(uint8_t* data);

        bool getTemperature(int16_t* out);

        void getRotation(int16_t* x, int16_t* y, int16_t* z);
        int16_t getRotationX();
        int16_t getRotationY();
        int16_t getRotationZ();

        bool getFIFOHeaderModeEnabled();
        void setFIFOHeaderModeEnabled(bool enabled);
        void resetFIFO();

        bool getFIFOCount(uint16_t* outCount);
        bool getFIFOBytes(uint8_t *data, uint16_t length);

        uint8_t getDeviceID();

        uint8_t getRegister(uint8_t reg);
        void setRegister(uint8_t reg, uint8_t data);

        bool getGyroDrdy();
        void waitForGyroDrdy();
        void waitForAccelDrdy();
        void waitForMagDrdy();
        bool getSensorTime(uint32_t *v_sensor_time_u32);
        void setMagDeviceAddress(uint8_t addr);
        bool setMagRegister(uint8_t addr, uint8_t value);

        bool getErrReg(uint8_t* out);
        int8_t getZXFactor();
        void selectFeaturePage(uint8_t page);
        bool performCRT(uint8_t &gainX, uint8_t &gainY, uint8_t &gainZ);
        void applyGyroGain(uint8_t &gainX, uint8_t &gainY, uint8_t &gainZ);

    private:
        uint8_t buffer[14];
        uint8_t devAddr;
};

#endif /* _BMI270_H_ */