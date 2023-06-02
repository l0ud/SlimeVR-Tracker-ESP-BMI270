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
#include "BMI270.h"
#include "I2Cdev.h"
#include <algorithm>
#include "BMI270_firmware.h"

#define BMI270_CHIP_ID 0x24
#define BMI270_POWERUP_DELAY_MS 100

/* Test the sign bit and set remaining MSBs if sign bit is set */
#define BMI270_SIGN_EXTEND(val, from) \
    (((val) & (1 << ((from) - 1))) ? (val | (((1 << (1 + (sizeof(val) << 3) - (from))) - 1) << (from))) : val)


/******************************************************************************/

class I2CdevMod : public I2Cdev {
    public:
        static bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
            uint8_t b;
            if (readByte(devAddr, regAddr, &b) != 0) {
                uint8_t mask = ((1 << length) - 1) << bitStart;
                data <<= bitStart; // shift data into correct position
                data &= mask; // zero all non-important bits in data
                b &= ~(mask); // zero all important bits in existing byte
                b |= data; // combine data with existing byte
                return writeByte(devAddr, regAddr, b);
            } else {
                return false;
            }
        }
        static int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout) {
            uint8_t count, b;
            if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
                uint8_t mask = ((1 << length) - 1) << bitStart;
                b &= mask;
                b >>= bitStart;
                *data = b;
            }
            return count;
        }
};

BMI270::BMI270() {};

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up).
 */
bool BMI270::initialize(uint8_t addr,
    BMI270GyroRate gyroRate, BMI270GyroRange gyroRange, BMI270DLPFMode gyroFilterMode,
    BMI270AccelRate accelRate, BMI270AccelRange accelRange, BMI270DLPFMode accelFilterMode
) {
    devAddr = addr;
    /* Issue a soft-reset to bring the device into a clean state */
    setRegister(BMI270_RA_CMD, BMI270_CMD_SOFT_RESET);
    delay(12);
    printf("BMI270: Status after soft reset: 0x%x\n", getInternalStatus());
    printf("BMI270: Uploading firmware...\n");
    if (!uploadFW()) {
        printf("BMI270: Failed to upload firmware, status: 0x%x\n", getInternalStatus());
        return false;
    }

    printf("BMI270: Status after fw upload: 0x%x\n", getInternalStatus());

    setGyroRate(gyroRate);
    setAccelRate(accelRate);
    setFullScaleGyroRange(gyroRange);
    setFullScaleAccelRange(accelRange);
    setGyroDLPFMode(gyroFilterMode);
    setAccelDLPFMode(accelFilterMode);

    I2CdevMod::readByte(devAddr, BMI270_RA_GYRO_CONF, buffer);
    printf("BMI270: GYRO_CONF: 0x%x\n", buffer[0]);
    I2CdevMod::readByte(devAddr, BMI270_RA_GYRO_RANGE, buffer);
    printf("BMI270: GYRO_RANGE: 0x%x\n", buffer[0]);
    I2CdevMod::readByte(devAddr, BMI270_RA_ACCEL_CONF, buffer);
    printf("BMI270: ACCEL_CONF: 0x%x\n", buffer[0]);
    I2CdevMod::readByte(devAddr, BMI270_RA_ACCEL_RANGE, buffer);
    printf("BMI270: ACCEL_RANGE: 0x%x\n", buffer[0]);

    I2CdevMod::readByte(devAddr, BMI270_RA_FIFO_CONFIG_0, buffer);
    printf("BMI270: FIFO_CONFIG_0: 0x%x\n", buffer[0]);
    I2CdevMod::readByte(devAddr, BMI270_RA_FIFO_CONFIG_1, buffer);
    printf("BMI270: FIFO_CONFIG_1: 0x%x\n", buffer[0]);

    I2CdevMod::readByte(devAddr, BMI270_RA_STATUS, buffer);
    printf("BMI270: STATUS: 0x%x\n", buffer[0]);

    printf("BMI270: ZX FACTOR: %d\n", getZXFactor());

    return true;
}

void BMI270::powerUp(uint8_t gyroscope, uint8_t accelerometer, uint8_t temperature) {
    // enable power to accelerometer, gyroscope and temperature sensor
    printf("Value to pwr ctrl 0x%x\n", (gyroscope << BMI270_PWR_GYR_BIT) | (accelerometer << BMI270_PWR_ACC_BIT) | (temperature << BMI270_PWR_TEMP_BIT));
    setRegister(BMI270_RA_PWR_CTRL, (gyroscope << BMI270_PWR_GYR_BIT) | (accelerometer << BMI270_PWR_ACC_BIT) | (temperature << BMI270_PWR_TEMP_BIT));
    delay(BMI270_POWERUP_DELAY_MS);

    I2CdevMod::readByte(devAddr, BMI270_RA_PWR_CTRL, buffer);
    printf("BMI270: PWR_CTRL: 0x%x\n", buffer[0]);
    printf("BMI270: Status after pwr ctrl: 0x%x\n", getInternalStatus());
}

bool BMI270::getErrReg(uint8_t* out) {
    bool ok = I2CdevMod::readByte(devAddr, BMI270_RA_ERR, buffer) >= 0;
    if (!ok) return false;
    *out = buffer[0];
    return true;
}

int8_t BMI270::getZXFactor()
{
    selectFeaturePage(0);
    I2CdevMod::readByte(devAddr, BMI270_RA_GYR_CAS, buffer);
    // convert 7-bit two's complement value to 8-bit one, copying 7th bit (sign) to 8th bit
    const uint8_t sign_byte = (buffer[0] << 1) & 0x80;
    return (int8_t)(buffer[0] | sign_byte);
}

void BMI270::setGyroIOC(bool enable)
{
    selectFeaturePage(1);
    I2CdevMod::readBytes(devAddr, BMI270_RA_GEN_SET_1, 2, buffer);

    uint16_t feature_reg = (((uint16_t)buffer[1]) << 8) | buffer[0];
    printf("BMI270: Features read: 0x%x\n", feature_reg);

    if (enable) {
        feature_reg |= ((uint16_t)1 << BMI270_GYR_SELF_OFFSET_BIT);
    }
    else {
        feature_reg &= ~((uint16_t)1 << BMI270_GYR_SELF_OFFSET_BIT);
    }
    printf("BMI270: Features to write: 0x%x\n", feature_reg);

    buffer[0] = (uint8_t)feature_reg;
    buffer[1] = (uint8_t)(feature_reg >> 8);
    I2CdevMod::writeBytes(devAddr, BMI270_RA_GEN_SET_1, 2, buffer);
}

void BMI270::selectFeaturePage(uint8_t page) {
    setRegister(BMI270_RA_FEAT_PAGE, page);
}

bool BMI270::performCRT(uint8_t &gainX, uint8_t &gainY, uint8_t &gainZ)
{
    I2CdevMod::writeBits(devAddr, BMI270_RA_OFFSET_6,
                          BMI270_GYR_GAIN_EN, 1, 1);
    // we assume accelerometer is enabled here
    I2CdevMod::writeBits(devAddr, BMI270_RA_GYR_CRT_CONF,
                          BMI270_CRT_RUNNING_BIT, 1, 1);

    I2CdevMod::readBytes(devAddr, BMI270_RA_GYR_USR_GAIN_0, 3, buffer);

    I2CdevMod::readBytes(devAddr, BMI270_RA_G_TRIG_1, 2, buffer);

    uint16_t trig_reg = (((uint16_t)buffer[1]) << 8) | buffer[0];

    trig_reg |= ((uint16_t)BMI270_FEATURE_CRT << BMI270_FEATURE_SELECT_BIT);
    trig_reg &= ~((uint16_t)1 << BMI270_FEATURE_BLOCK_BIT);
    buffer[0] = (uint8_t)trig_reg;
    buffer[1] = (uint8_t)(trig_reg >> 8);
    I2CdevMod::writeBytes(devAddr, BMI270_RA_G_TRIG_1, 2, buffer);
    I2CdevMod::writeByte(devAddr, BMI270_RA_CMD, BMI270_CMD_G_TRIGGER);
    delay(200);
    I2CdevMod::readByte(devAddr, BMI270_RA_GYR_CRT_CONF, buffer);
    while(buffer[0] & (1 << BMI270_CRT_RUNNING_BIT)) {
        printf("CRT running. DO NOT MOVE TRACKER!\n");
        delay(100);
        I2CdevMod::readByte(devAddr, BMI270_RA_GYR_CRT_CONF, buffer);
    }

    selectFeaturePage(0);
    I2CdevMod::readByte(devAddr, BMI270_RA_GYR_GAIN_STATUS, buffer);
    printf("BMI270: GYR_GAIN_STATUS: 0x%x\n", buffer[0]);
    const uint8_t status = buffer[0] >> BMI270_G_TRIG_STATUS_OFFSET;
    if (status != 0) {
        printf("CRT failed with status 0x%x\n", status);
        if (status == 0x03) {
            printf("You moved tracker!\n");
        }
        return false;
    }

    printf("CRT completed successfully!\n");

    I2CdevMod::readBytes(devAddr, BMI270_RA_GYR_USR_GAIN_0, 3, buffer);
    gainX = buffer[0];
    gainY = buffer[1];
    gainZ = buffer[2];
    return true;
}

void BMI270::applyGyroGain(uint8_t &gainX, uint8_t &gainY, uint8_t &gainZ)
{
    I2CdevMod::writeBits(devAddr, BMI270_RA_OFFSET_6,
                          BMI270_GYR_GAIN_EN, 1, 1);
    buffer[0] = gainX;
    buffer[1] = gainY;
    buffer[2] = gainZ;
    I2CdevMod::writeBytes(devAddr, BMI270_RA_GYR_USR_GAIN_0, 3, buffer);
}

void BMI270::setMagDeviceAddress(uint8_t addr) {
    setRegister(BMI270_RA_MAG_IF_0_DEVADDR, addr << 1); // 0 bit of address is reserved and needs to be shifted
}

bool BMI270::setMagRegister(uint8_t addr, uint8_t value) {
    setRegister(BMI270_RA_MAG_IF_4_WRITE_VALUE, value);
    setRegister(BMI270_RA_MAG_IF_3_WRITE_RA, addr);
    delay(3);
    I2CdevMod::readByte(devAddr, BMI270_RA_ERR, buffer);
    if (buffer[0] & BMI270_ERR_MASK_I2C_ERR) {
        printf("BMI270: mag register proxy write error\n");
        return false;
    }
    return true;
}

/** Get Device ID.
 * This register is used to verify the identity of the device (0b11010001, 0xD1).
 * @return Device ID (should be 0xD1)
 * @see BMI270_RA_CHIP_ID
 */
uint8_t BMI270::getDeviceID() {
    I2CdevMod::readByte(devAddr, BMI270_RA_CHIP_ID, buffer);
    return buffer[0];
}


uint8_t BMI270::getInternalStatus() {
        I2CdevMod::readByte(devAddr, BMI270_RA_INTERNAL_STATUS, buffer);
        return buffer[0];
}

bool BMI270::uploadFW() {
    I2CdevMod::writeByte(devAddr, BMI270_RA_PWR_CONF, 0x00); // disable power saving
    delay(1);
    
    I2CdevMod::writeByte(devAddr, BMI270_RA_INIT_CTRL, 0x00); // start initialization procedure

    for (uint16_t pos=0; pos<sizeof(bmi270_firmware);)
    {
        const uint16_t posDiv = pos >> 1;
        uint8_t pos_regs[2] = {(uint8_t)(posDiv & 0x0F), (uint8_t)(posDiv >> 4)};

        // tell the device current position
        I2CdevMod::writeBytes(devAddr, BMI_RA_INIT_ADDR_0, sizeof(pos_regs), pos_regs);
        // write actual payload chunk
        const uint16_t burst_write = std::min(sizeof(bmi270_firmware) - pos, (size_t)BMI270_BURST_PAYLOAD_LENGTH);
        I2CdevMod::writeBytes(devAddr, BMI270_RA_INIT_DATA, burst_write, const_cast<uint8_t*>(bmi270_firmware + pos));

        pos += burst_write;
    }

    I2CdevMod::writeByte(devAddr, BMI270_RA_INIT_CTRL, 0x01); // end initialization procedure
    delay(140);

    I2CdevMod::writeByte(devAddr, BMI270_RA_PWR_CONF, 0x02); // leave fifo_self_wakeup enabled

    return (getInternalStatus() & 0x01) == 0x01; // check if initialized bit is set
};

/** Verify the SPI connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool BMI270::testConnection() { return (BMI270_CHIP_ID == getDeviceID()); }

/** Set gyroscope output data rate.
 * @param rate New output data rate
 * @see getGyroRate()
 * @see BMI270_GYRO_RATE_25HZ
 * @see BMI270_RA_GYRO_CONF
 */
void BMI270::setGyroRate(uint8_t rate) {
    I2CdevMod::writeBits(devAddr, BMI270_RA_GYRO_CONF,
                   BMI270_GYRO_RATE_SEL_BIT,
                   BMI270_GYRO_RATE_SEL_LEN, rate);
}

/** Set accelerometer output data rate.
 * @param rate New output data rate
 * @see getAccelRate()
 * @see BMI270_RA_ACCEL_CONF
 */
void BMI270::setAccelRate(uint8_t rate) {
    I2CdevMod::writeBits(devAddr, BMI270_RA_ACCEL_CONF,
                   BMI270_ACCEL_RATE_SEL_BIT,
                   BMI270_ACCEL_RATE_SEL_LEN, rate);
}

/** Set gyroscope digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getGyroDLPFMode()
 */
void BMI270::setGyroDLPFMode(uint8_t mode) {
    I2CdevMod::writeBits(devAddr, BMI270_RA_GYRO_CONF,
                          BMI270_GYRO_DLPF_SEL_BIT,
                          BMI270_GYRO_DLPF_SEL_LEN, mode);
}

void BMI270::setGyroFilterPerfMode(bool highPerf) {
    I2CdevMod::writeBits(devAddr, BMI270_RA_GYRO_CONF,
                          BMI270_GYRO_NOISE_PERF_BIT,
                          1, highPerf ? 1 : 0);
}

/** Set accelerometer digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getAccelDLPFMode()
 */
void BMI270::setAccelDLPFMode(uint8_t mode) {
    I2CdevMod::writeBits(devAddr, BMI270_RA_ACCEL_CONF,
                          BMI270_ACCEL_DLPF_SEL_BIT,
                          BMI270_ACCEL_DLPF_SEL_LEN, mode);
}

/** Get full-scale gyroscope range.
 * The gyr_range parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 4 = +/-  125 degrees/sec
 * 3 = +/-  250 degrees/sec
 * 2 = +/-  500 degrees/sec
 * 1 = +/- 1000 degrees/sec
 * 0 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see BMI270_RA_GYRO_RANGE
 * @see BMI270GyroRange
 */
uint8_t BMI270::getFullScaleGyroRange() {
    I2CdevMod::readBits(devAddr, BMI270_RA_GYRO_RANGE,
                         BMI270_GYRO_RANGE_SEL_BIT,
                         BMI270_GYRO_RANGE_SEL_LEN, buffer);
    return buffer[0];
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleGyroRange()
 */
void BMI270::setFullScaleGyroRange(uint8_t range) {
    I2CdevMod::writeBits(devAddr, BMI270_RA_GYRO_RANGE,
                   BMI270_GYRO_RANGE_SEL_BIT,
                   BMI270_GYRO_RANGE_SEL_LEN, range);
}

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 *  0 = +/- 2g
 *  1 = +/- 4g
 *  2 = +/- 8g
 *  3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see BMI270_RA_ACCEL_RANGE
 * @see BMI270AccelRange
 */
uint8_t BMI270::getFullScaleAccelRange() {
    I2CdevMod::readBits(devAddr, BMI270_RA_ACCEL_RANGE,
                         BMI270_ACCEL_RANGE_SEL_BIT,
                         BMI270_ACCEL_RANGE_SEL_LEN, buffer);
    return buffer[0];
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 * @see BMI270AccelRange
 */
void BMI270::setFullScaleAccelRange(uint8_t range) {
    I2CdevMod::writeBits(devAddr, BMI270_RA_ACCEL_RANGE,
                   BMI270_ACCEL_RANGE_SEL_BIT,
                   BMI270_ACCEL_RANGE_SEL_LEN, range);
}

/** Get accelerometer offset compensation enabled value.
 * @see getXAccelOffset()
 * @see BMI270_RA_NVM_CONF
 */
bool BMI270::getAccelOffsetEnabled() {
    I2CdevMod::readBits(devAddr, BMI270_RA_NVM_CONF,
                            BMI270_ACC_OFFSET_EN,
                            1, buffer);
    return !!buffer[0];
}

/** Set accelerometer offset compensation enabled value.
 * @see getXAccelOffset()
 * @see BMI270_RA_NVM_CONF
 */
void BMI270::setAccelOffsetEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI270_RA_NVM_CONF,
                   BMI270_ACC_OFFSET_EN,
                   1, enabled ? 0x1 : 0);
}

/** Execute internal calibration to generate Accelerometer X-Axis offset value.
 * This populates the Accelerometer offset compensation value for the X-Axis only.
 * These can be retrieved using the getXAccelOffset() methods.
 * Note that this procedure may take up to 250ms to complete.
 *
 * IMPORTANT: The user MUST ensure NO movement and correct orientation of the
 * BMI270 device occurs while this auto-calibration process is active.
 * For example, to calibrate to a target of 0g on the X-axis, the BMI270 device
 * must be resting horizontally as shown in Section 5.2 of the BMI270 Data Sheet.
 *
 * To enable offset compensation, @see setAccelOffsetEnabled()
 *
 * @param target X-axis target value (0 = 0g, 1 = +1g, -1 = -1g)
 * @see setAccelOffsetEnabled()
 * @see getXAccelOffset()
 * @see BMI270_RA_FOC_CONF
 * @see BMI270_RA_CMD
 */
void BMI270::autoCalibrateXAccelOffset(int target) {
    uint8_t foc_conf;
    if (target == 1)
        foc_conf = (0x1 << BMI270_FOC_ACC_X_BIT);
    else if (target == -1)
        foc_conf = (0x2 << BMI270_FOC_ACC_X_BIT);
    else if (target == 0)
        foc_conf = (0x3 << BMI270_FOC_ACC_X_BIT);
    else
        return;  /* Invalid target value */

    I2CdevMod::writeByte(devAddr, BMI270_RA_FOC_CONF, foc_conf);
    I2CdevMod::writeByte(devAddr, BMI270_RA_CMD, BMI270_CMD_START_FOC);
    do {
        I2CdevMod::readBits(devAddr, BMI270_RA_STATUS,
                           BMI270_STATUS_FOC_RDY,
                           1, buffer);
        delay(1);
    } while (!buffer[0]);
}

/** Execute internal calibration to generate Accelerometer Y-Axis offset value.
 * This populates the Accelerometer offset compensation value for the Y-Axis only.
 * These can be retrieved using the getYAccelOffset() methods.
 * Note that this procedure may take up to 250ms to complete.
 *
 * IMPORTANT: The user MUST ensure NO movement and correct orientation of the
 * BMI270 device occurs while this auto-calibration process is active.
 * For example, to calibrate to a target of 0g on the Y-axis, the BMI270 device
 * must be resting horizontally as shown in Section 5.2 of the BMI270 Data Sheet.
 *
 * To enable offset compensation, @see setAccelOffsetEnabled()
 *
 * @param target Y-axis target value (0 = 0g, 1 = +1g, -1 = -1g)
 * @see setAccelOffsetEnabled()
 * @see getYAccelOffset()
 * @see BMI270_RA_FOC_CONF
 * @see BMI270_RA_CMD
 */
void BMI270::autoCalibrateYAccelOffset(int target) {
    uint8_t foc_conf;
    if (target == 1)
        foc_conf = (0x1 << BMI270_FOC_ACC_Y_BIT);
    else if (target == -1)
        foc_conf = (0x2 << BMI270_FOC_ACC_Y_BIT);
    else if (target == 0)
        foc_conf = (0x3 << BMI270_FOC_ACC_Y_BIT);
    else
        return;  /* Invalid target value */

    I2CdevMod::writeByte(devAddr, BMI270_RA_FOC_CONF, foc_conf);
    I2CdevMod::writeByte(devAddr, BMI270_RA_CMD, BMI270_CMD_START_FOC);
    do {
        I2CdevMod::readBits(devAddr, BMI270_RA_STATUS,
                           BMI270_STATUS_FOC_RDY,
                           1, buffer);
        delay(1);
    } while (!buffer[0]);
}

/** Execute internal calibration to generate Accelerometer Z-Axis offset value.
 * This populates the Accelerometer offset compensation value for the Z-Axis only.
 * These can be retrieved using the getZAccelOffset() methods.
 * Note that this procedure may take up to 250ms to complete.
 *
 * IMPORTANT: The user MUST ensure NO movement and correct orientation of the
 * BMI270 device occurs while this auto-calibration process is active.
 * For example, to calibrate to a target of +1g on the Z-axis, the BMI270 device
 * must be resting horizontally as shown in Section 5.2 of the BMI270 Data Sheet.
 *
 * To enable offset compensation, @see setAccelOffsetEnabled()
 *
 * @param target Z-axis target value (0 = 0g, 1 = +1g, -1 = -1g)
 * @see setAccelOffsetEnabled()
 * @see getZAccelOffset()
 * @see BMI270_RA_FOC_CONF
 * @see BMI270_RA_CMD
 */
void BMI270::autoCalibrateZAccelOffset(int target) {
    uint8_t foc_conf;
    if (target == 1)
        foc_conf = (0x1 << BMI270_FOC_ACC_Z_BIT);
    else if (target == -1)
        foc_conf = (0x2 << BMI270_FOC_ACC_Z_BIT);
    else if (target == 0)
        foc_conf = (0x3 << BMI270_FOC_ACC_Z_BIT);
    else
        return;  /* Invalid target value */

    I2CdevMod::writeByte(devAddr, BMI270_RA_FOC_CONF, foc_conf);
    I2CdevMod::writeByte(devAddr, BMI270_RA_CMD, BMI270_CMD_START_FOC);
    do {
        I2CdevMod::readBits(devAddr, BMI270_RA_STATUS,
                           BMI270_STATUS_FOC_RDY,
                           1, buffer);
        delay(1);
    } while (!buffer[0]);
}

/** Get offset compensation value for accelerometer X-axis data.
 * The value is represented as an 8-bit two-complement number in
 * units of 3.9mg per LSB.
 * @see BMI270_RA_OFFSET_0
 */
int8_t BMI270::getXAccelOffset() {
    I2CdevMod::readByte(devAddr, BMI270_RA_OFFSET_0, buffer);
    return buffer[0];
}

/** Set offset compensation value for accelerometer X-axis data.
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateXAccelOffset().
 * @see getXAccelOffset()
 * @see BMI270_RA_OFFSET_0
 */
void BMI270::setXAccelOffset(int8_t offset) {
    I2CdevMod::writeByte(devAddr, BMI270_RA_OFFSET_0, offset);
    getAccelerationX(); /* Read and discard the next data value */
}

/** Get offset compensation value for accelerometer Y-axis data.
 * The value is represented as an 8-bit two-complement number in
 * units of 3.9mg per LSB.
 * @see BMI270_RA_OFFSET_1
 */
int8_t BMI270::getYAccelOffset() {
    I2CdevMod::readByte(devAddr, BMI270_RA_OFFSET_1, buffer);
    return buffer[0];
}

/** Set offset compensation value for accelerometer Y-axis data.
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateYAccelOffset().
 * @see getYAccelOffset()
 * @see BMI270_RA_OFFSET_1
 */
void BMI270::setYAccelOffset(int8_t offset) {
    I2CdevMod::writeByte(devAddr, BMI270_RA_OFFSET_1, offset);
    getAccelerationY(); /* Read and discard the next data value */
}

/** Get offset compensation value for accelerometer Z-axis data.
 * The value is represented as an 8-bit two-complement number in
 * units of 3.9mg per LSB.
 * @see BMI270_RA_OFFSET_2
 */
int8_t BMI270::getZAccelOffset() {
    I2CdevMod::readByte(devAddr, BMI270_RA_OFFSET_2, buffer);
    return buffer[0];
}

/** Set offset compensation value for accelerometer Z-axis data.
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateZAccelOffset().
 * @see getZAccelOffset()
 * @see BMI270_RA_OFFSET_2
 */
void BMI270::setZAccelOffset(int8_t offset) {
    I2CdevMod::writeByte(devAddr, BMI270_RA_OFFSET_2, offset);
    getAccelerationZ(); /* Read and discard the next data value */
}

/** Get gyroscope offset compensation enabled value.
 * @see getXGyroOffset()
 * @see BMI270_RA_OFFSET_6
 */
bool BMI270::getGyroOffsetEnabled() {
    I2CdevMod::readBits(devAddr, BMI270_RA_OFFSET_6,
                            BMI270_GYR_OFFSET_EN,
                            1, buffer);
    return !!buffer[0];
}

/** Set gyroscope offset compensation enabled value.
 * @see getXGyroOffset()
 * @see BMI270_RA_OFFSET_6
 */
void BMI270::setGyroOffsetEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI270_RA_OFFSET_6,
                      BMI270_GYR_OFFSET_EN,
                      1, enabled ? 0x1 : 0);
}

/** Execute internal calibration to generate Gyro offset values.
 * This populates the Gyro offset compensation values for all 3 axes.
 * These can be retrieved using the get[X/Y/Z]GyroOffset() methods.
 * Note that this procedure may take up to 250ms to complete.
 *
 * IMPORTANT: The user MUST ensure that NO rotation of the BMI270 device
 * occurs while this auto-calibration process is active.
 *
 * To enable offset compensation, @see setGyroOffsetEnabled()
 * @see setGyroOffsetEnabled()
 * @see getXGyroOffset()
 * @see getYGyroOffset()
 * @see getZGyroOffset()
 * @see BMI270_RA_FOC_CONF
 * @see BMI270_RA_CMD
 */
void BMI270::autoCalibrateGyroOffset() {
    uint8_t foc_conf = (1 << BMI270_FOC_GYR_EN);
    I2CdevMod::writeByte(devAddr, BMI270_RA_FOC_CONF, foc_conf);
    I2CdevMod::writeByte(devAddr, BMI270_RA_CMD, BMI270_CMD_START_FOC);
    do {
        I2CdevMod::readBits(devAddr, BMI270_RA_STATUS,
                           BMI270_STATUS_FOC_RDY,
                           1, buffer);
        delay(1);
    } while (!buffer[0]);
}

/** Get offset compensation value for gyroscope X-axis data.
 * The value is represented as an 10-bit two-complement number in
 * units of 0.061 degrees/s per LSB (sign-extended for int16_t type).
 * @see BMI270_RA_OFFSET_3
 * @see BMI270_RA_OFFSET_6
 */
int16_t BMI270::getXGyroOffset() {
    I2CdevMod::readByte(devAddr, BMI270_RA_OFFSET_3, buffer);
    int16_t offset = buffer[0];
    I2CdevMod::readBits(devAddr, BMI270_RA_OFFSET_6,
                     BMI270_GYR_OFFSET_X_MSB_BIT,
                     BMI270_GYR_OFFSET_X_MSB_LEN,
                     buffer);
    offset |= (int16_t)(buffer[0]) << 8;
    return BMI270_SIGN_EXTEND(offset, 10);
}

/** Set offset compensation value for gyroscope X-axis data.
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateGyroOffset().
 * @see getXGyroOffset()
 * @see BMI270_RA_OFFSET_3
 * @see BMI270_RA_OFFSET_6
 */
void BMI270::setXGyroOffset(int16_t offset) {
    I2CdevMod::writeByte(devAddr, BMI270_RA_OFFSET_3, offset);
    I2CdevMod::writeBits(devAddr, BMI270_RA_OFFSET_6,
                   BMI270_GYR_OFFSET_X_MSB_BIT,
                   BMI270_GYR_OFFSET_X_MSB_LEN, offset >> 8);
    getRotationX(); /* Read and discard the next data value */
}

/** Get offset compensation value for gyroscope Y-axis data.
 * The value is represented as an 10-bit two-complement number in
 * units of 0.061 degrees/s per LSB (sign-extended for int16_t type).
 * @see BMI270_RA_OFFSET_4
 * @see BMI270_RA_OFFSET_6
 */
int16_t BMI270::getYGyroOffset() {
    I2CdevMod::readByte(devAddr, BMI270_RA_OFFSET_4, buffer);
    int16_t offset = buffer[0];
    I2CdevMod::readBits(devAddr, BMI270_RA_OFFSET_6,
                     BMI270_GYR_OFFSET_Y_MSB_BIT,
                     BMI270_GYR_OFFSET_Y_MSB_LEN, buffer);
    offset |= (int16_t)(buffer[0]) << 8;
    return BMI270_SIGN_EXTEND(offset, 10);
}

/** Set offset compensation value for gyroscope Y-axis data.
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateGyroOffset().
 * @see getYGyroOffset()
 * @see BMI270_RA_OFFSET_4
 * @see BMI270_RA_OFFSET_6
 */
void BMI270::setYGyroOffset(int16_t offset) {
    I2CdevMod::writeByte(devAddr, BMI270_RA_OFFSET_4, offset);
    I2CdevMod::writeBits(devAddr, BMI270_RA_OFFSET_6,
                   BMI270_GYR_OFFSET_Y_MSB_BIT,
                   BMI270_GYR_OFFSET_Y_MSB_LEN, offset >> 8);
    getRotationY(); /* Read and discard the next data value */
}

/** Get offset compensation value for gyroscope Z-axis data.
 * The value is represented as an 10-bit two-complement number in
 * units of 0.061 degrees/s per LSB (sign-extended for int16_t type).
 * @see BMI270_RA_OFFSET_5
 * @see BMI270_RA_OFFSET_6
 */
int16_t BMI270::getZGyroOffset() {
    I2CdevMod::readByte(devAddr, BMI270_RA_OFFSET_5, buffer);
    int16_t offset = buffer[0];
    I2CdevMod::readBits(devAddr, BMI270_RA_OFFSET_6,
                     BMI270_GYR_OFFSET_Z_MSB_BIT,
                     BMI270_GYR_OFFSET_Z_MSB_LEN, buffer);
    offset |= (int16_t)(buffer[0]) << 8;
    return BMI270_SIGN_EXTEND(offset, 10);
}

/** Set offset compensation value for gyroscope Z-axis data.
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateGyroOffset().
 * @see getZGyroOffset()
 * @see BMI270_RA_OFFSET_5
 * @see BMI270_RA_OFFSET_6
 */
void BMI270::setZGyroOffset(int16_t offset) {
    I2CdevMod::writeByte(devAddr, BMI270_RA_OFFSET_5, offset);
    I2CdevMod::writeBits(devAddr, BMI270_RA_OFFSET_6,
                   BMI270_GYR_OFFSET_Z_MSB_BIT,
                   BMI270_GYR_OFFSET_Z_MSB_LEN, offset >> 8);
    getRotationZ(); /* Read and discard the next data value */
}

/** Get accelerometer FIFO enabled value.
 * When set to 1, this bit enables accelerometer data samples to be
 * written into the FIFO buffer.
 * @return Current accelerometer FIFO enabled value
 * @see BMI270_RA_FIFO_CONFIG_1
 */
bool BMI270::getAccelFIFOEnabled() {
    I2CdevMod::readBits(devAddr, BMI270_RA_FIFO_CONFIG_1,
                     BMI270_FIFO_ACC_EN_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set accelerometer FIFO enabled value.
 * @param enabled New accelerometer FIFO enabled value
 * @see getAccelFIFOEnabled()
 * @see BMI270_RA_FIFO_CONFIG_1
 */
void BMI270::setAccelFIFOEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI270_RA_FIFO_CONFIG_1,
                   BMI270_FIFO_ACC_EN_BIT,
                   1, enabled ? 0x1 : 0);
}

/** Get gyroscope FIFO enabled value.
 * When set to 1, this bit enables gyroscope data samples to be
 * written into the FIFO buffer.
 * @return Current gyroscope FIFO enabled value
 * @see BMI270_RA_FIFO_CONFIG_1
 */
bool BMI270::getGyroFIFOEnabled() {
    I2CdevMod::readBits(devAddr, BMI270_RA_FIFO_CONFIG_1,
                     BMI270_FIFO_GYR_EN_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set gyroscope FIFO enabled value.
 * @param enabled New gyroscope FIFO enabled value
 * @see getGyroFIFOEnabled()
 * @see BMI270_RA_FIFO_CONFIG_1
 */
void BMI270::setGyroFIFOEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI270_RA_FIFO_CONFIG_1,
                   BMI270_FIFO_GYR_EN_BIT,
                   1, enabled ? 0x1 : 0);
}


/** Get magnetometer FIFO enabled value.
 * When set to 1, this bit enables magnetometer data samples to be
 * written into the FIFO buffer.
 * @return Current magnetometer FIFO enabled value
 * @see BMI270_RA_FIFO_CONFIG_1
 */
bool BMI270::getMagFIFOEnabled() {
    I2CdevMod::readBits(devAddr, BMI270_RA_FIFO_CONFIG_1,
                     BMI270_FIFO_MAG_EN_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set magnetometer FIFO enabled value.
 * @param enabled New magnetometer FIFO enabled value
 * @see getMagFIFOEnabled()
 * @see BMI270_RA_FIFO_CONFIG_1
 */
void BMI270::setMagFIFOEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI270_RA_FIFO_CONFIG_1,
                   BMI270_FIFO_MAG_EN_BIT,
                   1, enabled ? 0x1 : 0);
}

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer.
 *
 * In "headerless" FIFO mode, it is directly proportional to the number of
 * samples available given the set of sensor data bound to be stored in the
 * FIFO. See @ref getFIFOHeaderModeEnabled().
 *
 * @param outCount Current FIFO buffer size
 * @return Bool if value was read successfully
 * @see BMI270_RA_FIFO_LENGTH_0
 */
bool BMI270::getFIFOCount(uint16_t* outCount) {
    bool ok = I2CdevMod::readBytes(devAddr, BMI270_RA_FIFO_LENGTH_0, 2, buffer) >= 0;
    if (!ok) return false;
    *outCount = (((int16_t)buffer[1]) << 8) | buffer[0];
    return ok;
}

/** Reset the FIFO.
 * This command clears all data in the FIFO buffer.  It is recommended
 * to invoke this after reconfiguring the FIFO.
 *
 * @see BMI270_RA_CMD
 * @see BMI270_CMD_FIFO_FLUSH
 */
void BMI270::resetFIFO() {
    I2CdevMod::writeByte(devAddr, BMI270_RA_CMD, BMI270_CMD_FIFO_FLUSH);
}

/** Get FIFO Header-Mode enabled status.
 * When this bit is set to 0, the FIFO header-mode is disabled, and frames
 * read from the FIFO will be headerless (raw sensor data only).
 * When this bit is set to 1, the FIFO header-mode is enabled, and frames
 * read from the FIFO will include headers.
 *
 * For more information on the FIFO modes and data formats, please refer
 * to Section 2.5 of the BMI270 Data Sheet.
 *
 * @return Current FIFO Header-Mode enabled status
 * @see BMI270_RA_FIFO_CONFIG_1
 * @see BMI270_FIFO_HEADER_EN_BIT
 */
bool BMI270::getFIFOHeaderModeEnabled() {
    I2CdevMod::readBits(devAddr, BMI270_RA_FIFO_CONFIG_1,
                     BMI270_FIFO_HEADER_EN_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set FIFO Header-Mode enabled status.
 * @param enabled New FIFO Header-Mode enabled status
 * @see getFIFOHeaderModeEnabled()
 * @see BMI270_RA_FIFO_CONFIG_1
 * @see BMI270_FIFO_HEADER_EN_BIT
 */
void BMI270::setFIFOHeaderModeEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI270_RA_FIFO_CONFIG_1,
                   BMI270_FIFO_HEADER_EN_BIT,
                   1, enabled ? 0x1 : 0);
}

/** Get data frames from FIFO buffer.
 * This register is used to read and write data frames from the FIFO buffer.
 * Data is written to the FIFO in order of DATA register number (from lowest
 * to highest) corresponding to the FIFO data sources enabled (@see
 * getGyroFIFOEnabled() and getAccelFIFOEnabled()).
 *
 * The data frame format depends on the enabled data sources and also on
 * the FIFO header-mode setting (@see getFIFOHeaderModeEnabled()).
 *
 * It is strongly recommended, where possible, to read whole frames from the
 * FIFO.  Partially-read frames will be repeated until fully read out.
 *
 * If the FIFO buffer has filled to the point where subsequent writes may
 * cause data loss, the status bit ffull_int is automatically set to 1. This bit
 * is located in INT_STATUS[1]. When the FIFO buffer has overflowed, the oldest
 * data will be lost and new data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return a magic number
 * (@see BMI270_FIFO_DATA_INVALID) until new data is available. The user should
 * check FIFO_LENGTH to ensure that the FIFO buffer is not read when empty (see
 * @getFIFOCount()).
 *
 * @param data Data frames from FIFO buffer
 * @param length Buffer length
 * @return Bool if value was read successfully
 */
bool BMI270::getFIFOBytes(uint8_t *data, uint16_t length) {
    if (!length) return true;
    bool ok = I2CdevMod::readBytes(devAddr, BMI270_RA_FIFO_DATA, length, data) >= 0;
    return ok;
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see BMI270_RA_GYRO_X_L
 */
void BMI270::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    I2CdevMod::readBytes(devAddr, BMI270_RA_GYRO_X_L, 12, buffer);
    *gx = (((int16_t)buffer[1])  << 8) | buffer[0];
    *gy = (((int16_t)buffer[3])  << 8) | buffer[2];
    *gz = (((int16_t)buffer[5])  << 8) | buffer[4];
    *ax = (((int16_t)buffer[7])  << 8) | buffer[6];
    *ay = (((int16_t)buffer[9])  << 8) | buffer[8];
    *az = (((int16_t)buffer[11]) << 8) | buffer[10];
}

/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Output Data Rate
 * as configured by @see getAccelRate()
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Output Data Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale configured by
 * @setFullScaleAccelRange. For each full scale setting, the accelerometers'
 * sensitivity per LSB is shown in the table below:
 *
 * <pre>
 * Full Scale Range | LSB Sensitivity
 * -----------------+----------------
 * +/- 2g           | 8192 LSB/mg
 * +/- 4g           | 4096 LSB/mg
 * +/- 8g           | 2048 LSB/mg
 * +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see BMI270_RA_ACCEL_X_L
 */
void BMI270::getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    I2CdevMod::readBytes(devAddr, BMI270_RA_ACCEL_X_L, 6, buffer);
    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];
}

/** Get X-axis accelerometer reading.
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI270_RA_ACCEL_X_L
 */
int16_t BMI270::getAccelerationX() {
    I2CdevMod::readBytes(devAddr, BMI270_RA_ACCEL_X_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get Y-axis accelerometer reading.
 * @return Y-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI270_RA_ACCEL_Y_L
 */
int16_t BMI270::getAccelerationY() {
    I2CdevMod::readBytes(devAddr, BMI270_RA_ACCEL_Y_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get Z-axis accelerometer reading.
 * @return Z-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI270_RA_ACCEL_Z_L
 */
int16_t BMI270::getAccelerationZ() {
    I2CdevMod::readBytes(devAddr, BMI270_RA_ACCEL_Z_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get current internal temperature as a signed 16-bit integer.
 *  The resolution is typically 1/2^9 degrees Celcius per LSB, at an
 *  offset of 23 degrees Celcius.  For example:
 *
 * <pre>
 * Value    | Temperature
 * ---------+----------------
 * 0x7FFF   | 87 - 1/2^9 degrees C
 * ...      | ...
 * 0x0000   | 23 degrees C
 * ...      | ...
 * 0x8001   | -41 + 1/2^9 degrees C
 * 0x8000   | Invalid
 *
 * @param out Temperature reading in 16-bit 2's complement format
 * @return Bool if value was read successfully
 * @see BMI270_RA_TEMP_L
 */
bool BMI270::getTemperature(int16_t* out) {
    bool ok = I2CdevMod::readBytes(devAddr, BMI270_RA_TEMP_L, 2, buffer) >= 0;
    if (!ok) return false;
    *out = (((int16_t)buffer[1]) << 8) | buffer[0];
    return ok;
}

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Output Data Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale configured by
 * @setFullScaleGyroRange(). For each full scale setting, the gyroscopes'
 * sensitivity per LSB is shown in the table below:
 *
 * <pre>
 * Full Scale Range   | LSB Sensitivity
 * -------------------+----------------
 * +/- 125  degrees/s | 262.4 LSB/deg/s
 * +/- 250  degrees/s | 131.2 LSB/deg/s
 * +/- 500  degrees/s | 65.5  LSB/deg/s
 * +/- 1000 degrees/s | 32.8  LSB/deg/s
 * +/- 2000 degrees/s | 16.4  LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see BMI270_RA_GYRO_X_L
 */
void BMI270::getRotation(int16_t* x, int16_t* y, int16_t* z) {
    I2CdevMod::readBytes(devAddr, BMI270_RA_GYRO_X_L, 6, buffer);
    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];
}

/** Get X-axis gyroscope reading.
 * @return X-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI270_RA_GYRO_X_L
 */
int16_t BMI270::getRotationX() {
    I2CdevMod::readBytes(devAddr, BMI270_RA_GYRO_X_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get Y-axis gyroscope reading.
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI270_RA_GYRO_Y_L
 */
int16_t BMI270::getRotationY() {
    I2CdevMod::readBytes(devAddr, BMI270_RA_GYRO_Y_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get Z-axis gyroscope reading.
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI270_RA_GYRO_Z_L
 */
int16_t BMI270::getRotationZ() {
    I2CdevMod::readBytes(devAddr, BMI270_RA_GYRO_Z_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get magnetometer readings
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI270_RA_GYRO_Z_L
 */
void BMI270::getMagnetometer(int16_t* mx, int16_t* my, int16_t* mz) {
    I2CdevMod::readBytes(devAddr, BMI270_RA_MAG_X_L, 6, buffer);
    // *mx = (((int16_t)buffer[1])  << 8) | buffer[0];
    // *my = (((int16_t)buffer[3])  << 8) | buffer[2];
    // *mz = (((int16_t)buffer[5])  << 8) | buffer[4];
    *mx = ((int16_t)buffer[0] << 8) | buffer[1];
    *mz = ((int16_t)buffer[2] << 8) | buffer[3];
    *my = ((int16_t)buffer[4] << 8) | buffer[5];
}

void BMI270::getMagnetometerXYZBuffer(uint8_t* data) {
    I2CdevMod::readBytes(devAddr, BMI270_RA_MAG_X_L, 6, data);
}

/** Read a BMI270 register directly.
 * @param reg register address
 * @return 8-bit register value
 */
uint8_t BMI270::getRegister(uint8_t reg) {
    I2CdevMod::readByte(devAddr, reg, buffer);
    return buffer[0];
}

/** Write a BMI270 register directly.
 * @param reg register address
 * @param data 8-bit register value
 */
void BMI270::setRegister(uint8_t reg, uint8_t data) {
    I2CdevMod::writeByte(devAddr, reg, data);
}

bool BMI270::getGyroDrdy() {
    I2CdevMod::readBits(devAddr, BMI270_RA_STATUS, BMI270_STATUS_DRDY_GYR, 1, buffer);
    return buffer[0];
}

void BMI270::waitForGyroDrdy() {
    do {
        getGyroDrdy();
        if (!buffer[0]) delayMicroseconds(150);
    } while (!buffer[0]);
}

void BMI270::waitForAccelDrdy() {
    do {
        I2CdevMod::readBits(devAddr, BMI270_RA_STATUS, BMI270_STATUS_DRDY_ACC, 1, buffer);
        if (!buffer[0]) delayMicroseconds(150);
    } while (!buffer[0]);
}

void BMI270::waitForMagDrdy() {
    do {
        I2CdevMod::readBits(devAddr, BMI270_RA_STATUS, BMI270_STATUS_DRDY_MAG, 1, buffer);
        if (!buffer[0]) delay(5);
    } while (!buffer[0]);
}

bool BMI270::getSensorTime(uint32_t *v_sensor_time_u32) {
    bool ok = I2CdevMod::readBytes(devAddr, BMI270_RA_SENSORTIME, 3, buffer) >= 0;
    if (!ok) return false;
    *v_sensor_time_u32 = (uint32_t)(
        (((uint32_t)buffer[2]) << 16) |
        (((uint32_t)buffer[1]) << 8)  |
        ((uint32_t)buffer[0])
    );
    return ok;
}
