/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2023 S.J. Remington & SlimeVR contributors

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

#include "bmi323sensor.h"
#include "bmi323.h"
#include "Wire.h"

BMI3_INTF_RET_TYPE bmi3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    // Get sensor address from the interface pointer (intf_ptr)
    BMI323Sensor* sensorInstance = static_cast<BMI323Sensor*>(intf_ptr);
    uint8_t dev_addr = sensorInstance->address;

    // Read data from the sensor
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.endTransmission();
    Wire.requestFrom(dev_addr, len);
    for (auto i = 0u; i < len; i++) {
        reg_data[i] = Wire.read();
    }
    return 0;
}

BMI3_INTF_RET_TYPE bmi3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    // Get sensor address from the interface pointer (intf_ptr)
    BMI323Sensor* sensorInstance = static_cast<BMI323Sensor*>(intf_ptr);
    uint8_t dev_addr = sensorInstance->address;

    // Write data to the sensor
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    for (auto i = 0u; i < len; i++) {
        Wire.write(reg_data[i]);
    }
    Wire.endTransmission();
    return 0;
}

void bmi3_delay_us(uint32_t period, void *) {
    delayMicroseconds(period);
}

void BMI323Sensor::motionSetup() {
    // Create an instance of the struct
    struct bmi3_dev bmi323;

    // Assign values to its members
    bmi323.chip_id = sensorId;
    bmi323.intf = BMI3_I2C_INTF;
    bmi323.read = bmi3_i2c_read;
    bmi323.write = bmi3_i2c_write;
    bmi323.delay_us = bmi3_delay_us;
    bmi323.intf_ptr = this;

    // Initialize the sensor
    int8_t initResult = bmi323_init(&bmi323);
    if (initResult == BMI3_OK) {
        Serial.println("BMI323 initialized on address 0x" + String(address, HEX));
    } else {
        Serial.println("BMI323 initialization failed");
    }

    // Get status
    uint16_t status = 0;
    int8_t statusResult = bmi323_get_int2_status(&status, &bmi323);
    if (statusResult == BMI3_OK) {
        Serial.println("BMI323 status: 0x" + String(status, HEX));
    } else {
        Serial.println("BMI323 status read failed");
    }

    // Configure sensors
    bmi3_sens_config sensorsConfig[2];

    // Configure accelerometer
    sensorsConfig[0].type = BMI323_ACCEL;
    bmi3_accel_config accelConfig;
    accelConfig.odr = BMI3_ACC_ODR_100HZ; // Data rate
    accelConfig.bwp = BMI3_ACC_AVG2; // Smoothing
    accelConfig.acc_mode = BMI3_ACC_MODE_NORMAL; // Performance mode
    accelConfig.range = BMI3_ACC_RANGE_8G; // Max G force
    accelConfig.avg_num = 10; // Averaging
    sensorsConfig[0].cfg.acc = accelConfig;

    // Configure gyroscope
    sensorsConfig[1].type = BMI323_GYRO;
    bmi3_gyro_config gyroConfig;
    gyroConfig.odr = BMI3_GYR_ODR_200HZ; // Data rate
    gyroConfig.bwp = BMI3_GYR_AVG2; // Smoothing
    gyroConfig.gyr_mode = BMI3_GYR_MODE_NORMAL; // Performance mode
    gyroConfig.range = BMI3_GYR_RANGE_500DPS; // Sensitivity
    gyroConfig.avg_num = 10; // Averaging
    sensorsConfig[1].cfg.gyr = gyroConfig;

    // Apply the configuration
    bmi323_set_sensor_config(sensorsConfig, 2, &bmi323);

    // Try to read temperature data
    uint16_t rawTemp = 0;
    int8_t tempResult = bmi323_get_temperature_data(&rawTemp, &bmi323);
    if (tempResult == BMI3_OK) {
        float temp = (float)(((float)((int16_t)rawTemp)) / 512.0) + 23.0;
        Serial.print("Temperature: ");
        Serial.println(temp);

        Serial.print("Raw temperature: ");
        Serial.println(rawTemp);
    } else {
        Serial.println("Temperature read failed");
    }
}

void BMI323Sensor::motionLoop() {

}

void BMI323Sensor::sendData() {

}

void BMI323Sensor::startCalibration(int calibrationType) {

}