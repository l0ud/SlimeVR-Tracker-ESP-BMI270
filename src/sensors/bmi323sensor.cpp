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

int bmi323_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.endTransmission();
    Wire.requestFrom(dev_addr, len);
    for (int i = 0; i < len; i++) {
        reg_data[i] = Wire.read();
    }
    return 0;
}

bmi3_write_fptr_t bmi323_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    for (int i = 0; i < len; i++) {
        Wire.write(reg_data[i]);
    }
    Wire.endTransmission();
    return 0;
}

void BMI323Sensor::motionSetup() {
    // Create an instance of the struct
    struct bmi3_dev bmi323;

    // Assign values to its members
    bmi323.chip_id = sensorId;
    bmi323.intf = BMI3_I2C_INTF;
    bmi323.read = bmi323_i2c_read;
    bmi323.write = bmi323_i2c_write;
    

    // Initialize the sensor
    int8_t result = bmi323_init(&bmi323);
    if (result == BMI3_OK) {
        Serial.println("BMI323 initialized");
    } else {
        Serial.println("BMI323 initialization failed");
    }
}

void BMI323Sensor::motionLoop() {

}

void BMI323Sensor::sendData() {

}

void BMI323Sensor::startCalibration(int calibrationType) {

}