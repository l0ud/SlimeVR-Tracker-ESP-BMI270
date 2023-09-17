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
#include "GlobalVars.h"

BMI3_INTF_RET_TYPE bmiI2CRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
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

BMI3_INTF_RET_TYPE bmiI2CWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
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

void bmiDelayUs(uint32_t period, void *) {
    delayMicroseconds(period);
}

void BMI323Sensor::extractFrame(uint8_t *data, bmi3_fifo_sens_axes_data &accelData, bmi3_fifo_sens_axes_data &gyroData, bmi3_fifo_temperature_data &tempData) {
    /**
     * Data is an array of uint_8t divided into 18 bytes chunks
     * The first 3 words (6 bytes) are the accelerometer data (X, Y, Z)
     * The next 3 words (6 bytes) are the gyroscope data (X, Y, Z)
     * The next word (2 bytes) is the temperature data
     * The next word (2 bytes) is the time data
     * The last word (2 bytes) is empty
     * 
     * The least significant byte is first
     * The most significant byte is last
    */

   /*
    printf("\nframe ");
    for (int i=0; i<6+6+2+2; i++) {
        printf("%02x", data[i]);
    }
    printf("\n");
    */
    // Unpack accelerometer
    uint16_t fifoByteIndex = 0;
    accelData.x = (int16_t)((data[fifoByteIndex + 1] << 8) | data[fifoByteIndex]);
    accelData.y = (int16_t)((data[fifoByteIndex + 3] << 8) | data[fifoByteIndex + 2]);
    accelData.z = (int16_t)((data[fifoByteIndex + 5] << 8) | data[fifoByteIndex + 4]);
    if (accelData.x == BMI3_FIFO_ACCEL_DUMMY_FRAME) {
        Serial.println("BMI323 ACCEL dummy frame");
    }
    fifoByteIndex += 6;

    // Unpack gyroscope data
    gyroData.x = (int16_t)((data[fifoByteIndex + 1] << 8) | data[fifoByteIndex]);
    gyroData.y = (int16_t)((data[fifoByteIndex + 3] << 8) | data[fifoByteIndex + 2]);
    gyroData.z = (int16_t)((data[fifoByteIndex + 5] << 8) | data[fifoByteIndex + 4]);
    if (gyroData.x == BMI3_FIFO_GYRO_DUMMY_FRAME) {
        Serial.println("BMI323 GRYO dummy frame");
    }
    fifoByteIndex += 6;

    // Unpack temperature data
    tempData.temp_data = (int16_t)((data[fifoByteIndex + 1] << 8) | data[fifoByteIndex]);
    if (tempData.temp_data == BMI3_FIFO_TEMP_DUMMY_FRAME) {
        Serial.println("BMI323 TEMP dummy frame");
    }
    // Note: The time data (2 bytes) is not being extracted in this example.
    //fifoByteIndex += 2;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsbToDps(int16_t val, float dps, uint8_t bit_width) {
    double power = 2;
    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
    return (dps / (half_scale)) * (val);
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsbToMps2(int16_t val, int8_t g_range, uint8_t bit_width) {
    double power = 2;
    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
    return (CONST_EARTH_GRAVITY * val * g_range) / half_scale;
}

/* float getTemperature(struct bmi3_dev bmi323) {
    float temp = 0;
    uint16_t rawTemp = 0;
    uint16_t result = bmi323_get_temperature_data(&rawTemp, &bmi323);
    if (result == BMI3_OK) {
        float temp = (float)(((float)((int16_t)rawTemp)) / 512.0) + 23.0;
        Serial.print("Temperature: " + String(temp) + "°C");
    } else {
        Serial.println("Temperature read failed");
    }

    return temp;
} */

void BMI323Sensor::motionSetup() {
    // Assign values to its members
    bmi323.chip_id = sensorId;
    bmi323.intf = BMI3_I2C_INTF;
    bmi323.read = bmiI2CRead;
    bmi323.write = bmiI2CWrite;
    bmi323.delay_us = bmiDelayUs;
    bmi323.intf_ptr = this;

    int8_t result;

    // Initialize the sensor
    result = bmi323_init(&bmi323);
    if (result == BMI3_OK) {
        Serial.println("BMI323 initialized on address 0x" + String(address, HEX));
    } else {
        Serial.println("BMI323 initialization failed");
    }

    // Get status
    uint16_t status = 0;
    result = bmi323_get_int2_status(&status, &bmi323);
    if (result == BMI3_OK) {
        Serial.println("BMI323 status: 0x" + String(status, HEX));
    } else {
        Serial.println("BMI323 status read failed");
    }

    // Configure sensors
    struct bmi3_sens_config sensorsConfig[2];

    // Configure accelerometer
    sensorsConfig[0].type = BMI323_ACCEL;
    struct bmi3_accel_config accelConfig;
    accelConfig.odr = BMI3_ACC_ODR_25HZ; // Data rate
    accelConfig.bwp = BMI3_ACC_AVG2; // Smoothing
    accelConfig.acc_mode = BMI3_ACC_MODE_NORMAL; // Performance mode
    accelConfig.range = BMI3_ACC_RANGE_8G; // Max G force
    accelConfig.avg_num = 10; // Averaging
    sensorsConfig[0].cfg.acc = accelConfig;

    // Configure gyroscope
    sensorsConfig[1].type = BMI323_GYRO;
    struct bmi3_gyro_config gyroConfig;
    gyroConfig.odr = BMI3_GYR_ODR_25HZ; // Data rate
    gyroConfig.bwp = BMI3_GYR_AVG2; // Smoothing
    gyroConfig.gyr_mode = BMI3_GYR_MODE_NORMAL; // Performance mode
    gyroConfig.range = BMI3_GYR_RANGE_500DPS; // Sensitivity
    gyroConfig.avg_num = 10; // Averaging
    sensorsConfig[1].cfg.gyr = gyroConfig;

    // Apply the configuration
    result = bmi323_set_sensor_config(sensorsConfig, 2, &bmi323);
    if (result == BMI3_OK) {
        Serial.println("BMI323 sensors configured");
    } else {
        Serial.println("BMI323 sensor configuration failed");
    }

    // Enable FIFO
    result = bmi323_set_fifo_config(BMI3_FIFO_ACC_EN | BMI3_FIFO_GYR_EN | BMI3_FIFO_TEMP_EN | BMI3_FIFO_TIME_EN, 1, &bmi323);
    if (result == BMI3_OK) {
        Serial.println("BMI323 FIFO enabled");
    } else {
        Serial.println("BMI323 FIFO enable failed");
    }

    fifoFrame.data = fifoData;

    // Tell SlimeVR that the sensor is working and ready
    m_status = SensorStatus::SENSOR_OK;
    working = true;
}

void BMI323Sensor::motionLoop() {
    Serial.println("------------------------- motion loop -------------------------");

    // First we get the length of the data available in the fifo pile
    bmi323_get_fifo_length(&fifoFrame.available_fifo_len, &bmi323);

    // We set the length of the frame to the length of the data available times 2 (because each data point is 2 bytes)
    fifoFrame.length = (uint16_t)(fifoFrame.available_fifo_len * 2);
    constexpr auto frame_length_bytes = 6+6+2+2;
    constexpr auto dummy_bytes = 2;


    if (fifoFrame.length >= frame_length_bytes) {
        // Serial.print("\x1B[2J"); // Clear the screen
        // Serial.print("\x1B[H");
        m_Logger.info("BMI323 FIFO length bytes: %d", fifoFrame.length);
        // calculate our read
        fifoFrame.length = std::min(fifoFrame.length, static_cast<uint16_t>(I2C_BUFFER_LENGTH - dummy_bytes)) / frame_length_bytes * frame_length_bytes + dummy_bytes;
        m_Logger.info("BMI323 FIFO read bytes: %d", fifoFrame.length);

        // Then we read the data from the fifo pile
        uint16_t result = bmi323_read_fifo_data(&fifoFrame, &bmi323);

        if (result == BMI3_OK) {
            const auto fifo_layload_len = fifoFrame.length - dummy_bytes;
            uint8_t *fifo_payload = fifoFrame.data + dummy_bytes;

            const auto frame_count = fifo_layload_len / frame_length_bytes;

            for (auto i=0; i<frame_count; i++) {
                // Extract the data from the frame
                struct bmi3_fifo_sens_axes_data testAccel;
                struct bmi3_fifo_sens_axes_data testGyro;
                struct bmi3_fifo_temperature_data testTemp;

                extractFrame(&fifo_payload[i*frame_length_bytes], testAccel, testGyro, testTemp);
                float x = lsbToMps2(testAccel.x, 8, bmi323.resolution);
                float y = lsbToMps2(testAccel.y, 8, bmi323.resolution);
                float z = lsbToMps2(testAccel.z, 8, bmi323.resolution);
                Serial.println("["+ String(i) +"] Acceleration: " + String(x) + ", " + String(y) + ", " + String(z) + "            ");

                x = lsbToDps(testGyro.x, (float)500, bmi323.resolution);
                y = lsbToDps(testGyro.y, (float)500, bmi323.resolution);
                z = lsbToDps(testGyro.z, (float)500, bmi323.resolution);
                Serial.println("Gyroscope: " + String(x) + ", " + String(y) + ", " + String(z) + "           ");

                float temp = (float)(((float)((int16_t)testTemp.temp_data)) / 512.0) + 23.0;
                Serial.println("Temperature: " + String(temp) + "°C" + "           ");
            }

            Serial.print("\r");
            Serial.println("------------------------- Bosch way -------------------------");
            bmi323_extract_accel(accelData, &fifoFrame, &bmi323);
            bmi323_extract_gyro(gyroData, &fifoFrame, &bmi323);
            bmi323_extract_temperature(tempData, &fifoFrame, &bmi323);            
            for (auto i=0; i<frame_count; i++) {
                float x = lsbToMps2(accelData[i].x, 8, bmi323.resolution);
                float y = lsbToMps2(accelData[i].y, 8, bmi323.resolution);
                float z = lsbToMps2(accelData[i].z, 8, bmi323.resolution);
                Serial.println("["+ String(i) +"] BMI323 accel data: " + String(x) + ", " + String(y) + ", " + String(z));

                x = lsbToDps(gyroData[i].x, (float)500, bmi323.resolution);
                y = lsbToDps(gyroData[i].y, (float)500, bmi323.resolution);
                z = lsbToDps(gyroData[i].z, (float)500, bmi323.resolution);
                Serial.println("BMI323 gyro data: " + String(x) + ", " + String(y) + ", " + String(z));

                float temp = (float)(((float)((int16_t)tempData[i].temp_data)) / 512.0) + 23.0;
                Serial.println("BMI323 temperature data: " + String(temp) + "°C");
            }
        }
        else {
            Serial.println("BMI323 fifo data read failed");
        }
    }

    delay(200);
}

void BMI323Sensor::sendData() {
    // networkConnection.sendSensorData(sensorId, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

void BMI323Sensor::startCalibration(int calibrationType) {
    // bmi323_perform_gyro_sc
}