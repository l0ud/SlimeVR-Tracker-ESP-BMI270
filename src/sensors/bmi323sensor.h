/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 TheDevMinerTV

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

#ifndef SENSORS_BMI323_H
#define SENSORS_BMI323_H

#include "sensor.h"
#include "bmi323.h"

class BMI323Sensor : public Sensor
{
public:
    BMI323Sensor(uint8_t id, uint8_t address, float rotation, uint8_t sclPin, uint8_t sdaPin) :
        Sensor("BMI323Sensor", IMU_BMI323, id, address, rotation, sclPin, sdaPin), address(address) {};
    ~BMI323Sensor(){};
    
    SensorStatus getSensorState() override final {
        return m_status;
    }
    void motionSetup() override final;
    void motionLoop() override final;
    void sendData() override final;
    void startCalibration(int calibrationType) override final;

    uint8_t address;
    SensorStatus m_status = SensorStatus::SENSOR_OFFLINE;

private:
    bmi3_dev bmi323;

    uint16_t fifoByteIndex = 0;
    struct bmi3_fifo_frame fifoFrame = { 0 };
    uint8_t fifoData[1024] = { 0 };
    struct bmi3_fifo_sens_axes_data accelData[170]; // 2048 / BMI3_LENGTH_FIFO_ACC
    struct bmi3_fifo_sens_axes_data gyroData[170]; // 2048 / BMI3_LENGTH_FIFO_GYR
    struct bmi3_fifo_temperature_data tempData[170]; // 2048 / BMI3_LENGTH_FIFO_ACC -> Temperature runs based on Accel
};

#endif