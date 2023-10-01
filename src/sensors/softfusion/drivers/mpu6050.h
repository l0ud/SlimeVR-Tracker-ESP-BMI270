#pragma once

#include <cstdint>
#include <array>
#include <algorithm>

#include <MPU6050.h>

#define MPU6050_GYRO_SENS_250 131.0
#define MPU6050_GYRO_SENS_500 65.5
#define MPU6050_GYRO_SENS_1000 32.8
#define MPU6050_GYRO_SENS_2000 16.4

#define MPU6050_ACCEL_SENS_2G 16384.0
#define MPU6050_ACCEL_SENS_4G 8192.0
#define MPU6050_ACCEL_SENS_8G 4096.0
#define MPU6050_ACCEL_SENS_16G 2048.0

// Swap bytes because MPU is big-endian
#define MPU6050_CONVERT_WORD(x) (x = ((x << 8) | ((x >> 8) & 0xFF)))


namespace SlimeVR::Sensors::SoftFusion::Drivers
{

// Driver uses acceleration range at 8g
// and gyroscope range at 1000dps

template <template<uint8_t> typename I2CImpl>
struct MPU6050
{
    struct FifoSample {
        uint8_t accel_x_h, accel_x_l;
        uint8_t accel_y_h, accel_y_l;
        uint8_t accel_z_h, accel_z_l;

        // We don't need temperature in FIFO
        // uint8_t temp_h, temp_l;

        uint8_t gyro_x_h, gyro_x_l;
        uint8_t gyro_y_h, gyro_y_l;
        uint8_t gyro_z_h, gyro_z_l;
    };
    #define MPU6050_FIFO_VALUE(fifo, name) (((int16_t)fifo->name##_h << 8) | ((int16_t)fifo->name##_l))

    static constexpr uint8_t DevAddr = 0x68;
    static constexpr auto Name = "MPU-6050";
    static constexpr auto Type = IMU_SFMPU6050;

    static constexpr float Freq = 250;

    static constexpr float GyrTs = 1.0 / Freq;
    static constexpr float AccTs = 1.0 / Freq;
    static constexpr float MagTs = 1.0 / Freq;

    static constexpr float GyroSensitivity = MPU6050_GYRO_SENS_1000;
    static constexpr float AccelSensitivity = MPU6050_ACCEL_SENS_8G;

    using i2c = I2CImpl<DevAddr>;

    struct Regs {
        struct WhoAmI {
            static constexpr uint8_t reg = 0x75;
            static constexpr uint8_t value = 0x68;
        };

        struct UserCtrl {
            static constexpr uint8_t reg = 0x6A;
            static constexpr uint8_t fifoResetValue = (1 << MPU6050_USERCTRL_FIFO_EN_BIT) | (1 << MPU6050_USERCTRL_FIFO_RESET_BIT);
        };

        struct GyroConfig {
            static constexpr uint8_t reg = 0x1b;
            static constexpr uint8_t value = 0b10 << 3; // 1000dps
        };

        struct AccelConfig {
            static constexpr uint8_t reg = 0x1c;
            static constexpr uint8_t value = 0b10 << 3; // 8g
        };

        static constexpr uint8_t OutTemp = MPU6050_RA_TEMP_OUT_H;

        static constexpr uint8_t IntStatus = MPU6050_RA_INT_STATUS;

        static constexpr uint8_t FifoCount = MPU6050_RA_FIFO_COUNTH;
        static constexpr uint8_t FifoData = MPU6050_RA_FIFO_R_W;
    };

    void resetFIFO() {
        i2c::writeReg(Regs::UserCtrl::reg, Regs::UserCtrl::fifoResetValue);
    }

    bool initialize()
    {
        // Reset
        i2c::writeReg(MPU6050_RA_PWR_MGMT_1, 0x80); //PWR_MGMT_1: reset with 100ms delay (also disables sleep)
        delay(100);
        i2c::writeReg(MPU6050_RA_SIGNAL_PATH_RESET, 0x07); // full SIGNAL_PATH_RESET: with another 100ms delay
        delay(100);
        
        // Configure
        i2c::writeReg(MPU6050_RA_PWR_MGMT_1,   0x01); // 0000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
        i2c::writeReg(MPU6050_RA_USER_CTRL,    0x00); // 0000 0000 USER_CTRL: Disable FIFO / I2C master / DMP
        i2c::writeReg(MPU6050_RA_INT_ENABLE,   0x10); // 0001 0000 INT_ENABLE: only FIFO overflow interrupt
        i2c::writeReg(Regs::GyroConfig::reg, Regs::GyroConfig::value);
        i2c::writeReg(Regs::AccelConfig::reg, Regs::AccelConfig::value);
        i2c::writeReg(MPU6050_RA_CONFIG,       0x02); // 0000 0010 CONFIG: No EXT_SYNC_SET, DLPF set to 98Hz(also lowers gyro output rate to 1KHz)
        i2c::writeReg(MPU6050_RA_SMPLRT_DIV,   0x03); // 0000 0011 SMPLRT_DIV: Divides the internal sample rate 250Hz (Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))

        i2c::writeReg(MPU6050_RA_FIFO_EN,      0x78); // 0111 1000 FIFO_EN: All gyro axes + Accel

        resetFIFO();

        return true;
    }

    float getDirectTemp() const
    {
        auto value = i2c::readReg16(Regs::OutTemp);
        MPU6050_CONVERT_WORD(value);

        float result = (static_cast<int16_t>(value) / 340.0f) + 36.53f;

        return result;
    }

    template <typename AccelCall, typename GyroCall>
    void bulkRead(AccelCall &&processAccelSample, GyroCall &&processGyroSample) {
        const auto status = i2c::readReg(Regs::IntStatus);

        if (status & (1 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) {
            // Overflows make it so we lose track of which packet is which
            // This necessitates a reset
            printf("FIFO overflow!\n");
            resetFIFO();
            return;
        }

        std::array<uint8_t, 12 * 10> readBuffer; // max 10 packages of 12byte values (sample) of data form fifo
        auto byteCount = i2c::readReg16(Regs::FifoCount);
        MPU6050_CONVERT_WORD(byteCount);

        auto readBytes = min(static_cast<size_t>(byteCount), readBuffer.size()) / sizeof(FifoSample) * sizeof(FifoSample);
        if (!readBytes) {
            return;
        }

        i2c::readBytes(Regs::FifoData, readBytes, readBuffer.data());
        for (auto i = 0u; i < readBytes; i += sizeof(FifoSample)) {
            const FifoSample *sample = reinterpret_cast<FifoSample *>(&readBuffer[i]);
            
            int16_t xyz[3];

            xyz[0] = MPU6050_FIFO_VALUE(sample, accel_x);
            xyz[1] = MPU6050_FIFO_VALUE(sample, accel_y);
            xyz[2] = MPU6050_FIFO_VALUE(sample, accel_z);
            processAccelSample(xyz, AccTs);

            xyz[0] = MPU6050_FIFO_VALUE(sample, gyro_x);
            xyz[1] = MPU6050_FIFO_VALUE(sample, gyro_y);
            xyz[2] = MPU6050_FIFO_VALUE(sample, gyro_z);
            processGyroSample(xyz, GyrTs);
        }
    }


};

} // namespace
