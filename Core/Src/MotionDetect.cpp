/*
 * MotionDetect.cpp
 *
 *  Created on: Aug 20, 2025
 *      Author: alexl
 */




#include "MotionDetect.hpp"
#include <cstring>
#include <cstdio>
#include <cmath>
extern "C" {
  #include "stm32l4xx_hal.h"
}

// LSM6DSL registers/values
static constexpr uint8_t REG_WHO_AM_I = 0x0F;
static constexpr uint8_t REG_CTRL1_XL = 0x10; //accelerometer control register
static constexpr uint8_t REG_CTRL2_G  = 0x11; //gyroscope control register
static constexpr uint8_t REG_CTRL3_C  = 0x12; //general control register
static constexpr uint8_t REG_OUTX_L_A = 0x28; //first data register for axxel
static constexpr uint8_t WHO_AM_I_VAL = 0x6A; // LSM6DSL whoami

MotionDetect::MotionDetect(I2C_HandleTypeDef* i2c, UART_HandleTypeDef* uart, uint8_t i2c_addr_7bit)
: i2c_(i2c), uart_(uart), addr8_(static_cast<uint16_t>(i2c_addr_7bit << 1)) {}

bool MotionDetect::probe() {
	uint8_t id = 0;
	if (!memRead(REG_WHO_AM_I, &id, 1)) return false;
	return id == WHO_AM_I_VAL;
}

bool MotionDetect::init() {
	//TODO: update configuration for this. might need help from chat

	// CTRL1_XL: ODR=104 Hz (0101), FS=±4g (10), BW default -> 0b0101'10'00 = 0x58
	if (!memWrite(REG_CTRL1_XL, 0x58)) return false;

	//disable the gyroscope. not needed
	if (!memWrite(REG_CTRL2_G, 0x00)) return false;

    // CTRL3_C: BDU=1 (block data update), IF_INC=1 (auto-increment), SW_RESET=0, BOOT=0
    // IF_INC bit=2, BDU bit=6 → 0b0100'0100 = 0x44
	if(!memWrite(REG_CTRL3_C, 0x44)) return false;

	return true;
}

void MotionDetect::update(){
	float ax, ay, az;
	if(!readAccel(ax, ay, az)){
		uartPrint("Accel read fail\r\n");
		return;
	}
		//attempt to read and update the variables (possible because of pointers), skip this read if fails

	float mag = sqrtf(ax*ax + ay*ay + az*az);

	//crudely isolate the vibration by subtracting 1g baseline
	float vibration = fabsf(mag - 1.0f);
	lastVibration_ = vibration;

	bool shaking = (vibration > threshold_);

	uint32_t now = HAL_GetTick();
	bool shouldLog = false;
    if (logOnChangeOnly_) {
        if ((shaking != lastState_) && (now - lastLog_ >= logIntervalMs_)) {
            shouldLog = true;
        }
    } else {
        if (now - lastLog_ >= logIntervalMs_) {
            shouldLog = true;
        }
    }


    if (shouldLog) {
        char buf[64];
        int n = snprintf(buf, sizeof(buf),
                         "%s (|a|-1g)=%.3f\r\n", shaking ? "SHAKING" : "QUIET", vibration);
        if (n > 0) uartPrint(buf);
        lastLog_ = now;
        lastState_ = shaking;
    }

    if (sampleDelayMs_) {
        HAL_Delay(sampleDelayMs_);
    }

}


void MotionDetect::setThreshold(float g_threshold)      { threshold_ = g_threshold; }
void MotionDetect::setLogIntervalMs(uint32_t ms)        { logIntervalMs_ = ms; }
void MotionDetect::setSampleDelayMs(uint32_t ms)        { sampleDelayMs_ = ms; }
void MotionDetect::setLogOnChangeOnly(bool enable)      { logOnChangeOnly_ = enable; }

bool MotionDetect::memRead(uint8_t reg, uint8_t* buf, uint16_t len) {
    return HAL_I2C_Mem_Read(i2c_, addr8_, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK;
}
bool MotionDetect::memWrite(uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(i2c_, addr8_, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK;
}

float MotionDetect::rawToG(int16_t raw) {
    // ±4g → 0.122 mg/LSB = 0.000122 g/LSB
    return raw * 0.000122f;
}

bool MotionDetect::readAccel(float& ax, float& ay, float& az) {
    uint8_t b[6];
    if (!memRead(REG_OUTX_L_A, b, 6)) return false;
    int16_t x = static_cast<int16_t>((b[1] << 8) | b[0]);
    int16_t y = static_cast<int16_t>((b[3] << 8) | b[2]);
    int16_t z = static_cast<int16_t>((b[5] << 8) | b[4]);
    ax = rawToG(x);
    ay = rawToG(y);
    az = rawToG(z);
    return true;
}

void MotionDetect::uartPrint(const char* s) {
    if (!uart_) return;
    HAL_UART_Transmit(uart_, (uint8_t*)s, strlen(s), 100);
}
