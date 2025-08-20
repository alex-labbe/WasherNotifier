/*
 * MotionDetect.hpp
 *
 *  Created on: Aug 20, 2025
 *      Author: alexl
 */

#ifndef INC_MOTIONDETECT_HPP_
#define INC_MOTIONDETECT_HPP_

#pragma once
#include <cstdint>

// Forward declarations for HAL handles provided by CubeMX
extern "C" {
  #include "i2c.h"
  #include "usart.h"
}

class MotionDetect {
public:
    explicit MotionDetect(I2C_HandleTypeDef* i2c, UART_HandleTypeDef* uart,
                          uint8_t i2c_addr_7bit = 0x6A);

    bool probe(); //read WHO_AM_I

    bool init(); //Init and configure accel

    // Call in main loop
    void update();           // read accel, compute vibration, log per policy


    //TUNING TODO: update these
    void setThreshold(float g_threshold);          // default 0.05 g
    void setLogIntervalMs(uint32_t ms);            // default 500 ms
    void setSampleDelayMs(uint32_t ms);            // TODO: OPTIONAL: if you want the class to sleep
    void setLogOnChangeOnly(bool enable);          // default true

    // State query
    bool isShaking() const { return lastState_; }
    float lastVibration() const { return lastVibration_; }


private:
    bool memRead(uint8_t reg, uint8_t *buf, uint16_t len);
    bool memWrite(uint8_t reg, uint8_t val);

    static float rawToG(int16_t raw);

    bool readAccel(float &ax, float &ay, float &az);

    // logging
    void uartPrint(const char* s);

private:
    I2C_HandleTypeDef* i2c_;
    UART_HandleTypeDef* uart_;
    uint16_t addr8_; // 8-bit address for HAL
    float threshold_ = 0.05f;      // g
    uint32_t logIntervalMs_ = 500; // ms
    uint32_t sampleDelayMs_ = 0;   // optional delay inside update()
    bool logOnChangeOnly_ = true;

    // runtime state
    uint32_t lastLog_ = 0;
    bool lastState_ = false;       // false=quiet, true=shaking
    float lastVibration_ = 0.0f;
};

#endif /* INC_MOTIONDETECT_HPP_ */
