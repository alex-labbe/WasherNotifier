/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "../Inc/main.hpp"

#include "dfsdm.h"
#include "i2c.h"
#include "quadspi.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


static constexpr uint16_t LSM6DSL_ADDR = (0x6A << 1);

static constexpr uint8_t REG_WHO_AM_I  = 0x0F;  // expect 0x6A
static constexpr uint8_t REG_CTRL1_XL  = 0x10;  // accel ODR/FS
static constexpr uint8_t REG_CTRL3_C   = 0x12;  // IF_INC, BDU
static constexpr uint8_t REG_OUTX_L_XL = 0x28;  // accel data start

extern I2C_HandleTypeDef hi2c2; // struct for ic2 peripherals, calling in hi2c2
extern UART_HandleTypeDef huart1; // for logging

static constexpr float MOTION_THRESHOLD = 0.05f; //threshold for what to declare as movement
static uint32_t lastLog = 0; // in ms
static bool lastState = false; // false=quiet, true=shaking
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//reads len bytes starting at register reg from the accelerometer into buf
// reg is which register address to start reading from
// buf is where to store the bytes
// len is the number of bytes to read

//HAL_I2C_Mem_Read calls a read
// pass in pointer to which i2c peripheral to use
// pass in address to the accel
// 100 is the timeout
// 8 bit i2c memory address size
static HAL_StatusTypeDef i2c_mr(uint8_t reg, uint8_t* buf, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

static HAL_StatusTypeDef i2c_mw(uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static void uart_print(const char* s) {
  HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 100);
}

static bool lsm6dsl_init() {
  uint8_t who = 0;
  if (i2c_mr(REG_WHO_AM_I, &who, 1) != HAL_OK) {
    uart_print("I2C read WHO_AM_I failed\r\n");
    return false;
  }
  char buf[64];
  snprintf(buf, sizeof(buf), "WHO_AM_I=0x%02X\r\n", who);
  uart_print(buf);
  if (who != 0x6A) { // some boards use 0x6B addr; if you see 0x6B, change LSM6DSL_ADDR
    uart_print("Unexpected WHO_AM_I (expected 0x6A). Check addr/power.\r\n");
    // continue anyway so you can see what happens
  }

  // Enable auto-increment + BDU
  if (i2c_mw(REG_CTRL3_C, (1<<6) | (1<<2)) != HAL_OK) { // BDU=1, IF_INC=1
    uart_print("Write CTRL3_C failed\r\n"); return false;
  }

  // Accel ON: 104 Hz, ±4g
  uint8_t ctrl1 = (0b0101 << 4) | (0b01 << 2) | 0b00;
  if (i2c_mw(REG_CTRL1_XL, ctrl1) != HAL_OK) {
    uart_print("Write CTRL1_XL failed\r\n"); return false;
  }

  uart_print("Accel init OK (104Hz, +/-4g)\r\n");
  return true;
}

static inline float raw_to_g(int16_t raw) {
    return raw * 0.000122f; // ±4g scale → 0.122 mg/LSB
}

// using pointers here to directly update ax,.. in the logMotion function
static bool readAccel(float& ax, float& ay, float& az) { //returns true if succeeded, otherwise false
    uint8_t buf[6]; //inits a temp array to hold 6 bytes from the sensor
    if (i2c_mr(0x28, buf, 6) != HAL_OK) return false; // reads starting at 0x28, the first accel output register
    int16_t x = (int16_t)((buf[1] << 8) | buf[0]); // combine registers for x into a signed 16-bit value
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t z = (int16_t)((buf[5] << 8) | buf[4]);
    ax = raw_to_g(x); //convert raw counts to g units with our helper above
    ay = raw_to_g(y);
    az = raw_to_g(z);
    return true; //success
}

static void logMotion() { //motion logger
    float ax, ay, az; // init variables
    if (!readAccel(ax, ay, az)) return; //attempt a read, if fails, skip logging this time

    //compute the magnitude of acceleration vector
    float mag = sqrtf(ax*ax + ay*ay + az*az);

    // Isolate vibration by subtracting 1g baseline
    // crude way to estimate deviation from gravity
    float vibration = fabsf(mag - 1.0f);

    // isShaking if the calculated vibration is above our motion threshold
    bool shaking = (vibration > MOTION_THRESHOLD);

    //milliseconds since boot from HAL's system tick
    uint32_t now = HAL_GetTick();
    // if state change from last logged state
    if (shaking != lastState && now - lastLog > 500) { // log on change, 0.5s min
        const char* msg = shaking ? "SHAKING\r\n" : "QUIET\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        lastLog = now;
        lastState = shaking;
    }
}


class ButtonHandler {
public:
    ButtonHandler() { instance_ = this; }

    static void onISR() {
    	if (instance_) instance_->onButtonPressedISR();
    }

    void processPressInMainLoop() {
        if (pressedFlag_) {
            pressedFlag_ = false;
            onButtonPressed(); // your real work
        }
    }

private:
    void onButtonPressedISR() {
        uint32_t now = HAL_GetTick();
        if (now - lastTick_ > 50) { // debounce 50 ms
            pressedFlag_ = true;
            lastTick_ = now;
        }
    }

    void onButtonPressed() {
        // TODO: your action (toggle LED, send msg, etc.)
    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    }

    static ButtonHandler* instance_;
    volatile bool pressedFlag_{false};
    uint32_t lastTick_{0};
};

ButtonHandler* ButtonHandler::instance_ = nullptr;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  ButtonHandler button;

  const char* startMsg = "Motion logger started\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)startMsg, strlen(startMsg), 100);
  bool acc_ok = lsm6dsl_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  logMotion();
	  HAL_Delay(50);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13){
		ButtonHandler::onISR();
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
