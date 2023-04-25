/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "stm32l4xx_hal_conf.h"
#include "custom_bus.h"
#include "lsm6dsl.h"
#include "lsm6dsl_reg.h"
#include <stdio.h>


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
extern I2C_HandleTypeDef hi2c2;
LSM6DSL_Object_t gyro_obj;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int32_t platform_write(uint16_t Addr, uint16_t Reg, uint8_t *pBuffer, uint16_t Length) {
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Write(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Length, 1000);
  return (int32_t)status;
}

int32_t platform_read(uint16_t Addr, uint16_t Reg, uint8_t *pBuffer, uint16_t Length) {
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Length, 1000);
  return (int32_t)status;
}

int32_t MX_I2C2_Init_Wrapper(void) {
  //MX_I2C2_Init(&hi2c2);
	hi2c2.Instance = I2C2;
	  hi2c2.Init.Timing = 0x00707CBB;
	  hi2c2.Init.OwnAddress1 = 0;
	  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c2.Init.OwnAddress2 = 0;
	  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	  {
	    Error_Handler();
	  }
  return 0;
}

//void init_lsm6dsl_gyroscope(void) {
//  //LSM6DSL_Object_t gyro_obj;
//  LSM6DSL_IO_t io_ctx;
//
//  // Set I2C address and interface
//  io_ctx.BusType = LSM6DSL_I2C_BUS;
//  io_ctx.Address = LSM6DSL_I2C_ADD_L;
//  io_ctx.Init = MX_I2C2_Init_Wrapper;
//  io_ctx.WriteReg = platform_write;
//  io_ctx.ReadReg = platform_read;
//  LSM6DSL_RegisterBusIO(&gyro_obj, &io_ctx);
//
//  MX_I2C2_Init(&hi2c2);
//
//  // Enable gyroscope
//  LSM6DSL_Init(&gyro_obj);
//}
//
void send_data_usart(const char *data) {
  HAL_UART_Transmit(&huart1, (uint8_t *)data, strlen(data), 1000); // Replace huartX with your USART handler
}

void init_lsm6dsl_gyroscope(void) {
	  LSM6DSL_Object_t gyro_obj;
	  LSM6DSL_IO_t io_ctx;
	  uint8_t who_am_i;
	  uint8_t read_addr = 0xAF;
	  uint8_t write_addr = 0xAE;

	  // Set I2C address and interface
	  io_ctx.BusType = LSM6DSL_I2C_BUS;
	  io_ctx.Address = read_addr;
	  io_ctx.Init = MX_I2C2_Init_Wrapper;
	  io_ctx.WriteReg = platform_write;
	  io_ctx.ReadReg = platform_read;
	  LSM6DSL_RegisterBusIO(&gyro_obj, &io_ctx);

	  MX_I2C2_Init(&hi2c2);

	  // Check if the device is available
	  if (HAL_I2C_IsDeviceReady(&hi2c2, write_addr, 3, 100) != HAL_OK) {
	    Error_Handler(); // Implement an error handler if the device is not ready
	  }

	  // Read the WHO_AM_I register
	  platform_write(write_addr, 0x0F, NULL, 0); // Set the register address for WHO_AM_I
	  platform_read(read_addr, 0x0F, &who_am_i, 1); // Read a single byte from the read register

	  if (who_am_i != 0x6A) { // Check if the device is detected
	    Error_Handler(); // Implement an error handler if the device is not detected
	  }

	  // Enable gyroscope
	  LSM6DSL_Init(&gyro_obj);
}

void read_and_send_who_am_i(void) {
  LSM6DSL_IO_t io_ctx;
  uint8_t who_am_i = 0;
  char msg[64];

  // Set I2C address and interface
  io_ctx.BusType = LSM6DSL_I2C_BUS;
  io_ctx.Address = LSM6DSL_I2C_ADD_L;
  io_ctx.Init = MX_I2C2_Init_Wrapper;
  io_ctx.WriteReg = platform_write;
  io_ctx.ReadReg = platform_read;

  LSM6DSL_RegisterBusIO(&gyro_obj, &io_ctx); // Move this call to this function

  // Read the WHO_AM_I register
  LSM6DSL_ReadID(&gyro_obj, &who_am_i);

  // Check if the device is available
  if (who_am_i != LSM6DSL_ID) {
    Error_Handler(); // Implement an error handler if the device is not detected
  }

  // Convert the WHO_AM_I value to a string and send it to USART
  snprintf(msg, sizeof(msg), "WHO_AM_I: 0x%02X\r\n", who_am_i);
  send_data_usart(msg);
}
//
//void read_and_send_who_am_i(void) {
//  //LSM6DSL_Object_t gyro_obj;
//  LSM6DSL_IO_t io_ctx;
//  uint8_t who_am_i = 0;
//  char msg[64];
//
//  // Set I2C address and interface
//  io_ctx.BusType = LSM6DSL_I2C_BUS;
//  io_ctx.Address = LSM6DSL_I2C_ADD_L;
//  io_ctx.Init = MX_I2C2_Init_Wrapper;
//  io_ctx.WriteReg = platform_write;
//  io_ctx.ReadReg = platform_read;
//  LSM6DSL_RegisterBusIO(&gyro_obj, &io_ctx);
//
//  // Read the WHO_AM_I register
//  LSM6DSL_ReadID(&gyro_obj, &who_am_i);
//
//  if (who_am_i != LSM6DSL_ID) {
//     Error_Handler(); // Implement an error handler if the device is not detected
//   }
//
//  // Convert the WHO_AM_I value to a string and send it to USART
//  snprintf(msg, sizeof(msg), "WHO_AM_I: 0x%02X\r\n", who_am_i);
//  send_data_usart(msg);
//}
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
  MX_USART1_UART_Init();
  //MX_I2C2_Init(&hi2c2);
  /* USER CODE BEGIN 2 */
  //HAL_I2C_Mem_Write
  /* USER CODE END 2 */
  init_lsm6dsl_gyroscope();
  read_and_send_who_am_i();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
