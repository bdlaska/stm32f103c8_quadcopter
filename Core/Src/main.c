/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>  // Для використання printf

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MPU6500_ADDR 0x68 << 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Функція для запису в регістр MPU6500
HAL_StatusTypeDef MPU6500_Write(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    return HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, buf, 2, HAL_MAX_DELAY);
}

// Функція для читання з регістра MPU6500
HAL_StatusTypeDef MPU6500_Read(uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, &reg, 1, HAL_MAX_DELAY);
    return HAL_I2C_Master_Receive(&hi2c1, MPU6500_ADDR, data, len, HAL_MAX_DELAY);
}

// Ініціалізація MPU6500
void MPU6500_Init(void) {
    // Приклад ініціалізації: скидання MPU6500
    MPU6500_Write(0x6B, 0x00);  // Регістр PWR_MGMT_1: вимкнути sleep mode
    HAL_Delay(100);
    // Інші потрібні налаштування регістрів
}

void transform(uint8_t *arr, uint16_t *alt_arr)
{
  alt_arr[0] = (arr[0] << 8) | arr[1];
  alt_arr[1] = (arr[2] << 8) | arr[3];
  alt_arr[2] = (arr[4] << 8) | arr[5];
}

// USART функція для передачі даних
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// Функція для сканування I2C-шини
void I2C_Scanner() {
    printf("Scanning I2C bus...\n");

    // Проходимося по всіх можливих адресах
    for (uint8_t i = 1; i < 128; i++) {
        // Перевірка наявності пристрою на поточній адресі
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 1, 10) == HAL_OK) {
            printf("Device found at 0x%02X\n", i);
        }
    }

    printf("Scanning complete.\n");
}

// kvadrobers are better than lgbt

// Функція для форматованого виведення даних акселерометра і гіроскопа
void print_sensor_data(uint16_t *accel_data, uint16_t *giro_data) {
    // Виведення даних акселерометра
    printf("Accelerometer data:\n");
    printf("X: %d, Y: %d, Z: %d\n", accel_data[0], accel_data[1], accel_data[2]);

    // Виведення даних гіроскопа
    printf("Gyroscope data:\n");
    printf("X: %d, Y: %d, Z: %d\n", giro_data[0], giro_data[1], giro_data[2]);
}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MPU6500_Init(); // Ініціалізація MPU6500

  uint8_t accel_data[6];
  uint16_t accel_data_alt[3];
  uint8_t giro_data[6];
  uint16_t giro_data_alt[3];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        // Код для читання/обробки даних з MPU6500
  
        MPU6500_Read(0x3B, accel_data, 6); // Читання даних акселерометра
        MPU6500_Read(0x43, giro_data, 6);

        transform(accel_data, accel_data_alt);
        transform(giro_data, giro_data_alt);
        
        print_sensor_data(accel_data_alt, giro_data_alt);
        
        HAL_Delay(1000); // Пауза для тестування  

        //I2C_Scanner();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
