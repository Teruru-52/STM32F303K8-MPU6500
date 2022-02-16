/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADDRESS 0x68
#define WHO_AM_I 0x75

#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define GYRO_FACTOR 16.4
#define ACCEL_FACTOR 16384.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
  return len;
}

uint8_t read_byte1(uint8_t reg)
{
  uint8_t rx_data[2];
  uint8_t tx_data[2];

  tx_data[0] = reg | 0x80;
  tx_data[1] = 0x00; // dummy

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return rx_data[1];
}

void write_byte1(uint8_t reg, uint8_t data)
{
  uint8_t rx_data[2];
  uint8_t tx_data[2];

  tx_data[0] = reg & 0x7F;
  //   tx_data[0] = reg | 0x00;
  tx_data[1] = data; // write data

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CSピン立ち下げ
  HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // CSピン立ち上げ
}

uint8_t read_byte2(uint8_t reg)
{
  uint8_t rx_data[2];
  uint8_t tx_data[2];

  tx_data[0] = reg | 0x80;
  tx_data[1] = 0x00; // dummy

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

  return rx_data[1];
}

void write_byte2(uint8_t reg, uint8_t data)
{
  uint8_t rx_data[2];
  uint8_t tx_data[2];

  tx_data[0] = reg & 0x7F;
  //   tx_data[0] = reg | 0x00;
  tx_data[1] = data; // write data

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // CSピン立ち下げ
  HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); // CSピン立ち上げ
}

void MPU6500_1_Init()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  uint8_t who_am_i1;

  HAL_Delay(100);                            // wait start up
  who_am_i1 = read_byte1(WHO_AM_I);          // read who am i
  printf("who_am_i1 = 0x%x\r\n", who_am_i1); // check who am i value
  HAL_Delay(10);

  if (who_am_i1 != 0x70)
  {
    printf("mpu6500_1 error");
  }

  HAL_Delay(50);
  write_byte1(PWR_MGMT_1, 0x00); // set pwr_might (20MHz)
  HAL_Delay(50);
  write_byte1(CONFIG, 0x00); // set config (FSYNCはNC)
  HAL_Delay(50);
  write_byte1(GYRO_CONFIG, 0x18); // set gyro config (2000dps)
  HAL_Delay(50);
  write_byte1(ACCEL_CONFIG, 0x00); // set accel config (2g)
  HAL_Delay(50);
}

void MPU6500_2_Init()
{
  uint8_t who_am_i2;

  HAL_Delay(100);                            // wait start up
  who_am_i2 = read_byte2(WHO_AM_I);          // read who am i
  printf("who_am_i2 = 0x%x\r\n", who_am_i2); // check who am i value
  HAL_Delay(10);

  if (who_am_i2 != 0x70)
  {
    printf("mpu6500_2 error");
  }

  HAL_Delay(50);
  write_byte2(PWR_MGMT_1, 0x00); // set pwr_might (20MHz)
  HAL_Delay(50);
  write_byte2(CONFIG, 0x00); // set config (FSYNCはNC)
  HAL_Delay(50);
  write_byte2(GYRO_CONFIG, 0x18); // set gyro config (2000dps)
  HAL_Delay(50);
  write_byte2(ACCEL_CONFIG, 0x00); // set accel config (2g)
}

bool flag_offset;
bool flag_led;
float ax1, ay1, az1;
float gx1, gy1, gz1;
float ax1_offset, ay1_offset, az1_offset;
float gx1_offset, gy1_offset, gz1_offset;

void MPU6500_1_OffsetCalc()
{
  flag_offset = false;
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;
  float ax_sum = 0;
  float ay_sum = 0;
  float az_sum = 0;
  float gx_sum = 0;
  float gy_sum = 0;
  float gz_sum = 0;

  for (int i = 0; i < 1000; i++)
  {
    // H:8bit shift, Link h and l
    ax_raw = (int16_t)((uint16_t)(read_byte1(ACCEL_XOUT_H) << 8) | (uint16_t)read_byte1(ACCEL_XOUT_L));
    ay_raw = (int16_t)((uint16_t)(read_byte1(ACCEL_YOUT_H) << 8) | (uint16_t)read_byte1(ACCEL_YOUT_L));
    az_raw = (int16_t)((uint16_t)(read_byte1(ACCEL_ZOUT_H) << 8) | (uint16_t)read_byte1(ACCEL_ZOUT_L));
    gx_raw = (int16_t)((uint16_t)(read_byte1(GYRO_XOUT_H) << 8) | (uint16_t)read_byte1(GYRO_XOUT_L));
    gy_raw = (int16_t)((uint16_t)(read_byte1(GYRO_YOUT_H) << 8) | (uint16_t)read_byte1(GYRO_YOUT_L));
    gz_raw = (int16_t)((uint16_t)(read_byte1(GYRO_ZOUT_H) << 8) | (uint16_t)read_byte1(GYRO_ZOUT_L));

    ax1 = (float)(ax_raw / ACCEL_FACTOR);
    ay1 = (float)(ay_raw / ACCEL_FACTOR);
    az1 = (float)(az_raw / ACCEL_FACTOR);
    gx1 = (float)(gx_raw / GYRO_FACTOR); // dps to deg/sec
    gy1 = (float)(gy_raw / GYRO_FACTOR);
    gz1 = (float)(gz_raw / GYRO_FACTOR);

    ax_sum += ax1;
    ay_sum += ay1;
    az_sum += az1;
    gx_sum += gx1;
    gy_sum += gy1;
    gz_sum += gz1;
    HAL_Delay(1);
  }
  ax1_offset = ax_sum / 1000.0;
  ay1_offset = ay_sum / 1000.0;
  az1_offset = az_sum / 1000.0;
  gx1_offset = gx_sum / 1000.0;
  gy1_offset = gy_sum / 1000.0;
  gz1_offset = gz_sum / 1000.0;
}

float ax2, ay2, az2;
float gx2, gy2, gz2;
float ax2_offset, ay2_offset, az2_offset;
float gx2_offset, gy2_offset, gz2_offset;

void MPU6500_2_OffsetCalc()
{

  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;
  float ax_sum = 0;
  float ay_sum = 0;
  float az_sum = 0;
  float gx_sum = 0;
  float gy_sum = 0;
  float gz_sum = 0;

  for (int i = 0; i < 1000; i++)
  {
    // H:8bit shift, Link h and l
    ax_raw = (int16_t)((uint16_t)(read_byte2(ACCEL_XOUT_H) << 8) | (uint16_t)read_byte2(ACCEL_XOUT_L));
    ay_raw = (int16_t)((uint16_t)(read_byte2(ACCEL_YOUT_H) << 8) | (uint16_t)read_byte2(ACCEL_YOUT_L));
    az_raw = (int16_t)((uint16_t)(read_byte2(ACCEL_ZOUT_H) << 8) | (uint16_t)read_byte2(ACCEL_ZOUT_L));
    gx_raw = (int16_t)((uint16_t)(read_byte2(GYRO_XOUT_H) << 8) | (uint16_t)read_byte2(GYRO_XOUT_L));
    gy_raw = (int16_t)((uint16_t)(read_byte2(GYRO_YOUT_H) << 8) | (uint16_t)read_byte2(GYRO_YOUT_L));
    gz_raw = (int16_t)((uint16_t)(read_byte2(GYRO_ZOUT_H) << 8) | (uint16_t)read_byte2(GYRO_ZOUT_L));

    ax2 = (float)(ax_raw / ACCEL_FACTOR);
    ay2 = (float)(ay_raw / ACCEL_FACTOR);
    az2 = (float)(az_raw / ACCEL_FACTOR);
    gx2 = (float)(gx_raw / GYRO_FACTOR); // dps to deg/sec
    gy2 = (float)(gy_raw / GYRO_FACTOR);
    gz2 = (float)(gz_raw / GYRO_FACTOR);

    ax_sum += ax2;
    ay_sum += ay2;
    az_sum += az2;
    gx_sum += gx2;
    gy_sum += gy2;
    gz_sum += gz2;
    HAL_Delay(1);
  }
  ax2_offset = ax_sum / 1000.0;
  ay2_offset = ay_sum / 1000.0;
  az2_offset = az_sum / 1000.0;
  gx2_offset = gx_sum / 1000.0;
  gy2_offset = gy_sum / 1000.0;
  gz2_offset = gz_sum / 1000.0;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  flag_offset = true;
  flag_led = false;
}

const float mu = 9.0;
float Get_MPU6500_Data()
{
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;

  // H:8bit shift, Link h and l
  ax_raw = (int16_t)((uint16_t)(read_byte1(ACCEL_XOUT_H) << 8) | (uint16_t)read_byte1(ACCEL_XOUT_L));
  ay_raw = (int16_t)((uint16_t)(read_byte1(ACCEL_YOUT_H) << 8) | (uint16_t)read_byte1(ACCEL_YOUT_L));
  az_raw = (int16_t)((uint16_t)(read_byte1(ACCEL_ZOUT_H) << 8) | (uint16_t)read_byte1(ACCEL_ZOUT_L));
  gx_raw = (int16_t)((uint16_t)(read_byte1(GYRO_XOUT_H) << 8) | (uint16_t)read_byte1(GYRO_XOUT_L));
  gy_raw = (int16_t)((uint16_t)(read_byte1(GYRO_YOUT_H) << 8) | (uint16_t)read_byte1(GYRO_YOUT_L));
  gz_raw = (int16_t)((uint16_t)(read_byte1(GYRO_ZOUT_H) << 8) | (uint16_t)read_byte1(GYRO_ZOUT_L));

  // ax1 = (float)(ax_raw / ACCEL_FACTOR) - ax1_offset;
  // ay1 = (float)(ay_raw / ACCEL_FACTOR) - ay1_offset;
  // az1 = (float)(az_raw / ACCEL_FACTOR) - az1_offset;
  ax1 = (float)(ax_raw / ACCEL_FACTOR);
  ay1 = (float)(ay_raw / ACCEL_FACTOR);
  az1 = (float)(az_raw / ACCEL_FACTOR);
  gx1 = (float)(gx_raw / GYRO_FACTOR) - gx1_offset;
  gy1 = (float)(gy_raw / GYRO_FACTOR) - gy1_offset;
  gz1 = (float)(gz_raw / GYRO_FACTOR) - gz1_offset;

  ax_raw = (int16_t)((uint16_t)(read_byte2(ACCEL_XOUT_H) << 8) | (uint16_t)read_byte2(ACCEL_XOUT_L));
  ay_raw = (int16_t)((uint16_t)(read_byte2(ACCEL_YOUT_H) << 8) | (uint16_t)read_byte2(ACCEL_YOUT_L));
  az_raw = (int16_t)((uint16_t)(read_byte2(ACCEL_ZOUT_H) << 8) | (uint16_t)read_byte2(ACCEL_ZOUT_L));
  gx_raw = (int16_t)((uint16_t)(read_byte2(GYRO_XOUT_H) << 8) | (uint16_t)read_byte2(GYRO_XOUT_L));
  gy_raw = (int16_t)((uint16_t)(read_byte2(GYRO_YOUT_H) << 8) | (uint16_t)read_byte2(GYRO_YOUT_L));
  gz_raw = (int16_t)((uint16_t)(read_byte2(GYRO_ZOUT_H) << 8) | (uint16_t)read_byte2(GYRO_ZOUT_L));

  // ax2 = (float)(ax_raw / ACCEL_FACTOR) - ax2_offset;
  // ay2 = (float)(ay_raw / ACCEL_FACTOR) - ay2_offset;
  // az2 = (float)(az_raw / ACCEL_FACTOR) - az2_offset;
  ax2 = (float)(ax_raw / ACCEL_FACTOR);
  ay2 = (float)(ay_raw / ACCEL_FACTOR);
  az2 = (float)(az_raw / ACCEL_FACTOR);
  gx2 = (float)(gx_raw / GYRO_FACTOR) - gx2_offset;
  gy2 = (float)(gy_raw / GYRO_FACTOR) - gy2_offset;
  gz2 = (float)(gz_raw / GYRO_FACTOR) - gz2_offset;

  // printf("%f, %f, %f \r\n", ax1, ay1, az1);
  // printf("%f, %f, %f \r\n", ax2, ay2, az2);
  // printf("%f\r\n", ay2);
  float mx = az1 - mu * az2;
  float my = ay1 - mu * ay2;
  float theta_b = atan(-mx / my) * 180.0 / M_PI;
  // float theta_b = atan2(-mx, my) * 180.0 / M_PI;
  // printf("%f\r\n", theta_b);

  return theta_b;
}

int count = 0;
int led_count = 0;
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   if (htim == &htim3)
//     if (flag_offset == true)
//     {
//       {
//         count = (count + 1) % 160;
//         if (count == 0)
//         { // time interruption 100Hz
//           float theta = Get_MPU6500_Data();
//           led_count = (count + 1) % 100;
//           // if (led_count == 0)
//           // {
//           //   if (flag_led == true)
//           //     flag_led = false;
//           //   else
//           //     flag_led = true;
//           // }
//           if (led_count % 50 == 0)
//           {
//             // printf("%f, %f, %f, %f, %f, %f \r\n", ax, ay, az, gx, gy, gz);
//             printf("%f \r\n", theta);
//           }
//         }
//       }
//       // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
//       //   if (flag_led == true)
//       //     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
//       //   else
//       //     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
//     }
// }
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  setbuf(stdout, NULL);
  MPU6500_1_Init(); // who_am_i
  MPU6500_2_Init();
  MPU6500_1_OffsetCalc();
  MPU6500_2_OffsetCalc();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    float theta = Get_MPU6500_Data();
    printf("%f \r\n", theta);
    HAL_Delay(10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 30;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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

#ifdef USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
