/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LED.h"
#include "usart_app.h"
#include <stdio.h>
#include "pwm.h"
#include "MPU6050.h"
#include "attitude.h"
#include "pid.h"
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
MPU6050_Data_t MPU6050_Data;
Attitude_t Attitude;
PID_t Roll_PID;
float Servo_Output;
float Servo_Angle;
volatile uint32_t tick_cnt = 0;      /* TIM3 中断产生的控制节拍数 */
uint32_t tick_last = 0;     /* 主循环已经处理的控制节拍数 */
uint32_t miss_cnt = 0;             /* 主循环来不及处理、被跳过的控制周期数 */
uint8_t print_cnt = 0;                    /* 串口打印分频计数 */
#define LOOP_DT 0.01f
/*float Roll_Bias = 0;
float Pitch_Bias = 0;*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Control_Process(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(200);    // 等待系统和MPU6050电源稳定
  /* 初始化失败时停止启动，避免使用无效传感器数据 */
  if (MPU6050_Init() != HAL_OK)
  {
    printf("MPU6050 init failed\r\n");
    Error_Handler();
  }
  HAL_Delay(1000);		//等待传感器稳定
  /* 标定失败时停止启动，避免使用错误的姿态零偏 */
  if (MPU6050_Calibrate() != HAL_OK)
  {
    printf("MPU6050 calibration failed\r\n");
    Error_Handler();
  }
  PID_Init(&Roll_PID,2.0f,0.0f,0.1f,45.0f);//初始化PID
  PWM_Start();
  Servo_SetAngle(90);
  HAL_Delay(300);    // 等待舵机到中位
  HAL_TIM_Base_Start_IT(&htim3); //定时器3产生10ms定时中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Control_Process();

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
/* 主循环中执行一次控制处理，实际的传感器读取和算法都在这里完成 */
void Control_Process(void)
{
    uint32_t pending;
    uint32_t now_tick = tick_cnt;

    if (now_tick == tick_last)
    {
        return;
    }

    pending = now_tick - tick_last;
    tick_last = now_tick;

    /* 统计主循环没有及时处理的控制周期数 */
    if (pending > 1)
    {
        miss_cnt += pending - 1;
    }

    /* 读取失败时跳过本次控制计算 */
    if (MPU6050_GetData(&MPU6050_Data) != HAL_OK)
    {
        return;
    }

    Attitude_Update(&MPU6050_Data, &Attitude, LOOP_DT);

    Servo_Output = PID_Calculate(&Roll_PID, 0.0f, Attitude.Roll, LOOP_DT);

    Servo_Angle = 90.0f + Servo_Output;
    if (Servo_Angle > 135.0f) Servo_Angle = 135.0f;
    if (Servo_Angle < 45.0f) Servo_Angle = 45.0f;
    Servo_SetAngle(Servo_Angle);

    /* 每 10 个控制周期打印一次，约 10 Hz */
    print_cnt++;
    if (print_cnt >= 10)
    {
        print_cnt = 0;
        printf("Roll=%.2f PID=%.2f Servo=%.2f\r\n",
               Attitude.Roll,
               Servo_Output,
               Servo_Angle);
    }
}

/* TIM3 中断只记录控制节拍，耗时处理全部移到主循环 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        tick_cnt++;
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
