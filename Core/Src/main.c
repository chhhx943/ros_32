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
#include "can.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_motor.h"
#include "encoder.h"
#include "PID.h"
#include "bsp_bxcan.h"
#include "chassis_control.h"
#include "safety_manager.h"
#include "watchdog.h"
#ifdef AUTOTUNE_SAFE_PROFILE
#include "autotune_safe.h"
#endif
#ifdef BSP_BXCAN_RUN_LOOPBACK_SELF_TEST
#include "bsp_bxcan_loopback.h"
#endif
#if defined(CAN_MOTOR_BENCH_TEST) || defined(CALIBRATION_BENCH_TEST)
#include "can_motor_bench.h"
#endif
#ifdef STEERING_BENCH_TEST
#include "steering_bench.h"
#endif
#ifdef ACKERMANN_BENCH_TEST
#include "ackermann_bench.h"
#endif
#ifdef H4_CHARACTERIZATION_TEST
#include "h4_characterization.h"
#endif

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
#ifdef IWDG_STALL_TEST
volatile uint32_t g_iwdg_stall_result;
#endif
#ifdef SAFE_STOP_WATCHDOG_TEST
volatile uint32_t g_safe_stop_watchdog_result;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_USB_OTG_FS_USB_Init();
  BSP_Watchdog_Init();
#ifdef IWDG_STALL_TEST
  /* First boot deliberately stops feeding. After the watchdog reset, hold
     the target while feeding so the debugger can inspect the reset flag and
     result without another immediate reset. */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {
    g_iwdg_stall_result = 0x49574447UL;
    while (1) {
      BSP_Watchdog_Feed();
    }
  }
  while (1) {
  }
#endif
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);

#ifdef SAFE_STOP_WATCHDOG_TEST
  /* Inject the non-latched command-timeout condition and keep the controller
     in SAFE_STOP while the normal control path continues feeding IWDG. */
  /* A preceding IWDG probe can leave RCC_CSR_IWDGRST set across a debugger
     download.  Clear that stale reset cause before Safety_Manager_Init so
     this probe measures the normal SAFE_STOP path rather than the latched
     watchdog-reset recovery path. */
  __HAL_RCC_CLEAR_RESET_FLAGS();
  Chassis_ControlInit();
  BSP_BXCAN_SetFault(BSP_BXCAN_FAULT_COMMAND_TIMEOUT, 0U);
  Chassis_ControlProcess(HAL_GetTick());
  {
    uint32_t safe_stop_start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - safe_stop_start) < 3000U) {
      HAL_Delay(1U);
      Chassis_ControlProcess(HAL_GetTick());
      BSP_Watchdog_Feed();
    }
  }
  if (Safety_Manager_GetState() == SAFETY_STATE_SAFE_STOP) {
    g_safe_stop_watchdog_result = 0x53414645UL;
  }
  while (1) {
    HAL_Delay(10U);
    Chassis_ControlProcess(HAL_GetTick());
    BSP_Watchdog_Feed();
  }
#endif

#ifdef MOTOR_BENCH_TEST
  /* 一次性 PWM 台架，按 2026-08-29 spec：200/1000 占空比，结束时 COAST */
  Motor_Init();
  Motor_CoastAll();
  Motor_Drive(1U, 200);
  HAL_Delay(3000);
  Motor_Coast(1U);
  HAL_Delay(1000);
  Motor_Drive(2U, 200);
  HAL_Delay(3000);
  Motor_Coast(2U);
  while (1)
  {
  }
#elif defined(BSP_BXCAN_RUN_LOOPBACK_SELF_TEST)
  BSP_BXCAN_RunLoopbackSelfTest(&hcan1);
  while (1)
  {
  }
#elif defined(CAN_MOTOR_BENCH_TEST)
  CAN_Motor_Bench_Run();
  while (1)
  {
  }
#elif defined(CALIBRATION_BENCH_TEST)
  CAN_Motor_Bench_RunCalibration();
  while (1)
  {
  }
#elif defined(STEERING_BENCH_TEST)
  Steering_Bench_Run();
  while (1)
  {
  }
#elif defined(ACKERMANN_BENCH_TEST)
  Ackermann_Bench_Run();
  while (1)
  {
  }
#elif defined(H4_CHARACTERIZATION_TEST)
  H4_Characterization_Run();
  while (1)
  {
    Chassis_ControlProcessEvents(HAL_GetTick());
  }
#else
#ifdef AUTOTUNE_SAFE_PROFILE
  AutotuneSafe_Init();
#endif
  Chassis_ControlInit();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Chassis_ControlProcess(HAL_GetTick()); remains the legacy API spelling;
       production dispatch is event-driven below. */
    Chassis_ControlProcessEvents(HAL_GetTick());
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
