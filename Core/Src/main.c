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
#include "ecat_motion.h"
#include "usb_device.h"
#include "lcd_fsmc.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "gui.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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

  /* 复位原因诊断: 必须在HAL_Init()之前读, 读完立刻清掉标志位方便看下一次的.
     RCC->CSR 的 PINRSTF 位如果被置位, 就实锤是外部 NRST 引脚被拉低复位的
     (对应"一开串口板子就断"的怀疑: CH340 DTR/RTS 通过板子P4排针接到了 RESET)。 */
  g_reset_cause = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* --- LAN8720 硬件复位: 复位脚接在 PD3, 上电被10K电阻拉低=一直处于复位,
         必须由程序把 PD3 拉高才能松开复位、让网络芯片工作 --- */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  {
    GPIO_InitTypeDef phy_rst = {0};
    phy_rst.Pin   = GPIO_PIN_3;
    phy_rst.Mode  = GPIO_MODE_OUTPUT_PP;
    phy_rst.Pull  = GPIO_NOPULL;
    phy_rst.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &phy_rst);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); /* 拉低: 进入复位 */
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);   /* 拉高: 松开复位 */
    HAL_Delay(100);                                       /* 等 PHY 内部就绪 */
  }

  /* --- 心跳灯诊断: LED0=PF9, 低电平点亮(战神板寄存器例程实测).
         开机立即闪一下, 证明代码执行到这里(时钟+GPIO正常);
         后续心跳由 ecat_motion.c 在EtherCAT循环里接管闪烁,
         如果这一下都没闪说明卡在SystemClock_Config/HAL_Init之前(比如HSE不起振)。--- */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  {
    GPIO_InitTypeDef led = {0};
    led.Pin   = GPIO_PIN_9;
    led.Mode  = GPIO_MODE_OUTPUT_PP;
    led.Pull  = GPIO_NOPULL;
    led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &led);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);  /* 点亮 */
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);    /* 熄灭 */
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();   /* USB Slave(Micro-USB, PA11/PA12) 枚举成CDC虚拟串口 */

  /* --- 屏显GUI初始化: 总线/渲染/触摸/中文字库都在前几次提交里单独验证过了,
         这里正式建开机页并显示; ecat_motion_run()内部会在真实的EtherCAT
         启动阶段调gui_boot_xxx()把开机页往前推, 进待机后自动切主页。 --- */
  lcd_init();
  lv_init();
  lv_port_disp_init();
  lv_port_indev_init();
  gui_init();

  ecat_motion_run();   /* 扫从站 → CSV使能 → 伸出/缩回×5, 内部死循环, 不返回 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 正常情况下 ecat_motion_run() 内部死循环, 不会执行到这里;
       一旦跑到这, 说明它提前 return 了(比如没扫到从站/网卡初始化失败),
       用快闪(100ms)跟正常心跳区分, 方便直接用灯判断"提前退出"这种情况 */
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
    HAL_Delay(100);
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
  /* PLLQ=7: VCO(336MHz)/7=48MHz, USB OTG FS要求这里必须精确等于48MHz才能工作;
     原来的4会得到84MHz, USB完全不能用。PLLQ不影响SYSCLK(由PLLP决定), 改这里
     对EtherCAT的168MHz主频没有任何影响。 */
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
