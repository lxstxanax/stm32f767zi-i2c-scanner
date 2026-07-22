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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "xnx_i2c.h"
#include "tsc1641.h"
#include "ina228.h"
#include "mpu6050.h"
#include "mlx90614.h"
#include <stdio.h>
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
xnx_i2c_scan_t i2c_scan_result;
volatile uint32_t blink_count = 0;
volatile float psu_set_v = 0;   /* last voltage written to the DAC on PA5 */

/*
 * Four sensors share the single I2C1 bus (SCL = PB6, SDA = PB9):
 *   MPU6050   0x68  accelerometer and gyroscope
 *   MLX90614  0x5A  contact-free infrared thermometer
 *   TSC1641   0x40..0x43 (jumpers)  16-bit current and voltage monitor
 *   INA228    0x40..0x4F (jumpers)  20-bit current and voltage monitor
 *
 * Both power monitors measure the same load current: their shunts sit in
 * series in the same wire, which is what makes it possible to compare the
 * two against each other.
 */
mpu6050_t mpu;
mlx90614_t mlx;
tsc1641_t tsc;
ina228_t ina;

/* Newest reading of each sensor. These are global on purpose: the debugger
 * (Live Watch) can then show the measurements while the program runs. */
mpu6050_data_t mpu_data;
mlx90614_data_t mlx_data;
tsc1641_data_t tsc_data;
ina228_data_t ina_data;

/* Result of the start-up initialisation, kept so it can be reported again
 * later - a monitor that connects halfway would otherwise miss it. */
mpu6050_status_t mpu_st;
mlx90614_status_t mlx_st;
tsc1641_status_t tsc_st;
ina228_status_t ina_st;

/* A sensor is only polled if it answered at start-up. Skipping the silent
 * ones keeps the loop from stalling on I2C timeouts, so the rest of the
 * system keeps working when one sensor is unplugged. */
uint8_t mpu_ok = 0;
uint8_t mlx_ok = 0;
uint8_t tsc_ok = 0;
uint8_t ina_ok = 0;
/* USER CODE END PV */

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void DAC_PA5_Init(void);
void DAC_PA5_Set(float volts);
static void print_status(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);
  return ch;
}

/* What's on the bus and did the sensors come up. Repeated every few seconds
 * so a monitor or dashboard that attaches later still sees the state. */
static void print_status(void)
{
  printf("I2C scan: %u device(s):", i2c_scan_result.count);
  for (uint8_t i = 0; i < i2c_scan_result.count; i++) {
    printf(" 0x%02X", i2c_scan_result.addr[i]);
  }
  printf("\r\n");
  printf("MPU6050 init: %s\r\n", mpu6050_status_string(mpu_st));
  printf("MLX90614 init: %s, ID=0x%04X\r\n", mlx90614_status_string(mlx_st), mlx.id_word);
  printf("TSC1641 init: %s, addr=0x%02X, DIE_ID=0x%04X\r\n",
         tsc1641_status_string(tsc_st), tsc.address, tsc.die_id);
  printf("INA228 init: %s, addr=0x%02X, ID=0x%04X\r\n",
         ina228_status_string(ina_st), ina.address, ina.device_id);
}

/* Set up DAC channel 2 on PA5 by hand (the HAL DAC module isn't generated).
 * The output buffer stays on, so the pin can drive a control input directly. */
void DAC_PA5_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_DAC_CLK_ENABLE();

  DAC->DHR12R2 = 0;          /* start at 0 V */
  DAC->CR |= DAC_CR_EN2;     /* enable channel 2 = PA5 */
}

/* Set the PA5 output voltage, 0..3.3 V (assumes VDDA = 3.3 V).
 * The buffered output can't reach closer than ~0.2 V to the rails. */
void DAC_PA5_Set(float volts)
{
  if (volts < 0)
    volts = 0;
  if (volts > 3.3f)
    volts = 3.3f;
  psu_set_v = volts;
  DAC->DHR12R2 = (uint16_t)(volts / 3.3f * 4095.0f + 0.5f);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(300); /* let bus devices finish their own power-up before polling */
  xnx_i2c_scan(&hi2c1, &i2c_scan_result);

  DAC_PA5_Init();
  DAC_PA5_Set(0);

  /*
   * Finding the two power monitors.
   *
   * Both of them live in the 0x40..0x4F address range and both leave the
   * factory listening on 0x40, so the address each one ends up with depends
   * on how its jumpers are set. Instead of hard-coding that, every address
   * the bus scan reported is offered to both drivers in turn. Each driver
   * reads the chip's identification register before it writes anything, so
   * only the matching chip ever accepts the address and no driver can
   * misconfigure a stranger. The jumpers can then be set to anything.
   */
  ina228_config_t ina_cfg;
  ina228_get_default_config(&ina_cfg);

  tsc1641_config_t tsc_cfg;
  tsc1641_get_default_config(&tsc_cfg);

  /* Assume missing until a chip identifies itself. */
  ina_st = INA228_STATUS_DEVICE_NOT_FOUND;
  tsc_st = TSC1641_STATUS_DEVICE_NOT_FOUND;

  for (uint8_t i = 0; i < i2c_scan_result.count; i++) {
    uint8_t addr = i2c_scan_result.addr[i];

    if ((addr < 0x40) || (addr > 0x4F)) {
      continue;   /* outside the power-monitor range, leave it alone */
    }

    if (!ina_ok) {
      ina_st = ina228_init(&ina, &hi2c1, addr, &ina_cfg);
      ina_ok = (ina_st == INA228_STATUS_OK);
      if (ina_ok) {
        continue;   /* address taken, move on to the next one */
      }
    }

    if (!tsc_ok) {
      tsc_st = tsc1641_init(&tsc, &hi2c1, addr, &tsc_cfg);
      tsc_ok = (tsc_st == TSC1641_STATUS_OK);
    }
  }

  mpu6050_config_t mpu_cfg;
  mpu6050_get_default_config(&mpu_cfg);
  mpu_st = mpu6050_init(&mpu, &hi2c1, MPU6050_ADDRESS_AD0_LOW, &mpu_cfg);
  mpu_ok = (mpu_st == MPU6050_STATUS_OK);

  mlx_st = mlx90614_init(&mlx, &hi2c1, MLX90614_DEFAULT_ADDRESS);
  mlx_ok = (mlx_st == MLX90614_STATUS_OK);

  print_status();
  if (i2c_scan_result.count == 0) {
    printf("Nothing ACKed. Check: common GND? VDD? SCL=PB6/SDA=PB9 not swapped?\r\n");
  }
  /* USER CODE END 2 */

  /*
   * Main loop.
   *
   * Each group of sensors is read at its own rate, and the loop never
   * blocks: instead of waiting with HAL_Delay it compares the millisecond
   * counter against the time each group was last serviced. Motion needs a
   * fast rate to be useful, while temperature and current change slowly,
   * so reading everything at 100 ms would only flood the serial link.
   */
  /* USER CODE BEGIN WHILE */
  uint32_t t_mpu = 0, t_slow = 0, t_status = 0;  /* last service time, ms */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* Motion, 10 times a second. */
    if (mpu_ok && (now - t_mpu >= 100)) {
      t_mpu = now;
      if (mpu6050_read(&mpu, &mpu_data) == MPU6050_STATUS_OK) {
        /* Sent as whole milli-units (mg, mdps) so the line stays short
         * and the receiver never has to parse a decimal point. */
        printf("MPU A[mg]=%ld,%ld,%ld G[mdps]=%ld,%ld,%ld T[cC]=%ld\r\n",
               (long)(mpu_data.accel_x_g * 1000), (long)(mpu_data.accel_y_g * 1000),
               (long)(mpu_data.accel_z_g * 1000),
               (long)(mpu_data.gyro_x_dps * 1000), (long)(mpu_data.gyro_y_dps * 1000),
               (long)(mpu_data.gyro_z_dps * 1000),
               (long)(mpu_data.temperature_c * 100));
      }
    }

    /* Temperatures, load current and the heartbeat LED, twice a second. */
    if (now - t_slow >= 500) {
      t_slow = now;
      HAL_GPIO_TogglePin(GPIOB, LD1_Pin | LD2_Pin | LD3_Pin);
      blink_count++;

      if (mlx_ok) {
        if (mlx90614_read(&mlx, &mlx_data) == MLX90614_STATUS_OK) {
          printf("MLX Ta[cC]=%ld To[cC]=%ld\r\n",
                 (long)mlx_data.ambient_centi_c, (long)mlx_data.object_centi_c);
        }
      }

      /*
       * Both monitors report in amps, volts and watts. Six decimals are
       * needed because one step of the INA228 is only 19 uA, and the
       * shunt voltage is printed as well: it is the honest answer to
       * "is any current flowing at all", since it is what the chip
       * physically measures before any calibration is applied.
       */
      if (tsc_ok) {
        if (tsc1641_read(&tsc, &tsc_data) == TSC1641_STATUS_OK) {
          printf("TSC I[A]=%.6f U[V]=%.3f P[W]=%.3f Vsh[mV]=%.3f\r\n",
                 tsc_data.current_a, tsc_data.load_voltage_v,
                 tsc_data.power_w, tsc_data.shunt_voltage_mv);
        }
      }

      if (ina_ok) {
        if (ina228_read(&ina, &ina_data) == INA228_STATUS_OK) {
          printf("INA I[A]=%.6f U[V]=%.3f P[W]=%.3f T[C]=%.2f Vsh[mV]=%.3f\r\n",
                 ina_data.current_a, ina_data.bus_voltage_v, ina_data.power_w,
                 ina_data.die_temperature_c, ina_data.shunt_voltage_mv);
        }
      }

      printf("PSU set[V]=%.3f\r\n", psu_set_v);
    }

    /* Remind whoever is listening what the bus state is. */
    if (now - t_status >= 5000) {
      t_status = now;
      print_status();
    }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
