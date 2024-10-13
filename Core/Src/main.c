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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FIFO_SIZE 1024

// Buffer size to store timeStamp + sensor data before writing to microSD
#define BUFFER_SIZE 31456

// full packet: "+A.XXX +A.YYY +A.ZZZ +GGG.XX +GGG.YY +GGG.ZZ\r\n"
#define SD_PACKET_SIZE 46

// timestamp size: "T: YY-MM-GG hh:mm:ss\r\n"
#define SD_TIMESTAMP_SIZE 22

// FIFO packet size: "AxAxAyAyAzAzGxGxGyGyGzGz"
#define FIFO_PACKET_SIZE 12


/****** RTC Initialization variables: ******/
#define startHours 0x18
#define startMinutes 0x33
#define startSeconds 0x0

#define startDate 0x9
#define startMonth RTC_MONTH_OCTOBER
#define startYear 0x24

/****** MOTION DETECTION THRESHOLDS ******/
#define mot_th 10    // 1 LSB = 2 mg, Suggested value ~ 20mg
#define mot_dur 2    // 1 LSB = 1 ms --> The "old" documentation suggests duration = 1 ms

/****** ZERO MOTION DETECTION THRESHOLDS ******/
#define zeroMot_th 60    // 1 LSB = 2 mg
#define zeroMot_dur 150  // 1 LSB = 64 ms


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//TODO: comments on this section:

bool dmaTxDone = false;

// variables used for sleep cycle:
bool zero_mot_trg = false;
bool rtc_trg = false;

/*****************
 * RTC VARIABLES *
 *****************/
RTC_TimeTypeDef sTime;  // Track RTC time structure
RTC_DateTypeDef sDate;  // Track RTC data structure
HAL_StatusTypeDef statusRTC;  // RTC HAL status

/******* Sleep Variables ******/
uint32_t timerTickStart; // used in timer functions... TODO

// Used to update the sleep time depending on computation time:
uint16_t sleepTime; // in milliseconds!!

bool awake_status;



/*****************
 * IMU VARIABLES *
 *****************/

/****** ACCEL., GYRO. AND TEMPERATURE READINGS ******/
int16_t ax;
int16_t ay;
int16_t az;
double Ax, Ay, Az;

int16_t gx;
int16_t gy;
int16_t gz;
double Gx, Gy, Gz;

int16_t temp;
float curr_temp;

/*************************
 * CALIBRATION VARIABLES *
 *************************/
uint16_t readings = 1000;
uint8_t acel_deadzone = 8;  // Average accel. readings should be less than this threshold
uint8_t gyro_deadzone = 1;  // Average gyro readings should be less than this threshold

// last calibration results:
const int16_t AXoffset = 208;
const int16_t AYoffset = 53;
const int16_t AZoffset = 558;
const int16_t GXoffset = -88;
const int16_t GYoffset = -2;
const int16_t GZoffset = -9;

int16_t MPU6050_offsets[6] = {AXoffset, AYoffset, AZoffset, GXoffset, GYoffset, GZoffset};



/*********************
 * SD CARD VARIABLES *
 *********************/
// save them into SRAM2 since it allows buffering between different power down routines:
char mainBuff[BUFFER_SIZE] __attribute__((section(".ram2"))) __attribute__((aligned(4)));
uint16_t buff_Head __attribute__((section(".ram2"))) __attribute__((aligned(4)));


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

int _write(int file, char *ptr, int len);
uint8_t add_time_to_buff (char * buff, uint16_t* buff_Head);
uint8_t add_data_to_buff (char * buff, uint16_t* buff_Head, double ax, double ay, double az, double gx, double gy, double gz);
void Timer_On();
uint32_t Timer_Status();
uint16_t readIMUFifoBuffer(I2C_HandleTypeDef *I2Cx, uint8_t *temp_buff, bool rehabilitateFifo);
void process_and_save_ImuReadings(uint8_t *fifoBuff, uint16_t fifoSize, char *sramBuff, uint16_t *sramBuff_Head,
                        int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
void enterStandbyMode();

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef * hi2c);
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET){ // Recovering from standby

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);  // clear the StandBy flag

    /** Disable the WWAKEUP PIN **/
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN5);

    awake_status = MPU6050_getIntMotionStatus(&hi2c1);  // clear interrupt status register

    //reset all registers and re-initialize the device
    MPU6050_reset(&hi2c1);
    HAL_Delay(50);

    printf("MPU6050 Initialized!\r\n");


    /* mpu6050 setup zero motion interrupt: */
    MPU6050_setupZeroMotionInt(&hi2c1, zeroMot_dur, zeroMot_th, MPU6050_offsets);


    /* modify mpu6050 settings to store sensor readings in the FIFO: */

    // maximum divider -> New samples fill the FIFO with a rate of 4 Hz
    bool overflowEnabled = true;
    MPU6050_setupFifoBuffer(&hi2c1, MPU6050_DLPF_BW_188, 0xFF, overflowEnabled);



  } else { // 1st initialization:

    printf("First cycle!\r\n");

    // Initialize buffers stored in RAM2 section:
    memset(mainBuff, 0, sizeof(mainBuff));
    buff_Head = 0;

    // Initialize IMU for the 1st time
    printf("Starting IMU...\r\n");

    MPU6050_reset(&hi2c1);
    HAL_Delay(50);

    uint8_t check = MPU6050_Initialize(&hi2c1, A2G, G250DPS, MPU6050_offsets);

    if (check == 0){ // If no errors during initialization
      printf("IMU Initialized! \r\n");
    } else {
      printf("An error occured during initializiation! \r\n");

      while(1){/* block code execution */}
    }

    // Needed while loading the script (avoid conflict with ST-Link)
    HAL_Delay(500);

    // Uncomment here to perform a new calibration routine:
    // MPU6050_selfCalibration(&hi2c1, A2G, G250DPS, readings, acel_deadzone, gyro_deadzone);

    // Set-up motion interrupt
    printf("Setting up motion interrupt!\r\n\n");

    MPU6050_setupMotionInt(&hi2c1, mot_dur, mot_th, MPU6050_offsets);

    /** Now enter the standby mode **/
    enterStandbyMode();


    // Upon waking up, code will be executed from the beginning (like a reset)
  }

  /*
   *  This section is reached only after recovering from standby mode:
   */


  // local buff used to read FIFO data:
  uint8_t buff[FIFO_SIZE];
  memset(buff, 0, sizeof(buff));
  uint16_t fifoCount;

  /* Enable STOP2 mode*/

  /*
   * TODO calculate sleep time using functions
   * (changes according to sampling time and packet size)
   * This variable remains constant (used as a reference)
   */
  uint16_t defaultSleepTime = 21000;  // in milliseconds!!

  // initially sleep for the default time:
  sleepTime = defaultSleepTime;

  // TODO: comment here:

  // suspend tick and setup RTC interrupt
  HAL_SuspendTick();
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, sleepTime, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

  /* Enter Stop mode 2 */
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // awake here...
    // deactivate the RTC interrupt and resume settings
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    SystemClock_Config();
    HAL_ResumeTick();


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if(zero_mot_trg){ // if zero motion is triggered

      // prevent new interrupt generation
      MPU6050_setIntZeroMotionEnabled(&hi2c1, false);

      // get the full interrupt status vector:
      uint8_t interrupt_status = MPU6050_getIntStatus(&hi2c1);

      // check zero motion and overflow bits:
      bool zeroMotBit   = (interrupt_status >> MPU6050_INTERRUPT_ZMOT_BIT) & 0x01;
      bool fifoOflowBit = (interrupt_status >> MPU6050_INTERRUPT_FIFO_OFLOW_BIT) & 0x01;

      // check motiont_status_reg --> determine if the interrupt was caused by motion_to_zero_motion event or vice-versa
      bool mot_to_zeroMot = MPU6050_getZeroMotionDetected(&hi2c1);

      /* Determine which signal triggered the system activation: */

      // FIFO overflow event:
      if (fifoOflowBit) {
        // TODO manage overflow event
        printf("FIFO Oflow\r\n");
        while(1);
      }

      // Zero motion event caused by zero_motion_to_motion status
      if (zeroMotBit & !mot_to_zeroMot) {
        // TODO manage this event
        printf("Wrong zero mot\r\n");
        while(1);
      }

      // This point is reached only if the MCU was triggered by a real zero motion event

      // stop FIFO acquisition and read data:
      bool rehabilitateFifo = false;
      fifoCount = readIMUFifoBuffer(&hi2c1, buff, rehabilitateFifo);

      // process FIFO data
      process_and_save_ImuReadings(buff, fifoCount, mainBuff, &buff_Head, ax, ay, az, gx, gy, gz);

      // Set-up MPU6050 for motion interrupt:
      MPU6050_reset(&hi2c1);
      HAL_Delay(50);

      // Set-up motion interrupt
      printf("Setting up motion interrupt!\r\n\n");

      MPU6050_setupMotionInt(&hi2c1, mot_dur, mot_th, MPU6050_offsets);

      /** Now enter standby mode **/
      enterStandbyMode();

    } /* End of zero motion event: enter deep sleep mode */

    if(rtc_trg){ // if rtc flag is set

      /* read FIFO buffer: */
      bool rehabilitateFifo = true;
      fifoCount = readIMUFifoBuffer(&hi2c1, buff, rehabilitateFifo);


      // Start counting total time required by the MCU
      // This time will be subtracted from the total sleep time
      Timer_On();

      /* Process data and write into permanent memories */

      process_and_save_ImuReadings(buff, fifoCount, mainBuff, &buff_Head, ax, ay, az, gx, gy, gz);


      /*  setup RTC alarm */

      /*  RTC Wake-up Interrupt Generation:
          Wake-up Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI))
          ==> WakeUpCounter = Wake-up Time / Wake-up Time Base

          To configure the wake up timer to 20s the WakeUpCounter is set to 0xA017:
            RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16
            Wake-up Time Base = 16 /(32KHz) = 0.0005 seconds
            ==> WakeUpCounter = ~10s/0.0005s = 20000 = 0x4E20 */

      rtc_trg = false;
      sleepTime = defaultSleepTime - Timer_Status();
      uint32_t wakeUpCounter = (sleepTime/1e3)/(0.00048828125);

      // Enter STOP2 mode:
      HAL_SuspendTick();
      HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wakeUpCounter, RTC_WAKEUPCLOCK_RTCCLK_DIV16);


      /*  Enter Stop Mode 2 */
      HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

    } /* End of FIFO processing */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  hi2c1.Init.Timing = 0x00702991;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
    printf("\n\nWaking from StandBy mode, RTC skip!\r\n");
    return; // Exit the function to prevent reinitialize RTC
  } else {
    printf("\n\nInitializing RTC!\r\n");
  }

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  // Initialize RTC with user variables:
   sTime.Hours = startHours;
   sTime.Minutes = startMinutes;
   sTime.Seconds = startSeconds;
   sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
   sTime.StoreOperation = RTC_STOREOPERATION_RESET;
   if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
   {
     Error_Handler();
   }
   sDate.WeekDay = RTC_WEEKDAY_MONDAY;
   sDate.Month = startMonth;
   sDate.Date = startDate;
   sDate.Year = startYear;

   if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
   {
     Error_Handler();
   }

  /* USER CODE END RTC_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC6 PC7 PC8
                           PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA6
                           PA7 PA8 PA10 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB4 PB5 PB6
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_Int1_Pin */
  GPIO_InitStruct.Pin = IMU_Int1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IMU_Int1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Function to redirect printf output to UART
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}


// TODO
uint8_t add_data_to_buff (char* buff, uint16_t* buff_Head, double ax, double ay, double az, double gx, double gy, double gz) {

  // Check for buffer overflow
  uint16_t init_buff_head = *buff_Head;
  uint16_t available_space = BUFFER_SIZE - *buff_Head;
  if (available_space < SD_PACKET_SIZE) {  // +1 for null terminator
    return -1;  // Indicate buffer full
  }

  // Add Accelerometer:

  *buff_Head += snprintf(buff + *buff_Head, BUFFER_SIZE - *buff_Head,
                                "%+4.3f %+4.3f %+4.3f ",
                                ax,
                                ay,
                                az);

  // Add Gyroscope:
  *buff_Head += snprintf(buff + *buff_Head, BUFFER_SIZE - *buff_Head,
                                "%+07.2f %+07.2f %+07.2f\r\n",
                                gx,
                                gy,
                                gz);

  return *buff_Head - init_buff_head;
}


// TODO: add more comments here...
//add current time to main buffer, returns -1 if buffer full, number of bytes written otherwise
uint8_t add_time_to_buff (char * buff, uint16_t* buff_Head) {

  // Check for buffer overflow
  uint16_t init_buff_head = *buff_Head;
  uint16_t available_space = BUFFER_SIZE - *buff_Head;

  // Ensure that the buffer is not overflowed
  if (available_space < SD_TIMESTAMP_SIZE) {  // +1 for null terminator
    return -1;  // Indicate buffer full
  }

  // Get current time:
  statusRTC = HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  if (statusRTC != HAL_OK) {
    printf("RTC error!\r\n");
    while(1);
  }

  // Get current date:
  statusRTC = HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  if (statusRTC != HAL_OK) {
    printf("RTC error!\r\n");
    while(1);
  }

  // Add time stamp:
  *buff_Head += snprintf(buff + *buff_Head, available_space, "T: %02d-%02d-%02d %02d:%02d:%02d\r\n",
                         sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);


  return *buff_Head - init_buff_head;
}


// TODO
void Timer_On() {
  timerTickStart = HAL_GetTick();
}


// TODO
uint32_t Timer_Status() {
  return HAL_GetTick() - timerTickStart;
}


//TODO
uint16_t readIMUFifoBuffer(I2C_HandleTypeDef *I2Cx, uint8_t *temp_buff, bool rehabilitateFifo) {
  uint16_t fifoCount;

  // wait full packet received
  do {
   fifoCount = MPU6050_getFIFOCount(I2Cx);

  // TODO: add additional check to drop data if fifoCount%12 != 0
  } while(fifoCount % FIFO_PACKET_SIZE != 0);

  // prevent new incoming packets
  MPU6050_setFIFOEnabled(I2Cx, false);

  // prevent new data incoming in the FIFO while reading:
  MPU6050_setXGyroFIFOEnabled(I2Cx, false);
  MPU6050_setYGyroFIFOEnabled(I2Cx, false);
  MPU6050_setZGyroFIFOEnabled(I2Cx, false);
  MPU6050_setAccelFIFOEnabled(I2Cx, false);

  // Enable the FIFO for reading:
  MPU6050_setFIFOEnabled(I2Cx, true);

  // read FIFO here...
  //MPU6050_getFIFOBytes(&hi2c1, buff, fifoCount);

  HAL_I2C_Mem_Read_DMA(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_R_W, 1, temp_buff, fifoCount);

  // TODO remove this debug
  printf("Bytes: %d\r\n", fifoCount);

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  printf("T: %02d-%02d-%02d %02d:%02d:%02d\r\n",
           sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);

  while(!dmaTxDone);
  dmaTxDone = false;

  // here FIFO can be enabled again: (avoid losing samples)

  // prevent new incoming packets
  MPU6050_setFIFOEnabled(I2Cx, false);

  // activate sensors writing to FIFO
  MPU6050_setXGyroFIFOEnabled(I2Cx, true);
  MPU6050_setYGyroFIFOEnabled(I2Cx, true);
  MPU6050_setZGyroFIFOEnabled(I2Cx, true);
  MPU6050_setAccelFIFOEnabled(I2Cx, true);

  // reset FIFO:
  MPU6050_resetFIFO(I2Cx);

  // activate FIFO for a new cycle: (prevent activation if zero motion triggered)
  if (rehabilitateFifo) MPU6050_setFIFOEnabled(I2Cx, true);

  return fifoCount;
}


//TODO
void process_and_save_ImuReadings(uint8_t *fifoBuff, uint16_t fifoSize, char *sramBuff, uint16_t *sramBuff_Head,
                        int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz){


  // write time stamp into the sram2 buff:
  int8_t new_bytes = add_time_to_buff(sramBuff, sramBuff_Head);

  if (new_bytes == -1){
    // write to SD card here...
    printf("Buffer full, write to SD card\r\n");
    HAL_Delay(1000);

    // reset buffer:
    memset(sramBuff, 0, *sramBuff_Head);
    *sramBuff_Head = 0;

    // now add timestamp to main buff:
    add_time_to_buff(sramBuff, sramBuff_Head);
  }

  for (int i = 0; i < fifoSize; i += FIFO_PACKET_SIZE) {
    // combine raw data and convert into readable format:
    ax = (((int16_t)fifoBuff[i]) << 8) | fifoBuff[i+1];
    ay = (((int16_t)fifoBuff[i+2]) << 8) | fifoBuff[i+3];
    az = (((int16_t)fifoBuff[i+4]) << 8) | fifoBuff[i+5];

    gx = (((int16_t)fifoBuff[i+6]) << 8) | fifoBuff[i+7];
    gy = (((int16_t)fifoBuff[i+8]) << 8) | fifoBuff[i+9];
    gz = (((int16_t)fifoBuff[i+10]) << 8) | fifoBuff[i+11];

    // scale measurement depending on resolution:
    Ax = (double)ax * accelerationResolution;
    Ay = (double)ay * accelerationResolution;
    Az = (double)az * accelerationResolution;
    Gx = (double)gx * gyroscopeResolution;
    Gy = (double)gy * gyroscopeResolution;
    Gz = (double)gz * gyroscopeResolution;

//        printf("\nIndex: %d\r\n", i);
//        printf("Ax: %.3f g\r\n", Ax);
//        printf("Ay: %.3f g\r\n", Ay);
//        printf("Az: %.3f g\r\n", Az);
//        printf("Gx: %.3f °/s\r\n", Gx);
//        printf("Gy: %.3f °/s\r\n", Gy);
//        printf("Gz: %.3f °/s\r\n", Gz);
//
//        HAL_Delay(500);

    // write time stamp into the sram2 buff:
    uint8_t new_bytes = add_data_to_buff(sramBuff, sramBuff_Head, Ax, Ay, Az, Gx, Gy, Gz);

    // TODO: implement more effective way to manage overflow here
    // However this behavior is not expected!
    if(new_bytes == -1){
      // write to SD card here...
      printf("Buffer full, write to SD card\r\n");
      HAL_Delay(1000);

      // reset buffer:
      memset(sramBuff, 0, *sramBuff_Head);
      *sramBuff_Head = 0;

      // now add timestamp to main buff:
      add_data_to_buff(sramBuff, sramBuff_Head, Ax, Ay, Az, Gx, Gy, Gz);
    }
  }
}


void enterStandbyMode(){
  /* Clear the WU FLAG */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  printf("About to sleep...\r\n");

  // Enable pull down on SYS_WKUP pin
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, PWR_GPIO_BIT_5);
  HAL_PWREx_EnablePullUpPullDownConfig();

  // Enable RAM2 content retention
  HAL_PWREx_EnableSRAM2ContentRetention();

  /* Enable the WAKEUP PIN */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN5);

  printf("Entering standby mode\r\n");

  /* Finally enter the standby mode */
  HAL_PWR_EnterSTANDBYMode();
}


//TODO
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c) {
  dmaTxDone = true;
}


//TODO
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef * hi2c){
  printf("Err DMA!\r\n");
}

//TODO
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  SystemClock_Config();
  HAL_ResumeTick();

  // other code here...
  printf("wake from RTC\r\n");
  rtc_trg = true;
}

//TODO
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == IMU_Int1_Pin)
  {
    SystemClock_Config();
    HAL_ResumeTick();

    // code here...
    printf("wake from pin\r\n");
    zero_mot_trg = true;

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
