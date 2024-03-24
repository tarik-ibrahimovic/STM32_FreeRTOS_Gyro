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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ABS(x)         (x < 0) ? (-x) : x
//#define configTOTAL_HEAP_SIZE  ( 180 * 1024 )
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

__IO uint32_t ButtonPressed = 0;
extern __IO uint32_t Gv_EOA;
//USBD_HandleTypeDef USBD_Device;
TSC_HandleTypeDef TscHandle;
TSC_IOConfigTypeDef IoConfig;

int CallCount;
int Gyro;
int GyroPr;
/* USER CODE BEGIN PV */
TaskHandle_t xledTaskHandle = NULL;
TaskHandle_t xsaveTaskHandle = NULL;
TaskHandle_t xgyroTaskHandle = NULL;
TaskHandle_t xtouchTaskHandle = NULL;
TaskHandle_t xispisTaskHandle = NULL;
TaskHandle_t xispisAPTaskHandle = NULL;
TaskHandle_t xblinkTaskHandle = NULL;
TaskHandle_t xsveTaskHandle = NULL;
SemaphoreHandle_t xMutex1Handle = NULL;

static void Gyro_task_handler(void * p);
static void LED_task_handler(void * p);
static void Save_task_handler(void * p);
static void Touch_task_handler(void * p);
static void Ispis_task_handler(void * p);
static void IspisAP_task_handler(void * p);
static void Blink_task_handler(void * p);
static void Sve_task_handler(void * p);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TSC_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */
static void Demo(void);
static void LED_Test(void);
static void MEMS_Test(void);

static void USB_Demo(void);
static void USB_Test(void);
static void USB_GetPointerData_Test(uint8_t *pbuf);
static void USB_GetPointerData_Demo(uint8_t *pbuf);
static void Demo_GyroReadAngRate (float* pfData);
static void Gyro_Buffer (float* pfData, int postavi);
static void TSL_Test(void);

static void ProcessSensors(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


#define mainAUTO_RELOAD_TIMER_PERIOD pdMS_TO_TICKS( 500 )
#define Wait_Semaphore pdMS_TO_TICKS( 10 )
BaseType_t xTimerStarted;
static void prvAutoReloadTimerCallback( TimerHandle_t xTimer )
{
	//TickType_t xTimeNow;
	CallCount++;
	if(GyroPr==Gyro){
		  while (1)
		  {
			  BSP_LED_Off(LED3);
			  BSP_LED_Off(LED4);
			  BSP_LED_Off(LED5);
			  BSP_LED_Off(LED6);
			  HAL_Delay(100);
			  BSP_LED_On(LED3);
			  BSP_LED_On(LED4);
			  BSP_LED_On(LED5);
			  BSP_LED_On(LED6);
			  HAL_Delay(100);
		    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
		    /* USER CODE END W1_HardFault_IRQn 0 */
		  }
	}
	GyroPr=Gyro;
	//xTaskNotifyGive(xgyroTaskHandle);
	//xTaskNotifyGive(xtouchTaskHandle);
	//xTaskNotifyGive(xsveTaskHandle);
}

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    BSP_LED_Init(LED3);
    BSP_LED_Init(LED4);
    BSP_LED_Init(LED5);
    BSP_LED_Init(LED6);
    //BSP_LED_Init(LED7);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_I2C2_Init();
//  MX_SPI2_Init();
//  MX_TSC_Init();
//  MX_USB_PCD_Init();

  /* USER CODE BEGIN 2 */
  	  CallCount=0;
  	  Gyro=0;
  	  GyroPr=-1;
	TimerHandle_t xAutoReloadTimer;
	 xAutoReloadTimer = xTimerCreate("AutoReload", mainAUTO_RELOAD_TIMER_PERIOD, pdTRUE, 0, prvAutoReloadTimerCallback);
	 if( ( xAutoReloadTimer != NULL ) )
	  {
	  /* Start the software timers, using a block time of 0 (no block time). The scheduler has
	  not been started yet so any block time specified here would be ignored anyway. */
	  xTimerStarted = xTimerStart( xAutoReloadTimer, 0 );
	  /* The implementation of xTimerStart() uses the timer command queue, and xTimerStart()
	  will fail if the timer command queue gets full. The timer service task does not get
	  created until the scheduler is started, so all commands sent to the command queue will
	  stay in the queue until after the scheduler has been started. Check both calls to
	  xTimerStart() passed. */
	  if( xTimerStarted == pdPASS )
	  {
	  /* Start the scheduler. */
	  }
	  }
xMutex1Handle = xSemaphoreCreateMutex();

if (xMutex1Handle != NULL)
  {

  xTaskCreate(Gyro_task_handler, "Gyro Task", configMINIMAL_STACK_SIZE, NULL, 3, &xgyroTaskHandle);
  xTaskCreate(LED_task_handler, "LED Task", configMINIMAL_STACK_SIZE, NULL, 1, &xledTaskHandle);
  xTaskCreate(Save_task_handler, "Save Task", configMINIMAL_STACK_SIZE, NULL, 1, &xsaveTaskHandle);
  xTaskCreate(Touch_task_handler, "Touch Task", configMINIMAL_STACK_SIZE, NULL, 2, &xtouchTaskHandle);
  xTaskCreate(Ispis_task_handler, "Ispis Task", 2 *configMINIMAL_STACK_SIZE, NULL, 1, &xispisTaskHandle);
  xTaskCreate(IspisAP_task_handler, "APIspis Task", configMINIMAL_STACK_SIZE, NULL, 4, &xispisAPTaskHandle);
  //xTaskCreate(Blink_task_handler, "Blink Task", configMINIMAL_STACK_SIZE, NULL, 1, &xblinkTaskHandle);
  xTaskCreate(Sve_task_handler, "Sve Task", configMINIMAL_STACK_SIZE, NULL, 4, &xsveTaskHandle);
  }
    SEGGER_SYSVIEW_Conf();
    SEGGER_SYSVIEW_Start();

    vTaskStartScheduler();
  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TSC Initialization Function
  * @param None
  * @retval None
  */
static void MX_TSC_Init(void)
{

  /* USER CODE BEGIN TSC_Init 0 */

  /* USER CODE END TSC_Init 0 */

  /* USER CODE BEGIN TSC_Init 1 */

  /* USER CODE END TSC_Init 1 */

  /** Configure the TSC peripheral
  */
//  htsc.Instance = TSC;
//  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
//  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
//  htsc.Init.SpreadSpectrum = DISABLE;
//  htsc.Init.SpreadSpectrumDeviation = 1;
//  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
//  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
//  htsc.Init.MaxCountValue = TSC_MCV_8191;
//  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
//  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
//  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
//  htsc.Init.MaxCountInterrupt = DISABLE;
//  htsc.Init.ChannelIOs = TSC_GROUP1_IO3|TSC_GROUP2_IO3|TSC_GROUP3_IO2;
//  htsc.Init.ShieldIOs = 0;
//  htsc.Init.SamplingIOs = TSC_GROUP1_IO4|TSC_GROUP2_IO4|TSC_GROUP3_IO3;
//  if (HAL_TSC_Init(&htsc) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN TSC_Init 2 */

  /* USER CODE END TSC_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
//  hpcd_USB_FS.Instance = USB;
//  hpcd_USB_FS.Init.dev_endpoints = 8;
//  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
//  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
//  hpcd_USB_FS.Init.low_power_enable = DISABLE;
//  hpcd_USB_FS.Init.lpm_enable = DISABLE;
//  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
//  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin
                           LD4_Pin LD5_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/**
* @brief  Demo.
* @param  None
* @retval None
*/
static void Demo(void)
{
  /*  Blinking LEDs & Wait for User button to be pressed to switch to MEMES demo */
  while(BSP_PB_GetState(BUTTON_USER) != GPIO_PIN_SET)
  {
    /* Toggle LED3, LED4, LED5 & LED6 */
    BSP_LED_Toggle(LED3);
    BSP_LED_Toggle(LED4);
    BSP_LED_Toggle(LED5);
    BSP_LED_Toggle(LED6);

    /* Insert 100ms delay */
    HAL_Delay(100);
  }

  /* Wait for User button is released */
  while(BSP_PB_GetState(BUTTON_USER) != GPIO_PIN_RESET)
  {
  }

  /* Turn Off Leds */
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED6);

  /* Move Discovery board to execute MEMS demo, Mems detect the angular rate
  and led On corresponding to such direction*/
  MEMS_Test();

  /* Wait for User button to be pressed to switch to USB demo
  Mouse cursor moving corresponding to board move direction  */
 // USB_Demo();

  /* Wait for User button to be pressed to switch to Touch Sensor Test
  each bank pointed correspond to specific Leds On, test can performed
  in both direction */
  TSL_Test();
}

/**
* @brief  LED Test.
* @param  None
* @retval None
*/
static void LED_Test(void)
{
  while(ButtonPressed != 1)
  {
    /* Toggle LED3 */
    BSP_LED_Toggle(LED3);

    /* Insert 50ms delay */
    HAL_Delay(50);

    /* Toggle LED5 */
    BSP_LED_Toggle(LED5);

    /* Insert 50ms delay */
    HAL_Delay(50);

    if(ButtonPressed == 1)
    {
      BSP_LED_Off(LED3);
      BSP_LED_Off(LED4);
      BSP_LED_Off(LED5);
      BSP_LED_Off(LED6);
    }
    else
    {
      /* Toggle LED6 */
      BSP_LED_Toggle(LED6);

      /* Insert 50ms delay */
      HAL_Delay(50);

      /* Toggle LED4 */
      BSP_LED_Toggle(LED4);

      /* Insert 50ms delay */
      HAL_Delay(50);
    }
  }

  /* Wait for User button is released */
  while(BSP_PB_GetState(BUTTON_USER) != GPIO_PIN_RESET)
  {
  }

  /* Turn Off Leds */
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED6);
}

/**
* @brief  MEMS Test.
* @param  None
* @retval None
*/
#define size 30
static void Save(int ispisi){

	static float Buff[size][3];
	static char dioda[size];
	char str[8][10];
	char string[100];
	uint8_t Xval;
	uint8_t Yval;

	if(ispisi==1){
		//xSemaphoreTake(xMutex1Handle, Wait_Semaphore);
		for(int i=0; i<size; i++){
			itoa((int)Buff[i][0],str[0],10);
			itoa((int)Buff[i][1],str[1],10);
			itoa((int)Buff[i][2],str[2],10);
			//itoa((int)Buff[i][3],str[3],10);
			//itoa((int)Buff[i][4],str[4],10);
			//itoa((int)Buff[i][5],str[5],10);
			itoa((int)dioda[i],str[6],10);
			itoa(i+1,str[7],10);
			strcpy(string, "Podatak ");
			strcat(string, str[7]);
			strcat(string, ": ");
			strcat(string, str[0]);
			strcat(string, " ");
			strcat(string, str[1]);
			strcat(string, " ");
			strcat(string, str[2]);
			strcat(string, " ");
			/*strcat(string, str[3]);
			strcat(string, " ");
			strcat(string, str[4]);
			strcat(string, " ");
			strcat(string, str[5]);
			strcat(string, " ");*/
			strcat(string, str[6]);
			strcat(string, "\n");
		    //SEGGER_SYSVIEW_Print(string); //ovo zamijeniti sa usb transmit fs
			CDC_Transmit_FS((uint8_t*)string, strlen(string)); //ovo bi trebalo bit ok
		}
		//xSemaphoreGive(xMutex1Handle);
		return;
	}

	for(int i=size-1; i>0; i--){
		for(int j=0; j<3; j++)
			Buff[i][j]=Buff[i-1][j];
		dioda[i]=dioda[i-1];
	}

	float Buffer[6] = {0};
	Gyro_Buffer(Buffer, 0);
	for(int i=0; i<3; i++)
		Buff[0][i]=Buffer[i];

    Xval = ABS((int8_t)(Buffer[0]));
    Yval = ABS((int8_t)(Buffer[1]));

    if(Xval > Yval)
    {
      if(Buffer[0] > 15.0f) dioda[0]=5;
      if(Buffer[0] < -15.0f) dioda[0]=4;
    }
    else
    {
      if(Buffer[1] > 15.0f) dioda[0]=3;
      if(Buffer[1] < -15.0f) dioda[0]=6;
    }
	//char str[20];
	//int32_t x=xval;
	//itoa(x,str,10);
}

static void Gyro_Buffer (float* Buffer, int postavi){

	static float Buf[6]={0};
	if(postavi==1){
		for(int i=0; i<6; i++) Buf[i]=Buffer[i];
	}
	else{
		for(int i=0; i<6; i++) Buffer[i]=Buf[i];
	}
}

static void MEMS_Test(void)
{
  float Buffer[6] = {0};
  uint8_t Xval, Yval = 0;

  Gyro_Buffer(Buffer, 0);

    /* Update autoreload and capture compare registers value */
    Xval = ABS((int8_t)(Buffer[0]));
    Yval = ABS((int8_t)(Buffer[1]));
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED4);
    BSP_LED_Off(LED5);
    BSP_LED_Off(LED6);
    if(Xval > Yval)
    {
      if(Buffer[0] > 15.0f)
      {
        /*  LED5 On */
        //BSP_LED_Off(LED3);
        BSP_LED_Off(LED4);
        BSP_LED_On(LED5);
        //BSP_LED_Off(LED6);

        /* Insert 250ms delay */
        //HAL_Delay(250);
      }

      if(Buffer[0] < -15.0f)
      {
        /*  LED4 On */
        //BSP_LED_Off(LED3);
        BSP_LED_On(LED4);
        BSP_LED_Off(LED5);
        //BSP_LED_Off(LED6);

        /* Insert 250ms delay */
        //HAL_Delay(250);
      }
    }
    else
    {
      if(Buffer[1] > 15.0f)
      {
        /*  LED3 On */
        BSP_LED_On(LED3);
        //BSP_LED_Off(LED4);
        //BSP_LED_Off(LED5);
        BSP_LED_Off(LED6);

        /* Insert 250ms delay */
        //HAL_Delay(250);
      }

      if(Buffer[1] < -15.0f)
      {
        /*  LED6 On */
        BSP_LED_Off(LED3);
        //BSP_LED_Off(LED4);
        //BSP_LED_Off(LED5);
        BSP_LED_On(LED6);

        /* Insert 250ms delay */
        //HAL_Delay(250);
      }
    }


}

/**
* @brief  USB Mouse cursor moving
* @param  None
* @retval None
*/
static void USB_Demo(void)
{
//  uint8_t HID_Buffer[4];
//
//  BSP_LED_On(LED3);
//  BSP_LED_Off(LED6);
//
//  while ((BSP_PB_GetState(BUTTON_USER) != GPIO_PIN_SET))
//  {
//    USB_GetPointerData_Demo(HID_Buffer);
//
//    /* send data though IN endpoint*/
//    if((HID_Buffer[1] != 0) || (HID_Buffer[2] != 0))
//    {
//      USBD_HID_SendReport(&USBD_Device, HID_Buffer, 4);
//    }
//  }
//
//  /* Wait for User button is released */
//  while (BSP_PB_GetState(BUTTON_USER) != GPIO_PIN_RESET)
//  {}
//  /* Turn Off Leds */
//  BSP_LED_Off(LED3);
//  BSP_LED_Off(LED4);
//  BSP_LED_Off(LED5);
//  BSP_LED_Off(LED6);
}

/**
  * @brief  Calculate the angular Data rate Gyroscope.
  * @param  pfData : Data out pointer
  * @retval None
  */
static void Demo_GyroReadAngRate (float* pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;

  GYRO_IO_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
  GYRO_IO_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);

  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & L3GD20_BLE_MSB))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }

  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & L3GD20_FULLSCALE_SELECTION)
  {
  case L3GD20_FULLSCALE_250:
    sensitivity=L3GD20_SENSITIVITY_250DPS;
    break;

  case L3GD20_FULLSCALE_500:
    sensitivity=L3GD20_SENSITIVITY_500DPS;
    break;

  case L3GD20_FULLSCALE_2000:
    sensitivity=L3GD20_SENSITIVITY_2000DPS;
    break;

    default:
      sensitivity=L3GD20_SENSITIVITY_250DPS;
  }
  /* divide by sensitivity */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)(RawData[i] / sensitivity);
  }
}

/**
  * @brief  USB Test : Configure the USB
  * @param  None
  * @retval None
  */
static void USB_Test(void)
{
//  uint8_t HID_Buffer[4];
//
//  while ((BSP_PB_GetState(BUTTON_USER) != GPIO_PIN_SET))
//  {
//    USB_GetPointerData_Test(HID_Buffer);
//
//    /* send data though IN endpoint*/
//    if((HID_Buffer[1] != 0) || (HID_Buffer[2] != 0))
//    {
//      USBD_HID_SendReport(&USBD_Device, HID_Buffer, 4);
//      HAL_Delay (50);
//    }
//  }
//
//  /* Wait for User button is released */
//  while (BSP_PB_GetState(BUTTON_USER) != GPIO_PIN_RESET)
//  {}
//  /* Turn Off Leds */
//  BSP_LED_Off(LED3);
//  BSP_LED_Off(LED4);
//  BSP_LED_Off(LED5);
//  BSP_LED_Off(LED6);
}

/**
  * @brief  USBD_HID_GetPos
  * @param  None
  * @retval Pointer to report
*/
static void USB_GetPointerData_Test(uint8_t *pbuf)
{
 /* static int8_t x = 0;
  static int8_t y = 0;
  static int8_t Sens = 0;
  static int8_t Pas = 0;

  if (Pas == 20)
  {
    Pas=0;
    Sens++;
  }

  if(Sens == 0)
  {
    x=Pas++;
    y=0;
    BSP_LED_Toggle(LED5);
  }
  if(Sens == 1)
  {
    y=Pas++;
    x=0;
    BSP_LED_Toggle(LED6);
  }
  if (Sens == 2)
  {
    x=256-Pas++;
    y=0;
    BSP_LED_Toggle(LED4);
  }
  if (Sens == 3)
  {
    y=256-Pas++;
    x=0;
    BSP_LED_Toggle(LED3);
  }
  if (Sens == 4)
  {
    Sens=0;
    x=0;
    y=0;
  }

  pbuf[0] = 0;
  pbuf[1] = x;
  pbuf[2] = y;
  pbuf[3] = 0;
*/
}

/**
  * @brief  USBD_HID_GetPos
  * @param  None
  * @retval Pointer to report
  */
static void USB_GetPointerData_Demo(uint8_t *pbuf)
{
  /*static float Buffer[6] = {0};

  BSP_GYRO_Init();

   Read Gyro Angular data
  Demo_GyroReadAngRate(Buffer);

  pbuf[0] = 0;
  pbuf[1] = -(int8_t)(Buffer[2])/6;
  pbuf[2] = (int8_t)(Buffer[1])/6;
  pbuf[3] = 0;

  BSP_LED_Toggle(LED3);
  BSP_LED_Toggle(LED6);*/

}

/**
* @brief  TS Test.
* @param  None
* @retval None
*/
static void TSL_Test(void)
{
  /* Configure the TSC peripheral */
  TscHandle.Instance = TSCx;

  if (HAL_TSC_DeInit(&TscHandle) != HAL_OK)
  {
    Error_Handler();
  }

  TscHandle.Init.AcquisitionMode         = TSC_ACQ_MODE_NORMAL;
  TscHandle.Init.CTPulseHighLength       = TSC_CTPH_2CYCLES;
  TscHandle.Init.CTPulseLowLength        = TSC_CTPL_2CYCLES;
  TscHandle.Init.IODefaultMode           = /*TSC_IODEF_IN_FLOAT*/TSC_IODEF_OUT_PP_LOW;
  TscHandle.Init.MaxCountInterrupt       = DISABLE;
  TscHandle.Init.MaxCountValue           = TSC_MCV_8191;
  TscHandle.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV64;
  TscHandle.Init.SpreadSpectrum          = DISABLE;
  TscHandle.Init.SpreadSpectrumDeviation = 127;
  TscHandle.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  TscHandle.Init.SynchroPinPolarity      = TSC_SYNC_POLARITY_FALLING;
  /* All channel, shield and sampling IOs must be declared below */
  TscHandle.Init.ChannelIOs              = (TSC_GROUP1_IO3 | TSC_GROUP2_IO3 | TSC_GROUP3_IO2);
  TscHandle.Init.SamplingIOs             = (TSC_GROUP1_IO4 | TSC_GROUP2_IO4 | TSC_GROUP3_IO3);
  TscHandle.Init.ShieldIOs               = 0;

  if (HAL_TSC_Init(&TscHandle) != HAL_OK)
  {
    Error_Handler();
  }

  /* Init STMTouch driver */
  TSL_user_Init();





}

/**
  * @brief  Manage the activity on sensors when touched/released (example)
  * @param  None
  * @retval None
  */
static void ProcessSensors(void)
{
  //BSP_LED_Off(LED3);
  //BSP_LED_Off(LED4);
  //BSP_LED_Off(LED5);
  //BSP_LED_Off(LED6);
	uint8_t sen;
  if ((MyLinRots[0].p_Data->StateId == TSL_STATEID_DETECT) ||
      (MyLinRots[0].p_Data->StateId == TSL_STATEID_DEB_RELEASE_DETECT))
  {
    if (MyLinRots[0].p_Data->Position > 0)
    {
      //BSP_LED_On(LED6);
    	//SEGGER_SYSVIEW_Print("1");
    	sen=(uint8_t)0x00;
    }

    if (MyLinRots[0].p_Data->Position >= 48)
    {
    	//BSP_LED_On(LED5);
    	//SEGGER_SYSVIEW_Print("2");
    	sen=(uint8_t)0x10;
    }

    if (MyLinRots[0].p_Data->Position >= 80)
    {
    	//BSP_LED_On(LED3);
    	//SEGGER_SYSVIEW_Print("3");
    	sen=(uint8_t)0x10;
    }

    if (MyLinRots[0].p_Data->Position >= 112)
    {
    	//BSP_LED_On(LED4);
    	//SEGGER_SYSVIEW_Print("4");
    	sen=(uint8_t)0x20;
    }
    GYRO_IO_Write(&sen,L3GD20_CTRL_REG4_ADDR,1);
  }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == USER_BUTTON_PIN)
  {
    ButtonPressed = 0x01;
  }
}

/**
  * @brief  Acquisition completed callback in non blocking mode
  * @param  htsc: pointer to a TSC_HandleTypeDef structure that contains
  *         the configuration information for the specified TSC.
  * @retval None
  */
void HAL_TSC_ConvCpltCallback(TSC_HandleTypeDef* htsc)
{
#if TSLPRM_TSC_IODEF > 0 // Default = Input Floating
  /* Set IO default in Output PP Low to discharge all capacitors */
  HAL_TSC_IODischarge(htsc, ENABLE);
#endif

  Gv_EOA = 1; /* To inform the main loop routine of the End Of Acquisition */
}

/**
  * @brief  Error callback in non blocking mode
  * @param  htsc: pointer to a TSC_HandleTypeDef structure that contains
  *         the configuration information for the specified TSC.
  * @retval None
  */
void HAL_TSC_ErrorCallback(TSC_HandleTypeDef* htsc)
{
  Error_Handler();
}

static void LED_task_handler(void * p)
{

	uint32_t received_notification = 0;
	//BaseType_t notify_response = 0;
	while(1)
	{
		received_notification=ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		while(xSemaphoreTake(xMutex1Handle, Wait_Semaphore)!=pdTRUE){
			vTaskDelay(20/portTICK_PERIOD_MS);
		}
		MEMS_Test();
		xSemaphoreGive(xMutex1Handle);
		//HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		//vPortYield();
		//vTaskDelay(pdMS_TO_TICKS(750));

		vTaskDelay(10/portTICK_PERIOD_MS);
		//SEGGER_SYSVIEW_Print("LED");
	}
}


static void Save_task_handler(void * p)
{

	uint32_t received_notification = 0;
	//BaseType_t notify_response = 0;
	while(1)
	{
		received_notification=ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		Save(0);
		//SEGGER_SYSVIEW_Print("SAVE");
		//HAL_Delay(10);
	}
}

float procenat(int x, int y){
	if(y==0 && x>10) return 100;
	else if(y==0) return 0;
	float proc=1.0*x/y;
	proc-=1;
	proc*=100;
	proc=(proc<0) ? -proc : proc;
	return proc;
}

float aps(float x){
	return (x<0) ? -x : x;
}

#define THR 50

static void Gyro_task_handler(void * p)
{
	//uint32_t received_notification = 0;
	//BaseType_t notify_response = 0;
	float Buffer[6] = {0};
	float Buffer1[6] = {0};

	//uint8_t Xval, Yval = 0;

	if(BSP_GYRO_Init() != GYRO_OK)
	{
	  Error_Handler();
	}
	while(1)
	{
		Gyro++;
		while(xSemaphoreTake(xMutex1Handle, Wait_Semaphore)!=pdTRUE){
			vTaskDelay(20/portTICK_PERIOD_MS);
		}
		Gyro_Buffer(Buffer1, 0);
		Demo_GyroReadAngRate(Buffer);
		Gyro_Buffer(Buffer, 1);
		xSemaphoreGive(xMutex1Handle);
		auto x=ABS((int)Buffer[0]-(int)Buffer1[0]);
		auto y=ABS((int)Buffer[1]-(int)Buffer1[1]);

		if(x>THR || y>THR){

			xTaskNotifyGive( xispisAPTaskHandle );
		}

	    xTaskNotifyGive( xledTaskHandle );
		xTaskNotifyGive( xsaveTaskHandle );
	    //SEGGER_SYSVIEW_Print("GYRO");
	    vTaskDelay(100/portTICK_PERIOD_MS);
		//HAL_Delay(10);
	}
}


static void Touch_task_handler(void * p)
{
	//uint32_t received_notification = 0;
	//BaseType_t notify_response = 0;

	TSL_Test();
	while(1)
	{
	    if (TSL_user_Action() == TSL_STATUS_OK)
	    {
	      ProcessSensors(); // Execute sensors related tasks
	    }

	    //SEGGER_SYSVIEW_Print("TOUCH");
	    vTaskDelay(50/portTICK_PERIOD_MS);
		//HAL_Delay(10);
	}
}

static void Ispis_task_handler(void * p)
{
	//uint32_t received_notification = 0;
	//BaseType_t notify_response = 0;
	float Buffer[6] = {0};
	char str[3][20];
	char string[100];
	short int dioda;
	while(1)
	{
		vTaskDelay(300/portTICK_PERIOD_MS);
		while(xSemaphoreTake(xMutex1Handle, Wait_Semaphore)!=pdTRUE){
			vTaskDelay(20/portTICK_PERIOD_MS);
		}
	    Gyro_Buffer(Buffer, 0);
	    xSemaphoreGive(xMutex1Handle);
	    int32_t x=Buffer[0];
	    itoa(x,str[1],10);
	    //SEGGER_SYSVIEW_Print(str);

	    x=Buffer[1];
	    itoa(x,str[2],10);
	    uint8_t Xval = ABS((int8_t)(Buffer[0]));
	    uint8_t Yval = ABS((int8_t)(Buffer[1]));
	    if(Xval > Yval)
	        {
	          if(Buffer[0] > 15.0f) dioda=5;
	          else if(Buffer[0] < -15.0f) dioda=4;
	          else dioda=0;
	        }
	        else
	        {
	          if(Buffer[1] > 15.0f) dioda=3;
	          else if(Buffer[1] < -15.0f) dioda=6;
	          else dioda=0;
	    }
	    itoa(dioda,str[0],10);
	    strcpy(string, "GYR,");
	    strcat(string, str[0]);
	    strcat(string, ",");
	    strcat(string, str[1]);
	    strcat(string, ",");
	    strcat(string, str[2]);
	    strcat(string, "\n");
	    //SEGGER_SYSVIEW_Print(string); //zamijeniti sa usbom
	    CDC_Transmit_FS((uint8_t*)string, strlen(string));

	}
}

static void IspisAP_task_handler(void * p)
{
	uint32_t received_notification = 0;
	//BaseType_t notify_response = 0;
	float Buffer[6] = {0};
	char str[3][20];
	char string[100];
	short int dioda;
	while(1)
	{
		received_notification=ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		while(xSemaphoreTake(xMutex1Handle, Wait_Semaphore)!=pdTRUE){
			vTaskDelay(20/portTICK_PERIOD_MS);
		}
	    Gyro_Buffer(Buffer, 0);
	    xSemaphoreGive(xMutex1Handle);
	    int32_t x=Buffer[0];
	    	    itoa(x,str[1],10);
	    	    //SEGGER_SYSVIEW_Print(str);

	    	    x=Buffer[1];
	    	    itoa(x,str[2],10);
	    	    uint8_t Xval = ABS((int8_t)(Buffer[0]));
	    	    uint8_t Yval = ABS((int8_t)(Buffer[1]));
	    	    if(Xval > Yval)
	    	        {
	    	          if(Buffer[0] > 15.0f) dioda=5;
	    	          else if(Buffer[0] < -15.0f) dioda=4;
	    	          else dioda=0;
	    	        }
	    	        else
	    	        {
	    	          if(Buffer[1] > 15.0f) dioda=3;
	    	          else if(Buffer[1] < -15.0f) dioda=6;
	    	          else dioda=0;
	    	    }
	    	    itoa(dioda,str[0],10);
	    	    strcpy(string, "GYR_AP,");
	    	    strcat(string, str[0]);
	    	    strcat(string, ",");
	    	    strcat(string, str[1]);
	    	    strcat(string, ",");
	    	    strcat(string, str[2]);
	    	    strcat(string, "\n");
	    	    //SEGGER_SYSVIEW_Print(string); //Zamijeniti sa USBom
	    	    CDC_Transmit_FS((uint8_t*)string, strlen(string));
	}
}

static void Blink_task_handler(void * p)
{
	float Buffer[6] = {0};
	//uint32_t received_notification = 0;
	//BaseType_t notify_response = 0;

	while(1)
	{
		while(xSemaphoreTake(xMutex1Handle, Wait_Semaphore)!=pdTRUE){
			vTaskDelay(20/portTICK_PERIOD_MS);
		}
		Gyro_Buffer(Buffer, 0);
		xSemaphoreGive(xMutex1Handle);
		if((Buffer[0]<50 && Buffer[0]>-50) && (Buffer[1]<50 && Buffer[1]>-50)) BSP_LED_Off(LED6);
		else if((Buffer[0]<100 && Buffer[0]>-100) && (Buffer[1]<100 && Buffer[1]>-100)) BSP_LED_Toggle(LED6);
		else BSP_LED_On(LED6);
	    vTaskDelay(50/portTICK_PERIOD_MS);
	}
}

static void Sve_task_handler(void * p)
{
		//uint32_t received_notification = 0;
		//BaseType_t notify_response = 0;
		int32_t pinState = 0;
		int32_t oldState = pinState;
		int proslo=-1;
		while(1)
		{
			pinState = BSP_PB_GetState(BUTTON_USER);

					if (pinState != oldState)
					{
						if(pinState==1){
							proslo=CallCount;
						}
						if(pinState==0){
							if(CallCount>proslo+5) Save(1);
							//if(CallCount>proslo+10) BSP_LCD_GLASS_DisplayString(LCD_Task1String);
							//else BSP_LCD_GLASS_DisplayString(LCD_Task2String);
							//HAL_Delay(2000);

						}
						oldState = pinState;
						//xTaskNotifyGive(xTask2Handle);

					}

					vTaskDelay(100);

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
