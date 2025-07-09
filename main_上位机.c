/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <String.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum { MODE_VOLTAGE, MODE_CURRENT ,MODE_AUTO} CtrlMode;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim8_ch4_trig_com;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* �Զ���ȫ�ֱ��� */
vu8 ErrorCode=NO_ERROR,ValueKey=0xff;
struct FLAGTYPE Flag={0,0,0,0,0,0};		
vu16 TimeKey=0,TimeFlash=0,Screen_id,Control_id;
u8 X_Witch=8,Y_Witch=16,X_Witch_cn=16,Y_Witch_cn=16;
vu8 RxBuf[256],RxBufTop=0,RxBufBom=0,OV_State=STOP,ImgBuf[OV7725_H*OV7725_W_B];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* �Ա��ӳ������� */
void TestLedSpeak(void);				/* LED�������������뿪����ϵͳ���ʱ�Ӳ��� */
u8 ReadKey(void);								/* ������0��5 */
void TestStick(void);					/* ҡ�˿��ز��� */
void TestSpiLcd(void);					/* SPI�ӿ�Һ����ʾ��MzLH04���� */
void MzInit(void);			   		/* MzLH04��ʼ�� */
void MzClearScreen(void);		/* MzLH04���� */
void MzCommand(u8 i, u8 * Data);		/* ��MzLH04�������� */
void MzPutChar(u8 x,u8 y,u8 a); 		  	/* MzLH04��ָ��λ����ʾASCII�ַ� */
void MzPutString(u8 x,u8 y,u8 *p);		/* MzLH04��ָ��λ�ÿ�ʼ��ʾASCII�ַ��� */
void MzPutChar_cn(u8 x,u8 y,u8 * GB); 	/* MzLH04��ָ��λ����ʾ�����ַ� */
void MzPutString_cn(u8 x,u8 y,u8 *p);		/* MzLH04��ָ��λ�ÿ�ʼ��ʾ�����ַ��� */
void MzSetBackLight(u8 Deg); 	 			/* MzLH04���ñ�������ȵȼ� */
void MzShowChar(u8 x,u8 y,u8 a,u8 type); 	/* MzLH04��ָ��λ����ʾcharֵ */
void MzShowShort(u8 x,u8 y,u16 a,u8 type);/* MzLH04��ָ��λ����ʾshortֵ */
void TestAdc(void);						/* A/Dת������ */
//void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc);	/* ADת����ɻص����� */
void TestDac(void);						/* D/Aת������ */
void TestI2cEeprom(void);				/* I2C�ӿ�EEPROM��24C02������ */
void TestUsart2(void);					/* USART2���� */
void CopyStr(u8 *DstStr ,u8 * SrcStr);						/* �����ַ��� */
void SetBaudRate(u32 bps);			/* ���ò����� */
void USART2_SendStr(u8* Buf);			/* USART2 �����ַ��� */
void USART2_ReceiveStr(u8* Buf);		/* USART2 �����ַ��� */
void TestRtc(void);						/* RTC��BKP���� */
void Time_Adjust(void);			   		/* ����ʱ�� */
void Time_Show(void);					/* ��ʾʱ��ֵ */
//void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);		/* �����жϻص����� */
void TestTimePwm(void);					/* T1��PWM�����T2��PWM������� */
void TestEncoder(void);					/* T4�ı������������ */
void TestRF24L01(void);				/* ����ͨѶģ��nRF24L01���� */
void RF_Init(void);							/* RF24L01ģ���ʼ�� */
u8 RF_Read(u8 Cmd, u8 Num, u8* Buf);		/* RF24L01ִ�ж�����Cmd����Num�����ݶ��뻺����Buf */
u8 RF_Write(u8 Cmd, u8 Num, u8* Buf);		/* RF24L01ִ��д����Cmd����Num������д�뻺����Buf */
u8 BufCmp(u8 *Buf1, u8 *Buf2, u8 Num);		/* �Ƚ�����������Num�������Ƿ�һ�� */
void RF_Transmit(u8 Num, u8* Buf);		/* RF24L01���ͻ�����Buf�е�Num������ */
void TestOV7725(void);				/* ��ֵ������ͷOV7725���� */
u8 OV7725_Init(void);						/* OV7725��ʼ�� */
u8 SCCB_ReadReg(u8 OV_Reg);		/* SCCB�ӿڶ�ȡOV7725�Ĵ������� */
void OV_Bright(u8 Bright);		/* ����OV7725���� */
void OV_Cnst(u8 Cnst);		/* ����OV7725�Աȶ� */
void OV_Capture(void);				/* �ɼ�OV7725ͼ�� */
void OV_DMACaptureCplt(DMA_HandleTypeDef *hdma);	/* ͼ��ɼ�DMA������ɻص����� */
void MzShowPic(u8 x, u8 y,u8 w, u8 h);		/* ��MzLH04�ϴӣ�x��y����ʼ��ʾ��w*h���ߴ��ͼ�� */
void TestUsartLcd(void);				/* Usart�ӿڴ�������� */
void DC_BackColor(u16 Color);			/* ��������ñ���ɫ */
void DC_ClearScreen(void);				/* ��������� */
void USART2_SendCmd(u8 num,u8 * p);		/* USART2�������ݰ�������+���ݸ���Ϊnum������p��ַ�� */
void DC_DisplayScreen(u16 id);			/* ������л����� */
u8 DC_PressButton(void);				/* ��������ް�ť���� */
u16 DC_ReadScreen(void);				/* ������������ */
void EndTest(void);						/* �������� */

void SetLcd(float vv, float ii, float cv,float ci,CtrlMode mode);
void SetLcd_vi(float vv, float ii,CtrlMode mode);
void SetLcd_dc(float cv,float ci);
u8 DC_ReadButton(u8 id, u8* screen_id, u8* control_id);
float DC_ReadValue(u8 id);
void Send_irvr(uint32_t vr, uint32_t ir, u8 mode, bool if_send);

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
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		/*�Ա�������ʼ */
		//TestLedSpeak();					/* LED�������������뿪����ϵͳ���ʱ�Ӳ��� */
		//TestStick();					/* ҡ�˿��ز��� */
		//TestSpiLcd();					/* SPI�ӿ�Һ����ʾ��MzLH04���� */
		//TestAdc();						/* A/Dת������ */
		//TestDac();						/* D/Aת������ */
		//TestI2cEeprom();				/* I2C�ӿ�EEPROM��24C02������ */
		//TestUsart2();					/* USART2���� */
		//TestRtc();						/* RTC��BKP���� */
		//TestTimePwm();					/* T1��PWM�����T2��PWM������� */
		//TestEncoder();					/* T4�ı������������ */
		TestRF24L01();				/* ����ͨѶģ��24L01���� */
		//TestOV7725();					/* ��ֵ������ͷOV7725���� */
		//TestUsartLcd();					/* ���Usart�ӿ�LCD���� */
		
		EndTest();						/* �������� */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 6;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 7;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  hi2c2.Init.ClockSpeed = 200000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

		u16 i;

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  hrtc.Instance = RTC;			
	HAL_RTC_MspInit(&hrtc); 		/* �Ȱ�RTCʹ�ܣ����ܶ�дBKP2�Ĵ��� */
	i=HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);		 /*	��BKP2������ */
	if(i == 0xA5A5) 
	{										  /* �Ѿ����ù�RTC���˳�RTC��ʼ�������������в���������Time_Adjust */
		/**Enable the Alarm A */ 	/* ÿ�θ�λ��Ҫ��ʼ�����ӣ����򲻻����㱨ʱ */
		sAlarm.AlarmTime.Hours = 0;		/* ��������A����MX_RTC_Init��ʼ�������и��� */
		sAlarm.AlarmTime.Minutes = 59;
		sAlarm.AlarmTime.Seconds = 59;
		sAlarm.AlarmTime.SubSeconds = 51;		/* 51/256=0.2��������ǰ0.2���������� */
		sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
		sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS;
		sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
		sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
		sAlarm.AlarmDateWeekDay = 1;
		sAlarm.Alarm = RTC_ALARM_A;
		if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
		{
			Error_Handler();
		}
		Flag.RtcRst=0;
		return;
	}
  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR2,0xA5A5);
	Flag.RtcRst=1;				/* ��һ�����У���Ҫ�������µ�RTC��ʼ��������������ҲҪ����Time_Adjust */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 12;
  sTime.Minutes = 59;
  sTime.Seconds = 30;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 19;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 59;
  sAlarm.AlarmTime.Seconds = 59;
  sAlarm.AlarmTime.SubSeconds = 51;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 167;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0xffff;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPEAK_Pin|RF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_G_Pin|LED_Y_Pin|LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Mz_CS_Pin|Mz_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RF_CSN_GPIO_Port, RF_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : KEY3_Pin KEY4_Pin KEY5_Pin KEY6_Pin
                           KEY7_Pin KEY8_Pin KEY1_Pin KEY2_Pin */
  GPIO_InitStruct.Pin = KEY3_Pin|KEY4_Pin|KEY5_Pin|KEY6_Pin
                          |KEY7_Pin|KEY8_Pin|KEY1_Pin|KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SPEAK_Pin */
  GPIO_InitStruct.Pin = SPEAK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPEAK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_G_Pin LED_Y_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin|LED_Y_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Mz_CS_Pin Mz_RST_Pin */
  GPIO_InitStruct.Pin = Mz_CS_Pin|Mz_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EC_SW_Pin */
  GPIO_InitStruct.Pin = EC_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EC_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_CSN_Pin */
  GPIO_InitStruct.Pin = RF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RF_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_IRQ_Pin */
  GPIO_InitStruct.Pin = RF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_CE_Pin */
  GPIO_InitStruct.Pin = RF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RF_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OV_D0_Pin OV_D1_Pin OV_D2_Pin OV_D3_Pin
                           OV_D4_Pin OV_D5_Pin OV_D6_Pin OV_D7_Pin */
  GPIO_InitStruct.Pin = OV_D0_Pin|OV_D1_Pin|OV_D2_Pin|OV_D3_Pin
                          |OV_D4_Pin|OV_D5_Pin|OV_D6_Pin|OV_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VSYNC_Pin */
  GPIO_InitStruct.Pin = VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(VSYNC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/******************************************************************************/

void TestLedSpeak(void)					/* LED�������������뿪����ϵͳ���ʱ�Ӳ��� */
{										
	u8 KV;
										
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin | LED_G_Pin, GPIO_PIN_RESET);	  	/* ��ơ��Ƶơ��̵�=000 */		 
	
/* LED��˸��������2����1���� */
 	HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_SET);			  
 	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);		/* ��1������� */
	HAL_Delay(200);
 	HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_RESET);			  
 	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);		
	HAL_Delay(800);
 	HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_SET);			  
 	HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);		/* ��2���Ƶ��� */
	HAL_Delay(200);
 	HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_RESET);			  
 	HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_RESET);		
	HAL_Delay(800);
 	HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_SET);			  
 	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);		/* ��3���̵��� */
	HAL_Delay(500);
 	HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_RESET);			  
 	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);		
	
	Flag.KeyPress=0;
	
/* ��ơ��Ƶ����ݲ��뿪��λ���������̵�ʼ����˸ */	
LOOP:											  
	if(HAL_GPIO_ReadPin(KEY8_GPIO_Port,KEY8_Pin)==GPIO_PIN_SET ) HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);	/* ����1=OFFʱ�����˸ */
	if(HAL_GPIO_ReadPin(KEY7_GPIO_Port,KEY7_Pin)==GPIO_PIN_SET ) HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);	/* ����2=OFFʱ�Ƶ���˸ */
 	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);		/* �̵���˸ */
	HAL_Delay(250);
 	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin | LED_G_Pin, GPIO_PIN_RESET);			
	HAL_Delay(250);
	KV=ReadKey();
	if (KV==0xff) goto LOOP;			 /* ����һ���˳� */
}

/******************************************************************************/

u8 ReadKey(void)		/* ������0��5 */
{
	u8 KV,i;

	if(Flag.KeyPress==0) return 0xff;
	KV=ValueKey;
	Flag.KeyPress=0;
	for(i=0;i<6;i++)
	{
		if((KV & 0x01)==0) break;
		KV>>=1;
	}
	return i;
}

/******************************************************************************/

void TestStick(void)					/* ҡ�˿��ز��� */
{
	u8 KV,b,ModeLed;
	const u16 DataLed1[4]={0x0000,0x1000,0x3000,0x7000};		/* ModeLed=0 */
	const u16 DataLed2[4]={0x0000,0x1000,0x2000,0x4000};		/* ModeLed=1 */

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin | LED_G_Pin, GPIO_PIN_RESET);	  	/* ��ơ��Ƶơ��̵�=000 */		 
	b=0;										  
	ModeLed=0;			  		/* ����ģʽ */
	
/* �������ԣ���ơ��Ƶơ��̵��������������� */
LOOP:
	KV=ReadKey();
	switch(KV)
	{
		case UP:				/* �������� */
			if(b<3) b++;		
			ModeLed=0;
			break;
		case DOWN:				/* �������� */
			if(b!=0) b--;	   
			ModeLed=0;
			break;
		case LEFT:				/* ���ƽ��� */
			if(b!=0) b--;	   
			ModeLed=1;
			break;
		case RIGHT:				/* �������� */
			if(b<3) b++;	   
			ModeLed=1;
			break;
		case SEL:				/* ȫ����ȫ���л� */
			ModeLed=0;
			if(b==0)b=3;
			else b=0;
			break;
		case ESC:					/* ��ESC���������� */
			return;
		default:
			goto LOOP;
	}
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin | LED_G_Pin, GPIO_PIN_RESET);
	if(ModeLed==0) HAL_GPIO_WritePin(LED_R_GPIO_Port, DataLed1[b], GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED_R_GPIO_Port, DataLed2[b], GPIO_PIN_SET);
	goto LOOP;
}

/******************************************************************************/

void TestSpiLcd(void)					/* SPI�ӿ�Һ����ʾ��MzLH04���� */
{
	u8 KV,b;

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin, GPIO_PIN_RESET);	 /* ��ơ��Ƶơ��̵�=001 */
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);	 	

/* MzLH04��ʾ */
	MzInit();
	MzClearScreen();
	{
		u8 BUF[]={0x81,0x01};
		MzCommand(2, BUF);			  /* ѡ��ASCII�ַ�0(6X10),�ַ�ɫΪ1(��ɫ) */
		X_Witch = 6;
		Y_Witch = 10;
	}
	MzPutChar(10,0,'A');				 /* ��ָ��λ����ʾ�ַ�A */
	{
		u8 BUF[]={0x81,0x11};
		MzCommand(2, BUF);			  /* ѡ��ASCII�ַ�1(8X16),�ַ�ɫΪ1(��ɫ) */
		X_Witch = 8;
		Y_Witch = 16;
	}
	MzPutChar(20,0,'A');				 /* ��ָ��λ����ʾ�ַ�A */
	{
		u8 BUF[]={0x05,48,10,10};
		MzCommand(4, BUF);			  /* ��ָ��λ�û���һ��Բ�ο� */
	}
	{
		u8 BUF[]={0x04,64,0,127,20};
		MzCommand(5, BUF);			  /* ��ָ��λ�û���һ����ɫ�ľ��������� */
	}
	{
		u8 BUF[]={0x81,0x10};
		MzCommand(2, BUF);			  /* ѡ��ASCII�ַ�1(8X16),�ַ�ɫΪ0(��ɫ) */
	}
	{
		u8 BUF[]={0x89,0x01};
		MzCommand(2, BUF);			  /* �����ַ�����ģʽΪ��ֹ���ַ���������ɫΪ1 */
	}
	MzPutChar(70,1,'B');				 /* ��ָ��λ����ʾ�ַ�B */
	{
		u8 BUF[]={0x81,0x11};
		MzCommand(2, BUF);			  /* ѡ��ASCII�ַ�1(8X16),�ַ�ɫΪ1(��ɫ) */
	}
	{
		u8 BUF[]={0x89,0x10};
		MzCommand(2, BUF);			  /* �����ַ�����ģʽΪʹ�ܣ��ַ���������ɫΪ0 */
	}
	MzPutChar(80,1,'a');				 /* ��ָ��λ����ʾ�ַ�a */
	MzPutString(0,17,(u8 *)"EeDesign");		/* ��ָ��λ�ÿ�ʼ��ʾASCII�ַ��� */
	{
		u8 BUF[]={0x81,0x01};
		MzCommand(2, BUF);			  /* ѡ��ASCII�ַ�0(6X10),�ַ�ɫΪ1(��ɫ) */
		X_Witch = 6;
		Y_Witch = 10;
	}
	MzPutString(66,23,(u8 *)"--MzLH04");		/* ��ָ��λ�ÿ�ʼ��ʾASCII�ַ��� */
	{
		u8 BUF[]={0x82,0x11};
		MzCommand(2, BUF);			  /* ѡ�����ַ�1(16X16),�ַ�ɫΪ1(��ɫ) */
		X_Witch_cn = 16;
		Y_Witch_cn = 16;
	}
	MzPutChar_cn(10,33,(u8 *)"��");			  /* ��ָ��λ����ʾ���֡��ԡ� */
	MzPutString_cn(40,33,(u8 *)"�繤����");	  /* ��ָ��λ�ÿ�ʼ��ʾ�����ַ��� */
	{
		u8 BUF[]={0x82,0x01};
		MzCommand(2, BUF);			  /* ѡ�����ַ�1(12X12),�ַ�ɫΪ1(��ɫ) */
		X_Witch_cn = 12;
		Y_Witch_cn = 12;
	}
	MzPutChar_cn(10,50,(u8 *)"��");			 /* ��ָ��λ����ʾ���֡��ԡ� */
	MzPutString_cn(40,50,(u8 *)"���ֿ�Һ��");/* ��ָ��λ�ÿ�ʼ��ʾ�����ַ��� */  
/* ���ñ�������ȵȼ� */	
	b=0;
	MzSetBackLight(b);				
	MzShowChar(104,50,b,0);			/* ��ָ��λ����ʾ����ȼ� */
LOOP:
	KV=ReadKey();
	switch(KV)
	{
		case UP:					/* ���󱳹�ȼ� */
			if(b<127) b++;
			break;
		case DOWN:					/* ��С����ȼ� */
			if(b!=0) b--;
			break;
		case LEFT:				   	/* �������ȼ� */
			if(b>10) b-=10;
			else b=0;
			break;
		case RIGHT:					/* ��������ȼ� */
			if(b<117) b+=10;
			else b=127;
			break;
		case SEL:					/* ����ȼ���Ϊ100 */
			b=100;
			break;
		case ESC:					/* ��ESC���������� */
			return;
		default:
			goto LOOP;
	}
	MzSetBackLight(b);				/* ���ñ�������ȵȼ� */
	MzShowChar(104,50,b,0);			/* ��ָ��λ����ʾ����ȼ� */
	goto LOOP;
}

/*****************************************************************************/

void MzInit(void)			   /* MzLH04��ʼ�� */
{
	HAL_GPIO_WritePin(Mz_RST_GPIO_Port, Mz_RST_Pin, GPIO_PIN_RESET);		/* Mz_RST=0 */
	HAL_Delay(20);		/* MzLH04Ҫ��λ�͵�ƽ���ٳ���2ms */
	HAL_GPIO_WritePin(Mz_RST_GPIO_Port, Mz_RST_Pin, GPIO_PIN_SET);			/* Mz_RST=1 */
	HAL_Delay(100);		/* MzLH04Ҫ��λ��ߵ�ƽ���ٵȴ�10ms */
}

/*****************************************************************************/

void MzClearScreen(void)		/* MzLH04���� */
	{
		u8 BUF[]={0x80};
		MzCommand(1, BUF);			  
	}

/*****************************************************************************/

void MzCommand(u8 i, u8 * Data)		/* ��MzLH04�������� */
{
	HAL_StatusTypeDef State;

	HAL_GPIO_WritePin(Mz_CS_GPIO_Port, Mz_CS_Pin, GPIO_PIN_RESET);		/* Mz_CS=0 */
	State=HAL_SPI_Transmit (&hspi2, Data, i, 10);		/* ��ʱʱ����Ϊ10ms */
	while(State!=HAL_OK);
	HAL_GPIO_WritePin(Mz_CS_GPIO_Port, Mz_CS_Pin, GPIO_PIN_SET);			/* Mz_CS=1 */
}

/*****************************************************************************/

void MzPutChar(u8 x,u8 y,u8 a) 		  /* MzLH04��ָ��λ����ʾASCII�ַ� */
{
	u8 BUF[4];

	BUF[0]=	0x07;
	BUF[1]=x;
	BUF[2]=y;
	BUF[3]=a;
	MzCommand(4, BUF);			  
}

/*****************************************************************************/

void MzPutString(u8 x,u8 y,u8 *p)		 /* MzLH04��ָ��λ�ÿ�ʼ��ʾASCII�ַ��� */
{
	while(*p!=0)
	{
		MzPutChar(x,y,*p);
		x += X_Witch;
		if(x > (128-X_Witch) )
		{
			x = 0;
			y += Y_Witch;
			if(y > (64 - Y_Witch)) y=0;
		}
		p++;
	}
}

/*****************************************************************************/

void MzPutChar_cn(u8 x,u8 y,u8 * GB) 			  /* MzLH04��ָ��λ����ʾ�����ַ� */
{
	u8 BUF[5];

	BUF[0]=	0x08;
	BUF[1]=x;
	BUF[2]=y;
	BUF[3]=GB[0];
	BUF[4]=GB[1];
	MzCommand(5, BUF);			  
}

/*****************************************************************************/

void MzPutString_cn(u8 x,u8 y,u8 *p)		 /* MzLH04��ָ��λ�ÿ�ʼ��ʾ�����ַ��� */
{
	while(*p!=0)
	{
		if(*p<0x80)
		{
			MzPutChar(x,y,*p);
			x += X_Witch;
			if(x > (128-X_Witch) )
			{
				x = 0;
				y += Y_Witch;
				if(y > (64 - Y_Witch)) y=0;
			}
			p++;
		}
		else
		{
			MzPutChar_cn(x,y,p);
			x += X_Witch_cn;
			if(x > (128-X_Witch_cn) )
			{
				x = 0;
				y += Y_Witch_cn;
				if(y > (64 - Y_Witch_cn)) y=0;
			}
			p+=2;
		}
	}
}

/*****************************************************************************/

void MzSetBackLight(u8 Deg) 	 /* MzLH04���ñ�������ȵȼ� */
{
	u8 BUF[2];

	BUF[0]=0x8a;
	BUF[1]=Deg;
	MzCommand(2, BUF);			  
}

/*****************************************************************************/

void MzShowChar(u8 x,u8 y,u8 a,u8 type) 	 /* MzLH04��ָ��λ����ʾcharֵ */
{
	u8 BUF[5];

	BUF[0]=0x0b;
	BUF[1]=x;
	BUF[2]=y;
	BUF[3]=a;
	BUF[4]=type;	/* 0:3λ����1���������Ч����2���Ҷ�����Ч�� */
	MzCommand(5, BUF);			  
}

/*****************************************************************************/

void MzShowShort(u8 x,u8 y,u16 a,u8 type)    /* MzLH04��ָ��λ����ʾshortֵ */
{
	u8 BUF[6];

	BUF[0]=0x0c;
	BUF[1]=x;
	BUF[2]=y;
	BUF[3]=a>>8;
	BUF[4]=a;
	BUF[5]=type;		/* 0:5λ����1���������Ч����2���Ҷ�����Ч�� */
	MzCommand(6, BUF);			  
}

/*****************************************************************************/

void TestAdc(void)						/* A/Dת������ */
{
	u8 KV,Buf[8];
	u16 i;
	vu16  AdcBuf[10];
	float f;

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_G_Pin, GPIO_PIN_RESET);	 /* ��ơ��Ƶơ��̵�=010 */
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);	 	
	
	MzSetBackLight(100);				  
	MzClearScreen();
	{
		u8 BUF[]={0x81,0x01};
		MzCommand(2, BUF);			  /* ѡ��ASCII�ַ�0(6X10),�ַ�ɫΪ1(��ɫ) */
		X_Witch = 6;
		Y_Witch = 10;
	}
	{
		u8 BUF[]={0x89,0x10};
		MzCommand(2, BUF);			  /* �����ַ�����ģʽΪ�������ַ���������ɫΪ0 */
	}
	MzPutString(0,0,(u8 *)"AD1=");		MzPutString(42,0,(u8 *)"AD2=");	   MzPutString(85,0,(u8 *)"AD3=");
	MzPutString(0,21,(u8 *)"AD4=");		MzPutString(42,21,(u8 *)"AD5=");	 MzPutString(85,21,(u8 *)"AD6=");
	MzPutString(0,42,(u8 *)"Temp=");	MzPutString(42,42,(u8 *)"Vint=");	 MzPutString(85,42,(u8 *)"Vbat=");
	
/* A/Dת������ */
LOOP:
	if(Flag.Flash)					  /* ��ʾˢ��ʱ */
	{
		Flag.Temp=0;
		if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *)AdcBuf,9)!=HAL_OK)	/*ע�⣺�����еġ�9��һ��Ҫ��ADת��һ��ɨ���ͨ����һ�� */
		{
			/* Start Error */
			Error_Handler();
		}
		while(Flag.Temp==0);		/* �ȴ�ת����� */
		MzShowShort(0,10,AdcBuf[0],1);	MzShowShort(42,10,AdcBuf[1],1);  MzShowShort(85,10,AdcBuf[2],1);
		MzShowShort(0,31,AdcBuf[3],1);	MzShowShort(42,31,AdcBuf[4],1);  MzShowShort(85,31,AdcBuf[5],1);
//		MzShowShort(0,52,AdcBuf[6],1);	MzShowShort(42,52,AdcBuf[7],1);  MzShowShort(85,52,AdcBuf[8],1);
		f=0.24414*AdcBuf[6]-279;	/* ������¶ȣ�T=((D/4096)*2500-760)/2.5 + 25 �� */
		i=f*100;
		Buf[0]=i/1000+'0';	i=i%1000;
		Buf[1]=i/100+'0';	i=i%100; 
		Buf[2]='.';
		Buf[3]=i/10+'0';
		Buf[4]=i%10+'0';
		Buf[5]=0;
		MzPutString(0,52,Buf);
		MzPutString_cn(30,52,(u8 *)"��");
		f=0.61035*AdcBuf[7];	/* ����ɵ�ѹ��V=(D/4096)*2500 mV */
		i=f;
		Buf[0]=i/1000+'0';	i=i%1000;
		Buf[1]='.';
		Buf[2]=i/100+'0';	i=i%100;
		Buf[3]=i/10+'0';
		Buf[4]=i%10+'0';
		Buf[5]='V';
		Buf[6]=0;
		MzPutString(42,52,Buf);
		f=1.2207*AdcBuf[8];	/* ����ɵ�ѹ��V=(D/4096)*2500*2 mV */
		i=f;
		Buf[0]=i/1000+'0';	i=i%1000;
		Buf[1]='.';
		Buf[2]=i/100+'0';	i=i%100;
		Buf[3]=i/10+'0';
		Buf[4]=i%10+'0';
		Buf[5]='V';
		Buf[6]=0;
		MzPutString(85,52,Buf);
		Flag.Flash=0;
	}
	KV=ReadKey();
	if(KV==SEL)	
	{
		while(ReadKey()==0xff);			/* ��SEL����ͣ���� */
		goto LOOP;
	}
	if(KV==ESC) return;					/* ��ESC���������� */
	goto LOOP;
}

/*****************************************************************************/

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)	/* ADת����ɻص����� */
{
	HAL_ADC_Stop_DMA(hadc);
	Flag.Temp=1;
}

/*****************************************************************************/

void TestDac(void)						/* D/Aת������ */
{
	u8 KV;
	u16 i;
	vu16  AdcBuf[10];
	u32 i32,j32;

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);	 /* ��ơ��Ƶơ��̵�=011 */
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_Y_Pin | LED_G_Pin, GPIO_PIN_SET);	 	

	MzClearScreen();
	MzPutString(0,0,(u8 *)"DA1=");		MzPutString(42,0,(u8 *)"DA2=");	  
	MzPutString(0,21,(u8 *)"AD1=");		MzPutString(42,21,(u8 *)"AD2=");	   MzPutString(85,21,(u8 *)"AD3=");
	MzPutString(0,42,(u8 *)"AD4=");		MzPutString(42,42,(u8 *)"AD5=");	   MzPutString(85,42,(u8 *)"AD6=");
	i32=1000; j32=3000;
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, i32);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, j32);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

	/* D/Aת������ */
LOOP:
	if(Flag.Flash)					 /* ��ʾˢ��ʱ */
	{
		Flag.Temp=0;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)AdcBuf,9);	/* ��Ȼֻ��Ҫ6��ͨ��AD�����������ͨ����9����ָ��ͨ�����붼ת�� */
		while(Flag.Temp==0);		/* �ȴ�ת����� */
		MzShowShort(0,10,i32,1);	MzShowShort(42,10,j32,1);  
		MzShowShort(0,31,AdcBuf[0],1);	MzShowShort(42,31,AdcBuf[1],1);  MzShowShort(85,31,AdcBuf[2],1);
		MzShowShort(0,52,AdcBuf[3],1);	MzShowShort(42,52,AdcBuf[4],1);  MzShowShort(85,52,AdcBuf[5],1);
		Flag.Flash=0;
	}
	i=KEY8_GPIO_Port->IDR;		/* ���Բ��뿪��λ�� */
	KV=ReadKey();
	switch(KV)
	{
		case UP:								  /* DA1�Ӵ� */
			if((i & KEY8_Pin)==0)
			{
				if (i32<4085) i32+=10;	  /* ��� */
				else i32=4095;
			}
			else
			{
				if(i32<4095) i32++;			 /* ���� */
			}
			break;
		case DOWN:								 /* DA1��С */
			if((i & KEY8_Pin)==0)
			{
				if (i32>10) i32-=10;	 /* ��� */
				else i32=0;
			}
			else
			{
				if(i32!=0) i32--;				 /* ���� */
			}
			break;
		case LEFT:								 /* DA2��С */
			if((i & KEY7_Pin)==0)
			{
				if (j32>10) j32-=10;	/* ��� */
				else j32=0;
			}
			else
			{
				if(j32!=0) j32--;				/* ���� */
			}
			break;
		case RIGHT:								/* DA2�Ӵ� */
			if((i & KEY7_Pin)==0)
			{
				if (j32<4085) j32+=10;	/* ��� */
				else j32=4095;
			}
			else
			{
				if(j32<4095) j32++;		   /* ���� */
			}
			break;
		case SEL:							   /* DA1��DA2��ȱʡֵ */
			i32=1000;
			j32=3000;
			break;
		case ESC:
			return;						   /* ��ESC���������� */
		default:
			goto LOOP;
	}
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, i32);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, j32);
	goto LOOP;
}

/******************************************************************************/
                                      
void TestI2cEeprom(void)				/* I2C�ӿ�EEPROM��24C02������ */
{
	u8 KV,b;

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_Y_Pin | LED_G_Pin, GPIO_PIN_RESET);	 /* ��ơ��Ƶơ��̵�=100 */
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);	 	

/* 24C02 ���� */
LOOP1:
	MzClearScreen();
	MzPutString(0,0,(u8 *)"E2PROM(0x5A)=");			
	MzPutString(0,20,(u8 *)"Value=");	  
	MzPutString(0,30,(u8 *)"PRESS SEL TO SAVE...");	  
	/*	��������ַΪ0xa0���洢��ַΪ0x5a��EEPROM��ȡ1���ֽ����ݣ���ʱʱ��Ϊ1ms */	
	if(HAL_I2C_Mem_Read(&hi2c2, 0xa0, 0x5a, I2C_MEMADD_SIZE_8BIT, &b, 1, 1) != HAL_OK)		
 {
    /* Reading process Error */
    Error_Handler();
  }
	MzShowChar(78,0,b,1);	
	MzShowChar(36,20,b,1);  

LOOP2:
	KV=ReadKey();
	switch(KV)
	{
		case UP:
			if(b<255) b++;			/* ���� */
			break;
		case DOWN:
			if(b!=0) b--;			/* ���� */
			break;
		case LEFT:
			if(b>10) b-=10;			/* ��� */
			else b=0;
			break;
		case RIGHT:
			if(b<245) b+=10;		/* ��� */
			else b=255;
			break;
		case SEL:
			/*	��SEL����������ַΪ0xa0���洢��ַΪ0x5a��EEPROMд��1���ֽ����ݣ���ʱʱ��Ϊ1ms */	
			if(HAL_I2C_Mem_Write(&hi2c2, 0xa0, 0x5a, I2C_MEMADD_SIZE_8BIT, &b, 1, 1)!= HAL_OK)
			{
				/* Writing process Error */
				Error_Handler();
			}
			HAL_Delay(10);		/* �ȴ�EEPROM�ڲ�д��10ms��� */
			goto LOOP1;
		case ESC:					
			return;					/* ��ESC���������� */
		default:
			goto LOOP2;
	}
	MzShowChar(36,20,b,1);
	goto LOOP2;
}

/******************************************************************************/

void TestUsart2(void)					/* USART2���� */
{	/* ��HAL������ʼ������Ҫ�����������ݣ�������HAL���շ���ֻ����HAL�����Ĵ������� */
	u8 KV,bi=0,b=1;
	u8 BufTxdStr[][20]={"BaudRate =  19200","WordLength = 8b","StopBits = 1","Parity = No"};
	u8 BufBaudStr[][20]={"BaudRate =   9600","BaudRate =  19200","BaudRate =  38400","BaudRate = 115200"};
	u8 BufRxdStr[256];
	u32 bps[]={9600,19200,38400,115200};

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_Y_Pin, GPIO_PIN_RESET);	 /* ��ơ��Ƶơ��̵�=101 */
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_G_Pin, GPIO_PIN_SET);	 	
	
	RxBufTop=0;		/* ���ջ�����ָ������ */
	RxBufBom=0;
	SetBaudRate(bps[b]);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
	__HAL_UART_ENABLE(&huart2);											/* ������ */	
	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);	/* ����ձ�־ */
	
	MzClearScreen();
	MzPutString(0,0,BufBaudStr[b]);
	MzPutString(0,10,(u8 *)"PRESS SEL TO SEND...");
	MzPutString(0,20,(u8 *)"USART2 OUTPUT:");
	MzPutString(0,30,BufTxdStr[0]);
	MzPutString(0,40,(u8 *)"USART2 INPUT:");	
	
/*  USART2 ���� */
LOOP:
	KV=ReadKey();
	switch(KV)
	{
		case UP:
			if(b<3) b++;			/* ���������� */
			break;
		case DOWN:
			if(b!=0) b--;			/* �����ʼ�С */
			break;
		case SEL:
			USART2_SendStr(BufTxdStr[bi]);	 /* �����ַ��� */
			bi++;
			bi &=0x03;	
			HAL_Delay(10);	/* �ȴ���λ�Ĵ����ͷ��ͼĴ����е����ݷ������ */
			USART2_ReceiveStr(BufRxdStr);	 /* �����ַ��� */
			MzClearScreen();
			MzPutString(0,0,BufBaudStr[b]);
			MzPutString(0,10,(u8 *)"PRESS SEL TO SEND...");
			MzPutString(0,20,(u8 *)"USART2 OUTPUT:");
			MzPutString(0,30,BufTxdStr[bi]);
			MzPutString(0,40,(u8 *)"USART2 INPUT:");	
			MzPutString(0,50,BufRxdStr);
			goto LOOP;
		case ESC:
			__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
			__HAL_UART_DISABLE(&huart2);		/* �ش��� */
			return;					/* ��ESC���������� */
		default:
			goto LOOP;
	}
	CopyStr(BufTxdStr[0],BufBaudStr[b]);						/* �����ַ��� */
	MzPutString(0,0,BufBaudStr[b]);
	__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
	__HAL_UART_DISABLE(&huart2);		/* �ش��� */
	SetBaudRate(bps[b]);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
	__HAL_UART_ENABLE(&huart2);											/* ������ */	
	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);	/* ����ձ�־ */
	goto LOOP;
}

/******************************************************************************/

void CopyStr(u8 *DstStr ,u8 * SrcStr)						/* ��SrcStr�����ַ�����DstStr */
{
	while(*SrcStr!=0)
	{
		*DstStr = *SrcStr;
		DstStr++;
		SrcStr++;
	}
	*DstStr = *SrcStr;
}

/******************************************************************************/

void SetBaudRate(u32 bps)			/* ���ò����� */
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = bps;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}


/******************************************************************************/

void USART2_SendStr(u8* Buf)			 /* USART2 �����ַ��� */
{
	while(*Buf!=0)
	{
	 	while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == RESET);
		__HAL_UART_FLUSH_DRREGISTER(&huart2)=*Buf;		/* д���ͼĴ��� */
		Buf++;
	}
}

/******************************************************************************/

void USART2_ReceiveStr(u8* Buf)			 /* USART2 �����ַ��� */
{
	u8 bi=0;

  __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);	   /* ���ж� */
	while(RxBufBom!=RxBufTop)
	{
		Buf[bi]=RxBuf[RxBufBom];
		bi++;
		RxBufBom++;
	}
	Buf[bi]=0;
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
}

/*****************************************************************************/

void TestRtc(void)						/* RTC��BKP���� */
{
	u8 KV;
//	RTC_AlarmTypeDef sAlarm;

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);	 /* ��ơ��Ƶơ��̵�=110 */
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin, GPIO_PIN_SET);	 	

	MzClearScreen();
	if(Flag.RtcRst == 1) 	/* MX_RTC_Init��������û����룬��־λ���������� */
	{										  /* ��һ�����У���û������RTC */
		MzPutString(0,0,(u8 *)"RTC not yet configured...");				
		MzPutString(0,20,(u8 *)"PRESS ANY KEY TO CONTINUE...");
		while(ReadKey()==0xff)
   	Time_Adjust();					   	  /* ����ʱ�� */
		Flag.RtcRst=0;
		MzClearScreen();
		MzPutString(0,0,(u8 *)"RTC configured! ");	  
  }
  else
 	{										 /* �Ѿ����ù�RTC */
   	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) MzPutString(0,0,(u8 *)"Power On Reset occurred...");
   	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) MzPutString(0,0,(u8 *)"External Reset occurred...");
		else MzPutString(0,0,(u8 *)"No Reset occurred...");
		MzPutString(0,20,(u8 *)"No need to configure RTC!");	  
 	}
	__HAL_RCC_CLEAR_RESET_FLAGS();		 /* Clear reset flags */
	MzPutString(0,40,(u8 *)"PRESS ANY KEY TO CONTINUE...");
	while(ReadKey()==0xff);
LOOP1:
	MzClearScreen();
LOOP2:
 	Time_Show();					 /* ��ʾʱ��ֵ */
	KV=ReadKey();
	if(KV==ESC) return;
	if(KV==SEL) 
	{
		Time_Adjust();
		goto LOOP1;
	}
	goto LOOP2;
}

/*****************************************************************************/

void Time_Adjust(void)			   /* ����ʱ�� */
{
	u8 b,KV,Db[7];
	u8 strDate[]="Date:  -  -  ( )";
	u8 strTime[]="Time:  :  :  ";
	u8 str[7][16]={"Year=   ","Month=  ","Date=   ","WeekDay=","Hours=  ","Minutes=","Seconds="};
	uc8 Dmax[7]={99,12,31,7,23,59,59};
	uc8 Dmin[7]={0,1,1,1,0,0,0};
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	MzClearScreen();
	b=0;												/* Ҫ��������Ŀ��� */
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);	/* �ȶ�ʱ�������ڲ������ */
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	Db[0]=sDate.Year;	Db[1]=sDate.Month;	Db[2]=sDate.Date;	Db[3]=sDate.WeekDay;
	Db[4]=sTime.Hours;	Db[5]=sTime.Minutes;	Db[6]=sTime.Seconds;
LOOP0:
	sDate.Year=Db[0];	sDate.Month=Db[1];	sDate.Date=Db[2];	sDate.WeekDay=Db[3];
	sTime.Hours=Db[4];	sTime.Minutes=Db[5];	sTime.Seconds=Db[6];
		strDate[5]=sDate.Year/10+'0';	strDate[6]=sDate.Year%10+'0';
		strDate[8]=sDate.Month/10+'0';	strDate[9]=sDate.Month%10+'0';
		strDate[11]=sDate.Date/10+'0';	strDate[12]=sDate.Date%10+'0';
		strDate[14]=sDate.WeekDay%10+'0';
		MzPutString(0,0,strDate);
		strTime[5]=sTime.Hours/10+'0';	strTime[6]=sTime.Hours%10+'0';
		strTime[8]=sTime.Minutes/10+'0';	strTime[9]=sTime.Minutes%10+'0';
		strTime[11]=sTime.Seconds/10+'0';	strTime[12]=sTime.Seconds%10+'0';
		MzPutString(0,10,strTime);
LOOP1:	
	MzPutString(0,40,str[b]);				
	MzShowShort(48,40,Db[b],1); 
LOOP2: 
	KV=ReadKey();
	switch(KV)
	{
		case UP:					/* ʱ��ֵ��1 */
			Db[b]++;					
			if(Db[b]>Dmax[b]) Db[b]=Dmin[b];
			goto LOOP1;
		case DOWN:					/* ʱ��ֵ��1 */
			if(Db[b]==Dmin[b]) Db[b]=Dmax[b];		
			else Db[b]=Db[b]-1;
			goto LOOP1;
		case LEFT:					/* ʱ�������� */
			if(b==0) b=6;
			else b--;
			goto LOOP1;
		case RIGHT:					/* ʱ�������� */
			b++;
			if(b>6) b=0;
			goto LOOP1;
		case SEL:
			goto LOOP0;
		case ESC:
			goto Next;
		default:
			goto LOOP2;
	}
Next:
	MzPutString(0,50,(u8 *)"SAVE?");
LOOP3:
	KV=ReadKey();
	if(KV==ESC) return;			/* ׼���˳� */
	if(KV==SEL)						/* ������ֵ */
	{
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;		/* HAL_RTC_GetTime�����û����д�������������� */
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;	/* HAL_RTC_GetDate�����û����д�������������� */
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		return;
	}
	goto LOOP3;
}

/*****************************************************************************/

void Time_Show(void)		/* ��ʾʱ��ֵ */
{
  RTC_DateTypeDef dDate;
  RTC_TimeTypeDef dTime;
	u8 strDate[]="Date:  -  -  ( )";
	u8 strTime[]="Time:  :  :  ";
	static u8 Sec=255;

	HAL_RTC_GetTime(&hrtc, &dTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &dDate, RTC_FORMAT_BIN);
	if (dTime.Seconds != Sec)	  /* ��ʾÿ1��ˢ��һ�� */
  {
		strDate[5]=dDate.Year/10+'0';	strDate[6]=dDate.Year%10+'0';
		strDate[8]=dDate.Month/10+'0';	strDate[9]=dDate.Month%10+'0';
		strDate[11]=dDate.Date/10+'0';	strDate[12]=dDate.Date%10+'0';
		strDate[14]=dDate.WeekDay%10+'0';
		MzPutString(0,0,strDate);
		strTime[5]=dTime.Hours/10+'0';	strTime[6]=dTime.Hours%10+'0';
		strTime[8]=dTime.Minutes/10+'0';	strTime[9]=dTime.Minutes%10+'0';
		strTime[11]=dTime.Seconds/10+'0';	strTime[12]=dTime.Seconds%10+'0';
		MzPutString(0,10,strTime);
		Sec = dTime.Seconds;
  }
}

/*****************************************************************************/

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)						/* �����жϻص����� */
{
  RTC_DateTypeDef dDate;
  RTC_TimeTypeDef dTime;

	HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_SET);			/* ���㱨ʱ */  
LOOP1:
	HAL_RTC_GetTime(hrtc, &dTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &dDate, RTC_FORMAT_BIN);			/* ��ʹû��ҲҪ���ϣ������´ζ�Time�᲻�� */
//	HAL_Delay(200);		/* ������0.2�� */ /*�жϳ����е���HAL_Delay�����������еĵ��ó�ͻ */
	if (dTime.Seconds == 59) goto LOOP1;
	HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_RESET);			  
}

/*****************************************************************************/

void TestTimePwm(void)					/* T1��PWM�����T2��PWM������� */
{
	u8 KV;
	u16 i,i1,Period,Pulse;

//	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);	 
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin | LED_G_Pin, GPIO_PIN_SET);	 	/* ��ơ��Ƶơ��̵�=111 */

	MzClearScreen();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);			/* PWM�����ʼ */
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);	/* PWM���������ʼ */
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);			/* ���벶��1��ʼ */
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);			/* ���벶��2��ʼ */
	MzPutString(0,0,(u8 *)"PWMO on T1_C1 & C1N:");	
	MzPutString(0,10,(u8 *)"Period= 1000us");	
	MzPutString(0,20,(u8 *)"Pulse=  500us");	
	MzPutString(0,34,(u8 *)"PWMI on T2_C1:");	
	MzPutString(0,44,(u8 *)"Period= 1000us");	
	MzPutString(0,54,(u8 *)"Pulse=  500us");	
	i=1000;
	i1=500;
	__HAL_TIM_SET_AUTORELOAD(&htim1, i-1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, i1);
LOOP:
	if(Flag.Flash)					 /* ��ʾˢ��ʱ�䵽 */
	{
		Flag.Flash=0;
		if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1))				/* �������ź� */
		{
			Period = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1)+1;
			Pulse = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2)+1;
			__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1 | TIM_FLAG_CC1OF | TIM_FLAG_CC2 | TIM_FLAG_CC2OF);
			MzShowShort(42,44,Period,1);  
			MzShowShort(36,54,Pulse,1); 
		}
		else										 /* û�����ź� */
		{
			MzPutString(42,44,(u8 *)"-----");  
			MzPutString(36,54,(u8 *)"-----"); 
		}
	}
	KV=ReadKey();
	switch(KV)
	{
		case UP:
			if(i<0x8000) i=i<<1;					/* ���ڼӱ� */
			else i=0xffff;
			i1=i>>1;								/* ռ�ձȱ���50% */
			break;
		case DOWN:
			i=i>>1;								    /* ���ڼ��� */
			if(i<2) i=2;
			i1=i>>1;								/* ռ�ձȱ���50% */
			break;
		case LEFT:
			i1=i1>>1;								/* ռ�ձȼ��� */
			break;
		case RIGHT:
			if(i1<0x8000) i1=i1<<1;					/* ռ�ձȼӱ� */
			else i1=0xffff;
			if(i1>i) i1=i;
			if(i1==0) i1=1;
			break;
		case SEL:
			i=1000;									/* ����=1000us */
			i1=500;									/* ռ�ձ���Ϊ50% */
			break;
		case ESC:
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);			/* PWM�����ͣ */
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);		/* PWM���������ͣ */
			HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_1);			/* ���벶��1��ͣ */
			HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_2);			/* ���벶��2��ͣ */
			return;
		default:
			goto LOOP;
	}
	MzShowShort(42,10,i,1);  
	MzShowShort(36,20,i1,1); 
	__HAL_TIM_SET_AUTORELOAD(&htim1, i-1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, i1);
	goto LOOP;			
}

/******************************************************************************/

void TestEncoder(void)					/* T4�ı������������ */
{
	u8 KV,Buf[8];
	u16 i,i1;
	s16 c,r;
	s32 l;

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin | LED_G_Pin, GPIO_PIN_RESET);	 /* ��ơ��Ƶơ��̵�=000 */
//	HAL_GPIO_WritePin(LED3_GPIO_Port, LED1_Pin | LED2_Pin, GPIO_PIN_SET);	 	
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); 							// ����������

	MzClearScreen();
	MzPutString(0,0,(u8 *)"Test Encoder...");	
	MzPutString(0,10,(u8 *)"EC11->Time4:");	
	MzPutString(0,30,(u8 *)"Count = 00000");	
	MzPutString(0,40,(u8 *)"revolutions = 00000");	
	MzPutString(0,50,(u8 *)"degree = 000");	
	{
		u8 BUF[]={0x82,0x01};
		MzCommand(2, BUF);			  /* ѡ�����ַ�1(12X12),�ַ�ɫΪ1(��ɫ) */
		X_Witch_cn = 12;
		Y_Witch_cn = 12;
	}
	MzPutString_cn(72,50,(u8 *)"��");	
	r=0;
	Flag.Temp=0;
	Flag.Dir=0;						/* ����Flag.Dir������ֵ�������� */
LOOP:
	c=__HAL_TIM_GET_COUNTER(&htim4);

	if(Flag.Temp==1)				 /* ���ڿ���� */
	{
		if(Flag.Dir==0 && c<0)		 /* ԭ�����������ڱ为�� */
		{
			r++;					 /* ���������1 */
			Flag.Dir=1;
		}
		else if(Flag.Dir==1 && c>=0) /* ԭ���Ǹ������ڱ����� */
		{
			r--;					 /* ���������1 */
			Flag.Dir=0;
		}
	}
	if(c>0x4000) 					 /* ����0x4000����������� */
	{
		Flag.Temp=1;
		Flag.Dir=0;
	}
	else if(c<-0x4000) 				/* С��-0x4000���븺����� */
	{
		Flag.Temp=1;
		Flag.Dir=1;
	}
	else Flag.Temp=0;				/* -0x4000<=c<=0x4000���������� */
	l=r*0x10000+c;
	
	if(Flag.Flash)			   		/* ��ʾˢ��ʱ�䵽 */
	{
		Flag.Flash=0;
		MzShowShort(48,30,c,0);
		if(l<0)				
		{							/* ���Ƕ� */
			l=-l;
			MzPutChar(78,40,'-');	
		}
		else MzPutChar(78,40,' ');
		i=l/80;						/* 80=20*4��ÿȦ20��ÿ��4�����أ� */
		i1=l%80;		
		MzShowShort(84,40,i,1);
		l=i1*36;
		i=l/8;					   /* i=i1*360/80 */
		Buf[0]=i/100+'0';	i=i%100;
		Buf[1]=i/10+'0';
		Buf[2]=i%10+'0';
		Buf[3]=0;
		MzPutString(54,50,Buf);
			  
	}

	i=HAL_GPIO_ReadPin(EC_SW_GPIO_Port, EC_SW_Pin);
	if(i==0) HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
	   
	KV=ReadKey();
	if(KV!=ESC) goto LOOP;
	HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL); 							// �رձ�����
}

/****************************************************************************/

void TestRF24L01(void)				/* ����ͨѶģ��nRF24L01���� */
{
	u8 KV,RF_RxBuf[32],b;
	u16 dTx,dRx;
	u8 Buf[18];
	uint32_t vr,ir;
	

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin, GPIO_PIN_RESET);	 /* ��ơ��Ƶơ��̵�=001 */
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);	 	
	
	RF_Init();
	MzInit();
	MzClearScreen();
	MzPutString(0,0,(u8 *)"Test nRF24L01...");	
	RF_Read(nRF24L01_R_REGISTER + nRF24L01_TX_ADDR, 5, RF_RxBuf);			/* ��Tx�ڵ��ַ */
	b=BufCmp(RF_RxBuf, (u8 *)nRF24L01_ADDR, 5);	/* �Ƚ�Tx�ڵ��ַ�Ƿ����趨��ַһ�� */
	if(b!=0)
	{
		MzPutString(0,20,(u8 *)"nRF24L01 ERROR!");	
		do
		{
			KV=ReadKey();
		}while(KV!=ESC);
		return;
	}
	
	RxBufTop=0;		/* ���ջ�����ָ������ */
	RxBufBom=0;
	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);	/* ����ձ�־ */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
	__HAL_UART_ENABLE(&huart2);											/* ������ */	
	
	Buf[0]=0xb1;
	Buf[1]=0x00;
	Buf[2]=0x00;
	Buf[3]=0x04;
	USART2_SendCmd(4,Buf);	
	
	
	u8 if_button,mode;
	u8 screen, control;
	u8 new_mode;
	int32_t new_v,new_i;
	uint16_t dac_val,dac_iac;
	
LOOP:
	//�������
	if_button = DC_ReadButton(10, &screen, &control);
	if (if_button==0x00)
	{
		char buf[8];
		MzPutString(0,10, "OK");
		sprintf(buf, "%d ", control);
		MzPutString(50,10,(u8*)buf);
		
		if (control==10)
		{
			//�޸��趨ֵ
			float c_v,c_i;
			c_v=DC_ReadValue(2);
			c_i=DC_ReadValue(3);
			c_v=c_v-0.0014*c_v+0.0086;
			c_i=c_i-0.0043*c_i+0.4253;
			
			c_v=c_v* (1.76f / 10.0f);
			c_i=c_i* (2.38f / 10.0f);
			if (c_v<=1.76f && c_i<=23.8f)
			{
				new_v = (uint16_t)(c_v / 2.5f * 4095.0f + 0.5f);
				new_i = (uint16_t)(c_i/10.0f / 2.5f * 4095.0f + 0.5f);
			}
			mode=DC_ReadValue(1);
			
			Send_irvr(new_v,new_i,mode,0);
		}
		else if (control==6) 
		{
			//��ѹģʽ
			new_mode=0;
			Send_irvr(vr,ir,new_mode,0);
		}
		else if (control==7) 
		{
			//����ģʽ
			new_mode=1;
			Send_irvr(vr,ir,new_mode,0);
		}
		else if (control==8) 
		{
			//�Զ��л�ģʽ
			new_mode=2;
			Send_irvr(vr,ir,new_mode,0);
		}
		else if (control==9) 
		{
			//���͹ض�
			mode=DC_ReadValue(1);
			Send_irvr(vr,ir,mode,1);
		}
	}
	control=0;
	
	//��ȡ��������
	RF_Read(nRF24L01_R_REGISTER | nRF24L01_STATUS, 1, &b);
	if((b & 0x4E)==0x40)		/* 0ͨ�������� */
	{
		RF_Read(nRF24L01_R_RX_PAYLOAD, 10, RF_RxBuf);
		b=0x40;
		RF_Write(nRF24L01_W_REGISTER | nRF24L01_STATUS, 1, &b);	/* ���RX_DR��־ */
		
		if (RF_RxBuf[0]==0xAA)	//�����������趨ֵ
		{
			vr = ((uint32_t)RF_RxBuf[1] << 24) | ((uint32_t)RF_RxBuf[2] << 16) |
								((uint32_t)RF_RxBuf[3] << 8)  | ((uint32_t)RF_RxBuf[4]);

			ir = ((uint32_t)RF_RxBuf[5] << 24) | ((uint32_t)RF_RxBuf[6] << 16) |
										((uint32_t)RF_RxBuf[7] << 8)  | ((uint32_t)RF_RxBuf[8]);
			uint8_t mode = RF_RxBuf[9];
			float vv = ((float)vr / 4096.0f) * 2.5f;
			float ii = ((float)ir/ 4096.0f) * 2.5f;
			char buf[8];
			sprintf(buf, "%.1fV ", vv*5.68182f);	MzPutString(0,30, (u8*)buf);
			sprintf(buf, "%.0fmA ", ii*42.0168f);	MzPutString(40,30, (u8*)buf);
			MzShowShort(0,50,mode,1);  
			SetLcd_vi(vv*5.68182f,ii*42.0168f,mode);
		}
		else	//���������ǲⶨֵ 
		{
			uint32_t adv = ((uint32_t)RF_RxBuf[2] << 24) | ((uint32_t)RF_RxBuf[3] << 16) |
								((uint32_t)RF_RxBuf[4] << 8)  | ((uint32_t)RF_RxBuf[5]);

			uint32_t adi = ((uint32_t)RF_RxBuf[6] << 24) | ((uint32_t)RF_RxBuf[7] << 16) |
										((uint32_t)RF_RxBuf[8] << 8)  | ((uint32_t)RF_RxBuf[9]);
			float AdcV = ((float)adv/ 4096.0f) * 2.5f * 5.68182f*0.9775f-0.5876f;
			float AdcI = ((float)adi/ 4096.0f) * 2.5f * 42.0168f*1.0263f+0.0853f;
			char buf[8];
			sprintf(buf, "%.3fV ", AdcV);	MzPutString(0,40, (u8*)buf);
			sprintf(buf, "%.1fmA ", AdcI);	MzPutString(60,40, (u8*)buf);
			SetLcd_dc(AdcV,AdcI);
		}
	}
	
	goto LOOP;			
}

void Send_irvr(uint32_t vr, uint32_t ir, u8 mode, bool if_send)		//���߷���vr��ir��mode  
{
		uint8_t buf[30];
	
    buf[0] = (vr >> 24) & 0xFF;
    buf[1] = (vr >> 16) & 0xFF;
    buf[2] = (vr >> 8) & 0xFF;
    buf[3] = vr & 0xFF;

    buf[4] = (ir >> 24) & 0xFF;
    buf[5] = (ir >> 16) & 0xFF;
    buf[6] = (ir >> 8) & 0xFF;
    buf[7] = ir & 0xFF;
	
		buf[8] = (uint8_t)mode;
		buf[9] = (uint8_t)if_send;

		RF_Transmit(10, buf);
}

/****************************************************************************/

void 	RF_Init(void)							/* RF24L01ģ���ʼ�� */
{
	u8 b;
	
	HAL_GPIO_WritePin(RF_CE_GPIO_Port, RF_CE_Pin, GPIO_PIN_RESET);		/* RF_CE=0 ����״̬ */
	b=0x03;		/* SETUP_AW=0x03 */
	RF_Write(nRF24L01_W_REGISTER + nRF24L01_SETUP_AW, 1, &b);	/* ���õ�ַ���=5�ֽ� */
	RF_Write(nRF24L01_W_REGISTER + nRF24L01_TX_ADDR, 5, (u8 *)nRF24L01_ADDR);			/* дTx�ڵ��ַ */
	RF_Write(nRF24L01_W_REGISTER + nRF24L01_RX_ADDR_P0, 5, (u8 *)nRF24L01_ADDR);	/* дRxͨ��0�ڵ��ַ */
	b=0x01;		/* ENAA_P0=1 */
	RF_Write(nRF24L01_W_REGISTER + nRF24L01_EN_AA, 1, &b);	/* ʹ���Զ�Ӧ�� */
	b=0x01;		/* ERX_P0=1 */
	RF_Write(nRF24L01_W_REGISTER + nRF24L01_EN_RXADDR, 1, &b);	/* ʹ��Rxͨ��0 */
	b=0x1a;		/* SETUP_RETR=0x1a */
	RF_Write(nRF24L01_W_REGISTER + nRF24L01_SETUP_RETR, 1, &b);	/* �����Զ��ط���500us��ʱ+10���ط� */
	b=46;		/* RF_CH=46 */
	RF_Write(nRF24L01_W_REGISTER + nRF24L01_RF_CH, 1, &b);	/* ѡ��ͨ��Ƶ��=2400+46MHz */
	b=0x07;		/* RF_SETUP=0x07 */
	RF_Write(nRF24L01_W_REGISTER + nRF24L01_RF_SETUP, 1, &b);	/* ���÷����������������=1Mbps�����书��=0dBm������Ŵ�������=1 */
	b=10;		/* RX_PW_P0=2 */
	RF_Write(nRF24L01_W_REGISTER + nRF24L01_RX_PW_P0, 1, &b);	/* ѡ��ͨ��0 ��Ч���ݿ��=2 */
	RF_Write(nRF24L01_FLUSH_TX, 0, 0);					/* ���TX FIFO */
	RF_Write(nRF24L01_FLUSH_RX, 0, 0);					/* ���RX FIFO */
	b=0x70;
	RF_Write(nRF24L01_W_REGISTER | nRF24L01_STATUS, 1, &b);	/* ���RX_DR��TX_DS��MAX_RT��־ */
	b=0x0b;		/* CONFIG=0x0b */
	RF_Write(nRF24L01_W_REGISTER + nRF24L01_CONFIG, 1, &b);	/* ���ù���ģʽ��CRCʹ�ܡ�CRC����=1�ֽڡ��ϵ硢����ģʽ */
	HAL_Delay(10);				/* �ϵ����ʱ�ȴ�10ms������Ҫ1.5ms */
	HAL_GPIO_WritePin(RF_CE_GPIO_Port, RF_CE_Pin, GPIO_PIN_SET);			/* RF_CE=1�������״̬ */
}

/****************************************************************************/

u8 RF_Read(u8 Cmd, u8 Num, u8* Buf)		/* RF24L01ִ�ж�����Cmd����Num�����ݶ��뻺����Buf */
{
	HAL_StatusTypeDef State;
	u8 s;
	
	HAL_GPIO_WritePin(RF_CSN_GPIO_Port, RF_CSN_Pin, GPIO_PIN_RESET);		/* RF_CSN=0 ��ʼSPI���� */
	State=HAL_SPI_TransmitReceive(&hspi1, &Cmd, &s, 1, 10);		/* �����������ͬʱ����״̬�ֽڣ���ʱʱ����Ϊ10ms */
	while(State!=HAL_OK);
	if(Num!=0) 
	{
		State=HAL_SPI_Receive(&hspi1, Buf, Num, 10);		/* ���������ֽڣ���ʱʱ����Ϊ10ms */
		while(State!=HAL_OK);
	}
	HAL_GPIO_WritePin(RF_CSN_GPIO_Port, RF_CSN_Pin, GPIO_PIN_SET);		/* RF_CSN=1 ����SPI���� */
	return s;
}

/****************************************************************************/

u8 RF_Write(u8 Cmd, u8 Num, u8* Buf)		/* RF24L01ִ��д����Cmd����Num������д�뻺����Buf */
{
	HAL_StatusTypeDef State;
	u8 s;
	
	HAL_GPIO_WritePin(RF_CSN_GPIO_Port, RF_CSN_Pin, GPIO_PIN_RESET);		/* RF_CSN=0 ��ʼSPI���� */
	State=HAL_SPI_TransmitReceive(&hspi1, &Cmd, &s, 1, 10);		/* �����������ͬʱ����״̬�ֽڣ���ʱʱ����Ϊ10ms */
	while(State!=HAL_OK);
	if(Num!=0) 
	{
		State=HAL_SPI_Transmit(&hspi1, Buf, Num, 10);		/* ���������ֽڣ���ʱʱ����Ϊ10ms */
		while(State!=HAL_OK);
	}
	HAL_GPIO_WritePin(RF_CSN_GPIO_Port, RF_CSN_Pin, GPIO_PIN_SET);		/* RF_CSN=1 ����SPI���� */
	return s;
}

/****************************************************************************/

u8 BufCmp(u8 *Buf1, u8 *Buf2, u8 Num)		/* �Ƚ�����������Num�������Ƿ�һ�� */
{
	u8 i;
	
	for(i=0; i<Num; i++)
	{
		if(Buf1[i]!=Buf2[i]) return 1;
	}
	return 0;
	
}

/****************************************************************************/

void RF_Transmit(u8 Num, u8* Buf)		/* RF24L01���ͻ�����Buf�е�Num������ */
{
	u8 b;

	HAL_GPIO_WritePin(RF_CE_GPIO_Port, RF_CE_Pin, GPIO_PIN_RESET);		/* RF_CE=0 ����״̬ */
	b=0x0a;		/* CONFIG=0x0a */
	RF_Write(nRF24L01_W_REGISTER + nRF24L01_CONFIG, 1, &b);	/* ���ù���ģʽ��CRCʹ�ܡ�CRC����=1�ֽڡ��ϵ硢����ģʽ */
	RF_Write(nRF24L01_W_TX_PAYLOAD, Num, Buf);		/* д�뷢������ */
	HAL_GPIO_WritePin(RF_CE_GPIO_Port, RF_CE_Pin, GPIO_PIN_SET);			/* RF_CE=1���뷢��״̬ */
	HAL_Delay(1);			/* CE�ߵ�ƽ10us���� */
	HAL_GPIO_WritePin(RF_CE_GPIO_Port, RF_CE_Pin, GPIO_PIN_RESET);		/* RF_CE=0 ����״̬ */
LOOP:
	RF_Read(nRF24L01_R_REGISTER | nRF24L01_STATUS, 1, &b);
	if((b & 0x30)==0)	goto LOOP;	/* ����û��ɣ�ûӦ�����ط�û���� */
	if((b & 0x20)==0x20)		/* TX FIFO�е������ѷ��� */
	{
		b=0x20;
		RF_Write(nRF24L01_W_REGISTER | nRF24L01_STATUS, 1, &b);	/* ���TX_DS��־ */
		MzPutString(0,50,(u8 *)"Send Succeed!");  
	}
	if((b & 0x10)==0x10)		/* �����ط����� */
	{
		RF_Write(nRF24L01_FLUSH_TX, 0, 0);	/* ���TX FIFO */
		b=0x10;
		RF_Write(nRF24L01_W_REGISTER | nRF24L01_STATUS, 1, &b);	/* ���MAX_RT��־ */
		MzPutString(0,50,(u8 *)"Send Error!  ");  
	}
	b=0x0b;		/* CONFIG=0x0b */ 
	RF_Write(nRF24L01_W_REGISTER + nRF24L01_CONFIG, 1, &b);	/* ���ù���ģʽ��CRCʹ�ܡ�CRC����=1�ֽڡ��ϵ硢����ģʽ */
	HAL_GPIO_WritePin(RF_CE_GPIO_Port, RF_CE_Pin, GPIO_PIN_SET);			/* RF_CE=1�������״̬ */
}

/****************************************************************************/
void SetLcd_vi(float vv, float ii,CtrlMode mode)		//************************�����趨ֵ
{

	u8 Buf[18];
	char strV[20],strI[20],strMode[20];
	sprintf(strV, "%.1f", vv); 
	sprintf(strI, "%.0f", ii); 
	sprintf(strMode, "%.0f", (float)mode); 
	/*
	if (mode==MODE_VOLTAGE) strcpy(strMode,"V Loop");
	else if (mode==MODE_CURRENT) strcpy(strMode,"I Loop");
	else strcpy(strMode,"Auto");
	*/

	
	Buf[0]=0xb1;
	Buf[1]=0x00;
	Buf[2]=0x00;
	Buf[3]=0x04;
	USART2_SendCmd(4,Buf);	
	
	
	
	// ģʽ
	Buf[0]=0xB1;
	Buf[1]=0x10;
	Buf[2]=0x00;
	Buf[3]=0x04;
	Buf[4]=0x00;
	Buf[5]=0x01;
	Buf[6]=strMode[0];
	USART2_SendCmd(7,Buf);	
	
	//�ο���ѹ
	Buf[0]=0xB1;
	Buf[1]=0x10;
	Buf[2]=0x00;
	Buf[3]=0x04;
	Buf[4]=0x00;
	Buf[5]=0x02;
	for(int i=0;i<strlen(strV);i++)
	{
		Buf[6+i]=strV[i];
	}
	USART2_SendCmd(6+strlen(strV),Buf);	
	
	//�ο�����
	Buf[0]=0xB1;
	Buf[1]=0x10;
	Buf[2]=0x00;
	Buf[3]=0x04;
	Buf[4]=0x00;
	Buf[5]=0x03;
	for(int i=0;i<strlen(strI);i++)
	{
		Buf[6+i]=strI[i];
	}
	USART2_SendCmd(6+strlen(strI),Buf);	
	
}

void SetLcd_dc(float cv, float ci)			//*****************************************���ò���ֵ
{
	
	u8 Buf[18];
	char strV[20],strI[20],strAdcV[20],strAdcI[20],strMode[2];
	sprintf(strAdcV, "%.3f", cv); 
	sprintf(strAdcI, "%.2f", ci); 
	
	Buf[0]=0xb1;
	Buf[1]=0x00;
	Buf[2]=0x00;
	Buf[3]=0x04;
	USART2_SendCmd(4,Buf);	
	
	//������ѹ
	Buf[0]=0xB1;
	Buf[1]=0x10;
	Buf[2]=0x00;
	Buf[3]=0x04;
	Buf[4]=0x00;
	Buf[5]=0x04;
	for(int i=0;i<strlen(strAdcV);i++)
	{
		Buf[6+i]=strAdcV[i];
	}
	USART2_SendCmd(6+strlen(strAdcV),Buf);	
	
	//��������
	Buf[0]=0xB1;
	Buf[1]=0x10;
	Buf[2]=0x00;
	Buf[3]=0x04;
	Buf[4]=0x00;
	Buf[5]=0x05;
	for(int i=0;i<strlen(strAdcI);i++)
	{
		Buf[6+i]=strAdcI[i];
	}
	USART2_SendCmd(6+strlen(strAdcI),Buf);		
}

u8 DC_ReadButton(u8 id, u8* screen_id, u8* control_id)			//*****************************��ȡ����״̬ 
{
	union DATAINT Data;
	u8 KV,d;
	u16 i,i1;

	__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
LOOP:
	d=RxBufTop-RxBufBom;
	if(d<14) 
	{
		KV=0xFF;
		goto RET;
	}
	if(RxBuf[RxBufBom++]!=0xee)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xb1)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0x11)	goto LOOP;

	Data.db[1]=RxBuf[RxBufBom++];
	Data.db[0]=RxBuf[RxBufBom++];
	i = Data.dw;

	Data.db[1]=RxBuf[RxBufBom++];
	Data.db[0]=RxBuf[RxBufBom++];
	i1 = Data.dw;

	if(RxBuf[RxBufBom++]!=0x10)	goto LOOP;

	KV=RxBuf[RxBufBom++];
	RxBufBom++;  // skip 0x00

	if(RxBuf[RxBufBom++]!=0xff)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xfc)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xff)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xff)	goto LOOP;

	if (screen_id) *screen_id = i;
	if (control_id) *control_id = i1;

RET:
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
	return KV;
}


float DC_ReadValue(u8 id)			//��ȡ�ı���ֵ
{
	union DATAINT Data;
	u8 i,d,Buf[6];
	float value;
	char str[12] = {0};

	Buf[0]=0xb1;
	Buf[1]=0x11;
	Buf[2]=0x00;
	Buf[3]=0x04;
	Buf[4]=0x00;
	Buf[5]=id;
	USART2_SendCmd(6,Buf);	   		/* ���Ͷ�ȡ��ǰ�����IDֵ���� */
	HAL_Delay(100);	

	__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
LOOP:
	d=RxBufTop-RxBufBom;
	if (d < 10) goto RET; // ����̫�̣���ֹԽ��
	if(RxBuf[RxBufBom++]!=0xee)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xb1)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0x11)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0x00)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0x04)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0x00)	goto LOOP;
	if(RxBuf[RxBufBom++]!=id)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0x11)	goto LOOP;
	
	// ��ȡ�ַ�����ֱ������������0x00��
	for (i = 0; i < sizeof(str) - 1; i++) {
		u8 ch = RxBuf[RxBufBom++];
		if (ch == 0x00) break;
		str[i] = ch;
	}
	str[i] = '\0';  // ȷ���ַ�����0��β
	
	if(RxBuf[RxBufBom++]!=0xff)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xfc)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xff)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xff)	goto LOOP;
	char *endptr;
	value = strtof(str, &endptr);
RET:
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
	return value;
}


/****************************************************************************/

void TestUsartLcd(void)				/* Usart�ӿڴ�������� */
{
	u8 KV;
	u16 i;

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);	 /* ��ơ��Ƶơ��̵�=011 */
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_Y_Pin | LED_G_Pin, GPIO_PIN_SET);	 	
	
	RxBufTop=0;		/* ���ջ�����ָ������ */
	RxBufBom=0;
	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);	/* ����ձ�־ */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
	__HAL_UART_ENABLE(&huart2);											/* ������ */	

	MzClearScreen();
	MzPutString(0,0,(u8 *)"Test UsartLcd...");	
	MzPutString(0,10,(u8 *)"Screen_id = ");	
	MzPutChar(72,10,'R');
	DC_BackColor(0xf800);			/* ���ñ���ɫΪ��ɫ */
	DC_ClearScreen();				/* ���� */
	HAL_Delay(1000);	
	MzPutChar(72,10,'G');
	DC_BackColor(0x07e0);			/* ���ñ���ɫΪ��ɫ */
	DC_ClearScreen();				/* ���� */
	HAL_Delay(1000);	
	MzPutChar(72,10,'B');
	DC_BackColor(0x001f);			/* ���ñ���ɫΪ��ɫ */
	DC_ClearScreen();				/* ���� */
	HAL_Delay(1000);	
	
	for(i=0;i<7;i++)
	{
		MzShowShort(72,10,i,1);
		DC_DisplayScreen(i);	
		HAL_Delay(1000);	
	}		

	DC_DisplayScreen(i);	
	
	u8 Buf[18];
	
	Buf[0]=0xb1;
	Buf[1]=0x00;
	Buf[2]=0x00;
	Buf[3]=0x04;
	USART2_SendCmd(4,Buf);	   /* ������������ */
	
	
	
	Buf[0]=0xB1;
	Buf[1]=0x10;
	Buf[2]=0x00;
	Buf[3]=0x04;
	Buf[4]=0x00;
	Buf[5]=0x02;
	Buf[6]='1';
	Buf[7]='2';
	Buf[8]='v';
	USART2_SendCmd(9,Buf);	
	
	
	u8 RF_RxBuf[32],b;
	u16 dTx,dRx;

	
LOOP:
	KV=DC_PressButton();			/* ���ް�ť���� */
	if(KV==0x00)
	{
		HAL_Delay(100);	
		i=DC_ReadScreen();			/* ������� */
		MzShowShort(72,10,i,1);
	}
	KV=ReadKey();
	switch(KV)
	{
		case UP:
			i=6;					/* ������Ϊ6 */
			break;
		case DOWN:
			i=1;				   	/* ������Ϊ1 */
			break;
		case LEFT:
			if(i>1) i--;		   	/* �����1 */
			else i=6;								
			break;
		case RIGHT:
			if(i<6) i++;			/* �����1 */
			else i=1;
			break;
		case SEL:
			i=0;					/* �л�������0 */
			break;
		case ESC:
			DC_DisplayScreen(0);			/* �л�������0 */
			HAL_Delay(100);	
			__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
			__HAL_UART_DISABLE(&huart2);		/* �ش��� */
			return;
		default:
			goto LOOP;
	}
	MzShowShort(72,10,i,1);
	DC_DisplayScreen(i);			/* �л����� */
	goto LOOP;
}

/***************************************************************************/

void DC_BackColor(u16 Color)			/* ��������ñ���ɫ */
{
	union DATAINT Dint;
	u8 Buf[4];
	Buf[0]=0x42;
	Dint.dw=Color;
	Buf[1]=Dint.db[1];
	Buf[2]=Dint.db[0];
	USART2_SendCmd(3,Buf);	   		/* �������ñ���ɫ���� */
	
}

/*****************************************************************************/

void USART2_SendCmd(u8 num,u8 * p)	/* USART2�������ݰ�������+���ݸ���Ϊnum������p��ַ�� */
{
	u8 Index=0, TxBuf[256];

	TxBuf[Index++]=0xee;
	while(num--)
	{
		TxBuf[Index++]=*p;
		p++;
	}
	TxBuf[Index++]=0xff;
	TxBuf[Index++]=0xfc;
	TxBuf[Index++]=0xff;
 	TxBuf[Index++]=0xff;

	p=TxBuf;
//	while(GPIO_ReadInputDataBit(DC_BUSY)); 	/* ��BusyΪ�ߵ�ƽ,��ѭ���ȴ� */
	while(Index--)
	{
	 	while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == RESET);
		__HAL_UART_FLUSH_DRREGISTER(&huart2)=*p;		/* д���ͼĴ��� */
		p++;
	}
}

/**************************************************************************/

void DC_ClearScreen(void)				/* ��������� */
{
	u8 Buf[2];

	Buf[0]=0x01;
	USART2_SendCmd(1,Buf);	   /* ������������ */
}

/**************************************************************************/

void DC_DisplayScreen(u16 id)			/* ������л����� */
{
	union DATAINT Dint;
	u8 Buf[5];

	Buf[0]=0xb1;
	Buf[1]=0x00;
	Dint.dw=id;
	Buf[2]=Dint.db[1];
	Buf[3]=Dint.db[0];
	USART2_SendCmd(4,Buf);	   /* ������������ */
}

/**************************************************************************/

u8 DC_PressButton(void)				/* ��������ް�ť���� */
{
	union DATAINT Data;
	u8 KV,d;
	u16 i,i1;

	__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
LOOP:
	d=RxBufTop-RxBufBom;
	if(d<14) 
	{
		KV=0xFF;
		goto RET;
	}
	if(RxBuf[RxBufBom++]!=0xee)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xb1)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0x11)	goto LOOP;
	Data.db[1]=RxBuf[RxBufBom++];
	Data.db[0]=RxBuf[RxBufBom++];
	i=Data.dw;
	Data.db[1]=RxBuf[RxBufBom++];
	Data.db[0]=RxBuf[RxBufBom++];
	i1=Data.dw;
	if(RxBuf[RxBufBom++]!=0x10)	goto LOOP;
	KV=RxBuf[RxBufBom++];
	RxBufBom++;
	if(RxBuf[RxBufBom++]!=0xff)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xfc)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xff)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xff)	goto LOOP;
	Screen_id=i;
	Control_id=i1;
RET:
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
	return KV;
}

/*************************************************************************/

u16 DC_ReadScreen(void)				/* ������������ */
{
	union DATAINT Data;
	u8 d,Buf[3];
	u16 id;

	Buf[0]=0xb1;
	Buf[1]=0x01;
	USART2_SendCmd(2,Buf);	   		/* ���Ͷ�ȡ��ǰ�����IDֵ���� */
	HAL_Delay(100);	

	__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
LOOP:
	d=RxBufTop-RxBufBom;
	if(d<9) 
	{
		id=0xFFFF;
		goto RET;
	}
	if(RxBuf[RxBufBom++]!=0xee)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xb1)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0x01)	goto LOOP;
	Data.db[1]=RxBuf[RxBufBom++];
	Data.db[0]=RxBuf[RxBufBom++];
	id=Data.dw;
	if(RxBuf[RxBufBom++]!=0xff)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xfc)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xff)	goto LOOP;
	if(RxBuf[RxBufBom++]!=0xff)	goto LOOP;
	Screen_id=id;
RET:
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
	return id;
}

/*****************************************************************************/

void EndTest(void)						/* �������� */
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin | LED_G_Pin, GPIO_PIN_RESET);	 /* ��ơ��Ƶơ��̵�=000 */

	MzClearScreen();
	MzSetBackLight(0);				/* ���ñ�������ȵȼ� */
}

/******************************************************************************/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
