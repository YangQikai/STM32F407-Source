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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum { MODE_VOLTAGE, MODE_CURRENT , MODE_AUTO } CtrlMode;
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
vu16  AdcBuf[10],New_Buf[10];

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

void KeyTest(void);
u8 ReadKeyboard(void);
void FFT(void);
void SaveSettingsToEEPROM(uint32_t vr, uint32_t ir, CtrlMode mode);
void LoadSettingsFromEEPROM(uint32_t *vr, uint32_t *ir, CtrlMode *mode);
void Send_irvr(uint32_t vr, uint32_t ir,CtrlMode mode);
void Send_dc(uint32_t dcv, uint32_t dci);
bool Receive_vrir(bool if_send);
void GetAvgAdc(uint16_t *adc0, uint16_t *adc1);

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
		//KeyTest();
		//TestAdc();						/* A/Dת������ */
		TestDac();						/* D/Aת������ */
		//TestI2cEeprom();				/* I2C�ӿ�EEPROM��24C02������ */
		//TestUsart2();					/* USART2���� */
		//TestRtc();						/* RTC��BKP���� */
		//TestTimePwm();					/* T1��PWM�����T2��PWM������� */
		//TestEncoder();					/* T4�ı������������ */
		//TestRF24L01();				/* ����ͨѶģ��24L01���� */
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
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
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
  HAL_GPIO_WritePin(GPIOE, KEY3_Pin|KEY4_Pin|GPIO_PIN_10|LED_G_Pin
                          |LED_Y_Pin|LED_R_Pin|KEY1_Pin|KEY2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPEAK_Pin|RF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Mz_CS_Pin|Mz_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RF_CSN_GPIO_Port, RF_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : KEY3_Pin KEY4_Pin PE10 LED_G_Pin
                           LED_Y_Pin LED_R_Pin KEY1_Pin KEY2_Pin */
  GPIO_InitStruct.Pin = KEY3_Pin|KEY4_Pin|GPIO_PIN_10|LED_G_Pin
                          |LED_Y_Pin|LED_R_Pin|KEY1_Pin|KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY5_Pin KEY6_Pin KEY7_Pin KEY8_Pin */
  GPIO_InitStruct.Pin = KEY5_Pin|KEY6_Pin|KEY7_Pin|KEY8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SPEAK_Pin */
  GPIO_InitStruct.Pin = SPEAK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPEAK_GPIO_Port, &GPIO_InitStruct);

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
 	//HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_SET);			  
 	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);		/* ��1������� */
	HAL_Delay(200);
 	//HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_RESET);			  
 	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);		
	HAL_Delay(800);
 	//HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_SET);			  
 	HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);		/* ��2���Ƶ��� */
	HAL_Delay(200);
 	//HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_RESET);			  
 	HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_RESET);		
	HAL_Delay(800);
 	//HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_SET);			  
 	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);		/* ��3���̵��� */
	HAL_Delay(500);
 	//HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_RESET);			  
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
u8 ReadKeyboard(void)		/*  ��ȡ����ֵ		*/
{
    uint8_t row, col;
	const u8 KeyMap[4][4] = {
    {'1','2','3','a'},
    {'4','5','6','b'},
    {'7','8','9','c'},
    {'d','0','e','f'}
};

    // ����������
    GPIO_TypeDef* COL_PORT = GPIOE;
    uint16_t COL_PIN[4] = {KEY1_Pin, KEY2_Pin, KEY3_Pin, KEY4_Pin};


    // ����������
    GPIO_TypeDef* ROW_PORT = GPIOE;
    uint16_t ROW_PIN[4] = {KEY5_Pin, KEY6_Pin, KEY7_Pin, KEY8_Pin};

    for (col = 0; col < 4; col++) {
        for (int k = 0; k < 4; k++) {
            HAL_GPIO_WritePin(COL_PORT, COL_PIN[k], GPIO_PIN_SET);
        }

        // ��ǰ������
        HAL_GPIO_WritePin(COL_PORT, COL_PIN[col], GPIO_PIN_RESET);
        HAL_Delay(1); // ������ʱ����

        // ɨ��������
        for (row = 0; row < 4; row++) {
            if (HAL_GPIO_ReadPin(ROW_PORT, ROW_PIN[row]) == GPIO_PIN_RESET) {
                while (HAL_GPIO_ReadPin(ROW_PORT, ROW_PIN[row]) == GPIO_PIN_RESET); // �ȴ�����
                HAL_Delay(10); // ����
								return KeyMap[row][col];
            }
        }
    }

    return 0; // �ް�������
}

/*****************************************************************************/
void KeyTest(void)		/*���̲���*/
{
	u8 keyboard;
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin, GPIO_PIN_RESET);	 /* ��ơ��Ƶơ��̵�=111 */
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin | LED_G_Pin, GPIO_PIN_SET);	 	

	MzInit();
	MzSetBackLight(100);				  
	MzClearScreen();
	
LOOP:
	keyboard = ReadKeyboard();
	if (keyboard == '*') return;
	if (keyboard == 0) goto LOOP;
	MzPutString(0,0,(u8 *)"KEY=");	 
	MzPutChar(50,0,keyboard);
	
	goto LOOP;
}


/*****************************************************************************/

void TestAdc(void)						/* A/Dת������ */
{
	u8 KV,Buf[8];
	u16 i;
	vu16  AdcBuf[901];
	double f;

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_G_Pin, GPIO_PIN_RESET);	 /* ��ơ��Ƶơ��̵�=010 */
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);	 	
	
	MzSetBackLight(100);		
	MzInit();	
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
		if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *)AdcBuf,900)!=HAL_OK)	/*ע�⣺�����еġ�9��һ��Ҫ��ADת��һ��ɨ���ͨ����һ�� */
		{
			/* Start Error */
			Error_Handler();
		}
		while(Flag.Temp==0);		/* �ȴ�ת����� */
		MzShowShort(0,10,AdcBuf[0],1);	MzShowShort(42,10,AdcBuf[1],1);  MzShowShort(85,10,AdcBuf[2],1);
		MzShowShort(0,31,AdcBuf[3],1);	MzShowShort(42,31,AdcBuf[4],1);  MzShowShort(85,31,AdcBuf[5],1);
		MzShowShort(0,52,AdcBuf[6],1);	MzShowShort(42,52,AdcBuf[7],1);  MzShowShort(85,52,AdcBuf[8],1);
		
		for (int i = 0; i < 100; i++)//�������г����ֵ
		{
			printf("%d:\t%d\r\n", i, AdcBuf[i*9]);
		}

		
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
	u8 KEY;
	u16 i;
	//vu32  AdcBuf[10],New_Buf[10];
	u32 vr,ir;			//��ѹ�ο�ֵ������ο�ֵ
	u8 R;
	double set = 0.000f;		//�޸ĺ���ֵ
	int16_t dac_val = 0;			//DAC��ֵ
	char input_buf[5] = {'-','-','-','-',0}; // �����ݴ������ַ���
	uint8_t input_idx = 0;		//�ַ�����
	bool dot_entered = false;		//�Ƿ���ڸ���
	bool if_send=false;				//�Ƿ����ź�	
	//typedef enum { MODE_VOLTAGE, MODE_CURRENT } CtrlMode;		//����ģʽ
	s16 c,c_past;		//��¼��һ�α�������λ��
	int8_t fix=0;

	
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin | LED_G_Pin, GPIO_PIN_RESET);	 /* ��ơ��Ƶơ��̵�=000 */

	
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);	//�򿪱�����
	c_past=__HAL_TIM_GET_COUNTER(&htim4);

	MzInit();
	RF_Init();
	MzClearScreen();
	MzPutString(0,0,(u8 *)"Vref=");		MzPutString(62,0,(u8 *)"Iref=");	  
	MzPutString(0,21,(u8 *)"AD1=");		MzPutString(62,21,(u8 *)"AD2=");
	{
		u8 BUF[]={0x82,0x01};
		MzCommand(2, BUF);			  /* ѡ�����ַ�1(12X12),�ַ�ɫΪ1(��ɫ) */
		X_Witch_cn = 10;
		Y_Witch_cn = 10;
	}
	MzPutString_cn(0,50,(u8 *)"δ����");				//��ʼ����DA������Ϊ���0
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);  
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
	
	CtrlMode mode;
	LoadSettingsFromEEPROM(&vr, &ir, &mode);	// ��ȡ����ĳ�ʼ������
	Send_irvr(vr, ir, mode);
	if (mode==MODE_VOLTAGE) HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
	else if (mode==MODE_CURRENT) HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin|LED_Y_Pin, GPIO_PIN_SET);
	

LOOP:
	if(Flag.Flash)
	{
		
		//��ȡ��λ���趨
		
		
		bool if_change;
		if_change=Receive_vrir(if_send);
		if (if_change==true) if_send=!if_send;
		
			
		
LOOP1:
		
		//��ȡ�Ĵ�������
		LoadSettingsFromEEPROM(&vr, &ir, &mode);
		
		uint16_t avg_adc0, avg_adc1;
		GetAvgAdc(&avg_adc0, &avg_adc1);	//��500��adc
		AdcBuf[0]=avg_adc0; AdcBuf[1]=avg_adc1; 
		
		
		double vv = ((double)vr / 4096.0f) * 2.5f;
		double ii = ((double)ir/ 4096.0f) * 2.5f;
		char buf[8];
		sprintf(buf, "%.1fV ", vv*5.68182f);	MzPutString(0,10, (u8*)buf);
		sprintf(buf, "%.0fmA  ", ii*42.0168f);	MzPutString(62,10, (u8*)buf);
		if (mode == MODE_VOLTAGE) MzPutChar(50,10,'<'); else MzPutChar(50,10,' ');
		if (mode == MODE_CURRENT) MzPutChar(110,10,'<'); else MzPutChar(110,10,' ');
		if (mode == MODE_AUTO) { MzPutChar(50,10,'<'); MzPutChar(110,10,'<'); }

		double AdcV = ((double)AdcBuf[0]/ 4096.0f) * 2.5f * 5.68182f*0.9775f-0.5876f;
		double AdcI = ((double)AdcBuf[1]/ 4096.0f) * 2.5f * 42.0168f*1.0263f+0.0853f;
		sprintf(buf, "%.3fV ", AdcV);	MzPutString(0,31, (u8*)buf);
		sprintf(buf, "%.2fmA  ", AdcI);	MzPutString(62,31, (u8*)buf);
		
		Send_dc(AdcBuf[0],AdcBuf[1]);
		
		
		/*
		//�㹦��
		if (mode==MODE_P)
		{
			if (AdcI>(ii*42.0168f/2))
			{
				double P=vv*5.68182f*ii*42.0168f;
				double Set_p_v=P/AdcI;
				double SSS=Set_p_v-0.0014*Set_p_v+0.0086;
				double P_scaled = SSS * (1.76f / 10.0f);  // ת��
				double P_dac_val = (uint16_t)(P_scaled / 2.5f * 4095.0f + 0.5f);  // ��������
				if (P_dac_val > 4095) P_dac_val = 4095;
				uint32_t vvr=P_dac_val/1.0038f+0.8330737f;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vvr);
			}
			else if (AdcV>(vv*5.68182f/2))
			{
				double P=vv*5.68182f*ii*42.0168f;
				double Set_p_i=P/AdcV;
				double SSS=Set_p_i-0.0043*Set_p_i+0.04253;
				double P_scaled = SSS * (2.38f / 10.0f);  // ת��
				double P_dac_val = (uint16_t)(P_scaled / 2.5f * 4095.0f + 0.5f);  // ��������
				if (P_dac_val > 4095) P_dac_val = 4095;
				uint32_t iir=P_dac_val/1.0127f-36.964711f;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iir);
				if (iir<780) 
				{
					uint32_t iiir=iir*0.9716f+30.0916f;
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iiir);
				}
			}
		}
		*/
		

		//������ѹ���
		
		
		if (mode == MODE_VOLTAGE)
		{
			if (if_send==true && AdcI>102)
			{
				
				LOOPV:
				
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin, GPIO_PIN_RESET);
				HAL_Delay(50);
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin, GPIO_PIN_SET);
				HAL_Delay(50);
				HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_SET);				//������
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
				
				KEY=ReadKeyboard();			//�ڹ���״̬����ȡ������
				if (KEY=='a')
				{
					if (if_send==true)
					{
						if_send=false;
						HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
						HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
						{
							u8 BUF1[]={0x82,0x01};
							MzCommand(2, BUF1);		
							X_Witch_cn = 10;
							Y_Witch_cn = 10;
						}
						MzPutString_cn(0,50,(u8 *)"δ����");
						HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
					}
					goto LOOP1;
				}
				
				
				bool if_change;
				if_change=Receive_vrir(if_send);
				if (if_change==true) 
				{
					if_send=!if_send;
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
					goto LOOP1;
				}
				
				
				goto LOOPV;
			}
			HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin, GPIO_PIN_RESET);
			if (mode==MODE_VOLTAGE) HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			else HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
		}
		else if (mode==MODE_CURRENT)
		{
			if (if_send==true && AdcV>10.2)
			{
				
				LOOPI:
				
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin, GPIO_PIN_RESET);
				HAL_Delay(50);
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin, GPIO_PIN_SET);
				HAL_Delay(50);
				HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_SET);			//������
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
				
				KEY=ReadKeyboard();				//�ڹ�ѹ״̬����ȡ������
				if (KEY=='a')
				{
					if (if_send==true)
						{
							if_send=false;
							//HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
							//HAL_DAC_Stop(&hdac, DAC_CHANNEL_2);
							HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
							HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
							{
								u8 BUF1[]={0x82,0x01};
								MzCommand(2, BUF1);		
								X_Witch_cn = 10;
								Y_Witch_cn = 10;
							}
							MzPutString_cn(0,50,(u8 *)"δ����");
							HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
						}
						goto LOOP1;
				}
				
				
				bool if_change;
				if_change=Receive_vrir(if_send);
				if (if_change==true) 
				{
					if_send=!if_send;
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
					goto LOOP1;
				}
				
				
				goto LOOPI;
			}
			HAL_GPIO_WritePin(SPEAK_GPIO_Port, SPEAK_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin, GPIO_PIN_RESET);
			if (mode==MODE_VOLTAGE) HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			else HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
		}
		else  {HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin | LED_Y_Pin, GPIO_PIN_SET);}
		

		Flag.Flash=0;
	}

NEXT:
	
	KEY=ReadKeyboard();
	{
		u8 BUF[]={0x81,0x01};
		MzCommand(2, BUF);			  /* ѡ��ASCII�ַ�0(6X10),�ַ�ɫΪ1(��ɫ) */
		X_Witch = 6;
		Y_Witch = 10;
	}
	MzPutString(62,51,(u8 *)"KET=");
	MzPutString(92,51,(u8 *)input_buf);
	 switch(KEY)							//�жϰ���
    {
        case '0':
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
            if (input_idx < sizeof(input_buf) - 1) {
                input_buf[input_idx++] = KEY;
                //input_buf[input_idx] = '-';
            }
            break;
        case 'e':  // С����
            if (!dot_entered && input_idx < sizeof(input_buf) - 2) {
                input_buf[input_idx++] = '.';
                input_buf[input_idx] = '-';
                dot_entered = true;
            }
            break;
        case 'f':  // ȷ�ϼ�
        {
						char *endptr;
						double val = strtof(input_buf, &endptr);
						if (input_buf[0]== '-') break;
            if (val >= 0.0f && val <= 100.0f) 
						{
								if (mode == MODE_CURRENT) val=(double)val/10;
                
							  if (mode == MODE_VOLTAGE) {
									if (val >= 0.0f && val <= 10.0f) 
									{
										set=val-0.0014*val+0.0086;
										double scaled = set * (1.76f / 10.0f);  // ת��
										dac_val = (uint16_t)(scaled / 2.5f * 4095.0f + 0.5f);  // ��������
										if (dac_val > 4095) dac_val = 4095;
										vr = dac_val;
										uint32_t vvr=vr/1.0038f+0.8330737f;
										if (if_send==true) HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vvr);
									}
								} 
								else if (mode==MODE_CURRENT)
								{
									set=val-0.0043*val+0.04253;
									double scaled = set * (2.38f / 10.0f);  // ת��
									dac_val = (uint16_t)(scaled / 2.5f * 4095.0f + 0.5f);  // ��������
									if (dac_val > 4095) dac_val = 4095;
									ir = dac_val;
									uint32_t iir=ir/1.0127f-36.964711f;
									if (if_send==true) HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iir);
									if (iir<780) 
									{
										uint32_t iiir=iir*0.9716f+30.0916f;
										if (if_send==true) HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iiir);
									}
									
								}
            }
            // ������뻺����
            input_idx = 0;
            input_buf[0] = '-';input_buf[1] = '-';input_buf[2] = '-';input_buf[3] = '-';
            dot_entered = false;
						SaveSettingsToEEPROM(vr, ir, mode);	// ��������
						Send_irvr(vr, ir, mode);
            break;
        }
				case 'b':
						if (if_send==true) break;
					
						if (mode==MODE_CURRENT) mode=MODE_VOLTAGE;
						else if (mode==MODE_VOLTAGE) mode=MODE_CURRENT;
						else mode=MODE_VOLTAGE;
				
						SaveSettingsToEEPROM(vr, ir, mode);	// ��������
						Send_irvr(vr, ir, mode);
						if (mode==MODE_VOLTAGE)
						{
							HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_Y_Pin, GPIO_PIN_RESET);
						}
						else 
						{
							HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
						}
						break;
				case 'c':
						if (if_send==true) break;
						if (mode!=MODE_AUTO)
						{
							mode=MODE_AUTO;
							SaveSettingsToEEPROM(vr, ir, mode);	// ��������
							Send_irvr(vr, ir, mode);
							HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin|LED_Y_Pin, GPIO_PIN_SET);
						}
						break;
        case 'd':  // ȡ����
            input_idx = 0;
            input_buf[0] = '-';input_buf[1] = '-';input_buf[2] = '-';input_buf[3] = '-';
            dot_entered = false;
            break;
				case 'a':	//����
						if (if_send==false)
						{
							if_send=true;
							HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
							if (mode==MODE_VOLTAGE) 
							{
									uint32_t vvr=vr/1.0038f+0.8330737f;
									HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vvr);
									//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vr);
									HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
							}
							else if (mode==MODE_CURRENT) 
							{
									HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
									uint32_t iir=ir/1.0127f-36.964711f;
									HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iir);
									if (iir<780) 
									{
										uint32_t iiir=iir*0.9716f+30.0916f;
										HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iiir);
									}
							}
							else if (mode==MODE_AUTO)
							{
									uint32_t vvr=vr/1.0038f+0.8330737f;
									HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vvr);
									//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vr);
									uint32_t iir=ir/1.0127f-36.964711f;
									HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iir);
									//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, ir);
									if (iir<780) 
									{
										uint32_t iiir=iir*0.9716f+30.0916f;
										HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iiir);
									}
							}
							

							MzPutString_cn(0,50,(u8 *)"������");
							HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
						}
						else if (if_send==true)
						{
							if_send=false;
							HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
							//HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
							//HAL_DAC_Stop(&hdac, DAC_CHANNEL_2);
							/*****************************************************�޸�Ϊ���0��*/
							HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
							HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
							
							MzPutString_cn(0,50,(u8 *)"δ����");
							HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
						}
						break;

        default:
            break;
    }
		
	
		//��ת������
	c=__HAL_TIM_GET_COUNTER(&htim4);
	if (c!=c_past)
	{
		if (c>32767) c=c-65536;
		int16_t bi=(c-c_past);
		c_past=c;
		
		set = (double)bi*0.1f/4;
		int tmp;
		if (set>=0)	
		{
			tmp=(int)(set*10.0f+0.5f);
			if (tmp>=2) tmp=1;
		}
		else 	
		{
			tmp=(int)(set*10.0f-0.5f);
			if (tmp<=-2) tmp=-1;
		}
		set=(double)(tmp)/10;
		if (mode==MODE_VOLTAGE) set = (set * 10014) / 10000;
		else set = (set * 10043) / 10000;
		double scaled;
		if (mode==MODE_VOLTAGE) scaled= set * (1.76f / 10.0f);  // ת��
		else if (mode==MODE_CURRENT) scaled= set * (2.38f / 10.0f);
		
		if (tmp>0) dac_val = (int16_t)(scaled / 2.5f * 4095.0f +0.5f);  // ��������
		else dac_val = (int16_t)(scaled / 2.5f * 4095.0f -0.5f);
		if (tmp>0) fix++; 
		else if (tmp<0) fix--;
		
		if (fix==4)
		{
			fix=0;
			dac_val=dac_val-1;
		}
		else if (fix==-4)
		{
			fix=0;
			dac_val=dac_val+1;
		}
		
		if (mode == MODE_VOLTAGE) 
		{
			dac_val=dac_val+vr;
			if (dac_val > 4095) dac_val = 4095;
			if (dac_val < 0) dac_val = 0;
			vr = dac_val;
			uint32_t vvr=vr/1.0038f+0.8330737f;
			if (if_send==true) HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vvr);
		} 
		else if (mode==MODE_CURRENT)
		{
			dac_val=dac_val+ir;
			if (dac_val > 4095) dac_val = 4095;
			if (dac_val < 0) dac_val = 0;
			ir = dac_val;
			uint32_t iir=ir/1.0127f-36.964711f;
			if (if_send==true) HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iir);
			if (iir<780) 
			{
				uint32_t iiir=iir*0.9716f+30.0916f;
				if (if_send==true) HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iiir);
			}
		}
		SaveSettingsToEEPROM(vr, ir, mode);	// ��������
		Send_irvr(vr, ir, mode);
	}
goto LOOP;
	HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL); 
	HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Stop(&hdac, DAC_CHANNEL_2);
}

void GetAvgAdc(uint16_t *adc0, uint16_t *adc1)				//***********************************************��ȡ500��adc
{
    uint32_t sum0 = 0, sum1 = 0;
    for (int i = 0; i < 250; i++)
    {
        Flag.Temp = 0;
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)AdcBuf, 9);
        while (Flag.Temp == 0);
        sum0 += AdcBuf[0];
        sum1 += AdcBuf[1];
    }
    *adc0 = sum0 / 250;
    *adc1 = sum1 / 250;
}


bool Receive_vrir(bool if_send)			//*****************************************************ͨ�����߽��ܸı���趨ֵ
{
	

		bool bl;
		u8 RF_RxBuf[32],b;
		uint8_t if_change=false,if_=false;
		uint32_t vr,ir;
		CtrlMode mode;
	
		RxBufTop=0;		/* ���ջ�����ָ������ */
		RxBufBom=0;
		__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);	/* ����ձ�־ */
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	  /* ���ж� */
		__HAL_UART_ENABLE(&huart2);											/* ������ */	
	

		RF_Read(nRF24L01_R_REGISTER | nRF24L01_STATUS, 1, &b);
		if((b & 0x4E)==0x40)		/* 0ͨ�������� */
		{
			RF_Read(nRF24L01_R_RX_PAYLOAD, 10, RF_RxBuf);
			b=0x40;
			RF_Write(nRF24L01_W_REGISTER | nRF24L01_STATUS, 1, &b);	/* ���RX_DR��־ */
			

			vr = ((uint32_t)RF_RxBuf[0] << 24) | ((uint32_t)RF_RxBuf[1] << 16) |
										((uint32_t)RF_RxBuf[2] << 8)  | ((uint32_t)RF_RxBuf[3]);

			ir = ((uint32_t)RF_RxBuf[4] << 24) | ((uint32_t)RF_RxBuf[5] << 16) |
										((uint32_t)RF_RxBuf[6] << 8)  | ((uint32_t)RF_RxBuf[7]);
			mode = RF_RxBuf[8];
			if_change= RF_RxBuf[9];
			
			if (if_send==true && if_change==false) 
			{
				LoadSettingsFromEEPROM(&vr, &ir, &mode);
				Send_irvr(vr, ir, mode);
				return if_send;
			}
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);

			
			if (if_change==false) 
			{
				if (if_send == false)
				{
					SaveSettingsToEEPROM(vr, ir, mode);
					Send_irvr(vr, ir, mode);
				}
				else
				{
					uint32_t old_vr,old_ir;
					LoadSettingsFromEEPROM(&old_vr, &old_ir, &mode);
					SaveSettingsToEEPROM(vr, ir, mode);
					Send_irvr(vr, ir, mode);
				}
			}
			else
			{
				if (if_send==false)
				{
					bl=true;
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
					if (mode==MODE_VOLTAGE) 
					{
							uint32_t vvr=vr/1.0038f+0.8330737f;
							HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vvr);
							//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vr);
							HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
					}
					else if (mode==MODE_CURRENT) 
					{
							HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
							uint32_t iir=ir/1.0127f-36.964711f;
							HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iir);
							//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, ir);
							if (iir<780) 
							{
								uint32_t iiir=iir*0.9716f+30.0916f;
								HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iiir);
							}
					}
					else 
					{
							uint32_t vvr=vr/1.0038f+0.8330737f;
							HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vvr);
							//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vr);
							uint32_t iir=ir/1.0127f-36.964711f;
							HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iir);
							//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, ir);
							if (iir<780) 
							{
								uint32_t iiir=iir*0.9716f+30.0916f;
								HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iiir);
							}
					}

					MzPutString_cn(0,50,(u8 *)"������");
					HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				}
				else if(if_send==true)
				{
					bl=false;
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
					//HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
					//HAL_DAC_Stop(&hdac, DAC_CHANNEL_2);
					/*****************************************************�޸�Ϊ���0��*/
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
					//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);

					
					MzPutString_cn(0,50,(u8 *)"δ����");
					HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
				}
			}
			
		}
		if (if_change==true) if_=true;
		return if_change;
}

/******************************************************************************/

void SaveSettingsToEEPROM(uint32_t vr, uint32_t ir, CtrlMode mode)			//�����ʼ������
{
    uint8_t buf[9];
    buf[0] = (vr >> 24) & 0xFF;
    buf[1] = (vr >> 16) & 0xFF;
    buf[2] = (vr >> 8) & 0xFF;
    buf[3] = vr & 0xFF;

    buf[4] = (ir >> 24) & 0xFF;
    buf[5] = (ir >> 16) & 0xFF;
    buf[6] = (ir >> 8) & 0xFF;
    buf[7] = ir & 0xFF;

    buf[8] = (uint8_t)mode;

    //HAL_I2C_Mem_Write(&hi2c2, 0xA0, 0x00, I2C_MEMADD_SIZE_8BIT, buf, 9, 10);	//9�ֽ����ݣ���ʱ10ms
		for (int i = 0; i < 9; i++) {
			HAL_I2C_Mem_Write(&hi2c2, 0xA0, 0x00 + i, I2C_MEMADD_SIZE_8BIT, &buf[i], 1, 1);
			HAL_Delay(1);
		}
}

void LoadSettingsFromEEPROM(uint32_t *vr, uint32_t *ir, CtrlMode *mode)			//��ȡ��ʼ������
{
    uint8_t buf[9];
    if (HAL_I2C_Mem_Read(&hi2c2, 0xA0, 0x00, I2C_MEMADD_SIZE_8BIT, buf, 9, 10) == HAL_OK)
    {
        *vr = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
              ((uint32_t)buf[2] << 8) | buf[3];
        *ir = ((uint32_t)buf[4] << 24) | ((uint32_t)buf[5] << 16) |
              ((uint32_t)buf[6] << 8) | buf[7];
        *mode = (CtrlMode)buf[8];

        // ��� mode ���Ϸ�������Ϊ MODE_VOLTAGE
        if (*mode > MODE_AUTO) *mode = MODE_VOLTAGE;
    }
    else
    {
        *vr = 1000;     // Ĭ�ϵ�ѹ
        *ir = 3000;     // Ĭ�ϵ���
        *mode = MODE_VOLTAGE;
    }
}

void Send_irvr(uint32_t vr, uint32_t ir,CtrlMode mode)		//���߷���vr��ir��mode
{
		uint8_t buf[30];
		buf[0] = 0xAA;
	
    buf[1] = (vr >> 24) & 0xFF;
    buf[2] = (vr >> 16) & 0xFF;
    buf[3] = (vr >> 8) & 0xFF;
    buf[4] = vr & 0xFF;

    buf[5] = (ir >> 24) & 0xFF;
    buf[6] = (ir >> 16) & 0xFF;
    buf[7] = (ir >> 8) & 0xFF;
    buf[8] = ir & 0xFF;

    buf[9] = (uint8_t)mode;

		RF_Transmit(10, buf);
}

void Send_dc(uint32_t dcv, uint32_t dci)		//���߷���dcv��dci
{
		uint8_t buf[30];
	
		buf[0] = 0xBB;
		buf[1] = 0xCC;
	
		buf[2] = (dcv >> 24) & 0xFF;
    buf[3] = (dcv >> 16) & 0xFF;
    buf[4] = (dcv >> 8) & 0xFF;
    buf[5] = dcv & 0xFF;

    buf[6] = (dci >> 24) & 0xFF;
    buf[7] = (dci >> 16) & 0xFF;
    buf[8] = (dci >> 8) & 0xFF;
    buf[9] = dci & 0xFF;

		RF_Transmit(10, buf);
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



/****************************************************************************/

void TestRF24L01(void)				/* ����ͨѶģ��nRF24L01���� */
{
	u8 KV,RF_RxBuf[32],b;
	u16 dTx,dRx;

	
	RF_Init();
	MzInit();
	
LOOP:
	
	for (u8 i=0;i<=100;i++)
	{
		RF_Transmit(2, (u8 *)&i);
		HAL_Delay (500);
	}

	
	goto LOOP;			
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
		//MzPutString(0,50,(u8 *)"Send Succeed!");  
	}
	if((b & 0x10)==0x10)		/* �����ط����� */
	{
		RF_Write(nRF24L01_FLUSH_TX, 0, 0);	/* ���TX FIFO */
		b=0x10;
		RF_Write(nRF24L01_W_REGISTER | nRF24L01_STATUS, 1, &b);	/* ���MAX_RT��־ */
		//MzPutString(0,50,(u8 *)"Send Error!  ");  
	}
	b=0x0b;		/* CONFIG=0x0b */ 
	RF_Write(nRF24L01_W_REGISTER + nRF24L01_CONFIG, 1, &b);	/* ���ù���ģʽ��CRCʹ�ܡ�CRC����=1�ֽڡ��ϵ硢����ģʽ */
	HAL_GPIO_WritePin(RF_CE_GPIO_Port, RF_CE_Pin, GPIO_PIN_SET);			/* RF_CE=1�������״̬ */
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
		DC_DisplayScreen(i);		/* �л����� */
		HAL_Delay(1000);	
	}
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
