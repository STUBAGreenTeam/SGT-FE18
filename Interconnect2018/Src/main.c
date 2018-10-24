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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "CAN2018.h"
#include "dog_1701.h"
#include <font_16x32nums.h>
#include <font_6x8.h>
#include <font_8x16.h>
#include <font_8x8.h>
#include <logo_BLH.h>

#define TXSIZE 5
#define RXSIZE 5

#define VCC 3300
#define LEN 32   //pocet vzoriek spriemerovanych v ADC7

#define TEMP_H 45 //teplota pri ktorej zopnu vodne pumpy
#define TEMP_L 40 //teplota pri ktorej vypnu vodne pumpy

//pre telemetriu
#define RXBUFFERSIZE 100
#define RFTXSIZE 9

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

char string_buf[7];
uint8_t buzzed=0;
uint8_t brake_ok=0;
uint8_t pocitadlo=0;
uint8_t lv_count=0;
uint8_t rtd_ok=0;
uint8_t BSPD_OK=0;


uint8_t i,k,j,u,x;
uint8_t inertia,killswitch_R,killswitch_L; // snimanie shutdownu


double Time_Left[5],Time_Right[5],spd_L,spd_R,T_Left,T_Right;

#define TEETH 100
#define obvod 1.47655





uint16_t LV_voltage;   // [mV]
uint8_t inertia,killswitch_R,killswitch_L; // snimanie shutdownu

typedef struct message{
		uint16_t cell_voltage[7];	//[mV]
		uint8_t  cell_temp[7];	//[°C]
		int16_t  current;            	 		//[0.1A]
		uint8_t  disch_flags;
		uint8_t  SW_flags;
	}MESSAGE;
MESSAGE BMS_Data;

typedef struct adc_data{
	uint16_t LV_voltage;    //napatie meranie priamo interkonektom
	uint16_t sus_L;			//lavy tlmic
	uint16_t sus_R;			//pravy tlmic
	uint16_t stm_temp;
}ADC;
ADC adc_raw[LEN], adc_mv;
uint8_t pokus[]={1,3};

//buffer telemetrie
uint8_t RfTxBuffer[RFTXSIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Switch_action(uint8_t number_of_switch, uint8_t state);

static void digit5_to_ascii_int(int16_t value,char *buffer);
static void digit3_to_ascii(uint16_t value,char *buffer);
static void digit3_to_ascii_int(int8_t value,char *buffer);
static void digit5_to_ascii(uint16_t value,char *buffer);
static void digit_to_ascii_XX_X(uint16_t value,char *buffer);
static void digit_to_ascii_int_X_X(int16_t value,char *buffer);
static void print_display();
void adc_raw_to_mv(void);
void water_pump(void);

//fukncie pre telemetriu
static void tel_send_FU1(void);
//static void tel_send_bms(void);
static void tel_send_BBoxPower(void);
static void tel_send_ECU(void);
static void tel_send_INTERKONEKT(void);
//static void tel_send_BmsTemp1(void);
//static void tel_send_BmsTemp2(void);
//static void tel_send_BmsVoltage1(void);
//static void tel_send_BmsVoltage2(void);



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
CAN_FilterConfTypeDef sFilterConfig;

/// Struktury na ukladanie RAW data z CAN
extern CanTxMsgTypeDef CanTxMsg;
extern CanRxMsgTypeDef CanRxMsg;
extern BBOX_status_TypeDef BBOX_status_Data;
extern BBOX_power_TypeDef BBOX_power_Data;
extern BBOX_command_TypeDef BBOX_command_Data;
extern Interconnect_TypeDef Interconnect_Data;
extern FU_Values_1_TypeDef FU_Values_1_Data;
extern FU_Values_2_TypeDef FU_Values_2_Data;
extern ECU_State_TypeDef ECU_State_Data;
extern BMS_Command_TypeDef BMS_Command_Data;
extern wheel_RPM_TypeDef wheel_RPM_Data;


void initCanFilter(void)
{
    sFilterConfig.FilterNumber = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        sFilterConfig.BankNumber = 0;
        sFilterConfig.FilterActivation = ENABLE;

        HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if(j==5)j=0;
			Time_Left[j]=__HAL_TIM_GET_COUNTER(&htim3);;
			j++;
			__HAL_TIM_SET_COUNTER(&htim3,0);
		}

	if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			//  Time_Right*10 000 = time in seconds
			if(k==5)k=0;
			Time_Right[k]=__HAL_TIM_GET_COUNTER(&htim4);;
			k++;
			__HAL_TIM_SET_COUNTER(&htim4,0);
		}
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
 // HAL_GPIO_WritePin(SW6_GPIO_Port,SW6_Pin,GPIO_PIN_SET);

  initCanFilter();
  hcan1.pTxMsg = &CanTxMsg; //Pointer struktury hcan na data nastavi na struktur v CAN.c
  hcan1.pRxMsg = &CanRxMsg;
   /// Zakladne hodnoty pre struktury nasej zbernice CAN. Aplikuju sa podla nastavenia CAN sprav
   //canDefaults(&hcan1);
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0); //Interupt na prijatie prvej spravy. Dalej nastavujeme v evente.

  while(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_raw, (LEN*sizeof(ADC))/2);

  //nastavenie displaju
  initialize(30,22,24,26,28,DOGS102);
  view(VIEW_TOP);  //default viewing direction
  clear();  //clear whole display
  contrast(10);

  HAL_Delay(50);
  HAL_GPIO_WritePin(SW2_GPIO_Port,SW2_Pin,GPIO_PIN_SET);   // zopnutie napajanie shutdownu



  	  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
      HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);
      HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_3);
      HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_4);
      HAL_TIM_Base_Start_IT(&htim3);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {     HAL_GPIO_WritePin(SW6_GPIO_Port,SW6_Pin,GPIO_PIN_SET);
		switch (Interconnect_Data.car_state) {
		case 0:
			Interconnect_Data.car_state = 1;
			break;
		case 1:  ///wait for BMS and IMD ok
			if ((BBOX_status_Data.IMD_OK == 1))
				//	&& (BBOX_status_Data.BMS_OK == 1))
				Interconnect_Data.car_state = 2;
			break;
		case 2:  ///wait for SHDN OK
			if (BBOX_status_Data.TSMS > 0) {
				Interconnect_Data.car_state = 3;
			}
			else (Interconnect_Data.car_state=2);

			break;
		case 3:  /// wait for HV precharged all OK
			if ((BBOX_status_Data.AIR_N == 1) && (BBOX_status_Data.AIR_P == 1))
				Interconnect_Data.car_state = 4;
			break;
		case 4:  ///wait for rtd
				 //rtd_ok=0;
			if (FU_Values_2_Data.RTD == 1)
				rtd_ok = 1;
			if ((rtd_ok == 1) && (FU_Values_2_Data.RTD == 0)) {
				Interconnect_Data.car_state = 5;
				rtd_ok = 0;
			}
			//rtd_ok=0;
			break;
		case 5:		  ///wait for break press and release
			if ((BBOX_status_Data.TSMS < 1))
							Interconnect_Data.car_state = 1;

			if ((FU_Values_1_Data.brake1 + FU_Values_1_Data.brake2) > 10)
				brake_ok = 1;
			if ((brake_ok == 1)
					&& ((FU_Values_1_Data.brake1 + FU_Values_1_Data.brake2) > 10)
					&& (FU_Values_2_Data.RTD)) {

				Interconnect_Data.car_state = 6;

				HAL_GPIO_WritePin(SW3_GPIO_Port, SW3_Pin, GPIO_PIN_SET);
				Interconnect_Data.tsas = 1;
				HAL_Delay(1500);
				HAL_GPIO_WritePin(SW3_GPIO_Port, SW3_Pin, GPIO_PIN_RESET);
				Interconnect_Data.tsas = 0;
				//buzzed=0;
				 //Interconnect_Data.tsas=1;
			//	 if (buzzed==0) HAL_TIM_Base_Start_IT(&htim6);
			}
			break;
		case 6:		  /// all OK driving mode

			if ((BBOX_status_Data.TSMS < 1)) {
				Interconnect_Data.car_state = 1;

				Interconnect_Data.tsas = 0;
				rtd_ok=0;
			}
			if (((FU_Values_1_Data.apps1 - FU_Values_1_Data.apps2) > 20)
					|| ((FU_Values_1_Data.apps2 - FU_Values_1_Data.apps1) > 20))
				Interconnect_Data.car_state = 7;

			/*
			if (((FU_Values_1_Data.brake1 + FU_Values_1_Data.brake2) > 6)
					&& ((FU_Values_1_Data.apps1 + FU_Values_1_Data.apps2) > 50))
				Interconnect_Data.car_state = 8;
*/
			break;
		case 7:		  ///implausability check acc1 nad acc2

			if ((BBOX_status_Data.TSMS < 1)) {
				Interconnect_Data.car_state = 1;

				HAL_Delay(1000);
				Interconnect_Data.tsas = 0;
			}
			if (((FU_Values_1_Data.apps1 - FU_Values_1_Data.apps2) > 20)
					|| ((FU_Values_1_Data.apps2 - FU_Values_1_Data.apps1) > 20)
					)//|| (FU_Values_1_Data.error > 0))
				Interconnect_Data.car_state = 7;
			else
				Interconnect_Data.car_state = 6;
			break;
		case 8:		  ///plausability check acc and brake

			if ((BBOX_status_Data.TSMS < 1)) {
				Interconnect_Data.car_state = 1;
				HAL_Delay(1000);
				Interconnect_Data.tsas = 0;
			}
			if ((FU_Values_1_Data.apps1 + FU_Values_1_Data.apps2) < 11)
				Interconnect_Data.car_state = 6;
			break;
		default:
			break;
		}

		//zopnutie brzdoveho svetla
		if ((FU_Values_1_Data.brake1 > 3) || (FU_Values_1_Data.brake2 > 3)) {
			Interconnect_Data.brake_red = 1;
			HAL_GPIO_WritePin(SW4_GPIO_Port, SW4_Pin, GPIO_PIN_SET); //red
			HAL_GPIO_WritePin(SW5_GPIO_Port,SW5_Pin,GPIO_PIN_RESET);  // white turn off
			Interconnect_Data.brake_white=0;
		}
		else {
			Interconnect_Data.brake_red = 0;
			HAL_GPIO_WritePin(SW4_GPIO_Port, SW4_Pin, GPIO_PIN_RESET); //red
		}

		if((Interconnect_Data.car_state<3)&&(Interconnect_Data.brake_red==0)) {
			HAL_GPIO_WritePin(SW5_GPIO_Port,SW5_Pin,GPIO_PIN_SET);
			Interconnect_Data.brake_white=1;
		}

		else {
			HAL_GPIO_WritePin(SW5_GPIO_Port,SW5_Pin,GPIO_PIN_RESET);
			Interconnect_Data.brake_white=0;
		}


		water_pump();

		(HAL_GPIO_ReadPin(KSW_L_GPIO_Port,KSW_L_Pin))? (Interconnect_Data.killswitch_L=0):(Interconnect_Data.killswitch_L=1);
		(HAL_GPIO_ReadPin(KSW_R_GPIO_Port,KSW_R_Pin))?(Interconnect_Data.killswitch_R=0):(Interconnect_Data.killswitch_R=1);
		(HAL_GPIO_ReadPin(BSPD_OK_GPIO_Port,BSPD_OK_Pin))?(BSPD_OK=1):(BSPD_OK=0);
		HAL_UART_Receive_IT(&huart2, (uint8_t*)&BMS_Data,(uint16_t) sizeof(MESSAGE));

		LV_voltage=0;
		for(i=0;i<7;i++) {
			LV_voltage+= BMS_Data.cell_voltage[i];
		}
		Interconnect_Data.susp_RL=LV_voltage;
		Interconnect_Data.susp_RR=(uint16_t)(BMS_Data.current);
		adc_raw_to_mv();
		print_display();

		/*
		//posli telemetriu
		tel_send_FU1();
		tel_send_BBoxPower();
		tel_send_ECU();
		tel_send_INTERKONEKT();
*/
		//otáèky
						for(u=0;u<5;u++)
							  	  {
								  	  T_Left=T_Left + Time_Left[u];
								  	  T_Right=T_Right + Time_Right[u];
							  	  }
							  T_Left=T_Left/5.0;
							  T_Right=T_Right/5.0;
							  if (T_Left!=0)  //(T_Left!=0) && (T_Right!=0)
							  {
							  wheel_RPM_Data.rear_left = ((10000*obvod)/((T_Left)*TEETH));  	 // m/s
							  wheel_RPM_Data.rear_right = ((10000*obvod)/((T_Right)*TEETH));		//m/s
							  }
							  if(__HAL_TIM_GET_COUNTER(&htim4)>=5000)
							  	  {
								  for(u=0;u<5;u++)
								  	  	  {
								  		  	  Time_Left[u]=0;
								  		  	  Time_Right[u]=0;
								  		  	  spd_L=0;
								  		  	  spd_R=0;
								  		  	  T_Left=0;
								  		  	  T_Right=0;
								  		  	  k=0;
								  		  	  j=0;
								  	  	  }
							  	  }


				Interconnect_Data.reserve=BBOX_status_Data.STM_temp;


				x=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);





		HAL_UART_Transmit(&huart1, (uint8_t*)&BMS_Data,(uint16_t) sizeof(MESSAGE),10);

		//HAL_UART_Transmit(&huart1, (uint8_t*)&pokus, 2, 5 );

		Tx_Interconnect_Data(&hcan1, &Interconnect_Data);
		Tx_wheel_RPM_Data(&hcan1,&wheel_RPM_Data);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		HAL_Delay(15);

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* CAN1_TX_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  /* CAN1_RX1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* CAN1_SCE_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_5TQ;
  hcan1.Init.BS2 = CAN_BS2_2TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3600;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3600;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PB0   ------> ADCx_IN8
     PA15   ------> SPI3_NSS
     PC10   ------> USART3_TX
     PC11   ------> USART3_RX
     PC12   ------> USART3_CK
     PB3   ------> SPI3_SCK
     PB4   ------> SPI3_MISO
     PB5   ------> SPI3_MOSI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SW7_Pin|SW6_Pin|SW5_Pin|SW4_Pin 
                          |DISP_CD_Pin|DISP_CS_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SW3_Pin|SW2_Pin|SW1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin|GPIO_PIN_13|DISP_RESET_Pin 
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW7_Pin SW6_Pin SW5_Pin SW4_Pin 
                           DISP_CD_Pin DISP_CS_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = SW7_Pin|SW6_Pin|SW5_Pin|SW4_Pin 
                          |DISP_CD_Pin|DISP_CS_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SW3_Pin SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW3_Pin|SW2_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KSW_R_Pin KSW_L_Pin BSPD_OK_Pin */
  GPIO_InitStruct.Pin = KSW_R_Pin|KSW_L_Pin|BSPD_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LV_VOLTAGE_Pin */
  GPIO_InitStruct.Pin = LV_VOLTAGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(LV_VOLTAGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED3_Pin PB13 DISP_RESET_Pin 
                           PB15 */
  GPIO_InitStruct.Pin = LED4_Pin|LED3_Pin|GPIO_PIN_13|DISP_RESET_Pin 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_USART3_PARTIAL();

}

/* USER CODE BEGIN 4 */


void Switch_action(uint8_t number_of_switch, uint8_t state){

	switch(number_of_switch) {
			case (1): HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,state ); break;
			case (2): HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,state); break;
			case (3): HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0, state); break;
			case (4): HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,state); break;
			case (5): HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,state); break;
			case (6): HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,state); break;
			case (7): HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,state); break;
		}
}

static void digit5_to_ascii_int(int16_t value,char *buffer)
{
	buffer[0]=(value>=0)?'+':'-';
	value=(value>=0)?value:-value;
	buffer[1]=(value/10000)+'0';
	buffer[2]=((value%10000)/1000)+'0';
	buffer[3]=((value%1000)/100)+'0';
	buffer[4]=((value%100)/10)+'0';
	buffer[5]=(value%10)+'0';
	buffer[5]='\0';
}

static void digit5_to_ascii(uint16_t value,char *buffer)
{
	buffer[0]=(value/10000)+'0';
	buffer[1]=((value%10000)/1000)+'0';
	buffer[2]=((value%1000)/100)+'0';
	buffer[3]=((value%100)/10)+'0';
	buffer[4]=(value%10)+'0';
	buffer[5]='\0';
}


static void digit3_to_ascii(uint16_t value,char *buffer)
{

	buffer[0]=(value/100)+'0';
	buffer[1]=((value%100)/10)+'0';
	buffer[2]=(value%10)+'0';
	buffer[3]='\0';

}

static void digit3_to_ascii_int(int8_t value,char *buffer)
{
	buffer[0]=(value>=0)?'+':'-';
	value=value>=0?value:-value;
	buffer[1]=(value/100)+'0';
	buffer[2]=((value%100)/10)+'0';
	buffer[3]=(value%10)+'0';
	buffer[4]='\0';
}

static void digit4_to_ascii(uint16_t value,char *buffer)
{
	buffer[0]=(value/1000)+'0';
	buffer[1]=((value%1000)/100)+'0';
	buffer[2]=((value%100)/10)+'0';
	buffer[3]=(value%10)+'0';
	buffer[4]='\0';
}

static void digit_to_ascii_XX_X(uint16_t value,char *buffer)
{

	buffer[0]=(value/100)+'0';
	buffer[1]= ((value%100)/10) +'0';
	buffer[2]='.';
	buffer[3]= (value%10)+'0';
	buffer[4]='\0';

}

static void digit_to_ascii_int_X_X(int16_t value,char *buffer)
{
	buffer[0]=(value>=0)?'+':'-';
	value=(value>=0)?value:-value;
	buffer[0]= (value/10) +'0';
	buffer[1]='.';
	buffer[2]= (value%10)+'0';
	buffer[3]='\0';
}

static void print_display()
{

	 string(0, 0, font_6x8, "Ac1:");
	digit3_to_ascii(FU_Values_1_Data.apps1, string_buf);
	string(25, 0, font_6x8, string_buf);
	string(50, 0, font_6x8, "Ac2:");
	digit3_to_ascii(FU_Values_1_Data.apps2, string_buf);
	string(75, 0, font_6x8, string_buf);
	string(0, 1, font_6x8, "Br1:");
	digit3_to_ascii(FU_Values_1_Data.brake1, string_buf);
	string(25, 1, font_6x8, string_buf);
	string(50, 1, font_6x8, "Br2:");
	digit3_to_ascii(FU_Values_1_Data.brake2, string_buf);
	string(75, 1, font_6x8, string_buf);

	string(0, 2, font_6x8, "Err:");
	string_buf[0] = (FU_Values_1_Data.error & 0b10000) == 16 ? '1' : '0';
	string_buf[1] = (FU_Values_1_Data.error & 0b1000) == 8 ? '1' : '0';
	string_buf[2] = (FU_Values_1_Data.error & 0b100) == 4 ? '1' : '0';
	string_buf[3] = (FU_Values_1_Data.error & 0b10) == 2 ? '1' : '0';
	string_buf[4] = (FU_Values_1_Data.error & 0b1) == 1 ? '1' : '0';
	string_buf[5] = '\0';
	string(25, 2, font_6x8, string_buf);

	string(60, 2, font_6x8, "State:");
	string_buf[0] = Interconnect_Data.car_state + '0';
	string_buf[1] = '\0';
	string(96, 2, font_6x8, string_buf);

	string(0, 3, font_6x8, "Str:");
	digit3_to_ascii_int(FU_Values_2_Data.steer, string_buf);
	string(25, 3, font_6x8, string_buf);

	string(55, 3, font_6x8, "Rtd:");
	string_buf[0] = FU_Values_2_Data.RTD + '0';
	string_buf[1] = '\0';
	string(80, 3, font_6x8, string_buf);

	string(0, 5, font_6x8, "IMD:");
	//digit3_to_ascii(BBOX_status_Data.IMD_OK,string_buf);
	string_buf[0] = BBOX_status_Data.IMD_OK + '0';
	string_buf[1] = '\0';
	string(25, 5, font_6x8, string_buf);

	string(35, 5, font_6x8, "RES:");
	//digit3_to_ascii(BBOX_status_Data.AMS_OK,string_buf);
	string_buf[0] = BBOX_status_Data.SHD_RESET + '0';
	string_buf[1] = '\0';
	string(60, 5, font_6x8, string_buf);

	string(70, 5, font_6x8, "SHO:");
	//digit3_to_ascii(BBOX_status_Data.AMS_OK,string_buf);
	string_buf[0] = BBOX_status_Data.SHD_OUT + '0';
	string_buf[1] = '\0';
	string(96, 5, font_6x8, string_buf);

	string(0, 6, font_6x8, "Ulv:");
	digit_to_ascii_XX_X((LV_voltage/100), string_buf);
	string(24, 6, font_6x8, string_buf);
	string(48, 6, font_6x8, "V");

	string(55, 6, font_6x8, "Uhv:");
	if (BBOX_power_Data.voltage > 0) {
		digit3_to_ascii( BBOX_power_Data.voltage, string_buf);
		string(75, 6, font_6x8, string_buf);
	} else {
		string(75, 6, font_6x8, "000");
	}
	string(95, 6, font_6x8, "V");


	string(55, 7, font_6x8, "Ilv");
	digit_to_ascii_int_X_X(BMS_Data.current,string_buf);
	string(75, 7, font_6x8, string_buf);
	string(95, 7, font_6x8,"A");




	string(0, 7, font_6x8, "SHD:     ");
	if((BSPD_OK==1)&&(FU_Values_2_Data.INERTIA_SW==1)&&(FU_Values_2_Data.SHDB==1)&&(FU_Values_2_Data.BOTS==1)&&(Interconnect_Data.killswitch_L==1)
			&&(Interconnect_Data.killswitch_R==1)&&(BBOX_status_Data.IMD_OK)
			&&(BBOX_status_Data.SHD_EN)&&((BBOX_status_Data.BMS_OK)==0)&&(BBOX_status_Data.TSMS==1))
		string(25, 7, font_6x8, "ON");

	else if(BSPD_OK==0)							string(25, 7, font_6x8, "BSPD");
	else if(FU_Values_2_Data.INERTIA_SW==0) 	string(25, 7, font_6x8, "INT");
	else if(FU_Values_2_Data.SHDB==0) 			string(25, 7, font_6x8, "KSW_F");
	else if(FU_Values_2_Data.BOTS==0) 			string(25, 7, font_6x8, "BOTS");
	else if(Interconnect_Data.killswitch_L==0) 	string(25, 7, font_6x8, "KSW_L");
	else if(Interconnect_Data.killswitch_R==0) 	string(25, 7, font_6x8, "KSW_R");
	else if(BBOX_status_Data.IMD_OK==0)  		string(25, 7, font_6x8, "IMD");
	else if(BBOX_status_Data.SHD_EN==0)  		string(25, 7, font_6x8, "SHD_EN");
	else if(BBOX_status_Data.TSMS==0) 			string(25, 7, font_6x8, "TSMS");
	//else if(BBOX_status_Data.BMS_OK==0)  string(25, 7, font_6x8, "BMS");
	else       		  string(25, 7, font_6x8, "ERR");


	/*
	string(54, 7, font_6x8, "SHDN:");
	if (BBOX_status_Data.AIR_P == 1 && BBOX_status_Data.AIR_N == 1) {
		string(84, 7, font_6x8, "ON ");
	} else {
		string(84, 7, font_6x8, "OFF");
	}
    */



}

void adc_raw_to_mv(void){
	uint8_t i;
	adc_mv.LV_voltage=0;
	adc_mv.sus_L=0;
	adc_mv.sus_R=0;

	for(i=0;i<LEN;i++){
			adc_mv.LV_voltage+=(uint16_t)adc_raw[i].LV_voltage;
			adc_mv.sus_L+=(uint16_t)adc_raw[i].sus_L;
			adc_mv.sus_R+= (uint16_t)adc_raw[i].sus_R;
		}

		//prepocita na milivolty
		adc_mv.LV_voltage=adc_mv.LV_voltage *VCC/(4095*LEN);
		adc_mv.sus_L=adc_mv.sus_L*VCC/(4095*LEN);
		adc_mv.sus_R=adc_mv.sus_R*VCC/(4095*LEN);
}

void water_pump(void){

	if((ECU_State_Data.TempIGBT_H>TEMP_H)||(ECU_State_Data.TempInverter_H>TEMP_H)||(ECU_State_Data.TempMotor_H>TEMP_H)){
		HAL_GPIO_WritePin(SW1_GPIO_Port,SW1_Pin, GPIO_PIN_SET);  //pumpa R?
		HAL_GPIO_WritePin(SW2_GPIO_Port,SW2_Pin, GPIO_PIN_SET);  //pumpa_L?
		HAL_GPIO_WritePin(SW7_GPIO_Port,SW7_Pin, GPIO_PIN_SET);  //fany na chladice

	};



	if((ECU_State_Data.TempIGBT_H<TEMP_L)&&(ECU_State_Data.TempInverter_H<TEMP_L)&&(ECU_State_Data.TempMotor_H<TEMP_L)){
			HAL_GPIO_WritePin(SW1_GPIO_Port,SW1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SW2_GPIO_Port,SW2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SW7_GPIO_Port,SW7_Pin, GPIO_PIN_RESET);  //fany na chladice
	};
}


static void tel_send_FU1(void)
{
	HAL_UART_Transmit(&huart1,FU_Values_1_Data.apps1,1,1);
	HAL_UART_Transmit(&huart1,FU_Values_1_Data.apps2,1,1);
	HAL_UART_Transmit(&huart1,FU_Values_1_Data.brake1,1,1);
	HAL_UART_Transmit(&huart1,FU_Values_1_Data.brake2,1,1);
	HAL_UART_Transmit(&huart1,FU_Values_1_Data.error / 256,1,1);
	HAL_UART_Transmit(&huart1,FU_Values_1_Data.error % 256,1,1);
	HAL_UART_Transmit(&huart1,FU_Values_2_Data.brake_pos,1,1);
	HAL_UART_Transmit(&huart1,"b",1,1);
	HAL_UART_Transmit(&huart1,255,1,1);
}
/*
static void tel_send_bms(void)
{
	RfTxBuffer[0]=BMS_State_Data.BMS_Faults/256;
	RfTxBuffer[1]=BMS_State_Data.BMS_Faults%256;

	RfTxBuffer[2]=BMS_State_Data.BMS_Mode;
	RfTxBuffer[3]=BMS_State_Data.CellTemp_H;

	RfTxBuffer[4]=BMS_State_Data.CellTemp_L;
	RfTxBuffer[5]=BMS_State_Data.CellVolt_H;

	RfTxBuffer[6]=BMS_State_Data.CellVolt_L;
	RfTxBuffer[7]='b';

	RfTxBuffer[8]=255;
	HAL_UART_Transmit(&huart1,RfTxBuffer,9,3);
	//HAL_UART_Transmit(&huart1,"b",1,1);
}
*/

static void tel_send_BBoxPower(void)
{
	RfTxBuffer[0]=BBOX_status_Data.AIR_N*128 + BBOX_status_Data.AIR_P*64 + BBOX_status_Data.BMS_OK*32 + BBOX_status_Data.FANS*16 + BBOX_status_Data.IMD_OK*8 + BBOX_status_Data.POLARITY*4 + BBOX_status_Data.SHD_EN*2+ BBOX_status_Data.SHD_RESET;

	HAL_UART_Transmit(&huart1,BBOX_power_Data.current/256,1,1);
	HAL_UART_Transmit(&huart1,BBOX_power_Data.current%265,1,1);
	HAL_UART_Transmit(&huart1,BBOX_power_Data.power/256,1,1);
	HAL_UART_Transmit(&huart1,BBOX_power_Data.power%256,1,1);
	HAL_UART_Transmit(&huart1,BBOX_power_Data.power/256,1,1);
	HAL_UART_Transmit(&huart1,BBOX_power_Data.voltage%256,1,1);
	HAL_UART_Transmit(&huart1,RfTxBuffer,1,1);
	HAL_UART_Transmit(&huart1,"p",1,1);
	HAL_UART_Transmit(&huart1,255,1,1);

}

static void tel_send_ECU(void)
{
	HAL_UART_Transmit(&huart1,ECU_State_Data.ECU_Status,1,1);
	HAL_UART_Transmit(&huart1,ECU_State_Data.FL_AMK_Status,1,1);
	HAL_UART_Transmit(&huart1,ECU_State_Data.FR_AMK_Status,1,1);
	HAL_UART_Transmit(&huart1,ECU_State_Data.RL_AMK_Status,1,1);
	HAL_UART_Transmit(&huart1,ECU_State_Data.RR_AMK_Status,1,1);
	HAL_UART_Transmit(&huart1,ECU_State_Data.TempInverter_H,1,1);
	HAL_UART_Transmit(&huart1,ECU_State_Data.TempMotor_H,1,1);
	HAL_UART_Transmit(&huart1,"e",1,1);
	HAL_UART_Transmit(&huart1,255,1,1);
}
static void tel_send_INTERKONEKT(void)
{
	RfTxBuffer[0]=Interconnect_Data.tsas*128 + Interconnect_Data.right_w_pump*64 + Interconnect_Data.left_w_pump*32 + Interconnect_Data.killswitch_R*16 + Interconnect_Data.killswitch_L*8 + Interconnect_Data.brake_red*4 + Interconnect_Data.brake_white*2 ;
	HAL_UART_Transmit(&huart1,Interconnect_Data.car_state,1,1);
	HAL_UART_Transmit(&huart1,RfTxBuffer,1,1);
	HAL_UART_Transmit(&huart1,Interconnect_Data.susp_RL/256,1,1);
	HAL_UART_Transmit(&huart1,Interconnect_Data.susp_RL%256,1,1);
	HAL_UART_Transmit(&huart1,Interconnect_Data.susp_RR/256,1,1);
	HAL_UART_Transmit(&huart1,Interconnect_Data.susp_RR%256,1,1);
	HAL_UART_Transmit(&huart1,'i',1,1);
	HAL_UART_Transmit(&huart1,255,1,1);
}
/*
static void tel_send_ECU(void)
{
    RfTxBuffer[0]=ECU_State_Data.ECU_Status;
	RfTxBuffer[1]=ECU_State_Data.FL_AMK_Status;

	RfTxBuffer[2]=ECU_State_Data.FR_AMK_Status;
	RfTxBuffer[3]=ECU_State_Data.RL_AMK_Status;

	RfTxBuffer[4]=ECU_State_Data.RR_AMK_Status;
	//RfTxBuffer[5]=ECU_State_Data.TempIGBT_H;

	RfTxBuffer[5]=ECU_State_Data.TempInverter_H;
	RfTxBuffer[6]=ECU_State_Data.TempMotor_H;

	RfTxBuffer[7]='e';
	RfTxBuffer[8]=255;
	HAL_UART_Transmit(&huart1,RfTxBuffer,9,3);
	//HAL_UART_Transmit(&huart1,"e",1,1);
}*/
/*
static void tel_send_BBoxPower(void)
{
	RfTxBuffer[0]=BBOX_power_Data.current/256;
	RfTxBuffer[1]=BBOX_power_Data.current%265;

	RfTxBuffer[2]=BBOX_power_Data.power/256;
	RfTxBuffer[3]=BBOX_power_Data.power%256;

	RfTxBuffer[4]=BBOX_power_Data.voltage/256;
	RfTxBuffer[5]=BBOX_power_Data.voltage%256;

	RfTxBuffer[6]=BBOX_status_Data.AIR_N*128 + BBOX_status_Data.AIR_P*64 + BBOX_status_Data.BMS_OK*32 + BBOX_status_Data.FANS*16 + BBOX_status_Data.IMD_OK*8 + BBOX_status_Data.POLARITY*4 + BBOX_status_Data.SHD_EN*2+ BBOX_status_Data.SHD_RESET;
	RfTxBuffer[7]='p';

	RfTxBuffer[8]=255;
	HAL_UART_Transmit(&huart1,RfTxBuffer,9,3);
	//HAL_UART_Transmit(&huart1,"p",1,1);
}*/
/*
static void tel_send_INTERKONEKT(void)
{
    RfTxBuffer[0]=Interconnect_Data.car_state;
	RfTxBuffer[1]=Interconnect_Data.tsas*128 + Interconnect_Data.right_w_pump*64 + Interconnect_Data.left_w_pump*32 + Interconnect_Data.killswitch_R*16 + Interconnect_Data.killswitch_L*8 + Interconnect_Data.brake_red*4 + Interconnect_Data.brake_white*2 ;
	RfTxBuffer[2]=Interconnect_Data.susp_RL/256;
	RfTxBuffer[3]=Interconnect_Data.susp_RL%256;
	RfTxBuffer[4]=Interconnect_Data.susp_RR/256;
	RfTxBuffer[5]=Interconnect_Data.susp_RR%256;
	RfTxBuffer[6]='i';
	RfTxBuffer[7]=255;
	HAL_UART_Transmit(&huart1,RfTxBuffer,8,3);
	//HAL_UART_Transmit(&huart1,"i",1,1);
}*/
/*
static void tel_send_FU1(void)
{
	RfTxBuffer[0]=FU_Values_1_Data.apps1;
	RfTxBuffer[1]=FU_Values_1_Data.apps2;

	RfTxBuffer[2]=FU_Values_1_Data.brake1;
	RfTxBuffer[3]=FU_Values_1_Data.brake2;

	RfTxBuffer[4]=FU_Values_1_Data.error / 256;
	RfTxBuffer[5]=FU_Values_1_Data.error % 256;

	//RfTxBuffer[6]=FU_Values_2_Data.steer;
	RfTxBuffer[6]=FU_Values_2_Data.brake_pos;

	RfTxBuffer[7]='f';
	RfTxBuffer[8]=255;
    HAL_UART_Transmit(&huart1,RfTxBuffer,9,3);
}*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
