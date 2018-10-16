/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#define RXBUFFERSIZE 100
#define RFTXSIZE 9
#define BMS_SLAVE 0

//definy na prepocet
#define VBUS_DIV 26.8
//Mcu->T_igbt - 15170)/102;
#define T_IGBT_SUB 15170
#define T_IGBT_DIV 102
//Mcu->T_motor = (Mcu->T_motor - 10000)/41;
#define T_MOTOR_SUB 10000
#define T_MOTOR_DIV 41
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const uint8_t premennavoflash;
extern uint32_t master_counter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void tel_send_FU1(void);
static void tel_send_bms(void);
static void tel_send_BBoxPower(void);
static void tel_send_ECU(void);
static void tel_send_INTERKONEKT(void);
static void tel_send_BmsTemp1(void);
static void tel_send_BmsTemp2(void);
static void tel_send_BmsVoltage1(void);
static void tel_send_BmsVoltage2(void);
/*
static void tel_send_CCU(void);
static void tel_send_PSU(void);
static void tel_send_InterConnect(void);
static void tel_send_BMS_Slaves(void);
static void tel_send_MCU(void);
static void tel_send_MCU_slow_rate(void);
static void tel_send_master_info();
static void tel_Power_data(void);

static void digit1_to_ascii(uint8_t value,uint8_t *buffer);
static void digit2_to_ascii(uint8_t value,uint8_t *buffer);
static void digit3_to_ascii(uint8_t value,uint8_t *buffer);
static void digit4_to_ascii(uint16_t value,uint8_t *buffer);
static void digit5_to_ascii(uint16_t value,uint8_t *buffer);
static void digit3_to_ascii_int(int8_t value,uint8_t *buffer);
static void digit5_to_ascii_int(int16_t value,uint8_t *buffer);
static void digit5_to_ascii_int32(int32_t value,uint8_t *buffer);

static void digit3_to_ascii_16bit(uint16_t value,uint8_t *buffer);
*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t slave_iterator=BMS_SLAVE;
uint8_t i;

uint8_t aRxBuffer[RXBUFFERSIZE];
uint8_t RfTxBuffer[RFTXSIZE];

CAN_FilterConfTypeDef sFilterConfig;

/// Struktury na ukladanie RAW data z CAN
extern CanTxMsgTypeDef CanTxMsg;
extern CanRxMsgTypeDef CanRxMsg;

extern 	BBOX_power_TypeDef 		BBOX_power_Data;
extern  BBOX_status_TypeDef 	BBOX_status_Data;
extern  BMS_State_TypeDef 		BMS_State_Data;
extern  BMS_Voltages_TypeDef 	BMS_Voltages_Data;
extern  BMS_Temps_TypeDef 		BMS_Temps_Data;

extern  wheel_RPM_TypeDef 		wheel_RPM_Data;
extern  FU_Values_1_TypeDef 	FU_Values_1_Data;
extern  FU_Values_2_TypeDef	 	FU_Values_2_Data;

extern  ECU_State_TypeDef	 	ECU_State_Data;

extern  Interconnect_TypeDef 	Interconnect_Data;

//stare
/*
extern CanTxMsgTypeDef CanTxMsg;
extern CanRxMsgTypeDef CanRxMsg;
extern BBOX_Power_TypeDef BBOX_Power_Data;

extern psu_TypeDef psu_Data;
extern interconnect_TypeDef interconnect_Data;
extern CCU_TypeDef CCU_Data;
extern BBOX_Status_TypeDef BBOX_Status_Data;

extern MCU_CMD_TypeDef McuR_Cmd;
extern MCU_CMD_TypeDef McuL_Cmd;

extern MCU_DATA_TypeDef McuR_Data;
extern MCU_DATA_TypeDef McuL_Data;*/
//koniec starých


//extern LV_DATA_TypeDef LV_Data;
//extern LV_ERROR_TypeDef LV_Error;

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

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
 // HAL_Delay(100 );
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
    HAL_Delay(500);

    ///Tieto inicializacie su NUTNE!


        initCanFilter();

        /// Pointer struktury hcan na data treba nastavit na strukturu, ktoru mame vytvorenu v CAN.c
        hcan1.pTxMsg = &CanTxMsg;
        hcan1.pRxMsg = &CanRxMsg;

        /// Zakladne hodnoty pre struktury nasej zbernice CAN. Aplikuju sa podla nastavenia CAN sprav
        //canDefaults(&hcan1);

        /// Nastavim interrupt na prijatie prvej spravy. Dalej nastavujeme v evente.

        HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);

	 	  int i=0;
	 	  for (i=0;i<5;i++)
	 	  {

	 	  HAL_Delay(200);
	 	  tel_send_FU1();
	 	  tel_send_bms();
	 	  tel_send_BBoxPower();
	 	  tel_send_ECU();
	 	  tel_send_INTERKONEKT();
	 	  /*tel_send_PSU();
	 	  tel_send_CCU();
	 	  tel_send_master_info();
	 	  tel_Power_data();*/


	 	  //tel_send_Power();

	 	  }
	 	  /*tel_send_MCU();
	 	  tel_send_InterConnect();
	 	  tel_send_MCU_slow_rate();
	 	  tel_send_BMS_Slaves();*/
	 	  int FullMode =0;
	 	  if(FullMode)
	 	  {
	 		 tel_send_BmsTemp1();
	 		 tel_send_BmsTemp2();
	 		 tel_send_BmsVoltage1();
	 		 tel_send_BmsVoltage2();
	 	  }


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

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

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_2TQ;
  hcan1.Init.BS2 = CAN_BS2_5TQ;
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

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
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
}
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
}

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
}
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
}

static void tel_send_BmsTemp1(void)
{
	RfTxBuffer[0]=BMS_Temps_Data.BMS_TempIdent;
	RfTxBuffer[1]=BMS_Temps_Data.BMS_Temp1;

	RfTxBuffer[2]=BMS_Temps_Data.BMS_Temp2;
	RfTxBuffer[3]=BMS_Temps_Data.BMS_Temp3;

	RfTxBuffer[4]=BMS_Temps_Data.BMS_Temp4;
	RfTxBuffer[5]='T';

	RfTxBuffer[6]=0xff;
	HAL_UART_Transmit(&huart1,RfTxBuffer,7,3);
}
static void tel_send_BmsTemp2(void)
{
	RfTxBuffer[0]=BMS_Temps_Data.BMS_TempIdent;
	RfTxBuffer[1]=BMS_Temps_Data.BMS_Temp5;

	RfTxBuffer[2]=BMS_Temps_Data.BMS_Temp6;
	RfTxBuffer[3]=BMS_Temps_Data.BMS_Temp7;

	RfTxBuffer[4]='t';

	RfTxBuffer[5]=0xff;
	HAL_UART_Transmit(&huart1,RfTxBuffer,6,3);
}
static void tel_send_BmsVoltage1(void)
{
	RfTxBuffer[0]=BMS_Voltages_Data.BMS_VoltIdent;
	RfTxBuffer[1]=BMS_Voltages_Data.BMS_Volt1;

	RfTxBuffer[2]=BMS_Voltages_Data.BMS_Volt2;
	RfTxBuffer[3]=BMS_Voltages_Data.BMS_Volt3;

	RfTxBuffer[4]=BMS_Voltages_Data.BMS_Volt4;
	RfTxBuffer[5]='V';

	RfTxBuffer[6]=0xff;
	HAL_UART_Transmit(&huart1,RfTxBuffer,7,3);
}
static void tel_send_BmsVoltage2(void)
{
	RfTxBuffer[0]=BMS_Voltages_Data.BMS_VoltIdent;
	RfTxBuffer[1]=BMS_Voltages_Data.BMS_Volt5;

	RfTxBuffer[2]=BMS_Voltages_Data.BMS_Volt6;
	RfTxBuffer[3]=BMS_Voltages_Data.BMS_Volt7;

	RfTxBuffer[4]='v';

	RfTxBuffer[5]=0xff;
	HAL_UART_Transmit(&huart1,RfTxBuffer,6,3);
}
/*
static void tel_send_PSU(void)
{

	RfTxBuffer[0]=(psu_Data.acc1+psu_Data.acc2)/2;
	RfTxBuffer[1]=psu_Data.brake1;
	RfTxBuffer[2]=psu_Data.brake2;
	RfTxBuffer[3]=psu_Data.error;
	RfTxBuffer[4]='p';
	[5]=0xff;
	HAL_UART_Transmit(&huart1,RfTxBuffer,6,3);


}
*/
/*
static void tel_send_CCU(void)
{
		RfTxBuffer[0]=CCU_Data.RTD;
		RfTxBuffer[1]=(uint8_t)(CCU_Data.Steer+100);
		RfTxBuffer[2]='c';
		RfTxBuffer[3]=0xff;
		HAL_UART_Transmit(&huart1,RfTxBuffer,4,3);


}
*/
/*
static void tel_send_InterConnect()
{
	RfTxBuffer[0] = interconnect_Data.car_state;
	RfTxBuffer[1] = 16000/256;
	RfTxBuffer[2] = 16000%256;
	RfTxBuffer[3] = 'i';
	RfTxBuffer[4] = 0xff;
	HAL_UART_Transmit(&huart1, RfTxBuffer, 5, 3);

}
*/ /*

static void tel_send_master_info()
{
	RfTxBuffer[0] = BBOX_Status_Data.IMD_OK;
	RfTxBuffer[1] = BBOX_Status_Data.AMS_OK;
	RfTxBuffer[2] = BBOX_Status_Data.BSPD_OK;
	RfTxBuffer[3] = BBOX_Status_Data.BSPD_IN;
	RfTxBuffer[4] = BBOX_Status_Data.RESET_OK;
	RfTxBuffer[5] = BBOX_Status_Data.SHDN;
	RfTxBuffer[6] = BBOX_Status_Data.AIR_N;
	RfTxBuffer[7] = BBOX_Status_Data.AIR_P;
	RfTxBuffer[8] = 't';
	RfTxBuffer[9] = 0xff;
	HAL_UART_Transmit(&huart1, RfTxBuffer, 10, 3);
}*/
/*
static void tel_send_BMS_Slaves(void)
{
	if (master_counter<20)
	{
	RfTxBuffer[2] = 'b';
	RfTxBuffer[3] = 0xff;
	HAL_UART_Transmit(&huart1, RfTxBuffer, 4, 2);
	}

	HAL_UART_Transmit(&huart1,"s",1,1);
	digit3_to_ascii(slave_iterator,RfTxBuffer);
	HAL_UART_Transmit(&huart1,RfTxBuffer,3,1);


	for (i=0;i<11;i++)
	{
	//digit4_to_ascii(BMS_RX_Data.Voltages[slave_iterator+i],RfTxBuffer);
	HAL_UART_Transmit(&huart1,RfTxBuffer,4,1);
	//HAL_UART_Transmit(&huart1,",",1,1);
	//digit3_to_ascii(BMS_RX_Data.Temperatures[slave_iterator+i],RfTxBuffer);
	//HAL_UART_Transmit(&huart1,RfTxBuffer,3,1);
	}
	HAL_UART_Transmit(&huart1,"$",1,1);
	slave_iterator=slave_iterator+11;

	if (slave_iterator>=BMS_SLAVE+44) slave_iterator=BMS_SLAVE;

}
*/
/*
static void tel_send_MCU(void)
{

	digit5_to_ascii_int(McuL_Data.Trq_act,RfTxBuffer);
	HAL_UART_Transmit(&huart1,RfTxBuffer,6,1);
	digit5_to_ascii_int(McuR_Data.Trq_act,RfTxBuffer);
	HAL_UART_Transmit(&huart1,RfTxBuffer,6,1);

	digit5_to_ascii_int(McuL_Data.Speed_act,RfTxBuffer);
	HAL_UART_Transmit(&huart1,RfTxBuffer,6,1);
	digit5_to_ascii_int(McuR_Data.Speed_act,RfTxBuffer);
	HAL_UART_Transmit(&huart1,RfTxBuffer,6,1);

	//int16_t value= ((float)McuL_Data.V_bus/VBUS_DIV);
	//digit5_to_ascii_int(value,RfTxBuffer);
	digit5_to_ascii_int(McuL_Data.V_bus,RfTxBuffer);
	HAL_UART_Transmit(&huart1,RfTxBuffer,6,1);

	//value= ((float)McuR_Data.V_bus/VBUS_DIV);
	//digit5_to_ascii_int(value,RfTxBuffer);
	digit5_to_ascii_int(McuR_Data.V_bus,RfTxBuffer);
	HAL_UART_Transmit(&huart1,RfTxBuffer,6,1);


	digit5_to_ascii_int(McuL_Data.Power,RfTxBuffer);
	HAL_UART_Transmit(&huart1,RfTxBuffer,6,1);

	digit5_to_ascii_int(McuR_Data.Power,RfTxBuffer);
	HAL_UART_Transmit(&huart1,RfTxBuffer,6,1);

	RfTxBuffer[0] = 'm';
	RfTxBuffer[1] = 0xff;
	HAL_UART_Transmit(&huart1, RfTxBuffer, 2, 2);

}
*/
/*
static void tel_send_MCU_slow_rate(void)
{
	RfTxBuffer[0] = McuR_Data.T_motor / 256;
	RfTxBuffer[1] = McuR_Data.T_motor % 256;
	RfTxBuffer[2] = McuL_Data.T_motor / 256;
	RfTxBuffer[3] = McuL_Data.T_motor % 256;
	RfTxBuffer[4] = McuR_Data.T_igbt / 256;
	RfTxBuffer[5] = McuR_Data.T_igbt % 256;
	RfTxBuffer[6] = McuL_Data.T_igbt / 256;
	RfTxBuffer[7] = McuL_Data.T_igbt % 256;

	RfTxBuffer[8] = 's';
	RfTxBuffer[9] = 0xff;
	HAL_UART_Transmit(&huart1, RfTxBuffer, 10, 3);

}
*/
/*
static void tel_Power_data(void)
{
	RfTxBuffer[0] = BBOX_Power_Data.voltage;
	RfTxBuffer[1] = hv_current / 256;
	RfTxBuffer[2] = hv_current % 256;



	RfTxBuffer[3] = 'z';
	RfTxBuffer[4] = 0xff;
	HAL_UART_Transmit(&huart1, RfTxBuffer, 5, 3);

}
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{

  //UartReady = SET;


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{

  //UartReady = SET;




}
/* lebo su nepouživane
static void digit1_to_ascii(uint8_t value,uint8_t *buffer)
{
	buffer[0]=(value%10)+'0';
}
static void digit2_to_ascii(uint8_t value,uint8_t *buffer)
{
	buffer[0]=((value%100)/10)+'0';
	buffer[1]=((value%100)%10)+'0';
}
static void digit3_to_ascii(uint8_t value,uint8_t *buffer)
{

	buffer[0]=(value/100)+'0';
	buffer[1]=((value%100)/10)+'0';
	buffer[2]=(value%10)+'0';
}
static void digit3_to_ascii_16bit(uint16_t value,uint8_t *buffer)
{
	buffer[0]=(value/100)+'0';
		buffer[1]=((value%100)/10)+'0';
		buffer[2]=(value%10)+'0';
}
static void digit3_to_ascii_int(int8_t value,uint8_t *buffer)
{
	buffer[0]=(value>=0)?'+':'-';
	buffer[1]=(value/100)+'0';
	buffer[2]=((value%100)/10)+'0';
	buffer[3]=(value%10)+'0';
}
static void digit4_to_ascii(uint16_t value,uint8_t *buffer)
{
	buffer[0]=(value/1000)+'0';
	buffer[1]=((value%1000)/100)+'0';
	buffer[2]=((value%100)/10)+'0';
	buffer[3]=(value%10)+'0';
}
static void digit5_to_ascii(uint16_t value,uint8_t *buffer)
{
	buffer[0]=(value/10000)+'0';
	buffer[1]=((value%10000)/1000)+'0';
	buffer[2]=((value%1000)/100)+'0';
	buffer[3]=((value%100)/10)+'0';
	buffer[4]=(value%10)+'0';
}

static void digit5_to_ascii_int(int16_t value,uint8_t *buffer)
{
	buffer[0]=(value>=0)?'+':'-';
	value=(value>=0)?value:-value;
	buffer[1]=(value/10000)+'0';
	buffer[2]=((value%10000)/1000)+'0';
	buffer[3]=((value%1000)/100)+'0';
	buffer[4]=((value%100)/10)+'0';
	buffer[5]=(value%10)+'0';
}

static void digit5_to_ascii_int32(int32_t value,uint8_t *buffer)
{
	buffer[0]=(value>=0)?'+':'-';
	value=(value>=0)?value:-value;
	buffer[1]=(value/10000)+'0';
	buffer[2]=((value%10000)/1000)+'0';
	buffer[3]=((value%1000)/100)+'0';
	buffer[4]=((value%100)/10)+'0';
	buffer[5]=(value%10)+'0';
}
*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
