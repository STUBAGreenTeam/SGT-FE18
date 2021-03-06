
/**
  ******************************************************************************
  * @file    CAN2018.c
  * @author  SGT Generated by tool
  * @version V0.1.0 (generator)
  * @date    30-July-2018
  * @brief   CAN protocol application/PROFILE layer for use in SGT formula student electric 2018
  ******************************************************************************
  */
#include "stm32f1xx_hal.h"
#include "CAN2018.h"
CanTxMsgTypeDef CanTxMsg;
CanRxMsgTypeDef CanRxMsg;


#if defined(Tx_BBOX_power) || defined(Rx_BBOX_power)
    BBOX_power_TypeDef BBOX_power_Data = {
        .power = 0,
        .current = 0,
        .voltage = 0
     };
#endif
    
#ifdef Tx_BBOX_power
void Tx_BBOX_power_Data(CAN_HandleTypeDef* hcan, BBOX_power_TypeDef* BBOX_power_Data)
{
    hcan->pTxMsg = &CanTxMsg;
    hcan->pTxMsg->StdId = ID_BBOX_power;
    hcan->pTxMsg->DLC = DLC_BBOX_power;
    hcan->pTxMsg->ExtId = 0x0;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->IDE = CAN_ID_STD;
        
    hcan->pTxMsg->Data[0] = (uint8_t)((BBOX_power_Data->power >> 8)&0b11111111);
    hcan->pTxMsg->Data[1] = (uint8_t)((BBOX_power_Data->power)&0b1111111111111111);
    hcan->pTxMsg->Data[2] = (uint8_t)((BBOX_power_Data->current >> 8)&0b11111111);
    hcan->pTxMsg->Data[3] = (uint8_t)((BBOX_power_Data->current)&0b1111111111111111);
    hcan->pTxMsg->Data[4] = (uint8_t)((BBOX_power_Data->voltage >> 8)&0b11111111);
    hcan->pTxMsg->Data[5] = (uint8_t)((BBOX_power_Data->voltage)&0b1111111111111111);
                        
    HAL_CAN_Transmit_IT(hcan);
    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}
#endif
#ifdef Rx_BBOX_power
void Rx_BBOX_power_Data(CAN_HandleTypeDef* hcan, BBOX_power_TypeDef* BBOX_power_Data)
{
    BBOX_power_Data->power = (int16_t)(((hcan->pRxMsg->Data[0]&0b11111111) << 8) | (hcan->pRxMsg->Data[1]&0b1111111111111111));
    BBOX_power_Data->current = (int16_t)(((hcan->pRxMsg->Data[2]&0b11111111) << 8) | (hcan->pRxMsg->Data[3]&0b1111111111111111));
    BBOX_power_Data->voltage = (uint16_t)(((hcan->pRxMsg->Data[4]&0b11111111) << 8) | (hcan->pRxMsg->Data[5]&0b1111111111111111));
}
#endif


#if defined(Tx_wheel_RPM) || defined(Rx_wheel_RPM)
    wheel_RPM_TypeDef wheel_RPM_Data = {
        .front_right = 0,
        .front_left = 0,
        .rear_left = 0,
        .rear_right = 0
     };
#endif
    
#ifdef Tx_wheel_RPM
void Tx_wheel_RPM_Data(CAN_HandleTypeDef* hcan, wheel_RPM_TypeDef* wheel_RPM_Data)
{
    hcan->pTxMsg = &CanTxMsg;
    hcan->pTxMsg->StdId = ID_wheel_RPM;
    hcan->pTxMsg->DLC = DLC_wheel_RPM;
    hcan->pTxMsg->ExtId = 0x0;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->IDE = CAN_ID_STD;
        
    hcan->pTxMsg->Data[0] = (uint8_t)((wheel_RPM_Data->front_right >> 8)&0b11111111);
    hcan->pTxMsg->Data[1] = (uint8_t)((wheel_RPM_Data->front_right)&0b1111111111111111);
    hcan->pTxMsg->Data[2] = (uint8_t)((wheel_RPM_Data->front_left >> 8)&0b11111111);
    hcan->pTxMsg->Data[3] = (uint8_t)((wheel_RPM_Data->front_left)&0b1111111111111111);
    hcan->pTxMsg->Data[4] = (uint8_t)((wheel_RPM_Data->rear_left >> 8)&0b11111111);
    hcan->pTxMsg->Data[5] = (uint8_t)((wheel_RPM_Data->rear_left)&0b1111111111111111);
    hcan->pTxMsg->Data[6] = (uint8_t)((wheel_RPM_Data->rear_right >> 8)&0b11111111);
    hcan->pTxMsg->Data[7] = (uint8_t)((wheel_RPM_Data->rear_right)&0b1111111111111111);
                        
    HAL_CAN_Transmit_IT(hcan);
    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}
#endif
#ifdef Rx_wheel_RPM
void Rx_wheel_RPM_Data(CAN_HandleTypeDef* hcan, wheel_RPM_TypeDef* wheel_RPM_Data)
{
    wheel_RPM_Data->front_right = (uint16_t)(((hcan->pRxMsg->Data[0]&0b11111111) << 8) | (hcan->pRxMsg->Data[1]&0b1111111111111111));
    wheel_RPM_Data->front_left = (uint16_t)(((hcan->pRxMsg->Data[2]&0b11111111) << 8) | (hcan->pRxMsg->Data[3]&0b1111111111111111));
    wheel_RPM_Data->rear_left = (uint16_t)(((hcan->pRxMsg->Data[4]&0b11111111) << 8) | (hcan->pRxMsg->Data[5]&0b1111111111111111));
    wheel_RPM_Data->rear_right = (uint16_t)(((hcan->pRxMsg->Data[6]&0b11111111) << 8) | (hcan->pRxMsg->Data[7]&0b1111111111111111));
}
#endif


#if defined(Tx_BBOX_status) || defined(Rx_BBOX_status)
    BBOX_status_TypeDef BBOX_status_Data = {
        .TSMS = 0,
        .SHD_IN = 0,
        .SHD_OUT = 0,
        .AIR_N = 0,
        .AIR_P = 0,
        .PRECH_60V = 0,
        .IMD_OK = 0,
        .BMS_OK = 0,
        .SIGNAL_ERROR = 0,
        .SHD_RESET = 0,
        .SHD_EN = 0,
        .POLARITY = 0,
        .FANS = 0,
        .STM_temp = 0
     };
#endif
    
#ifdef Tx_BBOX_status
void Tx_BBOX_status_Data(CAN_HandleTypeDef* hcan, BBOX_status_TypeDef* BBOX_status_Data)
{
    hcan->pTxMsg = &CanTxMsg;
    hcan->pTxMsg->StdId = ID_BBOX_status;
    hcan->pTxMsg->DLC = DLC_BBOX_status;
    hcan->pTxMsg->ExtId = 0x0;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->IDE = CAN_ID_STD;
        
    hcan->pTxMsg->Data[0] = (uint8_t)((BBOX_status_Data->TSMS << 7)&0b11111111);
    hcan->pTxMsg->Data[0] |= (uint8_t)((BBOX_status_Data->SHD_IN << 6)&0b1111111);
    hcan->pTxMsg->Data[0] |= (uint8_t)((BBOX_status_Data->SHD_OUT << 5)&0b111111);
    hcan->pTxMsg->Data[0] |= (uint8_t)((BBOX_status_Data->AIR_N << 4)&0b11111);
    hcan->pTxMsg->Data[0] |= (uint8_t)((BBOX_status_Data->AIR_P << 3)&0b1111);
    hcan->pTxMsg->Data[0] |= (uint8_t)((BBOX_status_Data->PRECH_60V << 2)&0b111);
    hcan->pTxMsg->Data[0] |= (uint8_t)((BBOX_status_Data->IMD_OK << 1)&0b11);
    hcan->pTxMsg->Data[0] |= (uint8_t)((BBOX_status_Data->BMS_OK)&0b1);
    hcan->pTxMsg->Data[1] = (uint8_t)((BBOX_status_Data->SIGNAL_ERROR << 7)&0b11111111);
    hcan->pTxMsg->Data[1] |= (uint8_t)((BBOX_status_Data->SHD_RESET << 6)&0b1111111);
    hcan->pTxMsg->Data[1] |= (uint8_t)((BBOX_status_Data->SHD_EN << 5)&0b111111);
    hcan->pTxMsg->Data[1] |= (uint8_t)((BBOX_status_Data->POLARITY << 4)&0b11111);
    hcan->pTxMsg->Data[1] |= (uint8_t)((BBOX_status_Data->FANS << 3)&0b1111);
    hcan->pTxMsg->Data[1] |= (uint8_t)((BBOX_status_Data->STM_temp >> 5)&0b111);
    hcan->pTxMsg->Data[2] = (uint8_t)((BBOX_status_Data->STM_temp << 3)&0b11111111111);
                        
    HAL_CAN_Transmit_IT(hcan);
    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}
#endif
#ifdef Rx_BBOX_status
void Rx_BBOX_status_Data(CAN_HandleTypeDef* hcan, BBOX_status_TypeDef* BBOX_status_Data)
{
    BBOX_status_Data->TSMS = (uint8_t)(((hcan->pRxMsg->Data[0]&0b11111111) >> 7));
    BBOX_status_Data->SHD_IN = (uint8_t)(((hcan->pRxMsg->Data[0]&0b1111111) >> 6));
    BBOX_status_Data->SHD_OUT = (uint8_t)(((hcan->pRxMsg->Data[0]&0b111111) >> 5));
    BBOX_status_Data->AIR_N = (uint8_t)(((hcan->pRxMsg->Data[0]&0b11111) >> 4));
    BBOX_status_Data->AIR_P = (uint8_t)(((hcan->pRxMsg->Data[0]&0b1111) >> 3));
    BBOX_status_Data->PRECH_60V = (uint8_t)(((hcan->pRxMsg->Data[0]&0b111) >> 2));
    BBOX_status_Data->IMD_OK = (uint8_t)(((hcan->pRxMsg->Data[0]&0b11) >> 1));
    BBOX_status_Data->BMS_OK = (uint8_t)((hcan->pRxMsg->Data[0]&0b1));
    BBOX_status_Data->SIGNAL_ERROR = (uint8_t)(((hcan->pRxMsg->Data[1]&0b11111111) >> 7));
    BBOX_status_Data->SHD_RESET = (uint8_t)(((hcan->pRxMsg->Data[1]&0b1111111) >> 6));
    BBOX_status_Data->SHD_EN = (uint8_t)(((hcan->pRxMsg->Data[1]&0b111111) >> 5));
    BBOX_status_Data->POLARITY = (uint8_t)(((hcan->pRxMsg->Data[1]&0b11111) >> 4));
    BBOX_status_Data->FANS = (uint8_t)(((hcan->pRxMsg->Data[1]&0b1111) >> 3));
    BBOX_status_Data->STM_temp = (int8_t)(((hcan->pRxMsg->Data[1]&0b111) << 5) | ((hcan->pRxMsg->Data[2]&0b11111111111) >> 3));
}
#endif


#if defined(Tx_FU_Values_1) || defined(Rx_FU_Values_1)
    FU_Values_1_TypeDef FU_Values_1_Data = {
        .apps1 = 0,
        .apps2 = 0,
        .brake1 = 0,
        .brake2 = 0,
        .error = 0
     };
#endif
    
#ifdef Tx_FU_Values_1
void Tx_FU_Values_1_Data(CAN_HandleTypeDef* hcan, FU_Values_1_TypeDef* FU_Values_1_Data)
{
    hcan->pTxMsg = &CanTxMsg;
    hcan->pTxMsg->StdId = ID_FU_Values_1;
    hcan->pTxMsg->DLC = DLC_FU_Values_1;
    hcan->pTxMsg->ExtId = 0x0;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->IDE = CAN_ID_STD;
        
    hcan->pTxMsg->Data[0] = (uint8_t)((FU_Values_1_Data->apps1)&0b11111111);
    hcan->pTxMsg->Data[1] = (uint8_t)((FU_Values_1_Data->apps2)&0b11111111);
    hcan->pTxMsg->Data[2] = (uint8_t)((FU_Values_1_Data->brake1)&0b11111111);
    hcan->pTxMsg->Data[3] = (uint8_t)((FU_Values_1_Data->brake2)&0b11111111);
    hcan->pTxMsg->Data[4] = (uint8_t)((FU_Values_1_Data->error >> 8)&0b11111111);
    hcan->pTxMsg->Data[5] = (uint8_t)((FU_Values_1_Data->error)&0b1111111111111111);
                        
    HAL_CAN_Transmit_IT(hcan);
    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}
#endif
#ifdef Rx_FU_Values_1
void Rx_FU_Values_1_Data(CAN_HandleTypeDef* hcan, FU_Values_1_TypeDef* FU_Values_1_Data)
{
    FU_Values_1_Data->apps1 = (uint8_t)((hcan->pRxMsg->Data[0]&0b11111111));
    FU_Values_1_Data->apps2 = (uint8_t)((hcan->pRxMsg->Data[1]&0b11111111));
    FU_Values_1_Data->brake1 = (uint8_t)((hcan->pRxMsg->Data[2]&0b11111111));
    FU_Values_1_Data->brake2 = (uint8_t)((hcan->pRxMsg->Data[3]&0b11111111));
    FU_Values_1_Data->error = (uint16_t)(((hcan->pRxMsg->Data[4]&0b11111111) << 8) | (hcan->pRxMsg->Data[5]&0b1111111111111111));
}
#endif


#if defined(Tx_BBOX_command) || defined(Rx_BBOX_command)
    BBOX_command_TypeDef BBOX_command_Data = {
        .FANS = 0,
        .SHD_EN = 0
     };
#endif
    
#ifdef Tx_BBOX_command
void Tx_BBOX_command_Data(CAN_HandleTypeDef* hcan, BBOX_command_TypeDef* BBOX_command_Data)
{
    hcan->pTxMsg = &CanTxMsg;
    hcan->pTxMsg->StdId = ID_BBOX_command;
    hcan->pTxMsg->DLC = DLC_BBOX_command;
    hcan->pTxMsg->ExtId = 0x0;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->IDE = CAN_ID_STD;
        
    hcan->pTxMsg->Data[0] = (uint8_t)((BBOX_command_Data->FANS << 7)&0b11111111);
    hcan->pTxMsg->Data[0] |= (uint8_t)((BBOX_command_Data->SHD_EN << 6)&0b1111111);
                        
    HAL_CAN_Transmit_IT(hcan);
    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}
#endif
#ifdef Rx_BBOX_command
void Rx_BBOX_command_Data(CAN_HandleTypeDef* hcan, BBOX_command_TypeDef* BBOX_command_Data)
{
    BBOX_command_Data->FANS = (uint8_t)(((hcan->pRxMsg->Data[0]&0b11111111) >> 7));
    BBOX_command_Data->SHD_EN = (uint8_t)(((hcan->pRxMsg->Data[0]&0b1111111) >> 6));
}
#endif


#if defined(Tx_BMS_Command) || defined(Rx_BMS_Command)
    BMS_Command_TypeDef BMS_Command_Data = {
        .BMS_Balanc = 0,
        .BMS_FullMode = 0,
        .BMS_OK = 0,
        .BMS_ONOFF = 0,
        .BMS_CAN = 0
     };
#endif
    
#ifdef Tx_BMS_Command
void Tx_BMS_Command_Data(CAN_HandleTypeDef* hcan, BMS_Command_TypeDef* BMS_Command_Data)
{
    hcan->pTxMsg = &CanTxMsg;
    hcan->pTxMsg->StdId = ID_BMS_Command;
    hcan->pTxMsg->DLC = DLC_BMS_Command;
    hcan->pTxMsg->ExtId = 0x0;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->IDE = CAN_ID_STD;
        
    hcan->pTxMsg->Data[0] = (uint8_t)((BMS_Command_Data->BMS_Balanc << 6)&0b11111111);
    hcan->pTxMsg->Data[0] |= (uint8_t)((BMS_Command_Data->BMS_FullMode << 5)&0b111111);
    hcan->pTxMsg->Data[0] |= (uint8_t)((BMS_Command_Data->BMS_OK << 3)&0b11111);
    hcan->pTxMsg->Data[0] |= (uint8_t)((BMS_Command_Data->BMS_ONOFF << 1)&0b111);
    hcan->pTxMsg->Data[0] |= (uint8_t)((BMS_Command_Data->BMS_CAN >> 1)&0b1);
    hcan->pTxMsg->Data[1] = (uint8_t)((BMS_Command_Data->BMS_CAN << 7)&0b111111111);
                        
    HAL_CAN_Transmit_IT(hcan);
    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}
#endif
#ifdef Rx_BMS_Command
void Rx_BMS_Command_Data(CAN_HandleTypeDef* hcan, BMS_Command_TypeDef* BMS_Command_Data)
{
    BMS_Command_Data->BMS_Balanc = (uint8_t)(((hcan->pRxMsg->Data[0]&0b11111111) >> 6));
    BMS_Command_Data->BMS_FullMode = (uint8_t)(((hcan->pRxMsg->Data[0]&0b111111) >> 5));
    BMS_Command_Data->BMS_OK = (uint8_t)(((hcan->pRxMsg->Data[0]&0b11111) >> 3));
    BMS_Command_Data->BMS_ONOFF = (uint8_t)(((hcan->pRxMsg->Data[0]&0b111) >> 1));
    BMS_Command_Data->BMS_CAN = (uint8_t)(((hcan->pRxMsg->Data[0]&0b1) << 1) | ((hcan->pRxMsg->Data[1]&0b111111111) >> 7));
}
#endif


#if defined(Tx_BMS_State) || defined(Rx_BMS_State)
    BMS_State_TypeDef BMS_State_Data = {
        .BMS_Mode = 0,
        .BMS_Faults = 0,
        .CellVolt_L = 0,
        .CellVolt_H = 0,
        .CellTemp_L = 0,
        .CellTemp_H = 0,
        .BMS_Ident = 0
     };
#endif
    
#ifdef Tx_BMS_State
void Tx_BMS_State_Data(CAN_HandleTypeDef* hcan, BMS_State_TypeDef* BMS_State_Data)
{
    hcan->pTxMsg = &CanTxMsg;
    hcan->pTxMsg->StdId = ID_BMS_State;
    hcan->pTxMsg->DLC = DLC_BMS_State;
    hcan->pTxMsg->ExtId = 0x0;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->IDE = CAN_ID_STD;
        
    hcan->pTxMsg->Data[0] = (uint8_t)((BMS_State_Data->BMS_Mode)&0b11111111);
    hcan->pTxMsg->Data[1] = (uint8_t)((BMS_State_Data->BMS_Faults >> 8)&0b11111111);
    hcan->pTxMsg->Data[2] = (uint8_t)((BMS_State_Data->BMS_Faults)&0b1111111111111111);
    hcan->pTxMsg->Data[3] = (uint8_t)((BMS_State_Data->CellVolt_L)&0b11111111);
    hcan->pTxMsg->Data[4] = (uint8_t)((BMS_State_Data->CellVolt_H)&0b11111111);
    hcan->pTxMsg->Data[5] = (uint8_t)((BMS_State_Data->CellTemp_L)&0b11111111);
    hcan->pTxMsg->Data[6] = (uint8_t)((BMS_State_Data->CellTemp_H)&0b11111111);
    hcan->pTxMsg->Data[7] = (uint8_t)((BMS_State_Data->BMS_Ident)&0b11111111);
                        
    HAL_CAN_Transmit_IT(hcan);
    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}
#endif
#ifdef Rx_BMS_State
void Rx_BMS_State_Data(CAN_HandleTypeDef* hcan, BMS_State_TypeDef* BMS_State_Data)
{
    BMS_State_Data->BMS_Mode = (uint8_t)((hcan->pRxMsg->Data[0]&0b11111111));
    BMS_State_Data->BMS_Faults = (uint16_t)(((hcan->pRxMsg->Data[1]&0b11111111) << 8) | (hcan->pRxMsg->Data[2]&0b1111111111111111));
    BMS_State_Data->CellVolt_L = (uint8_t)((hcan->pRxMsg->Data[3]&0b11111111));
    BMS_State_Data->CellVolt_H = (uint8_t)((hcan->pRxMsg->Data[4]&0b11111111));
    BMS_State_Data->CellTemp_L = (uint8_t)((hcan->pRxMsg->Data[5]&0b11111111));
    BMS_State_Data->CellTemp_H = (uint8_t)((hcan->pRxMsg->Data[6]&0b11111111));
    BMS_State_Data->BMS_Ident = (uint8_t)((hcan->pRxMsg->Data[7]&0b11111111));
}
#endif


#if defined(Tx_ECU_State) || defined(Rx_ECU_State)
    ECU_State_TypeDef ECU_State_Data = {
        .ECU_Status = 0,
        .FL_AMK_Status = 0,
        .FR_AMK_Status = 0,
        .RL_AMK_Status = 0,
        .RR_AMK_Status = 0,
        .TempMotor_H = 0,
        .TempInverter_H = 0,
        .TempIGBT_H = 0
     };
#endif
    
#ifdef Tx_ECU_State
void Tx_ECU_State_Data(CAN_HandleTypeDef* hcan, ECU_State_TypeDef* ECU_State_Data)
{
    hcan->pTxMsg = &CanTxMsg;
    hcan->pTxMsg->StdId = ID_ECU_State;
    hcan->pTxMsg->DLC = DLC_ECU_State;
    hcan->pTxMsg->ExtId = 0x0;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->IDE = CAN_ID_STD;
        
    hcan->pTxMsg->Data[0] = (uint8_t)((ECU_State_Data->ECU_Status)&0b11111111);
    hcan->pTxMsg->Data[1] = (uint8_t)((ECU_State_Data->FL_AMK_Status)&0b11111111);
    hcan->pTxMsg->Data[2] = (uint8_t)((ECU_State_Data->FR_AMK_Status)&0b11111111);
    hcan->pTxMsg->Data[3] = (uint8_t)((ECU_State_Data->RL_AMK_Status)&0b11111111);
    hcan->pTxMsg->Data[4] = (uint8_t)((ECU_State_Data->RR_AMK_Status)&0b11111111);
    hcan->pTxMsg->Data[5] = (uint8_t)((ECU_State_Data->TempMotor_H)&0b11111111);
    hcan->pTxMsg->Data[6] = (uint8_t)((ECU_State_Data->TempInverter_H)&0b11111111);
    hcan->pTxMsg->Data[7] = (uint8_t)((ECU_State_Data->TempIGBT_H)&0b11111111);
                        
    HAL_CAN_Transmit_IT(hcan);
    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}
#endif
#ifdef Rx_ECU_State
void Rx_ECU_State_Data(CAN_HandleTypeDef* hcan, ECU_State_TypeDef* ECU_State_Data)
{
    ECU_State_Data->ECU_Status = (uint8_t)((hcan->pRxMsg->Data[0]&0b11111111));
    ECU_State_Data->FL_AMK_Status = (uint8_t)((hcan->pRxMsg->Data[1]&0b11111111));
    ECU_State_Data->FR_AMK_Status = (uint8_t)((hcan->pRxMsg->Data[2]&0b11111111));
    ECU_State_Data->RL_AMK_Status = (uint8_t)((hcan->pRxMsg->Data[3]&0b11111111));
    ECU_State_Data->RR_AMK_Status = (uint8_t)((hcan->pRxMsg->Data[4]&0b11111111));
    ECU_State_Data->TempMotor_H = (uint8_t)((hcan->pRxMsg->Data[5]&0b11111111));
    ECU_State_Data->TempInverter_H = (uint8_t)((hcan->pRxMsg->Data[6]&0b11111111));
    ECU_State_Data->TempIGBT_H = (uint8_t)((hcan->pRxMsg->Data[7]&0b11111111));
}
#endif


#if defined(Tx_FU_Values_2) || defined(Rx_FU_Values_2)
    FU_Values_2_TypeDef FU_Values_2_Data = {
        .steer = 0,
        .susp_FL = 0,
        .susp_FR = 0,
        .brake_pos = 0,
        .RTD = 0,
        .BOTS = 0,
        .SHDB = 0,
        .INERTIA_SW = 0,
        .reserve = 0
     };
#endif
    
#ifdef Tx_FU_Values_2
void Tx_FU_Values_2_Data(CAN_HandleTypeDef* hcan, FU_Values_2_TypeDef* FU_Values_2_Data)
{
    hcan->pTxMsg = &CanTxMsg;
    hcan->pTxMsg->StdId = ID_FU_Values_2;
    hcan->pTxMsg->DLC = DLC_FU_Values_2;
    hcan->pTxMsg->ExtId = 0x0;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->IDE = CAN_ID_STD;
        
    hcan->pTxMsg->Data[0] = (uint8_t)((FU_Values_2_Data->steer)&0b11111111);
    hcan->pTxMsg->Data[1] = (uint8_t)((FU_Values_2_Data->susp_FL >> 4)&0b11111111);
    hcan->pTxMsg->Data[2] = (uint8_t)((FU_Values_2_Data->susp_FL << 4)&0b1111111111111111);
    hcan->pTxMsg->Data[2] |= (uint8_t)((FU_Values_2_Data->susp_FR >> 8)&0b1111);
    hcan->pTxMsg->Data[3] = (uint8_t)((FU_Values_2_Data->susp_FR)&0b111111111111);
    hcan->pTxMsg->Data[4] = (uint8_t)((FU_Values_2_Data->brake_pos)&0b11111111);
    hcan->pTxMsg->Data[5] = (uint8_t)((FU_Values_2_Data->RTD << 7)&0b11111111);
    hcan->pTxMsg->Data[5] |= (uint8_t)((FU_Values_2_Data->BOTS << 6)&0b1111111);
    hcan->pTxMsg->Data[5] |= (uint8_t)((FU_Values_2_Data->SHDB << 5)&0b111111);
    hcan->pTxMsg->Data[5] |= (uint8_t)((FU_Values_2_Data->INERTIA_SW << 4)&0b11111);
    hcan->pTxMsg->Data[5] |= (uint8_t)((FU_Values_2_Data->reserve >> 4)&0b1111);
    hcan->pTxMsg->Data[6] = (uint8_t)((FU_Values_2_Data->reserve << 4)&0b111111111111);
                        
    HAL_CAN_Transmit_IT(hcan);
    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}
#endif
#ifdef Rx_FU_Values_2
void Rx_FU_Values_2_Data(CAN_HandleTypeDef* hcan, FU_Values_2_TypeDef* FU_Values_2_Data)
{
    FU_Values_2_Data->steer = (int8_t)((hcan->pRxMsg->Data[0]&0b11111111));
    FU_Values_2_Data->susp_FL = (uint16_t)(((hcan->pRxMsg->Data[1]&0b11111111) << 4) | ((hcan->pRxMsg->Data[2]&0b1111111111111111) >> 4));
    FU_Values_2_Data->susp_FR = (uint16_t)(((hcan->pRxMsg->Data[2]&0b1111) << 8) | (hcan->pRxMsg->Data[3]&0b111111111111));
    FU_Values_2_Data->brake_pos = (uint8_t)((hcan->pRxMsg->Data[4]&0b11111111));
    FU_Values_2_Data->RTD = (uint8_t)(((hcan->pRxMsg->Data[5]&0b11111111) >> 7));
    FU_Values_2_Data->BOTS = (uint8_t)(((hcan->pRxMsg->Data[5]&0b1111111) >> 6));
    FU_Values_2_Data->SHDB = (uint8_t)(((hcan->pRxMsg->Data[5]&0b111111) >> 5));
    FU_Values_2_Data->INERTIA_SW = (uint8_t)(((hcan->pRxMsg->Data[5]&0b11111) >> 4));
    FU_Values_2_Data->reserve = (uint8_t)(((hcan->pRxMsg->Data[5]&0b1111) << 4) | ((hcan->pRxMsg->Data[6]&0b111111111111) >> 4));
}
#endif


#if defined(Tx_Interconnect) || defined(Rx_Interconnect)
    Interconnect_TypeDef Interconnect_Data = {
        .car_state = 0,
        .left_w_pump = 0,
        .right_w_pump = 0,
        .brake_red = 0,
        .brake_white = 0,
        .tsas = 0,
        .killswitch_R = 0,
        .killswitch_L = 0,
        .reserve = 0,
        .susp_RR = 0,
        .susp_RL = 0
     };
#endif
    
#ifdef Tx_Interconnect
void Tx_Interconnect_Data(CAN_HandleTypeDef* hcan, Interconnect_TypeDef* Interconnect_Data)
{
    hcan->pTxMsg = &CanTxMsg;
    hcan->pTxMsg->StdId = ID_Interconnect;
    hcan->pTxMsg->DLC = DLC_Interconnect;
    hcan->pTxMsg->ExtId = 0x0;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->IDE = CAN_ID_STD;
        
    hcan->pTxMsg->Data[0] = (uint8_t)((Interconnect_Data->car_state)&0b11111111);
    hcan->pTxMsg->Data[1] = (uint8_t)((Interconnect_Data->left_w_pump << 7)&0b11111111);
    hcan->pTxMsg->Data[1] |= (uint8_t)((Interconnect_Data->right_w_pump << 6)&0b1111111);
    hcan->pTxMsg->Data[1] |= (uint8_t)((Interconnect_Data->brake_red << 5)&0b111111);
    hcan->pTxMsg->Data[1] |= (uint8_t)((Interconnect_Data->brake_white << 4)&0b11111);
    hcan->pTxMsg->Data[1] |= (uint8_t)((Interconnect_Data->tsas << 3)&0b1111);
    hcan->pTxMsg->Data[1] |= (uint8_t)((Interconnect_Data->killswitch_R << 2)&0b111);
    hcan->pTxMsg->Data[1] |= (uint8_t)((Interconnect_Data->killswitch_L << 1)&0b11);
    hcan->pTxMsg->Data[1] |= (uint8_t)((Interconnect_Data->reserve >> 7)&0b1);
    hcan->pTxMsg->Data[2] = (uint8_t)((Interconnect_Data->reserve << 1)&0b111111111);
    hcan->pTxMsg->Data[2] |= (uint8_t)((Interconnect_Data->susp_RR >> 11)&0b1);
    hcan->pTxMsg->Data[3] = (uint8_t)((Interconnect_Data->susp_RR >> 3)&0b111111111);
    hcan->pTxMsg->Data[4] = (uint8_t)((Interconnect_Data->susp_RR << 5)&0b11111111111111111);
    hcan->pTxMsg->Data[4] |= (uint8_t)((Interconnect_Data->susp_RL >> 7)&0b11111);
    hcan->pTxMsg->Data[5] = (uint8_t)((Interconnect_Data->susp_RL << 1)&0b1111111111111);
                        
    HAL_CAN_Transmit_IT(hcan);
    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}
#endif
#ifdef Rx_Interconnect
void Rx_Interconnect_Data(CAN_HandleTypeDef* hcan, Interconnect_TypeDef* Interconnect_Data)
{
    Interconnect_Data->car_state = (uint8_t)((hcan->pRxMsg->Data[0]&0b11111111));
    Interconnect_Data->left_w_pump = (uint8_t)(((hcan->pRxMsg->Data[1]&0b11111111) >> 7));
    Interconnect_Data->right_w_pump = (uint8_t)(((hcan->pRxMsg->Data[1]&0b1111111) >> 6));
    Interconnect_Data->brake_red = (uint8_t)(((hcan->pRxMsg->Data[1]&0b111111) >> 5));
    Interconnect_Data->brake_white = (uint8_t)(((hcan->pRxMsg->Data[1]&0b11111) >> 4));
    Interconnect_Data->tsas = (uint8_t)(((hcan->pRxMsg->Data[1]&0b1111) >> 3));
    Interconnect_Data->killswitch_R = (uint8_t)(((hcan->pRxMsg->Data[1]&0b111) >> 2));
    Interconnect_Data->killswitch_L = (uint8_t)(((hcan->pRxMsg->Data[1]&0b11) >> 1));
    Interconnect_Data->reserve = (uint8_t)(((hcan->pRxMsg->Data[1]&0b1) << 7) | ((hcan->pRxMsg->Data[2]&0b111111111) >> 1));
    Interconnect_Data->susp_RR = (uint16_t)(((hcan->pRxMsg->Data[2]&0b1) << 11) | ((hcan->pRxMsg->Data[3]&0b111111111) << 3) | ((hcan->pRxMsg->Data[4]&0b11111111111111111) >> 5));
    Interconnect_Data->susp_RL = (uint16_t)(((hcan->pRxMsg->Data[4]&0b11111) << 7) | ((hcan->pRxMsg->Data[5]&0b1111111111111) >> 1));
}
#endif


/** \brief Zmaze CAN error ak je
 *
 * \param hcan CAN_HandleTypeDef*
 * \return void
 *
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    hcan->ErrorCode = HAL_CAN_ERROR_NONE;
    hcan->Instance->MSR &= 0x1C;
}
/**
* @brief Event for CAN Rx message
* @param Can controller message structure
*/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
    switch (hcan->pRxMsg->StdId)
    {

#ifdef Rx_BBOX_power
        case ID_BBOX_power:
            Rx_BBOX_power_Data(hcan, &BBOX_power_Data);
            break;
#endif

#ifdef Rx_wheel_RPM
        case ID_wheel_RPM:
            Rx_wheel_RPM_Data(hcan, &wheel_RPM_Data);
            break;
#endif

#ifdef Rx_BBOX_status
        case ID_BBOX_status:
            Rx_BBOX_status_Data(hcan, &BBOX_status_Data);
            break;
#endif

#ifdef Rx_FU_Values_1
        case ID_FU_Values_1:
            Rx_FU_Values_1_Data(hcan, &FU_Values_1_Data);
            break;
#endif

#ifdef Rx_BBOX_command
        case ID_BBOX_command:
            Rx_BBOX_command_Data(hcan, &BBOX_command_Data);
            break;
#endif

#ifdef Rx_BMS_Command
        case ID_BMS_Command:
            Rx_BMS_Command_Data(hcan, &BMS_Command_Data);
            break;
#endif

#ifdef Rx_BMS_State
        case ID_BMS_State:
            Rx_BMS_State_Data(hcan, &BMS_State_Data);
            break;
#endif

#ifdef Rx_ECU_State
        case ID_ECU_State:
            Rx_ECU_State_Data(hcan, &ECU_State_Data);
            break;
#endif

#ifdef Rx_FU_Values_2
        case ID_FU_Values_2:
            Rx_FU_Values_2_Data(hcan, &FU_Values_2_Data);
            break;
#endif

#ifdef Rx_Interconnect
        case ID_Interconnect:
            Rx_Interconnect_Data(hcan, &Interconnect_Data);
            break;
#endif

        default:
            break;
    }
    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_6);
}
