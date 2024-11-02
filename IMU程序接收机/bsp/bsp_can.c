///**
//	* @file bsp_can.c
//	* @version 2.0
//	* @date 2020.1.3
//  *
//  * @brief  can驱动
//  *
//  *	@author YY
//  *
//  */

//#include "bsp_can.h"

//uint8_t palstance_can_tx_data[8];
//uint8_t angle_can_tx_data[8];
//float pitch_angle[2];
//CAN_TxHeaderTypeDef TxMessage;


///**
//  * @brief   can filter initialization
//  * @param   CAN_HandleTypeDef
//  * @retval  None
//  */
//void can_filter_init(void)
//{
//  //can filter config
//  CAN_FilterTypeDef  can_filter;

//	can_filter.FilterBank           = 0;
//  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
//  can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
//  can_filter.FilterIdHigh         = 0x0000;
//  can_filter.FilterIdLow          = 0x0000;
//  can_filter.FilterMaskIdHigh     = 0x0000;
//  can_filter.FilterMaskIdLow      = 0x0000;
//  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//	can_filter.SlaveStartFilterBank = 0;  
//  can_filter.FilterActivation     = ENABLE;

//  HAL_CAN_ConfigFilter(&hcan, &can_filter);
//  while (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK);
//}


///**
//  * @brief  init the can transmit and receive
//  * @param  None
//  */
//void can_device_init(void)
//{
//	can_filter_init();
//	HAL_Delay(100);
//	HAL_CAN_Start(&hcan);
//}


///**
//  * @brief  send palstance message
//  * @param  
//  */
//void send_palstance_message(void)
//{
//	uint8_t FreeTxNum = 0;  

// // TxMessage.StdId   = 0x600;
//	 TxMessage.StdId   = 0x001;
//  TxMessage.IDE     = CAN_ID_STD;
//  TxMessage.RTR     = CAN_RTR_DATA;
//  TxMessage.DLC     = 0x08;
//	
////	palstance_can_tx_data[0] = palstance_tx_data[4];	//pit
////	palstance_can_tx_data[1] = palstance_tx_data[5];
////	palstance_can_tx_data[2] = palstance_tx_data[6];
////	palstance_can_tx_data[3] = palstance_tx_data[7];
//	palstance_can_tx_data[0] = palstance_tx_data[0];	//pit
//	palstance_can_tx_data[1] = palstance_tx_data[1];
//	palstance_can_tx_data[2] = palstance_tx_data[2];
//	palstance_can_tx_data[3] = palstance_tx_data[3];
//	palstance_can_tx_data[4] = palstance_tx_data[8];	//yaw
//	palstance_can_tx_data[5] = palstance_tx_data[9];
//	palstance_can_tx_data[6] = palstance_tx_data[10];
//	palstance_can_tx_data[7] = palstance_tx_data[11];
//	
//	//查询发送邮箱是否为空
//	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);  
//	while(FreeTxNum == 0) 
//	{  
//    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);  
//  }
//	
//	HAL_CAN_AddTxMessage(&hcan, &TxMessage, palstance_can_tx_data,(uint32_t*)CAN_TX_MAILBOX1);
//}


///**
//  * @brief  send_angle_message
//  * @param  
//  */
//void send_angle_message(void)
//{
//	uint8_t FreeTxNum = 0;  

//  //TxMessage.StdId   = 0x601;
//	TxMessage.StdId   = 0x002;
//  TxMessage.IDE     = CAN_ID_STD;
//  TxMessage.RTR     = CAN_RTR_DATA;
//  TxMessage.DLC     = 0x08;

//	
//	angle_can_tx_data[0] = angle_tx_data[0];	//pit
//	angle_can_tx_data[1] = angle_tx_data[1];
//	angle_can_tx_data[2] = angle_tx_data[2];
//	angle_can_tx_data[3] = angle_tx_data[3];
////	angle_can_tx_data[0] = angle_tx_data[4];	//pit
////	angle_can_tx_data[1] = angle_tx_data[5];
////	angle_can_tx_data[2] = angle_tx_data[6];
////	angle_can_tx_data[3] = angle_tx_data[7];
//	angle_can_tx_data[4] = angle_tx_data[8];	//yaw
//	angle_can_tx_data[5] = angle_tx_data[9];
//	angle_can_tx_data[6] = angle_tx_data[10];
//	angle_can_tx_data[7] = angle_tx_data[11];
//	
//	memcpy(pitch_angle,angle_can_tx_data,8);
//	//查询发送邮箱是否为空
//	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);  
//	while(FreeTxNum == 0) 
//	{  
//    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);  
//  }
//	
//	HAL_CAN_AddTxMessage(&hcan, &TxMessage, angle_can_tx_data,(uint32_t*)CAN_TX_MAILBOX1);
//}

