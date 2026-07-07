#include "can_bsp.h"
#include "qd4310.h"

/**
************************************************************************
* @brief:       can_bsp_init(void)
* @param:       void
* @retval:      void
* @details:     CAN init
************************************************************************
**/
void can_bsp_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
************************************************************************
* @brief:       can_filter_init(void)
* @param:       void
* @retval:      void
* @details:     CAN filter init
************************************************************************
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;
	fdcan_filter.FilterIndex = 0;
	fdcan_filter.FilterType = FDCAN_FILTER_RANGE;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	fdcan_filter.FilterID1 = 0x0000;
	fdcan_filter.FilterID2 = 0x0000;
	if(HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter)!=HAL_OK)
	{
		Error_Handler();
	}
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
}

/**
************************************************************************
* @brief:       fdcanx_send_data
* @param:       hfdcan: FDCAN handle
* @param:       id: CAN device ID
* @param:       data: data to send
* @param:       len: data length
* @retval:      0=success, 1=fail
* @details:     send data
************************************************************************
**/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	FDCAN_TxHeaderTypeDef TxHeader;
	
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = len;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x00;
    
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data)!=HAL_OK) 
		return 1;
	return 0;	
}

/**
************************************************************************
* @brief:       fdcanx_receive
* @param:       hfdcan: FDCAN handle
* @param:       buf: receive buffer
* @retval:      received data length
* @details:     receive data
************************************************************************
**/
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
  if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &fdcan_RxHeader, buf)!=HAL_OK)
		return 0;
  return fdcan_RxHeader.DataLength>>16;	
}

/**
************************************************************************
* @brief:       fdcanx_receive_with_id
* @param:       hfdcan: FDCAN handle
* @param:       buf: receive buffer
* @param:       can_id: pointer to store received CAN ID
* @retval:      received data length
* @details:     receive data with CAN ID (for QD4310 feedback routing)
************************************************************************
**/
uint8_t fdcanx_receive_with_id(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf, uint32_t *can_id)
{
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
	if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_RxHeader, buf)!=HAL_OK)
		return 0;
	if(can_id != NULL)
		*can_id = fdcan_RxHeader.Identifier;
	return fdcan_RxHeader.DataLength>>16;
}

/**
************************************************************************
* @brief:       HAL_FDCAN_RxFifo0Callback
* @param:       hfdcan: FDCAN handle
* @param:       RxFifo0ITs: interrupt flags
* @retval:      void
* @details:     HAL FDCAN interrupt callback
************************************************************************
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
		if(hfdcan == &hfdcan1)
		{
			fdcan1_rx_callback();
		}
		if(hfdcan == &hfdcan2)
		{
			fdcan2_rx_callback();
		}
		if(hfdcan == &hfdcan3)
		{
			fdcan3_rx_callback();
		}
	}
}

/**
************************************************************************
* @brief:       fdcan rx callbacks
* @param:       void
* @retval:      void
* @details:     rx callbacks for each FDCAN, with QD4310 feedback routing
************************************************************************
**/
uint8_t rx_data1[8] = {0};
void fdcan1_rx_callback(void)
{
	uint32_t can_id = 0;
	uint8_t len = fdcanx_receive_with_id(&hfdcan1, rx_data1, &can_id);
	/* QD4310 feedback frame routing: ID = 0x500 + motor_id */
	if(len > 0 && can_id >= 0x500 && can_id < (0x500 + QD_MAX_MOTORS))
	{
		qd_update((uint8_t)(can_id - 0x500), rx_data1);
	}
}

uint8_t rx_data2[8] = {0};
void fdcan2_rx_callback(void)
{
	uint32_t can_id = 0;
	uint8_t len = fdcanx_receive_with_id(&hfdcan2, rx_data2, &can_id);
	/* QD4310 feedback frame routing: ID = 0x500 + motor_id */
	if(len > 0 && can_id >= 0x500 && can_id < (0x500 + QD_MAX_MOTORS))
	{
		qd_update((uint8_t)(can_id - 0x500), rx_data2);
	}
}

uint8_t rx_data3[8] = {0};
void fdcan3_rx_callback(void)
{
	uint32_t can_id = 0;
	uint8_t len = fdcanx_receive_with_id(&hfdcan3, rx_data3, &can_id);
	/* QD4310 feedback frame routing: ID = 0x500 + motor_id */
	if(len > 0 && can_id >= 0x500 && can_id < (0x500 + QD_MAX_MOTORS))
	{
		qd_update((uint8_t)(can_id - 0x500), rx_data3);
	}
}