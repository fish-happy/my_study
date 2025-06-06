/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* SPI1 init function */
void MX_SPI1_Init (void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = { 0 };

  LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_SPI1);

  LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init (GPIOA , &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init (GPIOA , &GPIO_InitStruct);

  /* SPI1 DMA Init */

  /* SPI1_RX Init */
  LL_DMA_SetDataTransferDirection (DMA1 , LL_DMA_CHANNEL_2 , LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel (DMA1 , LL_DMA_CHANNEL_2 , LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode (DMA1 , LL_DMA_CHANNEL_2 , LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode (DMA1 , LL_DMA_CHANNEL_2 , LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode (DMA1 , LL_DMA_CHANNEL_2 , LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize (DMA1 , LL_DMA_CHANNEL_2 , LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize (DMA1 , LL_DMA_CHANNEL_2 , LL_DMA_MDATAALIGN_BYTE);

  /* SPI1_TX Init */
  LL_DMA_SetDataTransferDirection (DMA1 , LL_DMA_CHANNEL_3 , LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel (DMA1 , LL_DMA_CHANNEL_3 , LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode (DMA1 , LL_DMA_CHANNEL_3 , LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode (DMA1 , LL_DMA_CHANNEL_3 , LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode (DMA1 , LL_DMA_CHANNEL_3 , LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize (DMA1 , LL_DMA_CHANNEL_3 , LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize (DMA1 , LL_DMA_CHANNEL_3 , LL_DMA_MDATAALIGN_BYTE);

  /* SPI1 interrupt Init */
  NVIC_SetPriority (SPI1_IRQn , NVIC_EncodePriority (NVIC_GetPriorityGrouping ( ) , 9 , 0));
  NVIC_EnableIRQ (SPI1_IRQn);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init (SPI1 , &SPI_InitStruct);
  /* USER CODE BEGIN SPI1_Init 2 */
//使能SPI1
  LL_SPI_Enable (SPI1);

  //NSS使能
  LL_SPI_SetNSSMode (SPI1 , LL_SPI_NSS_SOFT);
  //SET_BIT (SPI1->CR1 , SPI_CR1_BIDIOE); // 发送模式

  //
  /* USER CODE END SPI1_Init 2 */

}

/* USER CODE BEGIN 1 */



/**
 * @brief SPI连续传输
 * @param tx 发送缓冲区(NULL时发0xFF)
 * @param rx 接收缓冲区(NULL时丢弃数据)
 * @param len 数据长度(0-65535)
 * @retval 传输状态
 */
ErrorStatus my_SPI_Continuous_Swapbyte (uint8_t * tx , uint8_t * rx , uint16_t len)
{
  const bool only_tx = (rx == NULL);
  const bool only_rx = (tx == NULL);

  while (LL_SPI_IsActiveFlag_BSY (SPI1)); // 等待总线空闲

  if (len == 0) return SUCCESS;





  for (uint16_t i = 0; i < len; i++)
  {

    //发送首字节
    if (i == 0)
    {
      LL_SPI_TransmitData8 (SPI1 , only_rx ? 0xFF : tx[i]);
    }


    if (i > 0 && len > 1)
    {
      while (!LL_SPI_IsActiveFlag_TXE (SPI1));
      LL_SPI_TransmitData8 (SPI1 , only_rx ? 0xFF : tx[i]);


      while (!LL_SPI_IsActiveFlag_RXNE (SPI1));
      if (!only_tx) rx[i - 1] = LL_SPI_ReceiveData8 (SPI1);
      else   LL_SPI_ReceiveData8 (SPI1);//丢弃字节

    }

    if (i == len - 1)
    {
      // 阶段3：末字节接收
      while (!LL_SPI_IsActiveFlag_RXNE (SPI1));
      if (!only_tx) rx[len - 1] = LL_SPI_ReceiveData8 (SPI1);
      else   LL_SPI_ReceiveData8 (SPI1);//丢弃字节
    }


  }



  while (LL_SPI_IsActiveFlag_BSY (SPI1));
  return SUCCESS;
}












ErrorStatus my_W25Q16M_Init (void)//W25Q16M初始化函数
{


  //复位W25Q16M
  my_SPI_Ctrl_CS (SEL);

  my_SPI_SwapOneByte (W25Q16M_CMD_RESET_ENABLE);//val=0x66
  my_SPI_SwapOneByte (W25Q16M_CMD_RESET_MEMORY);//val=0x99
  // delay_ms (1);//等待复位完成
  my_SPI_Ctrl_CS (UNSEL);


  my_SPI_Ctrl_CS (SEL);
  my_SPI_SwapOneByte (W25Q16M_CMD_JEDEC_ID);
  uint8_t MID = my_SPI_SwapOneByte (W25Q16M_DUMMY_BYTE);   //获取MID
  uint16_t DID = (my_SPI_SwapOneByte (W25Q16M_DUMMY_BYTE)) << 8;    //读取DID的高字节
  DID |= my_SPI_SwapOneByte (W25Q16M_DUMMY_BYTE);//读取DID的低字节
  my_SPI_Ctrl_CS (UNSEL);


  //检查是否等于W25Q16M的制造商ID和设备ID  分别为:0xEF和0x4018
  if (MID == WINBOND_MANUFACTURER_ID && DID == W25Q16M_DEVICE_ID)
  {
    printf ("W25Q16M found!\r\n");
    return SUCCESS;
  }
  else
  {

    printf ("W25Q16M not found!\r\n");
    return ERROR;
  }

}












/* USER CODE END 1 */
