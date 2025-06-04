/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */






/* USER CODE END 0 */

/* USART1 init function */

void MX_USART1_UART_Init (void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = { 0 };

  LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init (GPIOA , &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init (GPIOA , &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority (USART1_IRQn , NVIC_EncodePriority (NVIC_GetPriorityGrouping ( ) , 7 , 0));
  NVIC_EnableIRQ (USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init (USART1 , &USART_InitStruct);
  LL_USART_ConfigAsyncMode (USART1);
  LL_USART_Enable (USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init (void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = { 0 };

  LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_USART2);

  LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init (GPIOA , &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init (GPIOA , &GPIO_InitStruct);

  /* USART2 DMA Init */

  /* USART2_RX Init */
  LL_DMA_SetDataTransferDirection (DMA1 , LL_DMA_CHANNEL_6 , LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel (DMA1 , LL_DMA_CHANNEL_6 , LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode (DMA1 , LL_DMA_CHANNEL_6 , LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode (DMA1 , LL_DMA_CHANNEL_6 , LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode (DMA1 , LL_DMA_CHANNEL_6 , LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize (DMA1 , LL_DMA_CHANNEL_6 , LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize (DMA1 , LL_DMA_CHANNEL_6 , LL_DMA_MDATAALIGN_BYTE);

  /* USART2_TX Init */
  LL_DMA_SetDataTransferDirection (DMA1 , LL_DMA_CHANNEL_7 , LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel (DMA1 , LL_DMA_CHANNEL_7 , LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode (DMA1 , LL_DMA_CHANNEL_7 , LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode (DMA1 , LL_DMA_CHANNEL_7 , LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode (DMA1 , LL_DMA_CHANNEL_7 , LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize (DMA1 , LL_DMA_CHANNEL_7 , LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize (DMA1 , LL_DMA_CHANNEL_7 , LL_DMA_MDATAALIGN_BYTE);

  /* USART2 interrupt Init */
  NVIC_SetPriority (USART2_IRQn , NVIC_EncodePriority (NVIC_GetPriorityGrouping ( ) , 8 , 0));
  NVIC_EnableIRQ (USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init (USART2 , &USART_InitStruct);
  LL_USART_ConfigAsyncMode (USART2);
  LL_USART_Enable (USART2);
  /* USER CODE BEGIN USART2_Init 2 */
  my_USART2_DMA1_Config ( );




  /* USER CODE END USART2_Init 2 */

}

/* USER CODE BEGIN 1 */



//开启接收中断和空闲中断

uint8_t USART2_Rx_Buffer[BUFFER_SIZE];
bool idx = 0;

void my_USART2_DMA1_Config (void)
{
  //MX中已经配置了DMA的初始化，这里只需配置:
  //1.缓冲区
  //2.缓冲区长度
  //3.使能Rx请求DMA传输通道
  //4.使能DMA1 channel6
  LL_DMA_SetPeriphAddress (DMA1 , LL_DMA_CHANNEL_6 , LL_USART_DMA_GetRegAddr (USART2));
  LL_DMA_SetMemoryAddress (DMA1 , LL_DMA_CHANNEL_6 , (uint32_t) USART2_Rx_Buffer);
  LL_DMA_SetDataLength (DMA1 , LL_DMA_CHANNEL_6 , BUFFER_SIZE);

  LL_USART_ClearFlag_IDLE (USART2);
  LL_USART_EnableIT_IDLE (USART2);//启用空闲中断
  //LL_DMA_EnableIT_TC (DMA1 , LL_DMA_CHANNEL_6);//使能DMA传输完成中断
  LL_USART_EnableDMAReq_RX (USART2);//使能DMA接收请求
  LL_DMA_EnableChannel (DMA1 , LL_DMA_CHANNEL_6);//使能DMA通道
}

//特别注意:
   //CIRCULAR模式下 , 如果DMA不关闭,数据CNDTR不能写入,
   //因为:如同timer的一样,CNDTR的值来自自动重装寄存器(ARR:auto reload register),
   //这是一个影子寄存器.
   //长度数据是先写入影子寄存器 , 再写入CNDTR,初始化以后,当CNDTR递减至发出至0下溢的信号,
   //影子寄存器将长度值写入CNDTR.
   //工作过程:
   //DMA传输开启,CNDTR初始值开始递减:
   //如果未至0:
   //DMA会一直等待数据写入,锁死CNDTR从影子寄存器读取长度值,
   //初始值至0:
   //发出下溢更新信号,才从影子重装寄存器读取长度值.
   //这么做的目的是为了保证数据的完整性.
   //所以,应该先关闭通道,才能更新CNDTR.
   //白费了一晚上的时间鼓捣,反复思考,才找到的症结之所在,发现是CNDTR数据没有更新
   //问deepseek才知道CNDTR重载数据的机制.
   //更要命的中文参考手册完全没有提到CNDTR的更新机制,
   //不是万能的deep seek提醒,我估计还要再黑暗中摸索
//此代码,靠一个空闲中断,两个变量,实现了环形缓冲区的接收,
  //而且环形缓冲区可以任意调整,指针回到基址,可以是一半,三分之二等等
  //比我看到的任何同类程序都优雅简单且健壮
  //因为程序第一步就是获取到了最重要的长度信息.
//然后接着只有一段代码执行当前的接收数据
//后面的都是为下次接收做准备.根本不用担心数据覆盖
void my_USART2_IDLE_DMA_callback (void)
{
  LL_USART_DisableDirectionRx (USART2);//关闭接收方向保障获取长度信息的原子性
  //首先获取长度信息,长度信息+缓冲区基址USART2_Rx_Buffer,恰好也是下次写入的起始指针
  uint32_t len = BUFFER_SIZE - LL_DMA_GetDataLength (DMA1 , LL_DMA_CHANNEL_6);
  //基址指针赋值给当前指针
  static uint8_t * current_ptr = USART2_Rx_Buffer;
  //处理当前数据,通过printf函数回显到上位机
  printf ("receive data:%s\n" , current_ptr);
  //下次接收数据的起始指针
  current_ptr = USART2_Rx_Buffer + len;
  //判断缓冲区指针是否过半,过半则将缓冲区基址写入当前指针,
  //目的是不让指针回卷,导致数据地址不连续
  if (current_ptr >= (USART2_Rx_Buffer + 200))//可以随意设置环形缓冲区长度,此处设置为200字节
  {
    //关闭通道
    LL_DMA_DisableChannel (DMA1 , LL_DMA_CHANNEL_6);//使能DMA通道
    //重置长度和缓冲区指针
    LL_DMA_SetDataLength (DMA1 , LL_DMA_CHANNEL_6 , BUFFER_SIZE);
    LL_DMA_SetMemoryAddress (DMA1 , LL_DMA_CHANNEL_6 , (uint32_t) USART2_Rx_Buffer);
    //将当前指针写入缓冲区首指针
    current_ptr = USART2_Rx_Buffer;
    //清空DMA接收缓冲区
    memset (USART2_Rx_Buffer , 0 , len);
    LL_DMA_EnableChannel (DMA1 , LL_DMA_CHANNEL_6);//使能DMA通道
  }
  LL_USART_EnableDirectionRx (USART2);//关闭接收方向
}












int fputc (int ch , FILE * f)
{
  while (!LL_USART_IsActiveFlag_TXE (USART2));
  LL_USART_TransmitData8 (USART2 , (uint8_t) ch);

  return ch;
}




/* USER CODE END 1 */
