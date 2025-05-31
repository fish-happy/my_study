/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* I2C2 init function */
void MX_I2C2_Init (void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = { 0 };

  LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_GPIOB);
  /**I2C2 GPIO Configuration
  PB10   ------> I2C2_SCL
  PB11   ------> I2C2_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init (GPIOB , &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_I2C2);

  /* I2C2 DMA Init */

  /* I2C2_RX Init */
  LL_DMA_SetDataTransferDirection (DMA1 , LL_DMA_CHANNEL_5 , LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel (DMA1 , LL_DMA_CHANNEL_5 , LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode (DMA1 , LL_DMA_CHANNEL_5 , LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode (DMA1 , LL_DMA_CHANNEL_5 , LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode (DMA1 , LL_DMA_CHANNEL_5 , LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize (DMA1 , LL_DMA_CHANNEL_5 , LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize (DMA1 , LL_DMA_CHANNEL_5 , LL_DMA_MDATAALIGN_BYTE);

  /* I2C2_TX Init */
  LL_DMA_SetDataTransferDirection (DMA1 , LL_DMA_CHANNEL_4 , LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel (DMA1 , LL_DMA_CHANNEL_4 , LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode (DMA1 , LL_DMA_CHANNEL_4 , LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode (DMA1 , LL_DMA_CHANNEL_4 , LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode (DMA1 , LL_DMA_CHANNEL_4 , LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize (DMA1 , LL_DMA_CHANNEL_4 , LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize (DMA1 , LL_DMA_CHANNEL_4 , LL_DMA_MDATAALIGN_BYTE);

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2 (I2C2);
  LL_I2C_DisableGeneralCall (I2C2);
  LL_I2C_EnableClockStretching (I2C2);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init (I2C2 , &I2C_InitStruct);
  LL_I2C_SetOwnAddress2 (I2C2 , 0);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/* USER CODE BEGIN 1 */
/*static 函数部分*/

//发送起始条件,并检测,超时则重启I2C,并返回错误
__STATIC_INLINE ErrorStatus my_I2C2_GenerateStartCondition_Check (I2C_TypeDef * I2Cx)
{
  LL_I2C_GenerateStartCondition (I2Cx);
  while (!LL_I2C_IsActiveFlag_SB (I2Cx))
  {
    if (I2Cx->SR1 & I2C_SR1_TIMEOUT)
    {
      //重启I2C
      LL_I2C_Disable (I2Cx);
      LL_I2C_Enable (I2Cx);
      return ERROR; //超时退出
    }
  }
  return SUCCESS;
}

//发送从机地址,并检测,同时清除addr标志位,超时则重启I2C,并返回错误,rw为读写标志位,0为写,1为读
__STATIC_INLINE ErrorStatus my_I2C2_TransmitAddress_Check (I2C_TypeDef * I2Cx , uint8_t addr , uint8_t rw)
{
  LL_I2C_TransmitData8 (I2Cx , addr | rw);  //发送从机地址
  while (!LL_I2C_IsActiveFlag_ADDR (I2Cx))
  {
    if (I2Cx->SR1 & I2C_SR1_TIMEOUT)
    {
      //重启I2C
      LL_I2C_Disable (I2Cx);
      LL_I2C_Enable (I2Cx);
      return ERROR; //超时退出
    }
  }
  LL_I2C_ClearFlag_ADDR (I2Cx);
  return SUCCESS;
}




//发送寄存器地址和数据,并检测,超时则重启I2C,并返回错误
__STATIC_INLINE ErrorStatus my_I2C2_TransmitData_Check (I2C_TypeDef * I2Cx , uint8_t reg_addr , uint8_t * p_data , uint32_t len)
{

  //在清除addr标志位以后,TxE和BTF位置位,可以直接发送寄存器地址
  
  LL_I2C_TransmitData8 (I2Cx , reg_addr);
  


  for (uint16_t i = 0; i < len; i++)
  {

    while (!LL_I2C_IsActiveFlag_TXE (I2Cx))
    {
      if (I2Cx->SR1 & I2C_SR1_TIMEOUT)
      {
        //重启I2C
        LL_I2C_Disable (I2Cx);
        LL_I2C_Enable (I2Cx);
        return ERROR; //超时退出
      }
    }


    LL_I2C_TransmitData8 (I2Cx , p_data[i]);  //发送数据

  }

  while (!LL_I2C_IsActiveFlag_BTF (I2Cx))
  {
    if (I2Cx->SR1 & I2C_SR1_TIMEOUT)
    {
      //重启I2C
      LL_I2C_Disable (I2Cx);
      LL_I2C_Enable (I2Cx);
      return ERROR; //超时退出
    }
  }

  //BTF位可以不必手动清除,在stop信号发送成功以后,会自动清除BTF位
  return SUCCESS;

}

//发送停止信号,并检测,超时则重启I2C,并返回错误
__STATIC_INLINE ErrorStatus my_I2C2_GenerateStopCondition_Check (I2C_TypeDef * I2Cx)
{
  LL_I2C_GenerateStopCondition (I2Cx);
  while (!LL_I2C_IsActiveFlag_BUSY (I2Cx))
  {
    if (I2Cx->SR1 & I2C_SR1_TIMEOUT)
    {
      //重启I2C
      LL_I2C_Disable (I2Cx);
      LL_I2C_Enable (I2Cx);
      return ERROR; //超时退出
    }
  }
  return SUCCESS;
}



//关闭I2C对DMA提起的传输请求和关闭DMA的传输
__STATIC_INLINE  void my_I2C_DMA_Disabled (I2C_TypeDef * I2Cx , DMA_TypeDef * DMAx , uint32_t CHx)
{

  //在总线上发送stop信号,进入空闲状态后,就可关闭I2C对DMA提起的传输请求和DMA的传输
  LL_I2C_DisableDMAReq_TX (I2Cx);
  LL_DMA_DisableChannel (DMAx , CHx);

}


//使能I2C对DMA提起的传输请求和开启DMA的传输
__STATIC_INLINE  void my_I2C_DMA_Enable (I2C_TypeDef * I2Cx , DMA_TypeDef * DMAx , uint32_t CHx)
{

  //注意:先使能请求,后使能DMA,顺序不能颠倒,否则会丢失数据,详细参考数据手册:DMA请求映像p147)
  LL_I2C_EnableDMAReq_TX (I2Cx);
  LL_DMA_EnableChannel (DMAx , CHx);

}

/// @brief I2C的DMA发送配置
/// @note  配置I2C的DMA的源地址和目标地址以及数据长度
/// @param I2Cx I2C实例
/// @param DMAx DMA实例
/// @param CHx 通道号
/// @param p_buf 数据缓冲区的指针
/// @param len 数据长度
__STATIC_INLINE void my_I2C_DMA_SendConfig (I2C_TypeDef * I2Cx , DMA_TypeDef * DMAx , uint32_t CHx , uint8_t * p_buf , uint32_t len)
{

  /*
    传输方向和字节对齐等设置,已经在MX生成的初始化程序中完成.
    此处,需要设置:
    1.目标地址 (I2C的DR)  使用函数LL_I2C_DMA_GetRegAddr(I2Cx)获取
    2.源地址 (内存地址 , 数据缓冲区的指针) ,
    3.数据长度 ,
  */

  //1设置目标地址:DMA外设地址
  LL_DMA_SetPeriphAddress (DMAx , CHx , LL_I2C_DMA_GetRegAddr (I2Cx));
  //2设置源地址:内存地址 
  LL_DMA_SetMemoryAddress (DMAx , CHx , (uint32_t) p_buf);
  //3设置数据长度
  LL_DMA_SetDataLength (DMAx , CHx , len);


}


/// @brief 轮询DMA的TC标志位等待DMA数据传输完成,轮询I2C的BTF标志位等待I2C数据传输完成,并清除DMA的TC标志位,超时则重启I2C,并返回错误
/// @param DMAx 实例
/// @param I2Cx 实例
__STATIC_INLINE ErrorStatus my_I2C_DMA_WiatTransmitComplete (DMA_TypeDef * DMAx , I2C_TypeDef * I2Cx)
{
  // 等待 DMA完成传送
  while (!LL_DMA_IsActiveFlag_TC4 (DMAx))
  {

    if (I2Cx->SR1 & I2C_SR1_TIMEOUT)
    {
      //重启I2C
      LL_I2C_Disable (I2Cx);
      LL_I2C_Enable (I2Cx);
      return ERROR; //超时退出

    }

  }



  //检测I2C最后一个字节是否发送完成
  while (!LL_I2C_IsActiveFlag_BTF (I2Cx))
  {

    if (I2Cx->SR1 & I2C_SR1_TIMEOUT)
    {
      //重启I2C
      LL_I2C_Disable (I2Cx);
      LL_I2C_Enable (I2Cx);
      return ERROR; //超时退出
    }

  }

  //注意:BTF标志位在STOP信号产生以后,会自动被清除

  //清除DMA的TC标志位
  LL_DMA_ClearFlag_TC4 (DMAx);

  return SUCCESS;
}








/*I2C的普通传输*/


/// @brief I2C发送数据
/// @param I2Cx 实例
/// @param p_buf 数据缓冲区
/// @param len 长度
/// @param addr 从机地址
/// @return 0=success:成功,1=error:失败
ErrorStatus my_I2C_TransmitData (I2C_TypeDef * I2Cx , uint8_t * p_buf , uint32_t len , uint8_t slave_addr , uint8_t reg_addr)
{


  ErrorStatus status = my_I2C2_GenerateStartCondition_Check (I2Cx);

  status |= my_I2C2_TransmitAddress_Check (I2Cx , slave_addr , 0);

  status |= my_I2C2_TransmitData_Check (I2Cx , reg_addr , p_buf , len);

  status |= my_I2C2_GenerateStopCondition_Check (I2Cx);

  return status;
}




/// @brief I2C接收数据
/// @param I2Cx 实例
/// @param p_buf 接收数据缓冲区指针
/// @param len 接收数据长度
/// @param slave_addr 从机地址
/// @param reg_addr 寄存器地址
/// @return 0=success:成功,1=error:失败
ErrorStatus my_I2C_ReceiveData (I2C_TypeDef * I2Cx , uint8_t * p_buf , uint32_t len , uint8_t slave_addr , uint8_t reg_addr)
{

  //发送起始条件,并检测,超时则重启I2C,并返回错误
  ErrorStatus status = my_I2C2_GenerateStartCondition_Check (I2Cx);

  //发送从机地址(写模式,读写标志位为0),并检测,同时清除addr标志位,超时则重启I2C,并返回错误
  status |= my_I2C2_TransmitAddress_Check (I2Cx , slave_addr , 0);

  //发送寄存器地址
  LL_I2C_TransmitData8 (I2Cx , reg_addr);

  //检测是否发送完毕
  while (!LL_I2C_IsActiveFlag_BTF (I2Cx))
  {
    if (I2Cx->SR1 & I2C_SR1_TIMEOUT)
    {
      //重启I2C
      LL_I2C_Disable (I2Cx);
      LL_I2C_Enable (I2Cx);
      return ERROR; //超时退出
    }
  }



  //发送重复起始条件,并检测,超时则重启I2C,并返回错误
  status = my_I2C2_GenerateStartCondition_Check (I2Cx);

  //发送从机地址(读模式,读写标志位为1),并检测,同时清除addr标志位,超时则重启I2C,并返回错误

  status |= my_I2C2_TransmitAddress_Check (I2Cx , slave_addr , 1);

  //开始接收数据

  for (uint16_t i = 0; i < len; i++)
  {
    if (i == len - 1)
    {
      //最后一个字节接收之前:关闭ACK,使能stop
      LL_I2C_AcknowledgeNextData (I2Cx , LL_I2C_NACK);
      LL_I2C_GenerateStopCondition (I2Cx);
    }
    while (!LL_I2C_IsActiveFlag_RXNE (I2Cx))
    {
      if (I2Cx->SR1 & I2C_SR1_TIMEOUT)
      {
        //重启I2C
        LL_I2C_Disable (I2Cx);
        LL_I2C_Enable (I2Cx);
        return ERROR; //超时退出
      }
    }
    p_buf[i] = LL_I2C_ReceiveData8 (I2Cx);  //接收数据
  }


  //检测空闲标志位,超时则重启I2C,并返回错误
  while (!LL_I2C_IsActiveFlag_BUSY (I2Cx))
  {
    if (I2Cx->SR1 & I2C_SR1_TIMEOUT)
    {
      //重启I2C
      LL_I2C_Disable (I2Cx);
      LL_I2C_Enable (I2Cx);
      return ERROR; //超时退出
    }
  }



  //使能应答位,准备下一次接收
  LL_I2C_AcknowledgeNextData (I2Cx , LL_I2C_ACK);

  return status;
}


















/*I2C的DMA发送*/




/// @brief I2C的DMA发送数据
/// @param I2Cx 实例
/// @param DMAx 实例
/// @param CHx 通道号
/// @param p_buf 数据缓冲区指针
/// @param len 数据长度
/// @param slave_addr 从机地址
/// @return 0=success:成功,1=error:失败
ErrorStatus my_I2C_DMA_TransmitData (I2C_TypeDef * I2Cx , DMA_TypeDef * DMAx , uint32_t CHx , uint8_t * p_buf , uint32_t len , uint8_t slave_addr)
{

  //配置DMA的目标地址和源地址及其长度
  my_I2C_DMA_SendConfig (I2Cx , DMAx , CHx , p_buf , len);

  //使能DMA请求,启动DMA传输
  my_I2C_DMA_Enable (I2Cx , DMAx , CHx);

  // 发送起始条件
  ErrorStatus status = my_I2C2_GenerateStartCondition_Check (I2Cx);

  //发送从机地址,清除addr标志位
  status |= my_I2C2_TransmitAddress_Check (I2Cx , slave_addr , 0);

  //等待DMA传输完成
  status |= my_I2C_DMA_WiatTransmitComplete (DMAx , I2Cx);

  //发送停止位
  status |= my_I2C2_GenerateStopCondition_Check (I2Cx);

  my_I2C_DMA_Disabled (I2Cx , DMAx , CHx);

  return status;
}








ErrorStatus my_I2C_DMA_ReceiveData (I2C_TypeDef * I2Cx , DMA_TypeDef * DMAx , uint32_t CHx , uint8_t * p_buf , uint32_t len , uint8_t slave_addr)
{
  return ERROR;
}



























/* USER CODE END 1 */
