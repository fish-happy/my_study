/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */





/* USER CODE END Private defines */

void MX_I2C1_Init(void);
void MX_I2C2_Init(void);

/* USER CODE BEGIN Prototypes */



     /*I2C的普通传输*/

  ErrorStatus my_I2C_ReceiveData (I2C_TypeDef * I2Cx , uint8_t * p_buf , uint32_t len , uint8_t slave_addr , uint8_t reg_addr);

  ErrorStatus my_I2C_TransmitData (I2C_TypeDef * I2Cx , uint8_t * p_buf , uint32_t len , uint8_t slave_addr , uint8_t reg_addr);



  /*I2C的DMA传输*/


  ErrorStatus my_I2C_DMA_TransmitData (I2C_TypeDef * I2Cx , DMA_TypeDef * DMAx , uint32_t CHx , uint8_t * p_buf , uint32_t len , uint8_t slave_addr , uint8_t reg_addr);


  ErrorStatus my_I2C_DMA_ReceiveData (I2C_TypeDef * I2Cx , DMA_TypeDef * DMAx , uint32_t CHx , uint8_t * p_buf , uint32_t len , uint8_t slave_addr , uint8_t reg_addr);



  /*I2C的普通传输:EEPROM:at24c256*/



  ErrorStatus my_I2C_At24c256_Write (I2C_TypeDef * I2Cx , uint8_t * p_data , uint32_t len , uint8_t slave_addr , uint16_t mem_addr);


  ErrorStatus my_I2C_At24c256_Read (I2C_TypeDef * I2Cx , uint8_t * p_data , uint32_t len , uint8_t slave_addr , uint8_t mem_addr);






/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

