/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */


  //定义全局变量缓冲区
#define BUFFER_SIZE 256
extern uint8_t USART2_Rx_Buffer[BUFFER_SIZE];
extern bool idx;

void my_USART2_DMA1_Config (void);

void my_USART2_IDLE_DMA_callback (void);




  __STATIC_INLINE void USART2_Redirected_test (void)
  {
    //测试USART串口重定向的效果:
    puts ("STM32F103CT86 Hello world!");
    printf ("system frequency=%dMHz" , SystemCoreClock);
    putchar ('\r');
    putchar ('\n');

  }





/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

