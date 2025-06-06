/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
  
  typedef enum
  {
    SEL = 0 ,//选中SPI设备
    UNSEL = 1//未选中SPI设备
  } CS_Status;

  typedef enum
  {
    H = 1 ,//高电平
    L = 0 //低电平
  } Level_Status;







  
//设备ID信息
#define WINBOND_MANUFACTURER_ID  0xEF
#define W25Q16M_DEVICE_ID         0x4018

//复位
#define W25Q16M_CMD_RELEASE_POWER_DOWN  0xAB  // 唤醒芯片并返回设备ID（退出低功耗）
#define W25Q16M_CMD_RESET_ENABLE  0x66 // 使能复位
#define W25Q16M_CMD_RESET_MEMORY  0x99 // 复位

// 辅助定义
#define W25Q16M_DUMMY_BYTE                      0xFF  // 哑元空操作字节（用于快速读取占位）
#define W25Q_TIMEOUT_PAGE_PROGRAM   10     // 页编程状态   10ms
#define W25Q_TIMEOUT_SECTOR_ERASE   1000   // 扇区擦除状态 1000ms
#define W25Q_TIMEOUT_BLOCK32_ERASE    3000   // 32K块擦除状态   3000ms (3s)
#define W25Q_TIMEOUT_BLOCK64_ERASE    5000   // 64K块擦除状态   5000ms (3s)
#define W25Q_TIMEOUT_CHIP_ERASE    120000   // 整片擦除状态     120000ms (120s)


// 基础控制命令
#define W25Q16M_CMD_WRITE_ENABLE                    0x06  // 写使能：允许后续写入/擦除操作
#define W25Q16M_CMD_WRITE_DISABLE                   0x04  // 写禁止：禁止写入操作（保护数据）
#define W25Q16M_CMD_POWER_DOWN                      0xB9  // 进入低功耗模式（需唤醒命令退出）
#define W25Q16M_CMD_HIGH_PERFORMANCE_MODE           0xA3  // 激活高性能模式（提升时钟速率）
#define W25Q16M_CMD_CONTINUOUS_READ_MODE_RESET      0xFF  // 退出连续读取模式（复位时序）

// 状态寄存器操作
#define W25Q16M_CMD_READ_STATUS_REGISTER_1          0x05  // 读取状态寄存器1（BUSY/WEL等标志位）
#define W25Q16M_CMD_READ_STATUS_REGISTER_2          0x35  // 读取状态寄存器2（QE/SRP等扩展标志）
#define W25Q16M_CMD_WRITE_STATUS_REGISTER           0x01  // 写入状态寄存器（配置保护位/QE位）

// 数据读取命令（单线/多线模式）
#define W25Q16M_CMD_READ_DATA                       0x03  // 标准单线读取（低速，无dummy周期）
#define W25Q16M_CMD_FAST_READ                       0x0B  // 快速单线读取（需1个dummy字节）
#define W25Q16M_CMD_FAST_READ_DUAL_OUTPUT           0x3B  // 双线输出读取（数据线D0+D1）
#define W25Q16M_CMD_FAST_READ_DUAL_IO               0xBB  // 双线输入输出（地址和数据复用D0+D1）
#define W25Q16M_CMD_FAST_READ_QUAD_OUTPUT           0x6B  // 四线输出读取（数据线D0-D3）
#define W25Q16M_CMD_FAST_READ_QUAD_IO               0xEB  // 四线输入输出（需QE位使能）
#define W25Q16M_CMD_OCTAL_WORD_READ_QUAD_IO         0xE3  // 八字节四线读取（高性能模式）

// 编程与擦除命令
#define W25Q16M_CMD_PAGE_PROGRAM                    0x02  // 页编程（最大256字节，需先擦除）
#define W25Q16M_CMD_QUAD_PAGE_PROGRAM               0x32  // 四线页编程（需QE位使能）
#define W25Q16M_CMD_SECTOR_ERASE_4KB                0x20  // 擦除4KB扇区（最小擦除单位）
#define W25Q16M_CMD_BLOCK_ERASE_32KB                0x52  // 擦除32KB块（部分型号支持）
#define W25Q16M_CMD_BLOCK_ERASE_64KB                0xD8  // 擦除64KB块（典型擦除单位）
#define W25Q16M_CMD_CHIP_ERASE                      0xC7  // 全片擦除（谨慎使用！）
#define W25Q16M_CMD_ERASE_SUSPEND                   0x75  // 暂停当前擦除操作（允许临时读取）
#define W25Q16M_CMD_ERASE_RESUME                    0x7A  // 恢复被挂起的擦除操作

// 芯片标识与唯一ID
#define W25Q16M_CMD_JEDEC_ID                        0x9F  // 读取JEDEC标准ID（3字节：厂商+设备号）
#define W25Q16M_CMD_MANUFACTURER_DEVICE_ID          0x90  // 传统厂商/设备ID读取（兼容旧命令）
#define W25Q16M_CMD_READ_UNIQUE_ID                  0x4B  // 读取64位唯一ID（出厂固化，不可修改）
#define W25Q16M_CMD_RELEASE_POWER_DOWN_HPM_DEVICE_ID 0xAB // 唤醒芯片并返回设备ID（退出低功耗）

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */
ErrorStatus my_SPI_Continuous_Swapbyte (uint8_t * tx , uint8_t * rx , uint16_t len);


static inline uint8_t my_SPI_SwapOneByte (uint8_t tx_data)
{
  uint8_t rx_data;
  my_SPI_Continuous_Swapbyte (&tx_data , &rx_data , 1);
  return rx_data;
}

/// @brief CS控制函数
/// @param status SEL =选中从机 UNSEL =未选中从机
/// @return None
static inline void my_SPI_Ctrl_CS (CS_Status status)
{
  status == SEL ? LL_GPIO_ResetOutputPin (GPIOA , LL_GPIO_PIN_4) : LL_GPIO_SetOutputPin (GPIOA , LL_GPIO_PIN_4);
}














ErrorStatus my_W25Q16M_Init (void);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

