#ifndef __MY_AT24C16_H__ 
#define __MY_AT24C16_H__ 

#include "main.h"



// devide base address=0xA0
#define AT24C16_DEVICE_ADDR (uint8_t)0xA0 
// page scope :0-7
#define AT24C16_DEVICE_PAGE_ADDR(page) (AT24C16_DEVICE_ADDR|((uint8_t)(page << 1))) 



ErrorStatus my_AT24C16_WriteByte (uint8_t byte , uint8_t Page_addr , uint8_t mem_addr);


ErrorStatus my_AT24C16_WriteData (uint8_t * p_data , uint32_t len , uint8_t Page_addr , uint8_t mem_addr);


ErrorStatus my_AT24C16_RedaData (uint8_t * p_buf , uint32_t len , uint8_t Page_addr , uint8_t mem_addr);



































#endif

