
#include "my_at24c16.h"

//非DMA模式下写一个字节
ErrorStatus my_AT24C16_WriteByte (uint8_t byte , uint8_t Page_addr , uint8_t mem_addr)
{
    ErrorStatus status = my_I2C_TransmitData (I2C1 , &byte , 1 , Page_addr , mem_addr);
    LL_mDelay (5);
    return status;
}

//非DMA模式下写多个字节
ErrorStatus my_AT24C16_WriteData (uint8_t * p_data , uint32_t len , uint8_t Page_addr , uint8_t mem_addr)
{

    ErrorStatus status = my_I2C_TransmitData (I2C1 , p_data , len , Page_addr , mem_addr);
    LL_mDelay (5);
    return status;
}

//非DMA模式下读多个字节
ErrorStatus my_AT24C16_RedaData (uint8_t * p_buf , uint32_t len , uint8_t Page_addr , uint8_t mem_addr)
{
    return my_I2C_ReceiveData (I2C1 , p_buf , len , Page_addr , mem_addr);
}







//DMA模式下写多个字节
ErrorStatus my_AT24C16_DMA_WriteData (uint8_t * p_data , uint32_t len , uint8_t Page_addr , uint8_t mem_addr)
{
    ErrorStatus status;

    //uint8_t page_overflow = (len % 255);
    //uint8_t page_size = (len / 255) + (len % 255 != 0);
    uint32_t hex_size = (len / 16U) + ((len % 16U) != 0);

    // uint32_t hex_overflow = (len % 16U);

    // uint32_t page_remaining = 256 - mem_addr;

    //if (len > page_remaining)


    for (uint32_t i = 0; i < hex_size; i++)
    {
        uint8_t offset = (i * 0x10);
        status = my_I2C_DMA_TransmitData (I2C1 , DMA1 , LL_DMA_CHANNEL_4 , p_data , 16 , Page_addr , mem_addr + offset);
        LL_mDelay (5);

        if (i != 0 && i % 16 == 0)
        {
            Page_addr += 2;
            mem_addr = 0;
        }

    }
    return status;
}


















//DMA模式下读多个字节
ErrorStatus my_AT24C16_DMA_RedaData (uint8_t * p_buf , uint32_t len , uint8_t Page_addr , uint8_t mem_addr)
{
    return my_I2C_DMA_ReceiveData (I2C1 , DMA1 , LL_DMA_CHANNEL_5 , p_buf , len , Page_addr , mem_addr);
}












































//end























































//end

