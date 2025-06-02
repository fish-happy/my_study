
#include "my_at24c16.h"


ErrorStatus my_AT24C16_WriteByte (uint8_t byte , uint8_t Page_addr , uint8_t mem_addr)
{
    ErrorStatus status = my_I2C_TransmitData (I2C2 , &byte , 1 , Page_addr , mem_addr);
    LL_mDelay (5);
    return status;
}


ErrorStatus my_AT24C16_WriteData (uint8_t * p_data , uint32_t len , uint8_t Page_addr , uint8_t mem_addr)
{

    ErrorStatus status = my_I2C_TransmitData (I2C2 , p_data , len , Page_addr , mem_addr);
    LL_mDelay (5);
    return status;
}


ErrorStatus my_AT24C16_RedaData (uint8_t * p_buf , uint32_t len , uint8_t Page_addr , uint8_t mem_addr)
{
    return my_I2C_ReceiveData (I2C2 , p_buf , len , Page_addr , mem_addr);
}

ErrorStatus my_AT24C16_DMA_WriteData (uint8_t * p_data , uint32_t len , uint8_t Page_addr , uint8_t mem_addr)
{

    ErrorStatus status;
    uint32_t hex_size = (len / 16U) + ((len % 16U) != 0);


    for (uint32_t i = 0; i < hex_size; i++)
    {
        status = my_I2C_DMA_TransmitData (I2C2 , DMA1 , LL_DMA_CHANNEL_4 , p_data , 16 , Page_addr , mem_addr + (i << 4));

    }




    LL_mDelay (5);
    return status;
}






ErrorStatus my_AT24C16_DMA_RedaData (uint8_t * p_buf , uint32_t len , uint8_t Page_addr , uint8_t mem_addr)
{
    return my_I2C_DMA_ReceiveData (I2C2 , DMA1 , LL_DMA_CHANNEL_5 , p_buf , len , Page_addr , mem_addr);
}












































//end























































//end

