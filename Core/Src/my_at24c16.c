
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

















































//end























































//end

