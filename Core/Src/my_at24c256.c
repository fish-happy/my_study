#include  "my_at24c256.h"
/*
地址空间扩展：

AT24C256 不需要页地址，直接使用两字节（16位）地址访问全部32KB空间。

发送地址时先低字节（address & 0xFF），后高字节（address >> 8）。

I²C总线负载：

总线上所有设备的地址引脚（A2/A1/A0）组合必须唯一。

典型应用最多连接 8个AT24C256（地址组合 0xA0-0xAE）。

WP引脚：接GND：允许读写。

*/


//  my_I2C_At24c256_Write (I2C_TypeDef * I2Cx , uint8_t * p_data , uint32_t len , uint8_t slave_addr , uint16_t mem_addr);


// ErrorStatus my_I2C_At24c256_Read (I2C_TypeDef * I2Cx , uint8_t * p_data , uint32_t len , uint8_t slave_addr , uint8_t mem_addr);







