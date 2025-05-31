#include  "MY_OLED091.h"


/*I2C驱动的硬件代码部分*/

void my_OLED091_Init (void)
{
  uint8_t OLED_Init_ARR [ ] = {
    0xAE, // 关闭显示
    0xD5, 0x80, // 时钟分频
    0xA8, 0x1F, // 多路复用比例（32行）
    0xD3, 0x00, // 显示偏移
    0x40,       // 起始行
    0x8D, 0x14, // 启用电荷泵

    0x20, 0x01, // 垂直寻址模式
    0x21, 0x00, 0x7F, // 列地址 0~127
    0x22, 0x00, 0x03, // 页地址 0~3（32行）

    0xDA, 0x02, // COM 引脚配置
    0x81, 0xCF, // 对比度
    0xD9, 0x22, // 预充电周期
    0xDB, 0x40, // VCOMH 电压

    0xAf,// 开启显示
    0xa4,

  };

  //利用初始化数组对OLED091进行初始化
  my_I2C_TransmitData (I2C2 , OLED_Init_ARR , sizeof (OLED_Init_ARR) , 0x78 , 0x00);

  // //屏幕测试
  //512个字节全部写入0xff
  uint8_t OLED_test_ARR[512];
  memset (OLED_test_ARR , 0xff , sizeof (OLED_test_ARR));
  my_I2C_TransmitData (I2C2 , OLED_test_ARR , sizeof (OLED_test_ARR) , 0x78 , 0x40);
  LL_mDelay (2000);
  // // //512个字节全部写入0
  memset (OLED_test_ARR , 0x00 , sizeof (OLED_test_ARR));
  my_I2C_TransmitData (I2C2 , OLED_test_ARR , sizeof (OLED_test_ARR) , 0x78 , 0x40);

}


ErrorStatus my_OLED091_SendOneCommand (uint8_t CMD)
{


  return my_I2C_TransmitData (I2C2 , &CMD , 1 , 0x78 , 0x00);
}


ErrorStatus my_OLED091_SendOneByte (uint8_t Byte)
{


  return my_I2C_TransmitData (I2C2 , &Byte , 1 , 0x78 , 0x40);
}


ErrorStatus my_OLED091_SendMultiBytes (uint8_t * P_Data , uint8_t len)
{


  return my_I2C_TransmitData (I2C2 , P_Data , len , 0x78 , 0x40);

}


