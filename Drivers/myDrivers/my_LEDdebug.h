#ifndef __MY_LEDDEBUG_H__ 
#define __MY_LEDDEBUG_H__ 


#include  "main.h" 


__STATIC_INLINE void my_Onboard_LED_msToggle (uint32_t Delay , uint8_t count)
{
    for (uint8_t i = 0;i < count;i++)
    {
        LL_GPIO_TogglePin (GPIOC , LL_GPIO_PIN_13);
        LL_mDelay (Delay);
    }

}


/// @brief 板载LED开关函数
/// @param sta 1:开 0:关
/// @return 返回灯的状态
__STATIC_INLINE bool my_Onboard_LED_switch (bool sta)
{
    if (sta)
    {
        LL_GPIO_ResetOutputPin (GPIOC , LL_GPIO_PIN_13);
        if (!LL_GPIO_IsOutputPinSet (GPIOC , LL_GPIO_PIN_13))
        {
            return  1;
        }


    }
    else
    {
        LL_GPIO_SetOutputPin (GPIOC , LL_GPIO_PIN_13);
        if (LL_GPIO_IsOutputPinSet (GPIOC , LL_GPIO_PIN_13))
        {

        }
    }
    return  0;
}


__STATIC_INLINE void my_Onboard_LED_Test (void)
{
    //测试板载灯和开关代码

    while (!my_Onboard_LED_switch (1));
    LL_mDelay (1000);
    while (my_Onboard_LED_switch (0));
    LL_mDelay (1000);

}




















































//end

























































//end







































#endif

