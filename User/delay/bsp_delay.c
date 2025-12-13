#include "delay/bsp_delay.h"

/**
  * @brief  粗略阻塞延时基本函数接口
  * @param  ncount：传入计数值
  * @note   软件延时函数，使用不同的系统时钟，延时不一样，还会存在函数调用以及其它计算损耗，只能粗略使用
  * @retval 无
  */
void Rough_Delay(__IO uint32_t ncount)
{
    for(uint32_t i = 0;i<ncount;i++)
    {
        __NOP();
    }
}

/**
  * @brief  粗略阻塞延时基本函数接口    单位：Us
  * @param  ncount：传入计数值
  * @note   软件延时函数，使用不同的系统时钟，延时不一样，还会存在函数调用以及其它计算损耗，只能粗略使用
  * @retval 无
  */
void Rough_Delay_Us(__IO uint32_t time)
{
    Rough_Delay(7*time);
}

/**
  * @brief  粗略阻塞延时基本函数接口    单位：Ms
  * @param  ncount：传入计数值
  * @note   软件延时函数，使用不同的系统时钟，延时不一样，还会存在函数调用以及其它计算损耗，只能粗略使用
  * @retval 无
  */
void Rough_Delay_Ms(__IO uint32_t time)
{
    Rough_Delay(0x3e8*7*time);
}

/**
  * @brief  粗略阻塞延时基本函数接口    单位：S
  * @param  ncount：传入计数值
  * @note   软件延时函数，使用不同的系统时钟，延时不一样，还会存在函数调用以及其它计算损耗，只能粗略使用
  * @retval 无
  */
void Rough_Delay_S(__IO uint32_t time)
{
    Rough_Delay(0x3e8*0x3e8*7*time);
}

/*****************************END OF FILE***************************************/
