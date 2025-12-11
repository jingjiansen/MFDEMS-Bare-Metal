/**
  ******************************************************************************
  * @file       bsp_gpio_tracke.c
  * @author     embedfire
  * @version     V1.0
  * @date        2024
  * @brief      反射传感器函数接口
  ******************************************************************************
  * @attention
  *
  * 实验平台  ：野火 STM32F103C8T6-STM32开发板 
  * 论坛      ：http://www.firebbs.cn
  * 官网      ：https://embedfire.com/
  * 淘宝      ：https://yehuosm.tmall.com/
  *
  ******************************************************************************
  */

#include "tracke/bsp_gpio_tracke.h"
#include "delay/bsp_delay.h"

/**
  * @brief  初始化控制 反射传感器 的IO
  * @param  无
  * @retval 无
  */
void TRACKE_GPIO_Config(void)
{
    /* 定义一个 GPIO 结构体 */
    GPIO_InitTypeDef gpio_initstruct = {0};
    
#if 1    
    
    /* 开启 TRACKE 相关的GPIO外设/端口时钟 */
    RCC_APB2PeriphClockCmd(TRACKE_DO_GPIO_CLK_PORT,ENABLE);
    
    /* IO输出状态初始化控制 */
    GPIO_SetBits(TRACKE_DO_GPIO_PORT,TRACKE_DO_GPIO_PIN);
    
    /*选择要控制的GPIO引脚、设置GPIO模式为 浮空输入、设置GPIO速率为50MHz*/
    gpio_initstruct.GPIO_Pin    = TRACKE_DO_GPIO_PIN;
    gpio_initstruct.GPIO_Mode   = GPIO_Mode_IN_FLOATING;
    gpio_initstruct.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(TRACKE_DO_GPIO_PORT,&gpio_initstruct);
   
#endif 
    
}

/**
  * @brief  检测 反射传感器 触发情况
  * @param  GPIOx：x 可以是 A，B，C等
  * @param  GPIO_Pin：待操作的pin脚号
  * @retval Bit_SET(外界环境反射强度弱)、Bit_RESET(外界环境反射强度强)
  */
BitAction TRACKE_Scan(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == Bit_RESET)
    {
        return Bit_RESET;
    }
    else
    {
        return Bit_SET;
    }
    
//    return GPIO_ReadInputDataBit(GPIOx,GPIO_Pin);
}

/*****************************END OF FILE***************************************/
