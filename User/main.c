/**
  ******************************************************************************
  * @file       main.c
  * @author     embedfire
  * @version     V1.0
  * @date        2024
  * @brief      桌面小摆件(按键版)
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

#include "main.h"
#include "stm32f10x.h"
#include "debug/bsp_debug.h"
#include "i2c/bsp_i2c.h" 
#include "dwt/bsp_dwt.h" 
#include "oled/bsp_i2c_oled.h"
#include "fonts/bsp_fonts.h"
#include "key/bsp_gpio_key.h"
#include "led/bsp_gpio_led.h"
#include "dht11/bsp_dht11.h"
#include "systick/bsp_systick.h"

/**
  ***************************************程序功能设计要求***************************************
  *
  *     一级功能：  
  *
  *         1-开机显示野火电子--上下左右居中--悬停1s
  *         2-第一步过后显示主页
  *         3-可通过板载按键key1/key2左右切换一级菜单(分别是主页、音乐、温湿度、灯具控制、木鱼)
  *         4-可通过扩展按键key进入\退回一级菜单(分别是主页、音乐、温湿度、灯具控制、木鱼)
  *
  *     二级功能：
  *
  *         1-主页内容：招呼图  
  *
  *         2-音乐内容：全屏位图
  *
  *         3-灯具内容：开关灯(LED4)
  *
  *         4-打坐内容：敲木鱼(板载按键key1/key2可以计数或清零敲击次数)
  *
  *         5-温湿度内容：500ms更新一次温湿度数据
  *       
  *       
  ******************************************************************************
  */

/**
  * @brief  主函数
  * @param  无 
  * @note   无
  * @retval 无
  */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    KEY_Init();
    LED_GPIO_Config();
    IIC_Init();
    DEBUG_USART_Init();
    DHT11_GPIO_Config();
    
    SysTick_Init();
    DWT_Init();
    
    OLED_Init();
    OLED_CLS();
    Dht11_TaskInit(500);
    
    Boot_Task();
    
    menu_show_flag = 1;
    
    while(1)
    {
        /* OLED显示屏任务 */
        Menu_Task();
        
        /* DHT11温湿度传感器任务 */
        Dht11_Task();
    }
        
}

