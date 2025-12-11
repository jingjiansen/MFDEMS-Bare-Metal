/**
  ******************************************************************************
  * @file       app_sensor_light.c
  * @author     embedfire
  * @version     V1.0
  * @date        2024
  * @brief      感应灯 应用层功能接口
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

#include "sensor_light/app_sensor_light.h"
#include "led/bsp_gpio_led.h"
#include "usart/usart_com.h"
#include "rd03/app_rd03.h"
#include "rd03/bsp_rd03.h"
#include "tracke/bsp_gpio_tracke.h"
#include "systick/bsp_systick.h"
#include "oled/app_oled.h" 

SL_TaskInfo sl_task  = {0};
static SL_StateInfo sl_device0 ={SL_MODE_AUTO};

/**
  * @brief  感应灯 计数复位
  * @param  无
  * @retval 无
  */
void SL_TaskReset(void)
{
    sl_task.timer = sl_task.cycle;
    sl_task.flag  = 0;

}

/**
  * @brief  感应灯 任务初始化
  * @param  sl_task_cycle: 任务轮询周期 单位ms(可修改系统节拍定时器)
  * @retval 无
  */
void SL_TaskInit(uint32_t sl_task_cycle)
{
    sl_task.cycle = sl_task_cycle;
    SL_SetMode(SL_MODE_AUTO);
    SL_TaskReset();
}

/**
  * @brief  感应灯 任务
  * @param  无
  * @retval 无
  */
void SL_Task(void)
{
    if(sl_task.flag)
    {
//        if(menu == 0x22)
//        {
//        
//        
//        }
        SL_Rd03Handle();
        SL_TrackeHandle();
        SL_TaskReset();
    }
}

/**
  * @brief  感应灯 模式配置（指定模式配置）
  * @param  sl_mode：感应灯模式
  * @retval 无
  */
void SL_SetMode(SL_ModeTpye sl_mode)
{
    switch(sl_mode)
    {
        case SL_MODE_AUTO :
            LED_OFF(R_LED_GPIO_PORT,R_LED_GPIO_PIN,LED_LOW_TRIGGER);
            LED_ON(G_LED_GPIO_PORT,G_LED_GPIO_PIN,LED_LOW_TRIGGER);
            LED_OFF(B_LED_GPIO_PORT,B_LED_GPIO_PIN,LED_LOW_TRIGGER);
            LED_OFF(LED4_GPIO_PORT,LED4_GPIO_PIN,LED_LOW_TRIGGER);
            led_state_flag = 0;
            printf("SL_MODE_AUTO\n");
            break;
        case SL_MODE_OPEN :
            LED_OFF(R_LED_GPIO_PORT,R_LED_GPIO_PIN,LED_LOW_TRIGGER);
            LED_OFF(G_LED_GPIO_PORT,G_LED_GPIO_PIN,LED_LOW_TRIGGER);
            LED_ON(B_LED_GPIO_PORT,B_LED_GPIO_PIN,LED_LOW_TRIGGER);
            LED_ON(LED4_GPIO_PORT,LED4_GPIO_PIN,LED_LOW_TRIGGER);
            led_state_flag = 1;
            printf("SL_MODE_OPEN\n");
            break;
        case SL_MODE_SHUT :
            LED_ON(R_LED_GPIO_PORT,R_LED_GPIO_PIN,LED_LOW_TRIGGER);
            LED_OFF(G_LED_GPIO_PORT,G_LED_GPIO_PIN,LED_LOW_TRIGGER);
            LED_OFF(B_LED_GPIO_PORT,B_LED_GPIO_PIN,LED_LOW_TRIGGER);
            LED_OFF(LED4_GPIO_PORT,LED4_GPIO_PIN,LED_LOW_TRIGGER);
            led_state_flag = 0;
            printf("SL_MODE_SHUT\n");
            break;
        default:
            break;
    }
    sl_device0.mode = sl_mode;  //记录更新生效的感应灯模式

}
/**
  * @brief  感应灯 模式配置（根据上一个模式自动配置下一个模式）
  * @param  无
  * @retval 无
  */
void SL_SwitchMode(void)
{
    int mode_temp = sl_device0.mode+1;
    
    if(mode_temp == SL_MODE_NUM)    //利用SL_MODE_NUM指示的枚举个数，让实际所有枚举值从头循环
    {
        mode_temp = SL_MODE_AUTO;
    }
    
    SL_SetMode((SL_ModeTpye)mode_temp);

}

/**
  * @brief  感应灯 RD03雷达反馈信息处理函数
  * @param  无
  * @retval 无
  */
void SL_Rd03Handle(void)
{
    if(sl_device0.mode == SL_MODE_AUTO)
    {
        Rd03_ResultInfo result_temp = Rd03_GetResult();
        if(result_temp.exist == RD03_NOEXIST)
        {
            LED_OFF(LED4_GPIO_PORT,LED4_GPIO_PIN,LED_LOW_TRIGGER);
            led_state_flag = 0;
        }
        else
        {
            LED_ON(LED4_GPIO_PORT,LED4_GPIO_PIN,LED_LOW_TRIGGER);
            led_state_flag = 1;
        }
    }
    else if(sl_device0.mode == SL_MODE_OPEN)
    {
        //Do nothing by default
    }
    else if(sl_device0.mode == SL_MODE_SHUT)    
    {
        //Do nothing by default
    }
    else;
}

/**
  * @brief  感应灯 反射传感器反馈信息处理函数
  * @param  无
  * @retval 无
  */
void SL_TrackeHandle(void)
{
    SL_SwitchSignalTpye current_signal = SL_SWITCH_SIGNAL_NONE;
    
    if(TRACKE_Scan(TRACKE_DO_GPIO_PORT,TRACKE_DO_GPIO_PIN) == Bit_RESET)
    {
        current_signal = SL_SWITCH_SIGNAL_TRIGGER;
    

    }
    else
    {
        current_signal = SL_SWITCH_SIGNAL_NONE;
    }
    
    if(sl_device0.last_signal == SL_SWITCH_SIGNAL_NONE && current_signal == SL_SWITCH_SIGNAL_TRIGGER)   //未感应到进入感应
    {
        sl_device0.last_signal          = SL_SWITCH_SIGNAL_TRIGGER;
        sl_device0.first_trigger_time    = SysTick_GetCount();           //第一次由不遮挡到遮挡的时间记录
    }
    else if(sl_device0.last_signal == SL_SWITCH_SIGNAL_TRIGGER && current_signal == SL_SWITCH_SIGNAL_NONE)   //感应到撤开感应
    {
        sl_device0.last_signal          = SL_SWITCH_SIGNAL_NONE;
        sl_device0.first_trigger_time = 0;
    }
    else if(sl_device0.last_signal == SL_SWITCH_SIGNAL_TRIGGER && current_signal == SL_SWITCH_SIGNAL_TRIGGER)   //感应到保持感应
    {
        uint64_t current_time = SysTick_GetCount();
        if(current_time - sl_device0.first_trigger_time > SL_SWITCH_HOLD_MS)
        {
            SL_SwitchMode();
            sl_device0.first_trigger_time = SysTick_GetCount();
        }

    }
    
    
}

/*****************************END OF FILE***************************************/
