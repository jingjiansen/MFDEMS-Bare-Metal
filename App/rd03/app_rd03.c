/**
  ******************************************************************************
  * @file       app_rd03.c
  * @author     embedfire
  * @version     V1.0
  * @date        2024
  * @brief      RD03雷达模块串口 应用层功能接口
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

#include "rd03/app_rd03.h"
#include "rd03/bsp_rd03.h"
#include "usart/usart_com.h"
#include <string.h>
#include "debug/bsp_debug.h"
#include <stdlib.h>

static Rd03_ResultInfo rd03_result = {RD03_NOEXIST,0};

/**
  * @brief  Rd03串口 接收缓冲区复位
  * @param  无
  * @retval 无
  */
void Rd03_ReadBufferReset(void)
{
    memset(rd03_receive.buffer,NULL,rd03_receive.len);
    rd03_receive.len = 0;
    rd03_receive.read_flag = 0;
}

/**
  * @brief  Rd03串口 任务初始化
  * @param  buffer：待处理的数据
  * @retval 无
  */
void Rd03_TaskInit(void)
{
    Rd03_ReadBufferReset();
}

/**
  * @brief  Rd03串口 运行模式数据
  * @param  无
  * @retval 无
  */
void Rd03_Running_Handle(uint8_t* buffer,uint32_t size)
{
    char state0[] = "ON";
    char state1[] = "OFF";
    char symbol[] = "Range ";
    
    if(size < 2)
    {
        return;
    }
    
    if(strstr((char*)buffer,state0) != NULL)
    {
        char *data_temp =NULL;
        if((data_temp = strstr((char*)buffer,symbol)) != NULL)
        {
            rd03_result.range = atoi(data_temp+strlen(symbol));
            rd03_result.exist = RD03_EXIST;
            printf("有人，距离：%d cm\n",rd03_result.range);
        }
    }
    else if(strstr((char*)buffer,state1) != NULL)
    {
        rd03_result.exist = RD03_NOEXIST;
        printf("无人\n");
    }
}

/**
  * @brief  Rd03串口 任务
  * @param  无
  * @retval 无
  */
void Rd03_Task(void)
{
    if(rd03_receive.read_flag)
    {
//        USARTX_SendString(DEBUG_USARTX,"MCU已接收Rd03雷达模块串口数据\n");
//        USARTX_SendArray(DEBUG_USARTX,rd03_receive.buffer,rd03_receive.len);
        Rd03_Running_Handle(rd03_receive.buffer,rd03_receive.len);
        Rd03_ReadBufferReset();
    }
}

/**
  * @brief  Rd03串口 目标信息获取
  * @param  无
  * @retval 目标信息
  */
Rd03_ResultInfo Rd03_GetResult(void)
{
    return rd03_result;

}

/*****************************END OF FILE***************************************/
