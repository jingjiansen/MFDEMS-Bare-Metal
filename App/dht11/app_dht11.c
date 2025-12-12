#include "dht11/app_dht11.h"
#include "dht11/bsp_dht11.h" 
#include "usart/usart_com.h"
#include "debug/bsp_debug.h"
#include "oled/app_oled.h" 

Dht11_TaskInfo dht11_rd_task  = {0};
DHT11_DATA_TYPEDEF dht11_data = {0};

/**
  * @brief  DHT11温湿度传感器数据读取 计数复位
  * @param  无
  * @retval 无
  */
void Dht11_TaskReset(void)
{
    dht11_rd_task.timer = dht11_rd_task.cycle;
    dht11_rd_task.flag  = 0;
}

/**
  * @brief  DHT11温湿度传感器数据读取 任务初始化
  * @param  dht11_rd_task_cycle: 任务轮询周期 单位ms(可修改系统节拍定时器)
  * @retval 无
  */
void Dht11_TaskInit(uint32_t dht11_rd_task_cycle)
{
    dht11_rd_task.cycle = dht11_rd_task_cycle;
    Dht11_TaskReset();
}

/**
 * @brief  DHT11任务  
 * @param  无
 * @retval 无
 */
void Dht11_Task(void)
{
    if(dht11_rd_task.flag == 1 && menu == 0x22)
    {
        if(DHT11_ReadData(&dht11_data) == SUCCESS)
        {
           printf("READ_DHT11_DATA SUCCESS! \r\n");
           if(dht11_data.humi_deci&0x80)//判断是否低于0
           {
               printf("湿度为 -%d.%d％ＲＨ \r\n",dht11_data.humi_int,dht11_data.humi_deci);
           
           }
           else
           {
               printf("湿度为 %d.%d％ＲＨ \r\n",dht11_data.humi_int,dht11_data.humi_deci);
           
           }
           if(dht11_data.temp_deci&0x80)//判断是否低于0
           {
               printf("温度为 -%d.%d ℃ \r\n",dht11_data.temp_int,dht11_data.temp_deci);
           
           }
           else
           {
               printf("温度为 %d.%d ℃ \r\n",dht11_data.temp_int,dht11_data.temp_deci);
             
           }
        }
        else
		{
            printf("READ_DHT11_DATA ERROR!\r\n");
		}
        content_show_flag = 1;
        Dht11_TaskReset();
    }
}

/*****************************END OF FILE***************************************/
