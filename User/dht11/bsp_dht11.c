#include "dht11/bsp_dht11.h" 
#include "dwt/bsp_dwt.h"  

/* DHT11 GPIO配置 */
void DHT11_GPIO_Config(void)
{
    GPIO_InitTypeDef gpio_initstruct = {0};                  /* 定义一个 GPIO 结构体 */
    RCC_APB2PeriphClockCmd(DHT11_DATA_GPIO_CLK_PORT,ENABLE); /* 开启 DHT11 相关的GPIO外设/端口时钟 */    
    gpio_initstruct.GPIO_Pin    = DHT11_DATA_GPIO_PIN;       /* 指定GPIO引脚 */
    gpio_initstruct.GPIO_Mode   = GPIO_Mode_IPD;             /* 设置GPIO模式为 下拉输入 */
    gpio_initstruct.GPIO_Speed  = GPIO_Speed_50MHz;          /* 设置GPIO速率为50MHz */
    GPIO_Init(DHT11_DATA_GPIO_PORT,&gpio_initstruct); 
}

/* DHT11 引脚模式配置 */
void DHT11_DataPinModeConfig(GPIOMode_TypeDef mode)
{
    /* 定义一个 GPIO 结构体 */
    GPIO_InitTypeDef gpio_initstruct = {0};
    gpio_initstruct.GPIO_Pin    = DHT11_DATA_GPIO_PIN;
    gpio_initstruct.GPIO_Mode   = mode;
    gpio_initstruct.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(DHT11_DATA_GPIO_PORT,&gpio_initstruct);
    
}

/* 读取1个字节，即8bit */
uint8_t DHT11_ReadByte(void)
{
    uint8_t dht11_readbyte_temp = 0;
    
    for(uint8_t i = 0;i<8;i++)
    {
        /*  从机发送数据：每位数据以54us低电平标置开始，
            高电平持续时间为23~27us表示“1”，
            高电平持续时间为68~74us表示“0” */

        // DWT_Delay_Us(54); /* 延时54us，这里起始无需精确延时，直接在while循环中延时即可 */
        while(DHT11_DATA_IN() == Bit_RESET); /* 继续延时，直到高电平出现 */        
        
        /* 高电平出现之后延时40us，如果仍为高电平表示数据“1”，如果为低电平表示数据“0” */
        /* 理解：如果是“1”，则40us之后仍旧是高电平，如果是“0”，则40us之后为低电平（进入下一位的接收） */
        DWT_DelayUs(40);
        
        if(DHT11_DATA_IN() == Bit_SET) /* 高电平：“1” */
        {
            while(DHT11_DATA_IN() == Bit_SET);
            dht11_readbyte_temp |=(uint8_t)(0x1<<(7-i)); /* 把位7-i位置1，MSB先行（字节高位先发送） */
        }
        else  /* 低电平：“0” */
        {
            dht11_readbyte_temp &=(uint8_t)(~(0x1<<(7-i))) ; /* 把位7-i位清0，MSB先行 */
        }
    }
    return dht11_readbyte_temp;
}

/**
 * @brief  一次完整的数据传输为40bit，高位先出,
 * @param  dht11_data:数据接收区
 * @note   8bit 湿度整数 + 8bit 湿度小数 + 8bit 温度整数 + 8bit 温度小数 + 8bit 校验和(每次读出的温湿度数值是上一次测量的结果，欲获取实时数据,需连续读取2次，但不建议连续多次读取传感器，每次读取传感器间隔大于2秒即可获得准确的数据。)
 * @retval ERROR：失败，SUCCESS：成功
 */
ErrorStatus DHT11_ReadData(DHT11_DATA_TYPEDEF *dht11_data)
{
    uint8_t count_timer_temp = 0;
    
    /* 步骤1：主机设置输出状态，拉低数据线18ms以上，然后设置为输入模式，释放数据线 */
    DHT11_DataPinModeConfig(GPIO_Mode_Out_OD); /* 配置为开漏输出模式，由外部上拉电阻拉高电平 */
    DHT11_DATA_OUT(0); /* 拉低数据总线（SDA）*/
    DWT_DelayMs(20); /* 阻塞等待20ms（确保DHT11能检测到起始信号）*/
    DHT11_DATA_OUT(1); /* 释放数据总线（SDA）*/
    DHT11_DataPinModeConfig(GPIO_Mode_IPU); /* 配置为上拉输入模式，由外部上拉电阻拉高电平 */
    DWT_DelayUs(20);//等待上拉生效且回应,主机发送开始信号结束后,延时等待20-40us后
    
    
    /* 步骤2：从机接收到起始信号后，先拉低数据线83us以上，然后拉高数据线87us以上 */
    /* 通过读取数据总线（SDA）的电平，判断从机是否有低电平响应信号 如不响应则跳出，响应则向下运行*/
    if(DHT11_DATA_IN()== Bit_RESET)
    {
        /* 低电平信号 */
        count_timer_temp = 0;
        /* 轮询检测DHT11发出的低电平信号的持续时间，由于之前等待上拉延时了20us，
           因此这里计数值应该不到83就会直接跳出 */
        while(DHT11_DATA_IN() == Bit_RESET)
        {
            if(count_timer_temp++ >83)
            {
                return ERROR;
            }  
            DWT_DelayUs(1);
        }
                
        /* 高电平信号 */
        count_timer_temp = 0;
        /* 轮询检测DHT11发出的高电平信号的持续时间（87us） */
        while(DHT11_DATA_IN() == Bit_SET)
        {
            if(count_timer_temp++ >87)
            {
                return ERROR;
            }  
            DWT_DelayUs(1);
        }
        
        /* 应答信号接收成功并且已建立连接，数据接收：连续读取40字节 */
        dht11_data->humi_int  = DHT11_ReadByte();        //湿度高8位
        dht11_data->humi_deci = DHT11_ReadByte();        //湿度低8位
        dht11_data->temp_int  = DHT11_ReadByte();        //温度高8位
        dht11_data->temp_deci = DHT11_ReadByte();        //温度低8位
        dht11_data->check_sum = DHT11_ReadByte();        //校验和
        
        /* 最后一位数据发送后，从机将拉低数据总线SDA54us，随后从机释放总线 */
        DWT_DelayUs(54);
        
        /* 校验和：加和前4个字节数据，校验和字节为加和结果的低8位 */
        if(dht11_data->check_sum == dht11_data->humi_int+dht11_data->humi_deci+dht11_data->temp_int+dht11_data->temp_deci)
        {
            return SUCCESS;
        }
        else
        {
            return ERROR;
        }
    }
    else
    {
        return ERROR;
    }
}

/*********************************************END OF FILE**********************/
