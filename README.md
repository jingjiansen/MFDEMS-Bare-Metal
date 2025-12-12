# MFDEMS-Bare-Metal

这是裸机版多功能桌面环境监测系统的实现，旨在实践学习到的嵌入式知识。

## 结果展示

https://github.com/user-attachments/assets/01c243bb-c6ad-4281-b750-c4530ed327f5

## 程序功能

### 一级功能

1. 开机连续显示3张小猫图像--上下左右居中--总共持续1s；
2. 第一步过后显示主页；
3. 可通过板载按键key1、key2左右切换一级菜单。分别是主页、音乐、温湿度、灯具控制、木鱼；
4. 可通过扩展按键key进入、退回一级菜单。分别是主页、音乐、温湿度、灯具控制、木鱼。

### 二级功能

1. 主页内容：招呼图；
2. 音乐内容：全屏位图；
3. 灯具内容：开关灯(LED4)；
4. 打坐内容：敲木鱼。使用板载按键key1、key2可以计数或清零敲击次数；
5. 温湿度内容：500ms更新一次温湿度数据。

## 功能实现

### 驱动部分

#### GPIO简介

本项目使用到GPIO的外接硬件部分有：外接LED灯、板载按键1、板载按键2、板载按键3、LED屏幕、DHT11温湿度传感器。

GPIO即通用输入输出，其具有多个端口GPIOA、GPIOB、GPIOC等，每个端口具有多个引脚。GPIO引脚有多种工作模式，这种工作模式因连接的外设和要求的初始状态不同而有所差异。

| 外设 | GPIO端口 | 引脚 |        工作模式        | 时钟 |
| :-----: | :--------: | :----: | :----------------------: | :----: |
| LED灯 |    B    |  13  |            `GPIO_Mode_Out_PP`            |   `RCC_APB2Periph_GPIOB`   |
| KEY1 |    A    |  0  |            `GPIO_Mode_IN_FLOATING`            |   `RCC_APB2Periph_GPIOA`   |
| KEY2 |    C    |  13  |            `GPIO_Mode_IN_FLOATING`            |   `RCC_APB2Periph_GPIOC`   |
| KEY3 |    B    |  15  |            `GPIO_Mode_IPD`            |   `RCC_APB2Periph_GPIOB`   |
| DHT11 |    B    |  12  | 初始为`GPIO_Mode_IPD`，运行过程中配置 |   `RCC_APB2Periph_GPIOB`   |

```C
typedef enum
{ 
  /* 模拟输入（ADC采集） */
  GPIO_Mode_AIN = 0x0,
  /* 模拟输入：内部无上拉下拉电阻，引脚电平完全由外部电路决定，容易受到外部干扰 */
  GPIO_Mode_IN_FLOATING = 0x04,
  /* 下拉输入：内部下拉电阻，默认低电平，外部高电平有效 */
  GPIO_Mode_IPD = 0x28,
  /* 上拉输入：内部上拉电阻，默认高电平，外部低电平有效 */
  GPIO_Mode_IPU = 0x48,
  /* 开漏输出：只能输出低电平，高电平由外部上拉电阻拉高（IIC使用） */
  GPIO_Mode_Out_OD = 0x14,
  /* 推挽输出：能够强力驱动高低电平输出（LED灯） */
  GPIO_Mode_Out_PP = 0x10,
  /* 复用开漏输出：同开漏输出，但由硬件外设控制 */
  GPIO_Mode_AF_OD = 0x1C,
  /* 复用推挽输出：同推挽输出，但由硬件外设控制 */
  GPIO_Mode_AF_PP = 0x18
}GPIOMode_TypeDef;

```

#### LED灯

##### 宏定义

LED4为外接LED灯，其使用GPIOB端口的PIN13引脚。

```C
// LED4灯相关宏定义
#define LED4_GPIO_PORT          GPIOB                           /* GPIO端口 */
#define LED4_GPIO_CLK_PORT      RCC_APB2Periph_GPIOB            /* GPIO端口时钟 */
#define LED4_GPIO_PIN           GPIO_Pin_5                      /* 对应PIN脚 */
```

##### 初始化及灯状态操作

LED灯为低电平触发，即端口输出低电平LED灯点亮，输出高电平灯灭。因此在引脚初始化时将其初始状态置为高电平状态。并且设置该GPIO引脚的工作模式为推挽输出`GPIO_Mode_Out_PP`。推挽输出能够同时提供强力的高电平和低电平，是一种驱动方式，与此常见的其他GPIO方式还有开漏输出（只有内部下拉，上拉需外部）等。

```C
// LED4GPIO引脚初始化
/* 开启 LED 相关的GPIO外设/端口时钟 */
RCC_APB2PeriphClockCmd(LED4_GPIO_CLK_PORT,ENABLE);
    
/* 初始化为高电平（灯灭） */
GPIO_SetBits(LED4_GPIO_PORT,LED4_GPIO_PIN);
    
/*选择要控制的GPIO引脚、设置GPIO模式为 推挽模式、设置GPIO速率为50MHz*/
gpio_initstruct.GPIO_Pin    = LED4_GPIO_PIN;
gpio_initstruct.GPIO_Mode   = GPIO_Mode_Out_PP;
gpio_initstruct.GPIO_Speed  = GPIO_Speed_50MHz;
GPIO_Init(LED4_GPIO_PORT,&gpio_initstruct);
```

此外还有一些对于灯状态的操作，如控制灯开、灯灭、翻转灯的状态。

低电平触发下，使用`GPIO_ResetBits(GPIOx,GPIO_Pin)`来点灯（即置GPIO引脚高电平），使用`GPIO_SetBits(GPIOx,GPIO_Pin)`来关灯（即置GPIO引脚低电平）。而在翻转灯状态时直接操作了控制GPIO引脚的输出电平寄存器`ODR`，通过异或操作实现翻转。ODR寄存器中的每一位对应一个GPIO引脚，通过`GPIO_Pin`掩码（对应引脚位置1）来保证只影响相关引脚，通过异或来翻转对应引脚状态。

```C
void LED_TOGGLE(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->ODR ^= GPIO_Pin;  // 直接操作输出数据寄存器进行位异或
}
```

#### 按键KEY

##### 宏定义与中断说明

KEY1使用GPIOA端口Pin0引脚；KEY2使用GPIOC端口引脚13。两者均使用按键中断进行状态检查、KEY3使用GPIOB端口Pin15引脚，非中断，在任务轮询中直接检查相关GPIO引脚状态来判断。

###### 关于按键中断

1. 按键一端与相关引脚相连，另一端接地（低电平有效）。没按下时相关引脚被内部上拉到高电平，按下瞬间引脚被拉低，出现一个下降沿；
2. 开启GPIO时钟之后将给GPIO相关模块供电，后续配置才会有效。配置引脚工作模式为浮空输入，即纯粹做输入；
3. 接着开启AFIO时钟，给引脚到EXTI（外部中断线）供电，后续配置AFIO寄存器才会有效；
4. 接着将相关引脚挂到外部中断线上；
5. 配置EXTI：配置相关外部中断线，将其设置为中断模式、下降沿触发并使能。此时硬件电路已经就绪，当按键引脚出现下降沿时，相关外部中断线EXTI将挂起中断标志位；
6. 配置NVIC，将相关的外部中断的抢占优先级、子优先级写入NVIC，并使能该通道。此时外部中断线一旦挂起标志位，NVIC就会去向量表里找函数入口；
7. 实现中断服务函数。首先通过宏定义`#define KEY1_EXTI_IRQHANDLER EXTI0_IRQHandler`为真正的中断向量入口起了一个别名。启动文件中已经将中断处理函数`EXTI0_IRQHandler和EXTI0_IRQHandler`硬编码进了向量表，在C文件中实现同名函数，这里为方便记忆为`KEY1_EXTI_IRQHANDLER`，链接器在最终链接阶段自动将其地址填到向量表对应的位置中去。

EXTI外部中断线与引脚编号一一对应，即同一端口的不同引脚不能共享一条EXTI线，而不同端口的同一引脚可共享一条EXTI线。在中断函数中通过读取引脚来区分。

```C
//KEY1
#define KEY1_GPIO_PORT          GPIOA                           /* GPIO端口 */
#define KEY1_GPIO_CLK_PORT      RCC_APB2Periph_GPIOA            /* GPIO端口时钟 */
#define KEY1_GPIO_PIN           GPIO_Pin_0                      /* 对应PIN脚 */

#define KEY1_EXTI_PORTSOURCE    GPIO_PortSourceGPIOA            /* 中断端口源 */
#define KEY1_EXTI_PINSOURCE     GPIO_PinSource0                 /* 中断PIN源 */
#define KEY1_EXTI_LINE          EXTI_Line0                      /* 中断线 */
#define KEY1_EXTI_IRQ           EXTI0_IRQn                      /* 外部中断向量号 */
#define KEY1_EXTI_IRQHANDLER    EXTI0_IRQHandler                /* 中断处理函数 */

//KEY2
#define KEY2_GPIO_PORT          GPIOC                           /* GPIO端口 */
#define KEY2_GPIO_CLK_PORT      RCC_APB2Periph_GPIOC            /* GPIO端口时钟 */
#define KEY2_GPIO_PIN           GPIO_Pin_13                     /* 对应PIN脚 */

#define KEY2_EXTI_PORTSOURCE    GPIO_PortSourceGPIOC            /* 中断端口源 */
#define KEY2_EXTI_PINSOURCE     GPIO_PinSource13                /* 中断PIN源 */
#define KEY2_EXTI_LINE          EXTI_Line13                     /* 中断线 */
#define KEY2_EXTI_IRQ           EXTI15_10_IRQn                  /* 外部中断向量号 */
#define KEY2_EXTI_IRQHANDLER    EXTI15_10_IRQHandler            /* 中断处理函数 */

//KEY3
#define KEY3_GPIO_PORT          GPIOB                           /* GPIO端口 */
#define KEY3_GPIO_CLK_PORT      RCC_APB2Periph_GPIOB            /* GPIO端口时钟 */
#define KEY3_GPIO_PIN           GPIO_Pin_15 
```

##### 数据结构设计

为方便统一管理按键，设计按键结构体，将与按键有关的所有元素统一管理。其中包含的其他数据结构定义详见代码部分`user/key/bsp_gpio_key.h`。

```C
/* 按键的结构体 */
typedef struct
{
    GPIO_TypeDef*       GPIOx;          /* 按键对应的GPIO端口 */
    uint16_t            GPIO_Pin;       /* 按键对应的GPIO引脚 */
    KEY_TriggerLevel    triggerlevel;   /* 按键对应的触发电平 */
    KEY_Status          status;         /* 按键的状态 */
    uint64_t            press_time;     /* 按键按下的时间 */
    uint64_t            release_time;   /* 按键释放的时间 */
    KEY_Event           event;          /* 按键的事件类型 */
    KEY_ClickType       clicktype;      /* 按键的点击类型 */
}KEY_Info;

extern KEY_Info key1_info;
extern KEY_Info key2_info;
```

##### GPIO配置（以按键1为例）

```C
/* 按键GPIO配置 */
void KEY_GPIO_Config(void)
{
    GPIO_InitTypeDef gpio_initstruct = {0};                 /* 按键GPIO结构体 */

    RCC_APB2PeriphClockCmd(KEY1_GPIO_CLK_PORT,ENABLE);      /* 开启KEY1相关的GPIO外设/端口时钟 */    
    GPIO_SetBits(KEY1_GPIO_PORT,KEY1_GPIO_PIN);             /* 按键为低电平触发，这里初始化为高电平 */    
    gpio_initstruct.GPIO_Pin    = KEY1_GPIO_PIN;
    gpio_initstruct.GPIO_Mode   = GPIO_Mode_IN_FLOATING;
    gpio_initstruct.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(KEY1_GPIO_PORT,&gpio_initstruct);
}
```

##### 外部中断线配置（以按键1为例）

```C
/* 外部中断线配置（GPIO>EXTI） */
void KEY_Mode_Config(void)
{
    EXTI_InitTypeDef exti_initstruct = {0};                         /* 定义一个 EXTI 初始化结构体 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);             /* 开启 AFIO 相关的时钟 */

    GPIO_EXTILineConfig(KEY1_EXTI_PORTSOURCE, KEY1_EXTI_PINSOURCE); /* 选择中断信号源，将KEY1的引脚映射到EXTI0 */
    exti_initstruct.EXTI_Line       = EXTI_Line0;                   /* 选择中断LINE */
    exti_initstruct.EXTI_Mode       = EXTI_Mode_Interrupt;          /* 选择中断模式*/
    exti_initstruct.EXTI_Trigger    = EXTI_Trigger_Falling;         /* 选择触发方式*/
    exti_initstruct.EXTI_LineCmd    = ENABLE;                       /* 使能中断*/
    EXTI_Init(&exti_initstruct);
}
```

##### 配置NVIC（以按键1为例）

```C
/* 配置NVIC中断，中断优先级和中断通道 */
void KEY_NVIC_Config(void)
{
    NVIC_InitTypeDef nvic_initstruct = {0};                                 /* 定义一个 NVIC 结构体 */

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);                     /* 开启 AFIO 相关的时钟 */

    nvic_initstruct.NVIC_IRQChannel                     = KEY1_EXTI_IRQ;    /* 配置中断源 */
    nvic_initstruct.NVIC_IRQChannelPreemptionPriority   =  1;               /* 配置抢占优先级 */
    nvic_initstruct.NVIC_IRQChannelSubPriority          =  0;               /* 配置子优先级 */
    nvic_initstruct.NVIC_IRQChannelCmd                  =  ENABLE;          /* 使能配置中断通道 */
    NVIC_Init(&nvic_initstruct);
}
```

##### 中断服务例程（以按键1为例）

```C
void KEY1_EXTI_IRQHANDLER(void)
{
    if(EXTI_GetFlagStatus(KEY1_EXTI_LINE) == SET)
    {
        left_shift_flag = 1; /* 标记左移 */
        menu_show_flag = 1;  /* 标记显示菜单 */
        EXTI_ClearITPendingBit(KEY1_EXTI_LINE); /* 清除中断标志位 */
    }
}
```




#### 温湿度传感器DHT11

DHT11为温湿度传感器，供4个引脚，是单线双向通信机制，即只使用一根数据线进行数据交换和控制。

数据格式为：8bit湿度整数数据+8bit湿度小数数据+8bit温度整数数据+8bit温度小数数据+8bit校验和数据。其中，湿度小数部分为0；当温度低于0℃时，温度数据的低8位的最高位为1。

数据传送正确时，校验和等于四字节数据之和。

通信时序为：建立连接、数据接收两部分。

| 引脚 |     作用     |     备注     |
| :----: | :------------: | :-------------: |
| VDD |     电源     |              |
| DATA | 输出串行总线 | 连接到GPIOB12 |
|  NC  |     空脚     |              |
| GND |     接地     |              |

##### 建立连接

1. DHT11上电后需等待1s才会稳定，在此期间不能发送任何指令，数据线SDA通过上拉电阻一直保持高电平状态。
2. 主机GPIO引脚设置为输出状态，拉低数据线SDA18ms以上，然后主机GPIO引脚设置为输入状态，释放数据线SDA，由于上拉电阻，数据线进入高电平状态；
3. 从机DHT11接收到起始信号后，先拉低数据线SDA（83us）作为应答信号，再拉高数据线SDA（87us）表示连接建立成功，通知主机准备接收数据；
4. 从机开始发送数据：从机发送40位数据，每位数据都以54us的低电平开始，主机通过判断低电平后的高电平时间来决定接收1还是0：

    1. “0”：54us的低电平和23~27us的高电平；
    2. “1”：54us的低电平和68~74us的高电平。

最后一位数据发送结束后，从机拉低数据线SDA54us，随后从机释放数据线SDA，数据线SDA进入空闲状态，由于上拉电阻，此时数据线SDA为高电平状态。

##### 宏定义

这里使用GPIOB12作为DHT11的数据总线，与其相连，进行数据传输。

```C
/* DHT11_DATA 引脚配置 */
#define DHT11_DATA_GPIO_PORT          GPIOB                           /* GPIO端口 */
#define DHT11_DATA_GPIO_CLK_PORT      RCC_APB2Periph_GPIOB            /* GPIO端口时钟 */
#define DHT11_DATA_GPIO_PIN           GPIO_Pin_12                     /* 对应PIN脚 */

#define DHT11_DATA_IN()         GPIO_ReadInputDataBit(DHT11_DATA_GPIO_PORT,DHT11_DATA_GPIO_PIN)
#define DHT11_DATA_OUT(VALUE)   if(VALUE)   GPIO_SetBits(DHT11_DATA_GPIO_PORT,DHT11_DATA_GPIO_PIN);\
                                else      GPIO_ResetBits(DHT11_DATA_GPIO_PORT,DHT11_DATA_GPIO_PIN)
```

##### 数据结构与函数声明

由于在进行数据传输时PB12需要既作为输入（接收来自DHT11的数据）、又作为输出（主机需主动拉低数据线以发送起始信号），因此需要实时改变引脚工作模式，使用函数`DHT11_DataPinModeConfig()`来进行模式切换。

函数`DHT11_ReadByte()`则负责读取1字节数据；

函数`DHT11_ReadData()`负责读取一帧数据，即完整的温湿度数据，并将其存储在指定缓冲区中。

```C
typedef struct
{                            
   uint8_t humi_int;        //湿度的整数部分
   uint8_t humi_deci;       //湿度的小数部分
   uint8_t temp_int;        //温度的整数部分
   uint8_t temp_deci;       //温度的小数部分
   uint8_t check_sum;       //校验和                              
}DHT11_DATA_TYPEDEF;                               
                        
void DHT11_GPIO_Config(void);
void DHT11_DataPinModeConfig(GPIOMode_TypeDef mode);
uint8_t DHT11_ReadByte(void);
ErrorStatus DHT11_ReadData(DHT11_DATA_TYPEDEF *dht11_data);
```

##### 函数实现

这里仅对主要的字节读取函数`DHT11_ReadByte()`和帧读取函数`DHT11_ReadData()`进行说明。

###### 帧读取`DHT11_ReadData()`

该函数负责读取一个完整数据帧（40字节数据）和前期的连接建立。具体过程见代码注释。

```C
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
```

###### 字节读取`DHT11_ReadByte()`

该函数用于在主机与DHT11建立连接之后读取数据字节。

```C
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
```

### 应用层部分
