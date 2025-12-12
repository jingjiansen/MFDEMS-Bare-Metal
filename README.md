# 项目1：裸机版多功能桌面环境监测系统

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

|外设|GPIO端口|引脚|工作模式|时钟|
| :-----: | :--------: | :----: | :----------------: | :--------------------: |
|LED灯|B|13|GPIO_Mode_Out_PP|RCC_APB2Periph_GPIOB|
|KEY1|A|0|||
|KEY2|C|13|||
|KEY3|B|15|||

GPIO即通用输入输出，其具有多个端口GPIOA、GPIOB、GPIOC等，每个端口具有多个引脚。GPIO引脚有多种工作模式，这种工作模式因连接的外设和要求的初始状态不同而有所差异。

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

低电平触发下，使用`GPIO_ResetBits(GPIOx,GPIO_Pin)`​来点灯（即置GPIO引脚高电平），使用`GPIO_SetBits(GPIOx,GPIO_Pin)`​来关灯（即置GPIO引脚低电平）。而在翻转灯状态时直接操作了控制GPIO引脚的输出电平寄存器`ODR`​，通过异或操作实现翻转。ODR寄存器中的每一位对应一个GPIO引脚，通过`GPIO_Pin`掩码（对应引脚置1）来保证只影响对应引脚，通过异或来翻转对应引脚状态。

```C
void LED_TOGGLE(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->ODR ^= GPIO_Pin;  // 直接操作输出数据寄存器进行位异或
}
```

#### 按键KEY

##### 宏定义

KEY1使用GPIOA端口Pin0引脚；KEY2使用GPIOC端口引脚13。两者均配置为中断触发、KEY3使用GPIOB端口Pin15引脚，非中断触发，系统时钟中断中进行状态检查。

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
```

### 应用层部分
