#ifndef __BSP_GPIO_TRACKE_H
#define __BSP_GPIO_TRACKE_H

#include "stm32f10x.h"

/* 定义 TRACKE 连接的GPIO端口, 用户只需要修改下面的代码即可改变控制的 TRACKE 引脚 */

//TRACKE_DO
#define TRACKE_DO_GPIO_PORT          GPIOA                           /* GPIO端口 */
#define TRACKE_DO_GPIO_CLK_PORT      RCC_APB2Periph_GPIOA            /* GPIO端口时钟 */
#define TRACKE_DO_GPIO_PIN           GPIO_Pin_11                      /* 对应PIN脚 */

void TRACKE_GPIO_Config(void);
BitAction TRACKE_Scan(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

#endif /* __BSP_GPIO_TRACKE_H */
